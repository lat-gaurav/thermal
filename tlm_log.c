/* tlm_log.c -- log gimbal telemetry with timestamps, and NOTHING else.
 *
 * Run this alongside record_raw during a capture. It never opens the camera, so
 * it cannot contend for it (the core allows one streamer) and it cannot cost
 * record_raw a single frame. It does no per-frame work at all: 36 B per sample
 * at ~1 kHz is 36 kB/s, which is 0.08% of the video's 43 MB/s.
 *
 *   ./record_raw -t 60 &        # frames + .idx timestamps
 *   ./tlm_log -t 60 -o rec.tlm.csv
 *   ./flow_post -i <capture.gray> -a rec.tlm.csv
 *
 * Wire format @ 921600: A5 5A | 8 x float32 LE | CRC16 (MCRF4XX over
 * bytes[2..33]), 36 B. float[0]=yaw, float[1]=pitch, degrees. Floats 2..7 are
 * undocumented but logged verbatim -- they cost nothing to keep and cannot be
 * recovered later. Stamped CLOCK_MONOTONIC on arrival, the same clock as
 * v4l2_buffer.timestamp, so the two files join with no conversion.
 *
 * Opened READ-ONLY: nothing is ever transmitted to the gimbal.
 *
 * Build:  gcc -O2 -Wall -Wextra -o tlm_log tlm_log.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define TLM_LEN    36
#define TLM_FLOATS 8
#define SERBUF     8192

static const char *TLM_LINK = "/dev/local_dds";
static const char *TLM_BYID_GLOB =
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK = "/dev/ttyUSB0";
static const char *REC_ROOT = "recordings";

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int s) { (void)s; stop_flag = 1; }

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fputc('\n', stderr); vfprintf(stderr, fmt, ap); va_end(ap); fputc('\n', stderr);
    exit(1);
}

static double now_mono(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int mkdir_p(const char *path)
{
    char tmp[600]; snprintf(tmp, sizeof tmp, "%s", path);
    for (char *q = tmp + 1; *q; q++) {
        if (*q != '/') continue;
        *q = '\0';
        if (mkdir(tmp, 0755) == -1 && errno != EEXIST) return -1;
        *q = '/';
    }
    if (mkdir(tmp, 0755) == -1 && errno != EEXIST) return -1;
    return 0;
}

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

static void usage(const char *me)
{
    printf(
"Log gimbal telemetry with CLOCK_MONOTONIC timestamps. Never opens the camera.\n"
"Safe to run alongside record_raw -- no contention, no per-frame cost.\n"
"\n"
"usage: %s [-t SECS] [-o CSV] [-g DEV] [-B BAUD] [-D ROOT] [-h]\n"
"\n"
"  -t SECS    stop after SECS seconds (default: until Ctrl-C)\n"
"  -o CSV     write here ('-' for stdout; default: a timestamped run folder)\n"
"  -g DEV     telemetry port (default %s, else the FTDI by-id path)\n"
"  -B BAUD    115200 or 921600 (default 921600)\n"
"  -D ROOT    run-folder root (default %s/)\n"
"\n"
"CSV: t_angle,yaw,pitch,f2,f3,f4,f5,f6,f7\n"
"t_angle is CLOCK_MONOTONIC seconds -- the same clock record_raw's .idx uses.\n",
        me, TLM_LINK, REC_ROOT);
}

int main(int argc, char **argv)
{
    const char *gdev_opt = NULL, *out = NULL, *root = REC_ROOT;
    double secs = 0;
    speed_t baud = B921600; long baud_n = 921600;
    int c;

    while ((c = getopt(argc, argv, "t:o:g:B:D:h")) != -1) {
        switch (c) {
        case 't': secs = atof(optarg); break;
        case 'o': out = optarg; break;
        case 'g': gdev_opt = optarg; break;
        case 'B':
            baud_n = strtol(optarg, NULL, 10);
            if (baud_n == 115200) baud = B115200;
            else if (baud_n == 921600) baud = B921600;
            else die("-B must be 115200 or 921600 (got %s)", optarg);
            break;
        case 'D': root = optarg; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }

    char *gdev = NULL;
    if (gdev_opt) gdev = strdup(gdev_opt);
    else if (access(TLM_LINK, F_OK) == 0) gdev = strdup(TLM_LINK);
    else {
        glob_t g;
        if (glob(TLM_BYID_GLOB, 0, NULL, &g) == 0 && g.gl_pathc > 0)
            gdev = strdup(g.gl_pathv[0]);
        globfree(&g);
        if (!gdev && access(TLM_FALLBACK, F_OK) == 0) gdev = strdup(TLM_FALLBACK);
    }
    if (!gdev)
        die("no telemetry port found (looked for %s, then %s, then %s)\n"
            "  is the FTDI cable plugged in?  ls -l /dev/serial/by-id/",
            TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);

    char outdir[512] = "", csvpath[640];
    int to_stdout = (out && strcmp(out, "-") == 0);
    if (out && !to_stdout) {
        snprintf(csvpath, sizeof csvpath, "%s", out);
        const char *slash = strrchr(csvpath, '/');
        if (slash && slash != csvpath)
            snprintf(outdir, sizeof outdir, "%.*s", (int)(slash - csvpath), csvpath);
    } else if (!out) {
        time_t tt = time(NULL); struct tm tm; localtime_r(&tt, &tm);
        unsigned dup = 0;
        for (;;) {
            int len;
            if (dup == 0)
                len = snprintf(outdir, sizeof outdir, "%s/%04d-%02d-%02d/%02d%02d%02d",
                    root, tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec);
            else
                len = snprintf(outdir, sizeof outdir, "%s/%04d-%02d-%02d/%02d%02d%02d-%u",
                    root, tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec, dup);
            if (len < 0 || (size_t)len >= sizeof outdir) die("-D path too long");
            if (access(outdir, F_OK) != 0) break;
            if (++dup > 999) die("cannot find an unused folder under %s", root);
        }
        snprintf(csvpath, sizeof csvpath, "%s/telemetry.csv", outdir);
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int fd = open(gdev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) die("cannot open %s: %s\n  in the dialout group? (id -nG)",
                    gdev, strerror(errno));
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) die("tcgetattr %s: %s", gdev, strerror(errno));
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud); cfsetospeed(&tio, baud);
    tio.c_cflag |= CLOCAL | CREAD; tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) < 0) die("tcsetattr: %s", strerror(errno));
    tcflush(fd, TCIFLUSH);

    FILE *csv = stdout;
    if (!to_stdout) {
        if (outdir[0] && mkdir_p(outdir) == -1)
            die("cannot create folder %s: %s", outdir, strerror(errno));
        csv = fopen(csvpath, "w");
        if (!csv) die("cannot create %s: %s", csvpath, strerror(errno));
    }
    fprintf(csv, "# tlm_log: gimbal telemetry, receive only\n");
    fprintf(csv, "# port=%s baud=%ld\n", gdev, baud_n);
    fprintf(csv, "# t_angle = CLOCK_MONOTONIC seconds at arrival\n");
    fprintf(csv, "# float[0]=yaw float[1]=pitch degrees; f2..f7 undocumented\n");
    fprintf(csv, "t_angle,yaw,pitch,f2,f3,f4,f5,f6,f7\n");

    fprintf(stderr, "telemetry   %s @ %ld, receive only\n", gdev, baud_n);
    if (!to_stdout) fprintf(stderr, "csv         %s\n", csvpath);
    fprintf(stderr, "logging     Ctrl-C to stop\n");

    uint8_t sbuf[SERBUF]; size_t slen = 0;
    unsigned long ok = 0, ncrc = 0;
    double t0 = now_mono(), t_status = 0;

    while (!stop_flag) {
        struct pollfd p = { .fd = fd, .events = POLLIN, .revents = 0 };
        int r = poll(&p, 1, 500);
        if (r < 0) { if (errno == EINTR) continue; break; }
        if (r > 0 && (p.revents & POLLIN)) {
            ssize_t n = read(fd, sbuf + slen, sizeof sbuf - slen);
            double t = now_mono();
            if (n > 0) {
                slen += (size_t)n;
                size_t i = 0;
                while (slen - i >= TLM_LEN) {
                    if (sbuf[i] != 0xA5 || sbuf[i+1] != 0x5A) { i++; continue; }
                    uint16_t want = (uint16_t)(sbuf[i+34] | (sbuf[i+35] << 8));
                    /* advance 1 on a CRC miss: a real frame can start one byte
                     * into a false A5 5A */
                    if (crc16(&sbuf[i+2], 32) != want) { ncrc++; i++; continue; }
                    float fl[TLM_FLOATS];
                    memcpy(fl, &sbuf[i+2], sizeof fl);
                    fprintf(csv, "%.6f", t);
                    for (int k = 0; k < TLM_FLOATS; k++)
                        fprintf(csv, ",%.4f", (double)fl[k]);
                    fputc('\n', csv);
                    ok++;
                    i += TLM_LEN;
                }
                memmove(sbuf, sbuf + i, slen - i);
                slen -= i;
                if (slen == sizeof sbuf) slen = 0;   /* pathological: resync */
            }
        }
        double now = now_mono();
        if (now - t_status >= 0.5) {
            t_status = now;
            double el = now - t0;
            fprintf(stderr, "\r  %6.1fs  %8lu samples  %6.1f Hz  crc err %lu   ",
                    el, ok, el > 0 ? ok / el : 0.0, ncrc);
            fflush(stderr);
        }
        if (secs > 0 && now - t0 >= secs) break;
    }

    double el = now_mono() - t0;
    close(fd);
    if (csv != stdout) fclose(csv);

    fprintf(stderr, "\nsamples     %lu in %.2f s  =  %.1f Hz\n", ok, el,
            el > 0 ? ok / el : 0.0);
    fprintf(stderr, "crc errors  %lu\n", ncrc);
    if (!to_stdout) fprintf(stderr, "csv         %s\n", csvpath);
    if (ok == 0) {
        fprintf(stderr, "\nFAILED      no telemetry decoded -- wrong baud, or the"
                        " gimbal is not streaming\n");
        free(gdev);
        return 1;
    }
    free(gdev);
    return 0;
}
