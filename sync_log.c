/* sync_log.c -- pair every camera frame with the gimbal angle that was true at
 * the moment it was captured, both stamped on the SAME clock.
 *
 * Ported from ~/drone_detection/sync_log.c. The logic is unchanged; what differs
 * is this repo's conventions -- devices pinned by USB serial rather than by
 * videoN/ttyUSBn, and output landing in the same timestamped run folder that
 * record_raw.c uses. The CSV columns are byte-for-byte the originals, so any
 * analysis already written against them still works.
 *
 * One process, one poll() loop, no threads. Two inputs:
 *
 *   video      V4L2 mmap capture. The timestamp used is the kernel's own
 *              v4l2_buffer.timestamp, taken when the frame's first USB payload
 *              lands -- NOT when this program dequeues it. That is ~30 ms
 *              earlier and far steadier than anything userspace can measure.
 *              Pixels are never read; the buffer is requeued immediately, so
 *              this costs nothing per frame.
 *
 *   telemetry  Gimbal telemetry @ 921600: A5 5A | 8 x float32 LE | CRC16
 *              (MCRF4XX over bytes[2..33]), 36 B, ~1 kHz. float[0]=yaw,
 *              float[1]=pitch, degrees. Stamped with CLOCK_MONOTONIC on
 *              arrival. Opened READ-ONLY: nothing is ever sent to the gimbal.
 *
 * Both clocks are CLOCK_MONOTONIC and that is verified, not assumed: uvcvideo's
 * module parameter reads clock=CLOCK_MONOTONIC, and every buffer comes back
 * flagged V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC. So the two streams are directly
 * comparable with no conversion. Measured on this Jetson: 1011 Hz angles vs
 * 32.8 fps frames = ~31 angle samples per frame, so the nearest match is
 * typically ~0.5 ms away.
 *
 * WHAT THIS CANNOT DO: the camera sends no clock of its own. The buffer flag
 * says TSTAMP_SRC_SOE ("start of exposure"), but uvcvideo here runs with
 * hwtimestamps=0 and the core's UVC payload headers carry neither PTS nor SCR,
 * so that stamp is really derived from payload arrival on the host. Frames and
 * angles are therefore consistently aligned RELATIVE to each other, but may be
 * jointly shifted from true exposure by an unknown fixed offset. To pin it
 * down, slew the gimbal sharply and see which frame the motion first appears in.
 *
 * NOTE ON RUNNING THIS: the core allows exactly one streamer. sync_log and
 * record_raw cannot both hold the camera, so this measures and records the
 * pairing but cannot annotate a record_raw capture -- see the README note.
 *
 * Build:  gcc -O2 -Wall -Wextra -o sync_log sync_log.c
 * Run:    ./sync_log -t 30          (Ctrl-C to stop; CSV -> run folder)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define NBUFS      4
#define TLM_LEN    36
#define TLM_FLOATS 8
#define RINGSZ     8192          /* ~8 s of 1 kHz angles */
#define SERBUF     8192
#define DEF_W      1280
#define DEF_H      1024

/* Same pinning rule as record_raw.c: by USB serial, never by videoN/ttyUSBn,
 * because those numbers drift across re-enumeration. */
static const char *THERMAL_LINK = "/dev/thermal0";
static const char *VID_BYID_GLOB =
    "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";
/* The project's own udev rule (99-drone-ports.rules) names the FTDI cable
 * local_dds; prefer it, then the serial-pinned path, then the bare tty. */
static const char *TLM_LINK = "/dev/local_dds";
static const char *TLM_BYID_GLOB =
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK = "/dev/ttyUSB0";

static const char *REC_ROOT = "recordings";

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fputc('\n', stderr);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(1);
}

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static double now_mono(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int mkdir_p(const char *path)
{
    char tmp[600];
    snprintf(tmp, sizeof tmp, "%s", path);
    for (char *q = tmp + 1; *q; q++) {
        if (*q != '/') continue;
        *q = '\0';
        if (mkdir(tmp, 0755) == -1 && errno != EEXIST) return -1;
        *q = '/';
    }
    if (mkdir(tmp, 0755) == -1 && errno != EEXIST) return -1;
    return 0;
}

/* first existing of: explicit, link, glob hit, fallback */
static char *resolve(const char *explicit_, const char *link,
                     const char *pattern, const char *fallback)
{
    if (explicit_) return strdup(explicit_);
    if (link && access(link, F_OK) == 0) return strdup(link);
    if (pattern) {
        glob_t g;
        if (glob(pattern, 0, NULL, &g) == 0 && g.gl_pathc > 0) {
            char *p = strdup(g.gl_pathv[0]);
            globfree(&g);
            return p;
        }
        globfree(&g);
    }
    if (fallback && access(fallback, F_OK) == 0) return strdup(fallback);
    return NULL;
}

/* -- gimbal telemetry ----------------------------------------------------- */

/* CRC-16/MCRF4XX, init 0xFFFF -- matches the gimbal firmware */
static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

struct angle { double t; float yaw, pitch; };
static struct angle ring[RINGSZ];
static unsigned ring_n = 0;              /* total pushed; index = n % RINGSZ */

static void ring_push(double t, float yaw, float pitch)
{
    struct angle *a = &ring[ring_n % RINGSZ];
    a->t = t; a->yaw = yaw; a->pitch = pitch;
    ring_n++;
}

/* Nearest sample to time t. Walks back from newest; the match is normally
 * within a few entries, so this is far cheaper than it looks. */
static const struct angle *ring_nearest(double t)
{
    if (ring_n == 0) return NULL;
    unsigned have = ring_n < RINGSZ ? ring_n : RINGSZ;
    const struct angle *best = NULL;
    double bestd = 1e18;
    for (unsigned k = 0; k < have; k++) {
        const struct angle *a = &ring[(ring_n - 1 - k) % RINGSZ];
        double d = t - a->t; if (d < 0) d = -d;
        if (d < bestd) { bestd = d; best = a; }
        else if (a->t < t) break;        /* moving away, older only gets worse */
    }
    return best;
}

static int open_serial(const char *dev, speed_t baud)
{
    /* O_RDONLY, not O_RDWR: this program has no business transmitting to the
     * gimbal, and the open mode is the cheapest way to guarantee it cannot. */
    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { close(fd); return -1; }
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* -- video ---------------------------------------------------------------- */

struct bufmap { void *start; size_t len; };
static struct bufmap vbufs[NBUFS];

static int open_video(const char *dev, unsigned w, unsigned h)
{
    int fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) return -1;

    struct v4l2_format f;
    memset(&f, 0, sizeof f);
    f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width = w;
    f.fmt.pix.height = h;
    f.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    f.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &f) < 0) { close(fd); return -1; }
    if (f.fmt.pix.width != w || f.fmt.pix.height != h ||
        f.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) {
        /* Timestamps stay valid under a substituted format, but silently
         * measuring a geometry nobody asked for is how a run gets misfiled. */
        fprintf(stderr, "warning: driver gave %ux%u instead of %ux%u\n",
                f.fmt.pix.width, f.fmt.pix.height, w, h);
    }
    fprintf(stderr, "video       %s  %ux%u GREY, %.2f MB/frame\n",
            dev, f.fmt.pix.width, f.fmt.pix.height, f.fmt.pix.sizeimage / 1e6);

    struct v4l2_requestbuffers rq;
    memset(&rq, 0, sizeof rq);
    rq.count = NBUFS;
    rq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rq.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rq) < 0) { close(fd); return -1; }

    for (unsigned i = 0; i < rq.count && i < NBUFS; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { close(fd); return -1; }
        vbufs[i].len = b.length;
        vbufs[i].start = mmap(NULL, b.length, PROT_READ, MAP_SHARED, fd,
                              b.m.offset);
        if (vbufs[i].start == MAP_FAILED) { close(fd); return -1; }
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { close(fd); return -1; }
    }
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &t) < 0) { close(fd); return -1; }
    return fd;
}

static void usage(const char *me)
{
    printf(
"Pair every camera frame with the gimbal angle true at its capture instant.\n"
"Both stamped on CLOCK_MONOTONIC. Pixels are never read.\n"
"\n"
"usage: %s [-t SECS] [-o CSV] [-v VIDEODEV] [-g TLMDEV] [-B BAUD]\n"
"          [-W W] [-H H] [-D ROOT] [-q] [-h]\n"
"\n"
"  -t SECS    stop after SECS seconds (default: until Ctrl-C)\n"
"  -o CSV     write here ('-' for stdout; default: a timestamped run folder)\n"
"  -v DEV     video device   (default %s, else %s)\n"
"  -g DEV     telemetry port (default %s, else the FTDI by-id path)\n"
"  -B BAUD    telemetry baud, 115200 or 921600 (default 921600)\n"
"  -W, -H     capture size   (default %dx%d)\n"
"  -D ROOT    run-folder root (default %s/)\n"
"  -q         no per-frame CSV rows, final stats only\n"
"\n"
"CSV columns (unchanged from the original sync_log):\n"
"  seq,t_frame,yaw,pitch,t_angle,dt_ms,n_angles\n"
"\n"
"t_frame is the kernel's v4l2 buffer stamp (first USB payload of that frame);\n"
"t_angle is when the matched telemetry frame arrived; dt_ms is t_frame-t_angle.\n",
        me, THERMAL_LINK, VID_BYID_GLOB, TLM_LINK, DEF_W, DEF_H, REC_ROOT);
}

int main(int argc, char **argv)
{
    const char *vdev_opt = NULL, *gdev_opt = NULL, *out = NULL;
    const char *root = REC_ROOT;
    unsigned W = DEF_W, H = DEF_H;
    double secs = 0;
    speed_t baud = B921600;
    long baud_n = 921600;
    int quiet = 0, c;

    while ((c = getopt(argc, argv, "t:o:v:g:B:W:H:D:qh")) != -1) {
        switch (c) {
        case 't': secs = atof(optarg); break;
        case 'o': out = optarg; break;
        case 'v': vdev_opt = optarg; break;
        case 'g': gdev_opt = optarg; break;
        case 'B':
            baud_n = strtol(optarg, NULL, 10);
            if (baud_n == 115200) baud = B115200;
            else if (baud_n == 921600) baud = B921600;
            else die("-B must be 115200 or 921600 (got %s)", optarg);
            break;
        case 'W': W = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'H': H = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'D': root = optarg; break;
        case 'q': quiet = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }

    char *vdev = resolve(vdev_opt, THERMAL_LINK, VID_BYID_GLOB, NULL);
    if (!vdev)
        die("no Sirius capture node found (looked for %s, then %s)\n"
            "  is the camera plugged in?  ls -l /dev/v4l/by-id/",
            THERMAL_LINK, VID_BYID_GLOB);
    char *gdev = resolve(gdev_opt, TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);
    if (!gdev)
        die("no telemetry port found (looked for %s, then %s, then %s)\n"
            "  is the FTDI cable plugged in?  ls -l /dev/serial/by-id/",
            TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);

    /* ---- output path: same run-folder shape as record_raw.c ------------- */
    char outdir[512] = "", csvpath[640];
    int to_stdout = (out && strcmp(out, "-") == 0);
    if (out && !to_stdout) {
        snprintf(csvpath, sizeof csvpath, "%s", out);
        const char *slash = strrchr(csvpath, '/');
        if (slash && slash != csvpath)
            snprintf(outdir, sizeof outdir, "%.*s",
                     (int)(slash - csvpath), csvpath);
    } else if (!out) {
        time_t tt = time(NULL);
        struct tm tm;
        localtime_r(&tt, &tm);
        unsigned dup = 0;
        for (;;) {
            int len;
            if (dup == 0)
                len = snprintf(outdir, sizeof outdir, "%s/%04d-%02d-%02d/%02d%02d%02d",
                               root, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                               tm.tm_hour, tm.tm_min, tm.tm_sec);
            else
                len = snprintf(outdir, sizeof outdir,
                               "%s/%04d-%02d-%02d/%02d%02d%02d-%u",
                               root, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                               tm.tm_hour, tm.tm_min, tm.tm_sec, dup);
            if (len < 0 || (size_t)len >= sizeof outdir) die("-D path too long");
            if (access(outdir, F_OK) != 0) break;
            if (++dup > 999) die("cannot find an unused folder under %s", root);
        }
        snprintf(csvpath, sizeof csvpath, "%s/sync.csv", outdir);
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int gfd = open_serial(gdev, baud);
    if (gfd < 0) die("telemetry %s: %s", gdev, strerror(errno));
    fprintf(stderr, "telemetry   %s @ %ld, receive only\n", gdev, baud_n);

    int vfd = open_video(vdev, W, H);
    if (vfd < 0)
        die("video %s: %s\n  another process may hold the camera"
            " (only one streamer allowed)", vdev, strerror(errno));

    FILE *csv = stdout;
    if (!to_stdout) {
        if (outdir[0] && mkdir_p(outdir) == -1)
            die("cannot create folder %s: %s", outdir, strerror(errno));
        csv = fopen(csvpath, "w");
        if (!csv) die("cannot create %s: %s", csvpath, strerror(errno));
        fprintf(stderr, "folder      %s/\ncsv         %s\n", outdir, csvpath);
    }
    /* A '#' preamble keeps the file self-describing without disturbing the
     * column layout -- pandas/awk skip it, and a stray CSV stays readable. */
    fprintf(csv, "# sync_log: frames paired with gimbal angles\n");
    fprintf(csv, "# video=%s telemetry=%s baud=%ld\n", vdev, gdev, baud_n);
    fprintf(csv, "# clock=CLOCK_MONOTONIC for both t_frame and t_angle\n");
    fprintf(csv, "# t_frame = v4l2_buffer.timestamp (first USB payload of frame)\n");
    if (!quiet)
        fprintf(csv, "seq,t_frame,yaw,pitch,t_angle,dt_ms,n_angles\n");

    uint8_t sbuf[SERBUF];
    size_t slen = 0;
    unsigned long nframe = 0, nangle = 0, ncrc = 0, nomatch = 0;
    unsigned long dropped = 0, seq_anom = 0;
    double worst_dt = 0, sum_dt = 0;
    double t0 = now_mono(), prev_ft = 0;
    double gap_min = 1e9, gap_max = 0;
    uint32_t last_seq = 0;
    int have_seq = 0, clock_checked = 0;

    while (!stop_flag) {
        struct pollfd p[2] = {
            { .fd = gfd, .events = POLLIN, .revents = 0 },
            { .fd = vfd, .events = POLLIN, .revents = 0 },
        };
        int r = poll(p, 2, 500);
        if (r < 0) { if (errno == EINTR) continue; break; }

        /* serial first, so the angle ring is as fresh as possible when a
         * frame is matched in the same wakeup */
        if (p[0].revents & POLLIN) {
            ssize_t n = read(gfd, sbuf + slen, sizeof sbuf - slen);
            double t = now_mono();
            if (n > 0) {
                slen += (size_t)n;
                size_t i = 0;
                while (slen - i >= TLM_LEN) {
                    if (sbuf[i] != 0xA5 || sbuf[i+1] != 0x5A) { i++; continue; }
                    uint16_t want = (uint16_t)(sbuf[i+34] | (sbuf[i+35] << 8));
                    /* Advance by 1, not 2, on a CRC miss: a real frame can
                     * start one byte into a false A5 5A, and skipping both
                     * magic bytes would step over it. */
                    if (crc16(&sbuf[i+2], 32) != want) { ncrc++; i++; continue; }
                    float fl[TLM_FLOATS];
                    memcpy(fl, &sbuf[i+2], sizeof fl);
                    ring_push(t, fl[0], fl[1]);
                    nangle++;
                    i += TLM_LEN;
                }
                memmove(sbuf, sbuf + i, slen - i);
                slen -= i;
                if (slen == sizeof sbuf) slen = 0;   /* pathological: resync */
            }
        }

        if (p[1].revents & POLLIN) {
            struct v4l2_buffer b;
            memset(&b, 0, sizeof b);
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            if (xioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
                /* Assert the clock rather than trusting the docstring: a driver
                 * stamping a different base would silently ruin every pairing. */
                if (!clock_checked) {
                    clock_checked = 1;
                    if ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                        != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                        die("video timestamps are not CLOCK_MONOTONIC "
                            "(flags 0x%08x) -- pairing would be meaningless",
                            b.flags);
                }
                /* kernel's own stamp: first USB payload of this frame */
                double ft = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;
                const struct angle *a = ring_nearest(ft);
                nframe++;
                if (prev_ft > 0) {
                    double g = (ft - prev_ft) * 1000.0;
                    if (g < gap_min) gap_min = g;
                    if (g > gap_max) gap_max = g;
                }
                prev_ft = ft;

                /* Underflow-safe: sequences can repeat when a stream never
                 * really starts, and unsigned subtraction there wraps to ~4e9. */
                if (have_seq) {
                    if (b.sequence > last_seq + 1)
                        dropped += b.sequence - last_seq - 1;
                    else if (b.sequence <= last_seq)
                        seq_anom++;
                }
                last_seq = b.sequence;
                have_seq = 1;

                if (a) {
                    double dt = (ft - a->t) * 1000.0;
                    double ad = dt < 0 ? -dt : dt;
                    if (ad > worst_dt) worst_dt = ad;
                    sum_dt += ad;
                    if (!quiet)
                        fprintf(csv, "%u,%.6f,%.4f,%.4f,%.6f,%.3f,%u\n",
                                b.sequence, ft, a->yaw, a->pitch, a->t, dt,
                                ring_n);
                } else {
                    nomatch++;
                    if (!quiet)
                        fprintf(csv, "%u,%.6f,,,,,0\n", b.sequence, ft);
                }
                /* pixels untouched -- straight back to the driver */
                xioctl(vfd, VIDIOC_QBUF, &b);
            }
        }

        if (secs > 0 && now_mono() - t0 >= secs) break;
    }

    double el = now_mono() - t0;
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(vfd, VIDIOC_STREAMOFF, &t);
    for (int i = 0; i < NBUFS; i++)
        if (vbufs[i].start) munmap(vbufs[i].start, vbufs[i].len);
    close(vfd);
    close(gfd);
    if (csv != stdout) fclose(csv);

    fprintf(stderr, "\n--- %.1f s ---\n", el);
    fprintf(stderr, "frames : %lu  (%.2f fps)  frame gap %.1f..%.1f ms\n",
            nframe, nframe / el, nframe > 1 ? gap_min : 0.0, gap_max);
    fprintf(stderr, "angles : %lu  (%.1f Hz)  crc errors %lu\n",
            nangle, nangle / el, ncrc);
    if (nframe)
        fprintf(stderr, "match  : mean |dt| %.3f ms, worst %.3f ms, "
                        "unmatched %lu\n",
                sum_dt / (double)(nframe - nomatch ? nframe - nomatch : 1),
                worst_dt, nomatch);
    if (dropped)
        fprintf(stderr, "DROPPED: %lu frame(s) lost upstream"
                        " (kernel sequence gaps)\n", dropped);
    if (seq_anom)
        fprintf(stderr, "SEQUENCE: %lu frame(s) did not advance the sequence"
                        " -- stream likely never started\n", seq_anom);
    fprintf(stderr, "both timestamps are CLOCK_MONOTONIC -- directly comparable\n");
    if (!to_stdout && outdir[0])
        fprintf(stderr, "csv    : %s\n", csvpath);

    int bad = (nframe == 0 || nangle == 0);
    if (bad)
        fprintf(stderr, "\nFAILED : captured nothing usable"
                        " (%lu frames, %lu angles)\n", nframe, nangle);
    free(vdev);
    free(gdev);
    return bad ? 1 : 0;
}
