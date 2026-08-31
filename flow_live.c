/* flow_live.c -- live: for every frame, check what arrived, compute the optical
 * flow for that frame in both axes, pair it with the gimbal angle, and log ONLY
 * the angles and the flow. No pixels are ever written to disk.
 *
 * Nothing is recorded, so there is no capture to protect: the whole CPU budget
 * is available for flow. Measured cost of the flow computation on this Jetson is
 * ~3.3 ms/frame at the default span, against a 30-40 ms frame interval, so this
 * runs with ~10x headroom. Frames are still counted and any loss is reported
 * loudly -- see the DROP CHECK note.
 *
 * TWO TIMESTAMPS PER FRAME, which is the point of this program:
 *
 *   t_first_byte  v4l2_buffer.timestamp -- taken by the kernel when the FIRST
 *                 USB payload of that frame lands on the Jetson. This is the
 *                 earliest instant the host knows anything about the frame, and
 *                 it is far steadier than anything userspace can measure.
 *
 *   t_arrival     CLOCK_MONOTONIC read the moment VIDIOC_DQBUF returns the
 *                 buffer to this process -- i.e. when the frame is fully
 *                 transferred, reassembled and handed over.
 *
 *   latency_ms    t_arrival - t_first_byte. That is transfer + reassembly +
 *                 queueing: how long after the first byte the frame became
 *                 usable. It is reported per row and summarised at the end.
 *
 * Both are CLOCK_MONOTONIC, the same clock the telemetry is stamped on, so
 * every column is directly comparable with no conversion. Verified rather than
 * assumed: uvcvideo runs clock=CLOCK_MONOTONIC and each buffer is checked for
 * V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC on arrival.
 *
 * A caveat worth stating plainly: the buffer flag says TSTAMP_SRC_SOE ("start of
 * exposure"), but uvcvideo here runs hwtimestamps=0 and the core's UVC payload
 * headers carry no PTS or SCR, so t_first_byte is really derived from payload
 * arrival on the host -- not from a camera-side clock. It is the first byte
 * hitting the Jetson, which is exactly what it is labelled, but it is NOT the
 * instant the sensor exposed the image; that sits an unknown fixed offset
 * earlier. Everything here is consistent relative to itself.
 *
 * WHICH FRAME THE FLOW BELONGS TO: a displacement only exists BETWEEN two
 * frames. dy_px/dx_px on a row is the motion from the previous frame to THIS
 * one, so it is the flow "of that frame" in the only sense that is defined.
 * t_flow is the midpoint of that interval, because attributing a between-frames
 * measurement to the later frame's timestamp pushes it dt/2 (~15-20 ms) late.
 * Use t_flow for correlation and t_first_byte to identify the frame.
 *
 * FLOW METHOD (from ~/drone_detection/flow_calc.c, same as flow_post.c). Each
 * frame collapses to two 1-D profiles, row means and column means, aligned
 * against the previous frame's by the shift minimising mean absolute difference.
 * Collapsing first turns an O(W*H*shifts) 2-D search into O((W+H)*shifts), which
 * is what makes it affordable live. Profiles are mean-removed so auto-exposure
 * hunting cannot masquerade as motion, and high-passed with a phase-neutral
 * forward+backward running mean, because row means are dominated by lens
 * vignetting and sensor shading fixed to the SENSOR, which do not move when the
 * scene does -- leave that in and the matcher locks onto zero no matter how far
 * the image travelled. The minimum is refined sub-pixel with a parabola.
 * Validated against synthetic known shifts to within 0.06 px on both axes.
 *
 * SIGN: dy_px > 0 = image content moved DOWN the sensor. dx_px > 0 = moved
 * RIGHT. Both axes are measured because the camera's mounting orientation
 * decides which image axis a given rotation appears on.
 *
 * DROP CHECK: b.sequence gaps mean frames were lost upstream, and if the flow
 * computation ever fails to keep up that is exactly how it shows. The final
 * report prints drops, the per-frame compute time, and the frame interval, so
 * the margin is visible rather than assumed. If compute ever exceeds the
 * interval it says so and names the fix (a smaller -s).
 *
 * Build:  gcc -O2 -Wall -Wextra -o flow_live flow_live.c
 * Run:    ./flow_live -t 60           (Ctrl-C to stop)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
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

#define NBUFS      8
#define TLM_LEN    36
#define TLM_FLOATS 8
#define RINGSZ     8192
#define SERBUF     8192
#define DEF_W      1280
#define DEF_H      1024
#define DEF_SPAN   300
#define MAXPROF    8192
#define MAXSPAN    1000
#define HP_WIN     101
#define LATN       200000        /* latency samples kept for percentiles */

static const char *THERMAL_LINK = "/dev/thermal0";
static const char *VID_BYID_GLOB =
    "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";
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

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
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

static char *resolve(const char *ex, const char *link, const char *pat,
                     const char *fb)
{
    if (ex) return strdup(ex);
    if (link && access(link, F_OK) == 0) return strdup(link);
    if (pat) {
        glob_t g;
        if (glob(pat, 0, NULL, &g) == 0 && g.gl_pathc > 0) {
            char *p = strdup(g.gl_pathv[0]); globfree(&g); return p;
        }
        globfree(&g);
    }
    if (fb && access(fb, F_OK) == 0) return strdup(fb);
    return NULL;
}

static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y ? 1 : 0;
}

/* -- telemetry ------------------------------------------------------------ */

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

struct angle { double t; float yaw, pitch; };
static struct angle ring[RINGSZ];
static unsigned ring_n = 0;
/* The ring is written by the telemetry thread and read by the video loop, so
 * every access is guarded. Contention is negligible: ~1010 short writes/s
 * against ~25 reads/s. */
static pthread_mutex_t ring_mtx = PTHREAD_MUTEX_INITIALIZER;

static void ring_push(double t, float yaw, float pitch)
{
    pthread_mutex_lock(&ring_mtx);
    struct angle *a = &ring[ring_n % RINGSZ];
    a->t = t; a->yaw = yaw; a->pitch = pitch;
    ring_n++;
    pthread_mutex_unlock(&ring_mtx);
}

/* Copies the nearest sample into *out. Returning a pointer would hand back
 * memory the reader thread may overwrite a millisecond later. */
static int ring_nearest(double t, struct angle *out, unsigned *total)
{
    int found = 0;
    pthread_mutex_lock(&ring_mtx);
    if (total) *total = ring_n;
    if (ring_n) {
        unsigned have = ring_n < RINGSZ ? ring_n : RINGSZ;
        double bestd = 1e18;
        for (unsigned k = 0; k < have; k++) {
            const struct angle *a = &ring[(ring_n - 1 - k) % RINGSZ];
            double d = t - a->t; if (d < 0) d = -d;
            if (d < bestd) { bestd = d; *out = *a; found = 1; }
            else if (a->t < t) break;
        }
    }
    pthread_mutex_unlock(&ring_mtx);
    return found;
}

/* Telemetry lives on its own thread, and that is not gratuitous. Arrival stamps
 * are only as good as how often the port is read: samples land ~1 ms apart, so a
 * read() returning k of them stamps all k identically. Draining from the video
 * loop means the flow computation sets the cadence -- measured at a 2.94 ms mean
 * match error with one drain per iteration, and still 2.13 ms with drains
 * sprinkled through the pixel pass, because the ~2 ms alignment search has no
 * drain point inside it. Chasing that with more call sites is whack-a-mole; a
 * thread that does nothing but poll the port keeps stamps at the ~1 ms the link
 * actually delivers, whatever the video path is doing. */
static uint8_t g_sbuf[SERBUF];
static size_t g_slen = 0;
static unsigned long g_nangle = 0, g_ncrc = 0;

static void drain_tlm(int gfd)
{
    for (;;) {
        if (g_slen >= sizeof g_sbuf) g_slen = 0;    /* pathological: resync */
        ssize_t n = read(gfd, g_sbuf + g_slen, sizeof g_sbuf - g_slen);
        if (n <= 0) return;                          /* EAGAIN: nothing waiting */
        double t = now_mono();
        g_slen += (size_t)n;
        size_t i = 0;
        while (g_slen - i >= TLM_LEN) {
            if (g_sbuf[i] != 0xA5 || g_sbuf[i+1] != 0x5A) { i++; continue; }
            uint16_t want = (uint16_t)(g_sbuf[i+34] | (g_sbuf[i+35] << 8));
            /* advance 1 on a CRC miss: a real frame can start one byte into a
             * false A5 5A */
            if (crc16(&g_sbuf[i+2], 32) != want) { g_ncrc++; i++; continue; }
            float fl[TLM_FLOATS];
            memcpy(fl, &g_sbuf[i+2], sizeof fl);
            ring_push(t, fl[0], fl[1]);
            g_nangle++;
            i += TLM_LEN;
        }
        memmove(g_sbuf, g_sbuf + i, g_slen - i);
        g_slen -= i;
        if ((size_t)n < sizeof g_sbuf - (g_slen + i)) return;  /* drained */
    }
}

/* Reader thread: poll the port and nothing else. */
static int g_tlm_fd = -1;
static void *tlm_thread(void *arg)
{
    (void)arg;
    while (!stop_flag) {
        struct pollfd p = { .fd = g_tlm_fd, .events = POLLIN, .revents = 0 };
        int r = poll(&p, 1, 100);
        if (r < 0) { if (errno == EINTR) continue; break; }
        if (r > 0 && (p.revents & POLLIN)) drain_tlm(g_tlm_fd);
    }
    return NULL;
}

static int open_serial(const char *dev, speed_t baud)
{
    /* O_RDONLY: never transmit to the gimbal. */
    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { close(fd); return -1; }
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud); cfsetospeed(&tio, baud);
    tio.c_cflag |= CLOCAL | CREAD; tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* -- flow ----------------------------------------------------------------- */

static double align_prof(const double *a, const double *b, int n, int span,
                         double *quality)
{
    static double sm[MAXPROF], ha[MAXPROF], hb[MAXPROF];
    static double cost[2 * MAXSPAN + 1];
    if (n > MAXPROF) n = MAXPROF;
    if (span > MAXSPAN) span = MAXSPAN;
    if (span > n / 2) span = n / 2;

    const double *src[2] = { a, b };
    double *dst[2] = { ha, hb };
    for (int p = 0; p < 2; p++) {
        double run = 0; int cnt = 0;
        for (int i = 0; i < n; i++) {
            run += src[p][i]; cnt++;
            if (cnt > HP_WIN) { run -= src[p][i - HP_WIN]; cnt--; }
            sm[i] = run / cnt;
        }
        run = 0; cnt = 0;
        for (int i = n - 1; i >= 0; i--) {
            run += sm[i]; cnt++;
            if (cnt > HP_WIN) { run -= sm[i + HP_WIN]; cnt--; }
            dst[p][i] = src[p][i] - run / cnt;
        }
    }
    double best = 1e300, sum = 0; int bestd = 0, nd = 0;
    for (int d = -span; d <= span; d++) {
        int i0 = d < 0 ? -d : 0, i1 = d < 0 ? n : n - d;
        double acc = 0; int m = 0;
        for (int i = i0; i < i1; i++) {
            double diff = ha[i] - hb[i + d];
            acc += diff < 0 ? -diff : diff;
            m++;
        }
        double cv = m ? acc / m : 1e300;
        cost[d + span] = cv; sum += cv; nd++;
        if (cv < best) { best = cv; bestd = d; }
    }
    double mean = nd ? sum / nd : 0;
    *quality = mean > 0 ? (mean - best) / mean : 0;
    double dy = bestd; int bi = bestd + span;
    if (bi > 0 && bi < 2 * span) {
        double lo = cost[bi - 1], hi = cost[bi + 1];
        double den = lo - 2 * best + hi;
        if (den != 0) {
            double adj = 0.5 * (lo - hi) / den;
            if (adj > -1 && adj < 1) dy = bestd + adj;
        }
    }
    return dy;
}

/* -- video ---------------------------------------------------------------- */

struct bufmap { void *start; size_t len; };
static struct bufmap vbufs[NBUFS];

static int open_video(const char *dev, unsigned w, unsigned h, unsigned *fsz)
{
    int fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) return -1;
    struct v4l2_format f;
    memset(&f, 0, sizeof f);
    f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width = w; f.fmt.pix.height = h;
    f.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    f.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &f) < 0) { close(fd); return -1; }
    /* Flow indexes the buffer as W*H bytes of GREY; a substituted format would
     * be measured wrong, so refuse rather than produce nonsense. */
    if (f.fmt.pix.width != w || f.fmt.pix.height != h ||
        f.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) { close(fd); return -2; }
    *fsz = f.fmt.pix.sizeimage;

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
        b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { close(fd); return -1; }
        vbufs[i].len = b.length;
        vbufs[i].start = mmap(NULL, b.length, PROT_READ, MAP_SHARED, fd, b.m.offset);
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
"Live: per frame, both arrival timestamps + optical flow (both axes) + gimbal\n"
"angle, logged to CSV. Pixels are never saved.\n"
"\n"
"usage: %s [-t SECS] [-o CSV] [-s SPAN] [-v VIDEODEV] [-g TLMDEV]\n"
"          [-B BAUD] [-W W] [-H H] [-D ROOT] [-q] [-h]\n"
"\n"
"  -t SECS    stop after SECS seconds (default: until Ctrl-C)\n"
"  -o CSV     write here ('-' for stdout; default: a timestamped run folder)\n"
"  -s SPAN    max shift searched, px (default %d). Cost is linear in SPAN:\n"
"             300 ~3.3 ms/frame, 150 ~2.5, 50 ~1.9. Must exceed the largest\n"
"             real between-frame shift or it cannot be found at all.\n"
"  -v DEV     video device   (default %s, else the by-id path)\n"
"  -g DEV     telemetry port (default %s, else the FTDI by-id path)\n"
"  -B BAUD    telemetry baud, 115200 or 921600 (default 921600)\n"
"  -W, -H     capture size   (default %dx%d)\n"
"  -D ROOT    run-folder root (default %s/)\n"
"  -q         no live status line\n"
"\n"
"CSV columns:\n"
"  seq,t_first_byte,t_arrival,latency_ms,\n"
"  yaw,pitch,t_angle,angle_dt_ms,n_ang_interval,\n"
"  t_flow,interval_ms,dy_px,dy_rate_px_s,qual_y,dx_px,dx_rate_px_s,qual_x,\n"
"  d_yaw_deg,d_pitch_deg,yaw_rate_dps,pitch_rate_dps\n"
"\n"
"t_first_byte = kernel stamp, first USB payload of the frame on the Jetson.\n"
"t_arrival    = when DQBUF handed the finished frame to userspace.\n"
"latency_ms   = the gap between those two: transfer + reassembly + queueing.\n"
"dy_px>0 = content moved DOWN; dx_px>0 = moved RIGHT. Low qual_* = untrusted.\n",
        me, DEF_SPAN, THERMAL_LINK, TLM_LINK, DEF_W, DEF_H, REC_ROOT);
}

int main(int argc, char **argv)
{
    const char *vdev_opt = NULL, *gdev_opt = NULL, *out = NULL, *root = REC_ROOT;
    unsigned W = DEF_W, H = DEF_H;
    int span = DEF_SPAN, quiet = 0, c;
    double secs = 0;
    speed_t baud = B921600; long baud_n = 921600;

    while ((c = getopt(argc, argv, "t:o:s:v:g:B:W:H:D:qh")) != -1) {
        switch (c) {
        case 't': secs = atof(optarg); break;
        case 'o': out = optarg; break;
        case 's': span = atoi(optarg); break;
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
    if (span < 1) die("-s must be >= 1");
    if (W < 2 || H < 2 || W > MAXPROF || H > MAXPROF)
        die("geometry %ux%u out of range", W, H);

    char *vdev = resolve(vdev_opt, THERMAL_LINK, VID_BYID_GLOB, NULL);
    if (!vdev) die("no Sirius capture node found (looked for %s, then %s)",
                   THERMAL_LINK, VID_BYID_GLOB);
    char *gdev = resolve(gdev_opt, TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);
    if (!gdev) die("no telemetry port found (looked for %s, then %s, then %s)",
                   TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);

    char outdir[512] = "", csvpath[640];
    int to_stdout = (out && strcmp(out, "-") == 0);
    if (out && !to_stdout) {
        snprintf(csvpath, sizeof csvpath, "%s", out);
        const char *sl = strrchr(csvpath, '/');
        if (sl && sl != csvpath)
            snprintf(outdir, sizeof outdir, "%.*s", (int)(sl - csvpath), csvpath);
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
        snprintf(csvpath, sizeof csvpath, "%s/flow_live.csv", outdir);
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int gfd = open_serial(gdev, baud);
    if (gfd < 0) die("telemetry %s: %s", gdev, strerror(errno));

    unsigned frame_sz = 0;
    int vfd = open_video(vdev, W, H, &frame_sz);
    if (vfd == -2)
        die("driver refused %ux%u GREY -- flow indexes the buffer as W*H bytes.\n"
            "  Run './record_raw -L' to see what this core offers.", W, H);
    if (vfd < 0)
        die("video %s: %s\n  another process may hold the camera"
            " (the core allows only one streamer)", vdev, strerror(errno));
    if (frame_sz < (unsigned)W * H)
        die("driver reports %u B/frame, less than %ux%u -- refusing to read"
            " past the buffer", frame_sz, W, H);

    FILE *csv = stdout;
    if (!to_stdout) {
        if (outdir[0] && mkdir_p(outdir) == -1)
            die("cannot create folder %s: %s", outdir, strerror(errno));
        csv = fopen(csvpath, "w");
        if (!csv) die("cannot create %s: %s", csvpath, strerror(errno));
    }

    fprintf(csv, "# flow_live: arrival timing + optical flow + gimbal angle\n");
    fprintf(csv, "# video=%s telemetry=%s baud=%ld\n", vdev, gdev, baud_n);
    fprintf(csv, "# geometry=%ux%u GREY  flow_span=%d px  (no pixels saved)\n",
            W, H, span);
    fprintf(csv, "# clock=CLOCK_MONOTONIC for every timestamp column\n");
    fprintf(csv, "# t_first_byte = kernel stamp, first USB payload on the host\n");
    fprintf(csv, "# t_arrival    = DQBUF handed the finished frame to userspace\n");
    fprintf(csv, "# latency_ms   = t_arrival - t_first_byte\n");
    fprintf(csv, "# dy_px>0 = content moved DOWN; dx_px>0 = moved RIGHT\n");
    fprintf(csv, "# t_flow = interval midpoint; correlate against t_flow\n");
    fprintf(csv, "seq,t_first_byte,t_arrival,latency_ms,"
                 "yaw,pitch,t_angle,angle_dt_ms,n_ang_interval,"
                 "t_flow,interval_ms,dy_px,dy_rate_px_s,qual_y,"
                 "dx_px,dx_rate_px_s,qual_x,"
                 "d_yaw_deg,d_pitch_deg,yaw_rate_dps,pitch_rate_dps\n");

    double *rprof = malloc(sizeof(double) * H), *rprev = malloc(sizeof(double) * H);
    double *cprof = malloc(sizeof(double) * W), *cprev = malloc(sizeof(double) * W);
    unsigned long *rsum = malloc(sizeof(unsigned long) * H);
    unsigned long *csum = malloc(sizeof(unsigned long) * W);
    double *lat = malloc(sizeof(double) * LATN);
    if (!rprof || !rprev || !cprof || !cprev || !rsum || !csum || !lat)
        die("out of memory");

    fprintf(stderr, "video       %s  %ux%u GREY\n", vdev, W, H);
    fprintf(stderr, "telemetry   %s @ %ld, receive only\n", gdev, baud_n);
    fprintf(stderr, "flow        span %d px, both axes\n", span);
    fprintf(stderr, "saving      angles + flow only -- no pixels\n");
    if (!to_stdout) fprintf(stderr, "csv         %s\n", csvpath);
    fprintf(stderr, "running     Ctrl-C to stop\n");

    g_tlm_fd = gfd;
    pthread_t tlm_tid;
    if (pthread_create(&tlm_tid, NULL, tlm_thread, NULL) != 0)
        die("cannot start telemetry thread: %s", strerror(errno));

    unsigned long nframe = 0, nomatch = 0;
#define nangle g_nangle
#define ncrc   g_ncrc
    unsigned long dropped = 0, seq_anom = 0, nflow = 0, lowq = 0, nbad = 0;
    unsigned nlat = 0;
    double sum_ang_dt = 0, worst_ang_dt = 0;
    double sum_cost = 0, max_cost = 0;
    double sum_absdy = 0, sum_absdx = 0, sum_qy = 0, sum_qx = 0;
    double sum_lat = 0, max_lat = 0, min_lat = 1e9;
    double t0 = now_mono(), prev_fb = 0, t_status = 0;
    double gap_min = 1e9, gap_max = 0;
    double prev_yaw = 0, prev_pitch = 0;
    double last_dy = 0, last_dx = 0;
    unsigned prev_ring = 0;
    uint32_t last_seq = 0;
    int have_seq = 0, have_prev = 0, have_prev_ang = 0, clock_checked = 0;
    int isatty_err = isatty(STDERR_FILENO);

    while (!stop_flag) {
        /* Video only: telemetry is the reader thread's job now. */
        struct pollfd p[1] = { { .fd = vfd, .events = POLLIN, .revents = 0 } };
        int r = poll(p, 1, 500);
        if (r < 0) { if (errno == EINTR) continue; break; }

        if (p[0].revents & POLLIN) {
            struct v4l2_buffer b;
            memset(&b, 0, sizeof b);
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            if (xioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
                /* FIRST thing after DQBUF returns, before any work: this is the
                 * arrival instant, and anything done before reading it would be
                 * charged to the frame's latency. */
                double t_arr = now_mono();
                double t_fb = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;
                /* VIDIOC_QBUF overwrites this struct, and the buffer is requeued
                 * early on purpose (before the alignment search) to give it back
                 * to the driver as soon as possible. So snapshot every field
                 * needed later NOW -- reading b.sequence after the QBUF below
                 * yields 0, which silently zeroed the CSV's seq column. */
                uint32_t fseq = b.sequence;
                uint32_t fflags = b.flags;
                uint32_t fbytes = b.bytesused;
                unsigned fidx = b.index;

                if (!clock_checked) {
                    clock_checked = 1;
                    if ((fflags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                        != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                        die("video timestamps are not CLOCK_MONOTONIC "
                            "(flags 0x%08x) -- every column would be unusable",
                            fflags);
                }

                nframe++;
                if (have_seq) {
                    if (fseq > last_seq + 1)
                        dropped += fseq - last_seq - 1;
                    else if (fseq <= last_seq)
                        seq_anom++;   /* repeats/rewinds: stream never started */
                }
                last_seq = fseq; have_seq = 1;

                /* Check what actually arrived before trusting the pixels. */
                int usable = !(fflags & V4L2_BUF_FLAG_ERROR)
                             && fbytes >= (unsigned)W * H;
                if (!usable) nbad++;

                double lat_ms = (t_arr - t_fb) * 1000.0;
                if (usable) {
                    sum_lat += lat_ms;
                    if (lat_ms > max_lat) max_lat = lat_ms;
                    if (lat_ms < min_lat) min_lat = lat_ms;
                    if (nlat < LATN) lat[nlat++] = lat_ms;
                }

                double tc0 = now_mono();
                if (usable) {
                    /* One sequential pass accumulating BOTH row and column sums.
                     * Columns in a separate y-inner loop would stride the cache
                     * and cost several times more. */
                    const uint8_t *img = (const uint8_t *)vbufs[fidx].start;
                    memset(csum, 0, sizeof(unsigned long) * W);
                    for (unsigned y = 0; y < H; y++) {
                        const uint8_t *row = img + (size_t)y * W;
                        unsigned long rs = 0;
                        for (unsigned x = 0; x < W; x++) {
                            unsigned v = row[x];
                            rs += v; csum[x] += v;
                        }
                        rsum[y] = rs;
                    }
                }
                /* Buffer back to the driver before the alignment search, so it is
                 * out of our hands for as little time as possible. */
                xioctl(vfd, VIDIOC_QBUF, &b);

                struct angle acopy;
                unsigned ring_total = 0;
                const struct angle *a = ring_nearest(t_fb, &acopy, &ring_total)
                                        ? &acopy : NULL;
                double ang_dt = 0;
                if (a) {
                    ang_dt = (t_fb - a->t) * 1000.0;
                    double ad = ang_dt < 0 ? -ang_dt : ang_dt;
                    if (ad > worst_ang_dt) worst_ang_dt = ad;
                    sum_ang_dt += ad;
                } else {
                    nomatch++;
                }
                unsigned n_ang_int = ring_total - prev_ring;
                prev_ring = ring_total;

                int flow_ok = 0;
                double dy = 0, dx = 0, qy = 0, qx = 0, tf = 0, dtf = 0;
                if (usable) {
                    double gs = 0;
                    for (unsigned y = 0; y < H; y++) {
                        rprof[y] = (double)rsum[y] / W; gs += rprof[y];
                    }
                    double gm = gs / H;
                    for (unsigned y = 0; y < H; y++) rprof[y] -= gm;
                    gs = 0;
                    for (unsigned x = 0; x < W; x++) {
                        cprof[x] = (double)csum[x] / H; gs += cprof[x];
                    }
                    gm = gs / W;
                    for (unsigned x = 0; x < W; x++) cprof[x] -= gm;

                    if (have_prev && prev_fb > 0) {
                        dy = align_prof(rprev, rprof, (int)H, span, &qy);
                        dx = align_prof(cprev, cprof, (int)W, span, &qx);
                        dtf = t_fb - prev_fb;
                        tf = prev_fb + dtf / 2.0;    /* midpoint, not this frame */
                        flow_ok = 1; nflow++;
                        sum_absdy += dy < 0 ? -dy : dy;
                        sum_absdx += dx < 0 ? -dx : dx;
                        sum_qy += qy; sum_qx += qx;
                        if (qy < 0.05 || qx < 0.05) lowq++;
                        last_dy = dy; last_dx = dx;
                        double g = dtf * 1000.0;
                        if (g < gap_min) gap_min = g;
                        if (g > gap_max) gap_max = g;
                    }
                    memcpy(rprev, rprof, sizeof(double) * H);
                    memcpy(cprev, cprof, sizeof(double) * W);
                    have_prev = 1;
                } else {
                    /* Garbage pixels would inject a bogus displacement, so the
                     * chain is broken rather than paired across the bad frame. */
                    have_prev = 0;
                }
                double cost = (now_mono() - tc0) * 1000.0;
                sum_cost += cost;
                if (cost > max_cost) max_cost = cost;

                fprintf(csv, "%u,%.6f,%.6f,%.3f", fseq, t_fb, t_arr, lat_ms);
                if (a)
                    fprintf(csv, ",%.4f,%.4f,%.6f,%.3f,%u",
                            a->yaw, a->pitch, a->t, ang_dt, n_ang_int);
                else
                    fprintf(csv, ",,,,,%u", n_ang_int);
                if (flow_ok)
                    fprintf(csv, ",%.6f,%.3f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f",
                            tf, dtf * 1000.0, dy, dtf > 0 ? dy / dtf : 0.0, qy,
                            dx, dtf > 0 ? dx / dtf : 0.0, qx);
                else
                    fprintf(csv, ",,,,,,,,");
                if (flow_ok && a && have_prev_ang) {
                    double dyaw = wrap180(a->yaw - prev_yaw);
                    double dpit = wrap180(a->pitch - prev_pitch);
                    fprintf(csv, ",%.4f,%.4f,%.3f,%.3f", dyaw, dpit,
                            dtf > 0 ? dyaw / dtf : 0.0, dtf > 0 ? dpit / dtf : 0.0);
                } else {
                    fprintf(csv, ",,,,");
                }
                fputc('\n', csv);

                if (a) { prev_yaw = a->yaw; prev_pitch = a->pitch; have_prev_ang = 1; }
                prev_fb = t_fb;
            }
        }

        double now = now_mono();
        if (!quiet && now - t_status >= 0.2) {
            t_status = now;
            double el = now - t0;
            fprintf(stderr,
                "\r  %6.1fs %5lu fr %5.1ffps | ang %6.1fHz | lat %5.1fms | "
                "dy %+7.2f dx %+7.2f | drop %lu%s",
                el, nframe, el > 0 ? nframe / el : 0.0,
                el > 0 ? nangle / el : 0.0,
                nlat ? sum_lat / nlat : 0.0, last_dy, last_dx, dropped,
                isatty_err ? "   " : "\n");
            fflush(stderr);
        }
        if (secs > 0 && now - t0 >= secs) break;
    }

    double el = now_mono() - t0;
    stop_flag = 1;                    /* the reader thread watches this */
    pthread_join(tlm_tid, NULL);
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(vfd, VIDIOC_STREAMOFF, &t);
    for (int i = 0; i < NBUFS; i++)
        if (vbufs[i].start) munmap(vbufs[i].start, vbufs[i].len);
    close(vfd); close(gfd);
    if (csv != stdout) fclose(csv);

    if (!quiet) fputc('\n', stderr);
    double interval_ms = nframe > 1 ? el / nframe * 1000.0 : 0;

    fprintf(stderr, "\n--- %.1f s ---\n", el);
    fprintf(stderr, "frames  : %lu  (%.2f fps)  interval %.1f..%.1f ms\n",
            nframe, nframe / el, nflow ? gap_min : 0.0, gap_max);
    fprintf(stderr, "angles  : %lu  (%.1f Hz)  crc errors %lu\n",
            nangle, nangle / el, ncrc);
    if (nlat) {
        qsort(lat, nlat, sizeof(double), cmp_d);
        fprintf(stderr, "latency : first-byte -> usable  mean %.2f ms  "
                        "min %.2f  p50 %.2f  p95 %.2f  max %.2f\n",
                sum_lat / nlat, min_lat, lat[nlat/2],
                lat[(int)(nlat * 0.95)], max_lat);
    }
    if (nframe - nbad)
        fprintf(stderr, "angle fit: mean |dt| %.3f ms, worst %.3f ms, unmatched %lu\n",
                sum_ang_dt / (double)(nframe - nomatch ? nframe - nomatch : 1),
                worst_ang_dt, nomatch);
    if (nflow)
        fprintf(stderr, "flow    : %lu values, mean |dy| %.3f px, |dx| %.3f px, "
                        "quality y=%.3f x=%.3f, low-quality %lu\n",
                nflow, sum_absdy / nflow, sum_absdx / nflow,
                sum_qy / nflow, sum_qx / nflow, lowq);
    else
        fprintf(stderr, "flow    : none computed\n");
    if (nframe)
        fprintf(stderr, "compute : mean %.2f ms/frame, max %.2f ms"
                        "  (interval %.1f ms => %.0f%% of budget)\n",
                sum_cost / nframe, max_cost, interval_ms,
                interval_ms > 0 ? 100.0 * (sum_cost / nframe) / interval_ms : 0.0);
    if (nbad)
        fprintf(stderr, "BAD     : %lu frame(s) flagged error or short -- flow not"
                        " paired across them\n", nbad);
    if (dropped) {
        fprintf(stderr, "DROPPED : %lu frame(s) lost upstream (sequence gaps)"
                        " -- %.1f%% of %lu\n",
                dropped, 100.0 * dropped / (double)(nframe + dropped),
                nframe + dropped);
        if (max_cost > interval_ms)
            fprintf(stderr, "          max compute %.2f ms exceeded the %.1f ms"
                            " interval -- lower -s\n", max_cost, interval_ms);
        else
            fprintf(stderr, "          compute stayed inside the interval, so this"
                            " was the link/driver, not this program\n");
    } else {
        fprintf(stderr, "dropped : 0 -- kept up with the camera\n");
    }
    if (seq_anom)
        fprintf(stderr, "SEQUENCE: %lu frame(s) did not advance the sequence"
                        " -- stream start transient\n", seq_anom);
    fprintf(stderr, "all timestamps are CLOCK_MONOTONIC -- directly comparable\n");
    if (!to_stdout) fprintf(stderr, "csv     : %s\n", csvpath);

    int bad = (nframe == 0 || nangle == 0 || nflow == 0);
    if (bad)
        fprintf(stderr, "\nFAILED  : nothing usable (%lu frames, %lu angles,"
                        " %lu flow)\n", nframe, nangle, nflow);
    free(rprof); free(rprev); free(cprof); free(cprev);
    free(rsum); free(csum); free(lat);
    free(vdev); free(gdev);
    return bad ? 1 : 0;
}
