/* record_raw.c -- raw V4L2 capture for the Artosyn "Sirius" thermal core.
 *
 * Writes exactly the bytes the kernel hands back, frame after frame, to one flat
 * file. No compression, no conversion, no container, no padding.
 *
 * Deliberately NOT linked against libv4l2. libv4l2's open()/ioctl() wrappers
 * transparently emulate formats and convert pixels, so a request for GREY can be
 * satisfied by decoding something else -- the bytes you get are then libv4l2's,
 * not the sensor's. Going straight at the kernel with ioctl() on videodev2.h is
 * the only way to be sure the file holds what the core actually sent.
 *
 * Only ioctls, mmap, poll and write are used:
 *   VIDIOC_QUERYCAP  -> confirm capture + streaming
 *   VIDIOC_S_FMT     -> 1280x1024 GREY, then verify the driver did not adjust it
 *   VIDIOC_REQBUFS   -> N mmap buffers
 *   VIDIOC_QUERYBUF / mmap
 *   VIDIOC_QBUF / VIDIOC_STREAMON / poll / VIDIOC_DQBUF -> write() -> VIDIOC_QBUF
 *   VIDIOC_STREAMOFF
 *
 * Build:  gcc -O2 -Wall -Wextra -o record_raw record_raw.c
 * Run:    ./record_raw                        (Ctrl-C to stop)
 *
 * Each run writes into its own date/time folder, e.g.
 *     recordings/2026-08-27/153812/raw-1280x1024-GREY.gray  (+ .meta, + .idx)
 *
 * The core delivers ~32.8 fps in mode 50 and 25.04 in mode 25; 1280x1024 GREY at
 * a true 50 fps is 524 Mbps, over the 480 Mbps line rate of the high-speed link
 * it enumerates on, so USB cannot carry 50. See set_fps_jetson.py.
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
#include <sys/statvfs.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define DEF_W      1280
#define DEF_H      1024
#define DEF_FOURCC V4L2_PIX_FMT_GREY
#define DEF_NBUF   8              /* ~10 MB of ring at 1.25 MB/frame */
#define MAX_NBUF   64
#define STATUS_HZ  10             /* status-line repaint rate */
#define INST_WIN   1.0            /* seconds in the instantaneous-fps window */
#define FLUSH_MB   64             /* push this much to disk, then drop it from cache */
#define KEEP_MB    16             /* ...but keep this much of the tail cached */

/* Pin the node by USB serial, never by videoN -- the numbers drift across
 * re-enumeration. -video-index0 is the capture node; index1 is metadata-only and
 * enumerates no capture formats. /dev/thermal0 is the RPi's custom udev rule. */
static const char *THERMAL_LINK = "/dev/thermal0";
static const char *BYID_GLOB =
    "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";

/* Each run gets its own folder under here, grouped by day:
 *     recordings/2026-08-27/153812/raw-1280x1024-GREY.gray + .meta + .idx
 * One folder per run keeps a capture with its own sidecars -- a .gray separated
 * from its .meta is unplayable, since raw video carries no geometry. Grouping by
 * day keeps the listing usable after a few hundred runs. Local time, not UTC. */
static const char *REC_ROOT = "recordings";

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

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

/* ioctls are interruptible; a signal must not look like a device failure. */
static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static double now_mono(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static void fourcc_str(uint32_t f, char out[5])
{
    out[0] = f & 0xff; out[1] = (f >> 8) & 0xff;
    out[2] = (f >> 16) & 0xff; out[3] = (f >> 24) & 0xff; out[4] = 0;
    for (int i = 0; i < 4; i++)
        if (out[i] < 32 || out[i] > 126) out[i] = '?';
}

/* The playback hint is only useful if it names the format actually recorded --
 * a hardcoded "gray" is wrong the moment -f is used. */
static const char *ff_pix_fmt(uint32_t f)
{
    switch (f) {
    case V4L2_PIX_FMT_GREY: return "gray";
    case V4L2_PIX_FMT_NV12: return "nv12";
    case V4L2_PIX_FMT_YUV420: return "yuv420p";
    case V4L2_PIX_FMT_YUYV: return "yuyv422";
    default: return NULL;
    }
}

static uint32_t parse_fourcc(const char *s)
{
    char b[4] = {' ', ' ', ' ', ' '};
    size_t n = strlen(s);
    if (n == 0 || n > 4) die("fourcc must be 1-4 characters: %s", s);
    memcpy(b, s, n);
    return v4l2_fourcc(b[0], b[1], b[2], b[3]);
}

/* write() is free to return short. Anything less than the whole frame on disk
 * silently shifts every following frame, so loop until it is all out. */
static int write_all(int fd, const void *buf, size_t n)
{
    const char *p = buf;
    while (n) {
        ssize_t w = write(fd, p, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    return 0;
}

static char *resolve_device(void)
{
    if (access(THERMAL_LINK, F_OK) == 0)
        return strdup(THERMAL_LINK);
    glob_t g;
    if (glob(BYID_GLOB, 0, NULL, &g) == 0 && g.gl_pathc > 0) {
        char *p = strdup(g.gl_pathv[0]);
        if (g.gl_pathc > 1)
            fprintf(stderr, "note: %zu cores present, using %s (pick with -d)\n",
                    g.gl_pathc, p);
        globfree(&g);
        return p;
    }
    globfree(&g);
    die("no Sirius capture node found (looked for %s, then %s)\n"
        "  is the camera plugged in?  ls -l /dev/v4l/by-id/", THERMAL_LINK, BYID_GLOB);
    return NULL;
}

static void list_formats(const char *dev)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0) die("cannot open %s: %s", dev, strerror(errno));
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof cap);
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
        printf("device   %s\ndriver   %s\ncard     %s\nbus      %s\n",
               dev, cap.driver, cap.card, cap.bus_info);
    for (uint32_t i = 0;; i++) {
        struct v4l2_fmtdesc fd_ = {0};
        fd_.index = i;
        fd_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_ENUM_FMT, &fd_) == -1) break;
        char fc[5];
        fourcc_str(fd_.pixelformat, fc);
        printf("format   [%u] %s  %s\n", i, fc, fd_.description);
        for (uint32_t j = 0;; j++) {
            struct v4l2_frmsizeenum fs = {0};
            fs.index = j;
            fs.pixel_format = fd_.pixelformat;
            if (xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == -1) break;
            if (fs.type != V4L2_FRMSIZE_TYPE_DISCRETE) continue;
            printf("           %ux%u\n", fs.discrete.width, fs.discrete.height);
        }
    }
    close(fd);
}

/* mkdir -p. Every component may already exist; only a real failure matters. */
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

static void usage(const char *me)
{
    printf(
"Raw V4L2 recorder for the Sirius thermal core. Uncompressed, unconverted.\n"
"\n"
"usage: %s [-o FILE] [-D ROOT] [-d DEV] [-n FRAMES] [-t SECONDS] [-b BUFS]\n"
"          [-W W] [-H H] [-f FOURCC] [-q] [-I] [-L] [-h]\n"
"\n"
"  -o FILE    write to exactly this path instead of a timestamped folder\n"
"             ('-' streams raw frames to stdout, no folder, no sidecars)\n"
"  -D ROOT    recordings root (default %s/)\n"
"  -d DEV     capture node (default: %s, else %s)\n"
"  -n FRAMES  stop after FRAMES frames (default: until Ctrl-C)\n"
"  -t SECONDS stop after SECONDS seconds (default: until Ctrl-C)\n"
"  -b BUFS    mmap buffers, 2..%d (default %d). More absorbs longer write stalls.\n"
"  -W, -H     frame geometry (default %dx%d, the only size the core offers)\n"
"  -f FOURCC  pixel format (default GREY; core also has NV12, YU12, YUYV)\n"
"  -q         no status line\n"
"  -I         no .idx sidecar\n"
"  -L         list the core's formats and exit\n"
"\n"
"Each run creates its own folder, named for the date and time it started:\n"
"\n"
"  %s/YYYY-MM-DD/HHMMSS/raw-<W>x<H>-<FOURCC>.gray   raw frames, back to back\n"
"  %s/YYYY-MM-DD/HHMMSS/raw-<W>x<H>-<FOURCC>.gray.meta   geometry, so it plays\n"
"  %s/YYYY-MM-DD/HHMMSS/raw-<W>x<H>-<FOURCC>.gray.idx    per-frame seq/timestamp\n"
"\n"
"The .meta is what makes the raw file playable later; the .idx per-frame kernel\n"
"sequence and timestamp is what lets you prove which frames were dropped.\n",
        me, REC_ROOT, THERMAL_LINK, BYID_GLOB, MAX_NBUF, DEF_NBUF, DEF_W, DEF_H,
        REC_ROOT, REC_ROOT, REC_ROOT);
}

int main(int argc, char **argv)
{
    const char *dev_opt = NULL, *out_opt = NULL, *root = REC_ROOT;
    unsigned long want_frames = 0;
    double want_secs = 0;
    unsigned nbuf = DEF_NBUF, W = DEF_W, H = DEF_H;
    uint32_t fourcc = DEF_FOURCC;
    int quiet = 0, no_idx = 0, do_list = 0, opt;

    while ((opt = getopt(argc, argv, "o:D:d:n:t:b:W:H:f:qILh")) != -1) {
        switch (opt) {
        case 'o': out_opt = optarg; break;
        case 'D': root = optarg; break;
        case 'd': dev_opt = optarg; break;
        case 'n': want_frames = strtoul(optarg, NULL, 10); break;
        case 't': want_secs = atof(optarg); break;
        case 'b': nbuf = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'W': W = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'H': H = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'f': fourcc = parse_fourcc(optarg); break;
        case 'q': quiet = 1; break;
        case 'I': no_idx = 1; break;
        case 'L': do_list = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (nbuf < 2 || nbuf > MAX_NBUF) die("-b must be 2..%d", MAX_NBUF);

    char *dev = dev_opt ? strdup(dev_opt) : resolve_device();
    if (do_list) { list_formats(dev); free(dev); return 0; }

    /* ---- output folder and name ------------------------------------------ */
    /* The name is decided here but nothing is created yet: the folder is made at
     * open time, after the device and format have been accepted, so a failed run
     * does not litter the tree with empty timestamped directories. */
    char outdir[512] = "", outbuf[640];
    if (out_opt) {
        snprintf(outbuf, sizeof outbuf, "%s", out_opt);
    } else {
        time_t t = time(NULL);
        struct tm tm;
        localtime_r(&t, &tm);
        char fc[5];
        fourcc_str(fourcc, fc);
        /* Two runs can start inside the same second; give the later one a suffix
         * rather than silently overwriting the earlier one's frames. */
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
            if (len < 0 || (size_t)len >= sizeof outdir)
                die("-D path too long");
            if (access(outdir, F_OK) != 0) break;
            if (++dup > 999) die("cannot find an unused folder under %s", root);
        }
        snprintf(outbuf, sizeof outbuf, "%s/raw-%ux%u-%s.gray", outdir, W, H, fc);
    }
    int to_stdout = (strcmp(outbuf, "-") == 0);

    /* An explicit -o may name directories too; honour them the same way. Doing
     * this from outbuf covers both branches, so there is one mkdir path only. */
    if (!to_stdout && !outdir[0]) {
        const char *slash = strrchr(outbuf, '/');
        if (slash && slash != outbuf)
            snprintf(outdir, sizeof outdir, "%.*s", (int)(slash - outbuf), outbuf);
    }

    /* ---- open device --------------------------------------------------- */
    int fd = open(dev, O_RDWR | O_CLOEXEC);
    if (fd < 0) die("cannot open %s: %s\n  in the video group? (id -nG)",
                    dev, strerror(errno));

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof cap);
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        die("VIDIOC_QUERYCAP on %s: %s (not a V4L2 device?)", dev, strerror(errno));
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        die("%s cannot capture video (caps 0x%08x).\n"
            "  The core's second node is metadata-only -- use -video-index0.",
            dev, cap.capabilities);
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        die("%s does not support streaming I/O (caps 0x%08x)", dev, cap.capabilities);

    /* ---- format: ask, then check what we actually got ------------------ */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = W;
    fmt.fmt.pix.height = H;
    fmt.fmt.pix.pixelformat = fourcc;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
        die("VIDIOC_S_FMT: %s", strerror(errno));

    /* S_FMT is a negotiation, not a command: the driver may hand back something
     * else. Accepting that silently is how a raw file ends up unreadable, with
     * nothing in it to say why -- so refuse instead of recording the wrong thing. */
    if (fmt.fmt.pix.pixelformat != fourcc ||
        fmt.fmt.pix.width != W || fmt.fmt.pix.height != H) {
        char want[5], got[5];
        fourcc_str(fourcc, want);
        fourcc_str(fmt.fmt.pix.pixelformat, got);
        die("driver refused the format.\n"
            "  asked for %ux%u %s, got %ux%u %s\n"
            "  run with -L to see what this core offers",
            W, H, want, fmt.fmt.pix.width, fmt.fmt.pix.height, got);
    }
    const uint32_t frame_sz = fmt.fmt.pix.sizeimage;
    const uint32_t stride = fmt.fmt.pix.bytesperline;
    char fcs[5];
    fourcc_str(fourcc, fcs);

    /* ---- buffers -------------------------------------------------------- */
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof rb);
    rb.count = nbuf;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
        die("VIDIOC_REQBUFS (%u buffers): %s", nbuf, strerror(errno));
    if (rb.count < 2) die("driver gave only %u buffers, need >= 2", rb.count);
    if (rb.count != nbuf)
        fprintf(stderr, "note: asked for %u buffers, driver gave %u\n", nbuf, rb.count);
    nbuf = rb.count;

    void **bufs = calloc(nbuf, sizeof *bufs);
    size_t *blen = calloc(nbuf, sizeof *blen);
    if (!bufs || !blen) die("out of memory");
    for (unsigned i = 0; i < nbuf; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) == -1)
            die("VIDIOC_QUERYBUF %u: %s", i, strerror(errno));
        blen[i] = b.length;
        bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd, b.m.offset);
        if (bufs[i] == MAP_FAILED) die("mmap buffer %u: %s", i, strerror(errno));
    }

    /* ---- output files --------------------------------------------------- */
    int ofd;
    if (to_stdout) {
        ofd = STDOUT_FILENO;
    } else {
        if (outdir[0] && mkdir_p(outdir) == -1)
            die("cannot create folder %s: %s", outdir, strerror(errno));
        ofd = open(outbuf, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (ofd < 0) die("cannot create %s: %s", outbuf, strerror(errno));
    }
    FILE *idx = NULL;
    char idxpath[700], metapath[700];
    snprintf(idxpath, sizeof idxpath, "%s.idx", outbuf);
    snprintf(metapath, sizeof metapath, "%s.meta", outbuf);
    if (!no_idx && !to_stdout) {
        idx = fopen(idxpath, "w");
        if (!idx) die("cannot create %s: %s", idxpath, strerror(errno));
        fprintf(idx, "# frame kernel_seq buf_timestamp_s bytes flags\n");
    }

    /* Space check, only when the run length is known up front. */
    if (!to_stdout && (want_frames || want_secs > 0)) {
        double est_frames = want_frames ? (double)want_frames : want_secs * 33.0;
        double need = est_frames * frame_sz;
        struct statvfs vfs;
        if (statvfs(outdir[0] ? outdir : ".", &vfs) == 0) {
            double avail = (double)vfs.f_bavail * vfs.f_frsize;
            if (need > avail)
                die("not enough space: need ~%.1f GB, %.1f GB free",
                    need / 1e9, avail / 1e9);
            if (!quiet)
                fprintf(stderr, "estimate    %.1f GB of %.1f GB free\n",
                        need / 1e9, avail / 1e9);
        }
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* ---- queue everything, then start ---------------------------------- */
    for (unsigned i = 0; i < nbuf; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QBUF, &b) == -1)
            die("VIDIOC_QBUF %u: %s", i, strerror(errno));
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1)
        die("VIDIOC_STREAMON: %s\n  another process may hold %s", strerror(errno), dev);

    if (!quiet) {
        fprintf(stderr, "device      %s\n", dev);
        fprintf(stderr, "format      %ux%u %s  %u B/frame  %u B/line  (%s)\n",
                W, H, fcs, frame_sz, stride, "uncompressed, unconverted");
        fprintf(stderr, "buffers     %u mmap (%.1f MB ring)\n",
                nbuf, (double)nbuf * frame_sz / 1e6);
        if (!to_stdout && outdir[0]) fprintf(stderr, "folder      %s/\n", outdir);
        fprintf(stderr, "output      %s\n", to_stdout ? "(stdout)" : outbuf);
        if (idx) fprintf(stderr, "index       %s\n", idxpath);
        fprintf(stderr, "recording   Ctrl-C to stop\n");
    }

    /* ---- capture loop --------------------------------------------------- */
    const double t0 = now_mono();
    double t_status = 0, t_win = t0;
    unsigned long frames = 0, win_frames = 0, dropped = 0, err_frames = 0,
                  short_frames = 0, timeouts = 0, seq_anomalies = 0;
    unsigned long long bytes = 0, last_flush = 0;
    uint32_t last_seq = 0;
    int have_seq = 0;
    double inst_fps = 0.0;
    const char *stop_why = "signal";
    int isatty_err = isatty(STDERR_FILENO);

    while (!g_stop) {
        if (want_frames && frames >= want_frames) { stop_why = "frame count"; break; }
        if (want_secs > 0 && now_mono() - t0 >= want_secs) { stop_why = "duration"; break; }

        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 2000);
        if (pr == -1) {
            if (errno == EINTR) continue;      /* Ctrl-C */
            die("poll: %s", strerror(errno));
        }
        if (pr == 0) {
            timeouts++;
            fprintf(stderr, "\nwarning: no frame for 2s (timeout %lu)\n", timeouts);
            if (timeouts >= 3) { stop_why = "device stopped delivering frames"; break; }
            continue;
        }

        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) == -1) {
            if (errno == EAGAIN) continue;
            die("VIDIOC_DQBUF: %s", strerror(errno));
        }
        timeouts = 0;

        /* buf.sequence counts frames the driver produced. A forward jump means
         * frames were lost upstream of us (USB or driver), which no amount of
         * buffering here would have caught -- so it is counted, not hidden.
         *
         * It can also repeat or rewind: a stream that never really started hands
         * back error buffers all stamped 0. Subtracting unsigned there underflows
         * to ~4.29e9 per frame, which is how a failed capture came to claim
         * 8589934590 dropped frames. Count only genuine forward gaps and track
         * the non-monotonic case separately. */
        if (have_seq) {
            if (b.sequence > last_seq + 1)
                dropped += b.sequence - last_seq - 1;
            else if (b.sequence <= last_seq)
                seq_anomalies++;
        }
        last_seq = b.sequence;
        have_seq = 1;

        if (b.flags & V4L2_BUF_FLAG_ERROR) err_frames++;
        if (b.bytesused != frame_sz) short_frames++;

        /* Exactly bytesused bytes, nothing added, nothing dropped. */
        if (b.bytesused && write_all(ofd, bufs[b.index], b.bytesused) == -1)
            die("write to %s: %s", to_stdout ? "stdout" : outbuf, strerror(errno));
        bytes += b.bytesused;
        frames++;
        win_frames++;

        if (idx)
            fprintf(idx, "%lu %u %ld.%06ld %u 0x%08x\n", frames - 1, b.sequence,
                    (long)b.timestamp.tv_sec, (long)b.timestamp.tv_usec,
                    b.bytesused, b.flags);

        if (xioctl(fd, VIDIOC_QBUF, &b) == -1)
            die("VIDIOC_QBUF %u: %s", b.index, strerror(errno));

        /* Long runs otherwise fill the page cache with data nobody will re-read;
         * push it out and let the kernel forget all but the recent tail. */
        if (!to_stdout && bytes - last_flush >= (unsigned long long)FLUSH_MB << 20) {
            sync_file_range(ofd, (off_t)last_flush, (off_t)(bytes - last_flush),
                            SYNC_FILE_RANGE_WRITE);
            if (bytes > ((unsigned long long)KEEP_MB << 20))
                posix_fadvise(ofd, 0,
                              (off_t)(bytes - ((unsigned long long)KEEP_MB << 20)),
                              POSIX_FADV_DONTNEED);
            last_flush = bytes;
        }

        /* ---- status line ------------------------------------------------ */
        double now = now_mono();
        if (now - t_win >= INST_WIN) {
            inst_fps = win_frames / (now - t_win);
            win_frames = 0;
            t_win = now;
        }
        if (!quiet && now - t_status >= 1.0 / STATUS_HZ) {
            t_status = now;
            double el = now - t0;
            /* No instantaneous rate exists until the first window closes; showing
             * the average there beats printing a 0.00 that looks like a stall. */
            double shown = inst_fps > 0.0 ? inst_fps : (el > 0 ? frames / el : 0.0);
            fprintf(stderr,
                    "\r  %6.1fs  %8lu fr  %6.2f fps now  %6.2f avg  "
                    "%7.2f GB  %5.1f MB/s  drop %lu%s",
                    el, frames, shown, el > 0 ? frames / el : 0.0,
                    bytes / 1e9, el > 0 ? bytes / el / 1e6 : 0.0, dropped,
                    isatty_err ? "   " : "\n");
            fflush(stderr);
        }
    }

    /* ---- stop and summarise -------------------------------------------- */
    double el = now_mono() - t0;
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
        fprintf(stderr, "\nwarning: VIDIOC_STREAMOFF: %s\n", strerror(errno));
    for (unsigned i = 0; i < nbuf; i++) munmap(bufs[i], blen[i]);
    free(bufs);
    free(blen);
    close(fd);
    if (idx) fclose(idx);
    if (!to_stdout) {
        if (fdatasync(ofd) == -1)
            fprintf(stderr, "\nwarning: fdatasync: %s\n", strerror(errno));
        close(ofd);
    }

    if (!quiet) fputc('\n', stderr);
    double mean = el > 0 ? frames / el : 0.0;

    if (!to_stdout) {
        FILE *m = fopen(metapath, "w");
        if (m) {
            time_t wall = time(NULL);
            fprintf(m,
                "# raw capture written by record_raw.c -- no compression, no conversion\n"
                "file            %s\n"
                "device          %s\n"
                "card            %s\n"
                "driver          %s\n"
                "width           %u\n"
                "height          %u\n"
                "fourcc          %s\n"
                "bytes_per_line  %u\n"
                "bytes_per_frame %u\n"
                "frames          %lu\n"
                "bytes           %llu\n"
                "duration_s      %.3f\n"
                "mean_fps        %.4f\n"
                "dropped_frames  %lu\n"
                "error_frames    %lu\n"
                "short_frames    %lu\n"
                "seq_anomalies   %lu\n"
                "finished_unix   %ld\n"
                "layout          frames stored back to back, no header, no padding\n",
                outbuf, dev, cap.card, cap.driver, W, H, fcs, stride, frame_sz,
                frames, bytes, el, mean, dropped, err_frames, short_frames,
                seq_anomalies, (long)wall);
            fclose(m);
        }
    }

    fprintf(stderr, "stopped     %s\n", g_stop ? "signal" : stop_why);
    fprintf(stderr, "frames      %lu in %.2f s  =  %.2f fps mean\n", frames, el, mean);
    fprintf(stderr, "written     %llu bytes (%.2f GB) at %.1f MB/s\n",
            bytes, bytes / 1e9, el > 0 ? bytes / el / 1e6 : 0.0);
    if (dropped)
        fprintf(stderr, "DROPPED     %lu frame(s) lost upstream (kernel sequence gaps)"
                        " -- %.2f%% of %lu produced\n",
                dropped, 100.0 * dropped / (double)(frames + dropped),
                frames + dropped);
    else
        fprintf(stderr, "dropped     0 -- kernel sequence numbers are contiguous\n");
    if (err_frames)
        fprintf(stderr, "ERRORS      %lu frame(s) flagged V4L2_BUF_FLAG_ERROR"
                        " (content unreliable, still written)\n", err_frames);
    if (short_frames)
        fprintf(stderr, "SHORT       %lu frame(s) were not %u bytes"
                        " -- see the .idx file\n", short_frames, frame_sz);
    if (seq_anomalies)
        fprintf(stderr, "SEQUENCE    %lu frame(s) did not advance the kernel sequence"
                        " -- the stream likely never started\n", seq_anomalies);
    if (!to_stdout) {
        if (outdir[0]) fprintf(stderr, "folder      %s/\n", outdir);
        fprintf(stderr, "output      %s\n", outbuf);
        fprintf(stderr, "metadata    %s\n", metapath);
        if (idx) fprintf(stderr, "index       %s\n", idxpath);
        const char *pf = ff_pix_fmt(fourcc);
        if (pf)
            fprintf(stderr, "\nplay it back:\n"
                            "  ffplay -f rawvideo -pixel_format %s -video_size %ux%u"
                            " -framerate %.2f %s\n",
                    pf, W, H, mean > 0 ? mean : 25.0, outbuf);
        else
            fprintf(stderr, "\nraw %s, %ux%u, %u B/frame -- no ffmpeg pixel-format\n"
                            "name is known for this fourcc; see %s\n",
                    fcs, W, H, frame_sz, metapath);
    }
    free(dev);

    /* An empty or all-error capture must not exit 0: a script that only checks the
     * status would file a folder of zero frames as a good recording. */
    if (frames == 0 || bytes == 0) {
        fprintf(stderr, "\nFAILED      captured nothing usable"
                        " (%lu frames, %llu bytes) -- exiting non-zero\n",
                frames, bytes);
        return 1;
    }
    if (err_frames == frames) {
        fprintf(stderr, "\nFAILED      every frame was flagged bad"
                        " -- exiting non-zero\n");
        return 1;
    }
    return 0;
}
