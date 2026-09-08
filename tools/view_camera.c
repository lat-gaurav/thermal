/* view_camera.c -- watch the thermal camera from a browser, without ever
 * costing the camera a frame.
 *
 *   ./view_camera                      then open http://<jetson-ip>:8080/
 *
 * WHY THE DISPLAY CANNOT DROP A CAMERA FRAME. Acquisition does exactly three
 * things per frame: DQBUF, copy the pixels into a spare buffer, QBUF. The copy is
 * a bounded ~0.3 ms memcpy of 1.31 MB; the buffer is back with the driver
 * immediately. JPEG encoding and every byte of network I/O happen on other
 * threads, and they hold nothing the driver needs.
 *
 * Two buffers are swapped under a mutex rather than copied twice: the acquisition
 * thread fills `spare`, swaps it with `pending`, and carries on. The encoder takes
 * `pending` whenever it is free. If the encoder or the network is slower than the
 * camera, the acquisition thread simply overwrites `spare` again -- so DISPLAY
 * frames are dropped and CAMERA frames are not. That asymmetry is the whole point
 * of the design, and the status line reports both numbers separately so you can
 * see it holding.
 *
 * A slow client cannot stall the others either: sockets are non-blocking, each
 * client has its own send buffer, and a client still transmitting the previous
 * frame is skipped for this one instead of being waited on.
 *
 * DOWNSCALE. The default -d 2 sends 640x512. That is not an arbitrary choice:
 * 0x1C core info reports the detector is 640x512 and the core upscales 2x before
 * sending over UVC, so -d 2 is the true sensor resolution with no interpolated
 * pixels. It also cuts the JPEG to a quarter. Use -d 1 for the full 1280x1024 if
 * you want to see exactly what is recorded.
 *
 * ONE STREAMER ONLY. The core allows a single process to stream. This program
 * holds the camera while it runs, so it cannot be used at the same time as
 * flow_stamp or record_raw -- stop those first.
 *
 * Build:  gcc -O2 -Wall -Wextra -o view_camera view_camera.c -ljpeg -lpthread
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <jpeglib.h>   /* after stdio.h: jpeglib.h uses FILE without including it */
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define DEF_W      1280
#define DEF_H      1024
#define DEF_PORT   8080
#define DEF_QUAL   70
#define DEF_DOWN   2
#define DEF_FPS    0        /* 0 = uncapped: measured to cost the camera nothing */
#define DEF_NBUF   8
#define MAX_NBUF   32
#define MAXCLIENT  8
#define CLIBUF     (1u << 20)      /* 1 MB per client is far more than a frame */
#define BOUNDARY   "thermalframe"

static const char *THERMAL_LINK = "/dev/thermal0";
static const char *VID_BYID_GLOB =
    "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fputc('\n', stderr); vfprintf(stderr, fmt, ap); va_end(ap); fputc('\n', stderr);
    exit(1);
}

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r; do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static double now_mono(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

static char *resolve_video(const char *ex)
{
    if (ex) return strdup(ex);
    if (access(THERMAL_LINK, F_OK) == 0) return strdup(THERMAL_LINK);
    glob_t g;
    if (glob(VID_BYID_GLOB, 0, NULL, &g) == 0 && g.gl_pathc > 0) {
        char *p = strdup(g.gl_pathv[0]); globfree(&g); return p;
    }
    globfree(&g);
    return NULL;
}

/* ---- the one-slot frame handoff --------------------------------------- */

static unsigned g_W = DEF_W, g_H = DEF_H;
static uint8_t *g_spare, *g_pending;       /* full-size grey frames */
static int g_pending_ready = 0;
static uint64_t g_pending_seq = 0;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

/* stats */
static unsigned long s_frames = 0, s_dropped = 0, s_seqanom = 0, s_bad = 0;
static unsigned long s_shown = 0, s_skip_cap = 0, s_skip_busy = 0;
static double s_enc_sum = 0, s_enc_max = 0;
static size_t s_jpeg_last = 0;

/* ---- JPEG ------------------------------------------------------------- */

/* Downscale by an integer factor with a box average, then compress. Averaging
 * rather than point-sampling matters on a thermal image: nearest-neighbour makes
 * sensor noise look like structure, which is misleading when the whole point is
 * to judge what the camera is seeing. */
static size_t encode_jpeg(const uint8_t *src, unsigned W, unsigned H,
                          unsigned down, int quality,
                          uint8_t **out, unsigned long *out_cap)
{
    unsigned ow = W / down, oh = H / down;
    static uint8_t *row = NULL;
    static unsigned row_cap = 0;
    if (row_cap < ow) { free(row); row = malloc(ow); row_cap = ow; }
    if (!row) return 0;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, out, out_cap);
    cinfo.image_width = ow;
    cinfo.image_height = oh;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < oh) {
        unsigned y0 = cinfo.next_scanline * down;
        for (unsigned x = 0; x < ow; x++) {
            unsigned sum = 0;
            for (unsigned dy = 0; dy < down; dy++) {
                const uint8_t *s = src + (size_t)(y0 + dy) * W + x * down;
                for (unsigned dx = 0; dx < down; dx++) sum += s[dx];
            }
            row[x] = (uint8_t)(sum / (down * down));
        }
        JSAMPROW r = row;
        jpeg_write_scanlines(&cinfo, &r, 1);
    }
    jpeg_finish_compress(&cinfo);
    size_t len = *out_cap;
    jpeg_destroy_compress(&cinfo);
    return len;
}

/* ---- HTTP ------------------------------------------------------------- */

struct client {
    int fd;
    int streaming;                  /* headers sent, now pushing parts */
    uint8_t *buf; size_t len, sent;
};
static struct client g_cli[MAXCLIENT];
static pthread_mutex_t g_cli_mtx = PTHREAD_MUTEX_INITIALIZER;

static const char PAGE[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
    "<!doctype html><title>Thermal camera</title>"
    "<style>body{margin:0;background:#111;color:#ccc;font:13px system-ui;"
    "display:flex;flex-direction:column;align-items:center;gap:8px;padding:12px}"
    "img{max-width:100%;image-rendering:pixelated;border:1px solid #333}</style>"
    "<img src=\"/stream\">"
    "<div>thermal core &mdash; live MJPEG</div>";

static const char STREAM_HDR[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
    "Cache-Control: no-store\r\nConnection: close\r\n\r\n";

static void client_close(struct client *c)
{
    if (c->fd >= 0) close(c->fd);
    c->fd = -1; c->streaming = 0; c->len = c->sent = 0;
}

/* Push whatever is pending for this client. Never blocks: a partial write just
 * leaves the rest for next time, and a client mid-frame is skipped rather than
 * waited on. */
static void client_pump(struct client *c)
{
    while (c->sent < c->len) {
        ssize_t w = send(c->fd, c->buf + c->sent, c->len - c->sent, MSG_DONTWAIT);
        if (w > 0) { c->sent += (size_t)w; continue; }
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        if (w < 0 && errno == EINTR) continue;
        client_close(c);
        return;
    }
}

static void broadcast(const uint8_t *jpg, size_t jlen)
{
    char head[256];
    int hl = snprintf(head, sizeof head,
                      "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
                      "Content-Length: %zu\r\n\r\n", jlen);
    pthread_mutex_lock(&g_cli_mtx);
    for (int i = 0; i < MAXCLIENT; i++) {
        struct client *c = &g_cli[i];
        if (c->fd < 0 || !c->streaming) continue;
        client_pump(c);
        if (c->fd < 0) continue;
        if (c->sent < c->len) { s_skip_busy++; continue; }  /* still busy: skip */
        if ((size_t)hl + jlen + 2 > CLIBUF) continue;
        memcpy(c->buf, head, hl);
        memcpy(c->buf + hl, jpg, jlen);
        memcpy(c->buf + hl + jlen, "\r\n", 2);
        c->len = hl + jlen + 2; c->sent = 0;
        client_pump(c);
    }
    pthread_mutex_unlock(&g_cli_mtx);
}

static int g_listen = -1;

static void *net_thread(void *arg)
{
    (void)arg;
    while (!g_stop) {
        struct pollfd p[1 + MAXCLIENT];
        int n = 0;
        p[n].fd = g_listen; p[n].events = POLLIN; p[n].revents = 0; n++;
        pthread_mutex_lock(&g_cli_mtx);
        int map[MAXCLIENT];
        for (int i = 0; i < MAXCLIENT; i++) {
            map[i] = -1;
            if (g_cli[i].fd < 0) continue;
            map[i] = n;
            p[n].fd = g_cli[i].fd;
            p[n].events = POLLIN | (g_cli[i].sent < g_cli[i].len ? POLLOUT : 0);
            p[n].revents = 0; n++;
        }
        pthread_mutex_unlock(&g_cli_mtx);

        if (poll(p, n, 200) <= 0) continue;

        if (p[0].revents & POLLIN) {
            struct sockaddr_in sa; socklen_t sl = sizeof sa;
            int fd = accept(g_listen, (struct sockaddr *)&sa, &sl);
            if (fd >= 0) {
                int one = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
                fcntl(fd, F_SETFL, O_NONBLOCK);
                pthread_mutex_lock(&g_cli_mtx);
                int slot = -1;
                for (int i = 0; i < MAXCLIENT; i++) if (g_cli[i].fd < 0) { slot = i; break; }
                if (slot < 0) { close(fd); }
                else { g_cli[slot].fd = fd; g_cli[slot].streaming = 0;
                       g_cli[slot].len = g_cli[slot].sent = 0; }
                pthread_mutex_unlock(&g_cli_mtx);
            }
        }

        pthread_mutex_lock(&g_cli_mtx);
        for (int i = 0; i < MAXCLIENT; i++) {
            struct client *c = &g_cli[i];
            if (c->fd < 0 || map[i] < 0) continue;
            short re = p[map[i]].revents;
            if (re & (POLLHUP | POLLERR)) { client_close(c); continue; }
            if ((re & POLLIN) && !c->streaming) {
                char req[1024];
                ssize_t r = recv(c->fd, req, sizeof req - 1, MSG_DONTWAIT);
                if (r <= 0) { if (r == 0) client_close(c); continue; }
                req[r] = '\0';
                if (strstr(req, "GET /stream")) {
                    memcpy(c->buf, STREAM_HDR, sizeof STREAM_HDR - 1);
                    c->len = sizeof STREAM_HDR - 1; c->sent = 0;
                    c->streaming = 1;
                    client_pump(c);
                } else {
                    memcpy(c->buf, PAGE, sizeof PAGE - 1);
                    c->len = sizeof PAGE - 1; c->sent = 0;
                    client_pump(c);
                    /* one-shot page: close once it is out */
                    if (c->sent >= c->len) client_close(c);
                }
            } else if (re & POLLOUT) {
                client_pump(c);
                if (c->fd >= 0 && !c->streaming && c->sent >= c->len) client_close(c);
            }
        }
        pthread_mutex_unlock(&g_cli_mtx);
    }
    return NULL;
}

/* ---- encoder ----------------------------------------------------------- */

static int g_down = DEF_DOWN, g_qual = DEF_QUAL;
/* 0 means uncapped; main() sets this from -f. Not written as 1.0/DEF_FPS
 * because DEF_FPS is 0 and that is a compile-time division by zero. */
static double g_min_dt = 0.0;

static void *enc_thread(void *arg)
{
    (void)arg;
    uint8_t *mine = malloc((size_t)g_W * g_H);
    if (!mine) die("encoder: out of memory");
    uint8_t *jpg = NULL; unsigned long jcap = 0;
    double last = 0;

    while (!g_stop) {
        pthread_mutex_lock(&g_mtx);
        while (!g_pending_ready && !g_stop) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 100 * 1000 * 1000;
            if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
            pthread_cond_timedwait(&g_cv, &g_mtx, &ts);
        }
        if (g_stop) { pthread_mutex_unlock(&g_mtx); break; }
        /* take the pending frame by swapping, so no second copy is needed */
        uint8_t *t = g_pending; g_pending = mine; mine = t;
        g_pending_ready = 0;
        pthread_mutex_unlock(&g_mtx);

        /* Rate-cap the display; the camera keeps running at full speed.
         * The 0.9 tolerance matters: capping at exactly the camera rate makes the
         * comparison beat against frame-interval jitter, and frames arriving a
         * few hundred microseconds early get thrown away -- measured as 15.6 fps
         * shown out of 25 with -f 25 before this factor was added.
         * g_min_dt == 0 means uncapped. */
        double now = now_mono();
        if (g_min_dt > 0 && now - last < g_min_dt * 0.9) { s_skip_cap++; continue; }
        last = now;

        double t0 = now_mono();
        size_t jlen = encode_jpeg(mine, g_W, g_H, g_down, g_qual, &jpg, &jcap);
        double cost = (now_mono() - t0) * 1000.0;
        s_enc_sum += cost; if (cost > s_enc_max) s_enc_max = cost;
        if (!jlen) continue;
        s_jpeg_last = jlen;
        broadcast(jpg, jlen);
        s_shown++;
    }
    free(mine);
    if (jpg) free(jpg);
    return NULL;
}

static void usage(const char *me)
{
    printf(
"Serve the thermal camera as MJPEG over HTTP. Acquisition is never delayed by\n"
"the display: encoding and network run on other threads, and DISPLAY frames are\n"
"dropped rather than camera frames.\n"
"\n"
"usage: %s [-p PORT] [-d DOWN] [-Q QUAL] [-f FPS] [-b BUFS]\n"
"          [-W W] [-H H] [-v DEV] [-q]\n"
"\n"
"  -p PORT    listen port (default %d)\n"
"  -d DOWN    integer downscale, box-averaged (default %d -> %dx%d, the\n"
"             detector's true resolution; use 1 for full size)\n"
"  -Q QUAL    JPEG quality 1..100 (default %d)\n"
"  -f FPS     cap the DISPLAY rate, 0 = uncapped (default %d; the camera\n"
"             always runs at full speed regardless)\n"
"  -b BUFS    v4l2 buffers, 3..%d (default %d)\n"
"  -W, -H     capture geometry (default %dx%d)\n"
"  -v DEV     video device override\n"
"  -q         no live status line\n"
"\n"
"Then open  http://<jetson-ip>:%d/  in a browser on your laptop.\n"
"Only one process may stream from this core, so stop flow_stamp/record_raw first.\n",
        me, DEF_PORT, DEF_DOWN, DEF_W/DEF_DOWN, DEF_H/DEF_DOWN, DEF_QUAL,
        DEF_FPS, MAX_NBUF, DEF_NBUF, DEF_W, DEF_H, DEF_PORT);
}

int main(int argc, char **argv)
{
    int port = DEF_PORT, nbuf = DEF_NBUF, quiet = 0, c;
    int maxfps = DEF_FPS;
    const char *vdev_opt = NULL;

    while ((c = getopt(argc, argv, "p:d:Q:f:b:W:H:v:qh")) != -1) {
        switch (c) {
        case 'p': port = atoi(optarg); break;
        case 'd': g_down = atoi(optarg); break;
        case 'Q': g_qual = atoi(optarg); break;
        case 'f': maxfps = atoi(optarg); break;
        case 'b': nbuf = atoi(optarg); break;
        case 'W': g_W = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'H': g_H = (unsigned)strtoul(optarg, NULL, 10); break;
        case 'v': vdev_opt = optarg; break;
        case 'q': quiet = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (g_down < 1 || g_down > 8) die("-d must be 1..8");
    if (g_W % g_down || g_H % g_down)
        die("-d %d does not divide %ux%u evenly", g_down, g_W, g_H);
    if (g_qual < 1 || g_qual > 100) die("-Q must be 1..100");
    if (maxfps < 0) die("-f must be >= 0 (0 = uncapped)");
    if (nbuf < 3 || nbuf > MAX_NBUF) die("-b must be 3..%d", MAX_NBUF);
    if (port < 1 || port > 65535) die("-p must be 1..65535");
    g_min_dt = maxfps > 0 ? 1.0 / maxfps : 0.0;

    signal(SIGPIPE, SIG_IGN);          /* a client vanishing must not kill us */
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL);

    char *vdev = resolve_video(vdev_opt);
    if (!vdev) die("no Sirius capture node found (looked for %s, then %s)\n"
                   "  is the camera plugged in?  ls -l /dev/v4l/by-id/",
                   THERMAL_LINK, VID_BYID_GLOB);

    int vfd = open(vdev, O_RDWR | O_CLOEXEC);
    if (vfd < 0) die("cannot open %s: %s\n  in the video group? (id -nG)",
                    vdev, strerror(errno));

    struct v4l2_capability cap; memset(&cap, 0, sizeof cap);
    if (xioctl(vfd, VIDIOC_QUERYCAP, &cap) == -1)
        die("VIDIOC_QUERYCAP: %s", strerror(errno));
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        die("%s cannot capture video -- the core's second node is metadata only,\n"
            "  use -video-index0", vdev);

    struct v4l2_format fmt; memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = g_W; fmt.fmt.pix.height = g_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(vfd, VIDIOC_S_FMT, &fmt) == -1) die("VIDIOC_S_FMT: %s", strerror(errno));
    if (fmt.fmt.pix.width != g_W || fmt.fmt.pix.height != g_H ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY)
        die("driver refused %ux%u GREY; got %ux%u. The encoder indexes the buffer\n"
            "  as W*H bytes, so a substituted format would render as garbage.",
            g_W, g_H, fmt.fmt.pix.width, fmt.fmt.pix.height);
    const uint32_t frame_sz = fmt.fmt.pix.sizeimage;

    struct v4l2_requestbuffers rb; memset(&rb, 0, sizeof rb);
    rb.count = nbuf; rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd, VIDIOC_REQBUFS, &rb) == -1)
        die("VIDIOC_REQBUFS (%d): %s", nbuf, strerror(errno));
    if (rb.count < 3) die("driver gave only %u buffers, need >= 3", rb.count);
    nbuf = rb.count;

    void **bufs = calloc(nbuf, sizeof *bufs);
    size_t *blen = calloc(nbuf, sizeof *blen);
    if (!bufs || !blen) die("out of memory");
    for (int i = 0; i < nbuf; i++) {
        struct v4l2_buffer b; memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (xioctl(vfd, VIDIOC_QUERYBUF, &b) == -1) die("VIDIOC_QUERYBUF %d: %s", i, strerror(errno));
        blen[i] = b.length;
        bufs[i] = mmap(NULL, b.length, PROT_READ, MAP_SHARED, vfd, b.m.offset);
        if (bufs[i] == MAP_FAILED) die("mmap %d: %s", i, strerror(errno));
        if (xioctl(vfd, VIDIOC_QBUF, &b) == -1) die("VIDIOC_QBUF %d: %s", i, strerror(errno));
    }

    g_spare   = malloc((size_t)g_W * g_H);
    g_pending = malloc((size_t)g_W * g_H);
    if (!g_spare || !g_pending) die("out of memory for the frame slots");

    /* listening socket */
    g_listen = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen < 0) die("socket: %s", strerror(errno));
    int one = 1;
    setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in sa2; memset(&sa2, 0, sizeof sa2);
    sa2.sin_family = AF_INET; sa2.sin_addr.s_addr = htonl(INADDR_ANY);
    sa2.sin_port = htons(port);
    if (bind(g_listen, (struct sockaddr *)&sa2, sizeof sa2) < 0)
        die("bind port %d: %s\n  is something already listening? try -p", port, strerror(errno));
    if (listen(g_listen, 8) < 0) die("listen: %s", strerror(errno));
    fcntl(g_listen, F_SETFL, O_NONBLOCK);
    for (int i = 0; i < MAXCLIENT; i++) {
        g_cli[i].fd = -1;
        g_cli[i].buf = malloc(CLIBUF);
        if (!g_cli[i].buf) die("out of memory for client buffers");
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(vfd, VIDIOC_STREAMON, &type) == -1)
        die("VIDIOC_STREAMON: %s\n  another process may hold %s"
            " (the core allows one streamer)", strerror(errno), vdev);

    pthread_t tid_net, tid_enc;
    if (pthread_create(&tid_net, NULL, net_thread, NULL) != 0) die("net thread");
    if (pthread_create(&tid_enc, NULL, enc_thread, NULL) != 0) die("encoder thread");

    if (!quiet) {
        fprintf(stderr, "video       %s  %ux%u GREY  %u buffers\n", vdev, g_W, g_H, nbuf);
        fprintf(stderr, "serving     %ux%u JPEG q%d, display capped at %d fps\n",
                g_W / g_down, g_H / g_down, g_qual, maxfps);
        fprintf(stderr, "open        http://<jetson-ip>:%d/\n", port);
        fprintf(stderr, "            e.g. http://192.168.2.200:%d/  (ethernet)\n", port);
        fprintf(stderr, "Ctrl-C to stop\n");
    }

    double t0 = now_mono(), tstat = 0, prev_fb = 0;
    uint32_t last_seq = 0; int have_seq = 0;
    int isatty_err = isatty(2);

    while (!g_stop) {
        struct pollfd p = { .fd = vfd, .events = POLLIN, .revents = 0 };
        int pr = poll(&p, 1, 500);
        if (pr < 0) { if (errno == EINTR) continue; die("poll: %s", strerror(errno)); }
        if (pr == 0) continue;

        struct v4l2_buffer b; memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(vfd, VIDIOC_DQBUF, &b) == -1) {
            if (errno == EAGAIN) continue;
            die("VIDIOC_DQBUF: %s", strerror(errno));
        }
        uint32_t seq = b.sequence, flags = b.flags, used = b.bytesused;
        unsigned idx = b.index;
        double tfb = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;
        s_frames++;

        if (have_seq) {
            if (seq > last_seq + 1) s_dropped += seq - last_seq - 1;
            else if (seq <= last_seq) s_seqanom++;
        }
        last_seq = seq; have_seq = 1;

        int usable = !(flags & V4L2_BUF_FLAG_ERROR) && used >= frame_sz && tfb > 0;
        if (!usable) s_bad++;

        /* Copy, then requeue. The copy is the only pixel work on this thread. */
        if (usable) memcpy(g_spare, bufs[idx], (size_t)g_W * g_H);
        if (xioctl(vfd, VIDIOC_QBUF, &b) == -1)
            die("VIDIOC_QBUF %u: %s", idx, strerror(errno));

        if (usable) {
            pthread_mutex_lock(&g_mtx);
            uint8_t *t = g_spare; g_spare = g_pending; g_pending = t;
            if (g_pending_ready) s_skip_busy++;  /* overwrote an unshown frame */
            g_pending_ready = 1; g_pending_seq++;
            pthread_cond_signal(&g_cv);
            pthread_mutex_unlock(&g_mtx);
        }
        prev_fb = tfb;

        double now = now_mono();
        if (!quiet && now - tstat >= 0.25) {
            tstat = now;
            double el = now - t0;
            int nc = 0;
            pthread_mutex_lock(&g_cli_mtx);
            for (int i = 0; i < MAXCLIENT; i++) if (g_cli[i].fd >= 0) nc++;
            pthread_mutex_unlock(&g_cli_mtx);
            fprintf(stderr,
                "\r  %6.1fs cam %6.2ffps %7lu fr | shown %5.1ffps %6lu | "
                "skip %6lu | jpeg %5.0fkB | cli %d | DROP %lu%s",
                el, el > 0 ? s_frames / el : 0, s_frames,
                el > 0 ? s_shown / el : 0, s_shown, s_skip_cap + s_skip_busy,
                s_jpeg_last / 1024.0, nc, s_dropped, isatty_err ? "  " : "\n");
            fflush(stderr);
        }
    }
    (void)prev_fb;

    double el = now_mono() - t0;
    g_stop = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_join(tid_enc, NULL);
    pthread_join(tid_net, NULL);
    xioctl(vfd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < nbuf; i++) munmap(bufs[i], blen[i]);
    close(vfd); close(g_listen);
    for (int i = 0; i < MAXCLIENT; i++) { if (g_cli[i].fd >= 0) close(g_cli[i].fd); free(g_cli[i].buf); }

    if (!quiet) fputc('\n', stderr);
    printf("\n=== view_camera ===\n");   /* stdout: pipeable */
    printf("  ran              %.2f s\n", el);
    printf("  camera frames    %lu  (%.2f fps)\n", s_frames, el > 0 ? s_frames / el : 0);
    printf("  displayed        %lu  (%.2f fps)\n", s_shown, el > 0 ? s_shown / el : 0);
    printf("  skipped by cap   %lu  (rate limit, not congestion -- raise -f or use -f 0)\n",
           s_skip_cap);
    printf("  skipped as busy  %lu  (encoder or a client behind; display only)\n",
           s_skip_busy);
    if (s_shown)
        printf("  jpeg encode      mean %.2f ms   max %.2f ms\n",
               s_enc_sum / s_shown, s_enc_max);
    if (s_bad) printf("  unusable frames  %lu (error/short)\n", s_bad);
    if (s_seqanom) printf("  seq anomalies    %lu\n", s_seqanom);
    if (s_dropped)
        printf("\n  CAMERA FRAMES DROPPED: %lu (kernel sequence gaps).\n"
               "  Display skips do not cause this -- acquisition only ever does\n"
               "  DQBUF, one memcpy, QBUF. Suspect the USB link or the fps mode.\n",
               s_dropped);
    else
        printf("\n  no camera frames dropped -- the display cost nothing\n");
    free(g_spare); free(g_pending); free(bufs); free(blen); free(vdev);
    return s_dropped ? 1 : 0;
}
