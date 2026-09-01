/* blob_log.c -- log gimbal IMU at ~1 kHz and blob detections at camera rate,
 * with an annotated live preview that is allowed to drop frames.
 *
 *   ./blob_log -t 30
 *   then open  http://<jetson-ip>:8080/  from the laptop
 *
 * STRICT PRIORITY ORDER. This is the whole point of the design, so it is worth
 * stating before the code:
 *
 *   1. frame acquisition -- the acquisition thread only ever does
 *                           DQBUF -> snapshot -> pair an IMU sample -> QBUF.
 *                           No file I/O, no pixel work, no allocation, no
 *                           socket, and it never waits on another thread.
 *   2. IMU logging       -- its own thread on its own CPU. It reads the serial
 *                           port and writes imu.csv. Nothing else runs there.
 *   3. blob detection    -- its own thread, niced down, holding at most QDEPTH
 *                           of the driver's buffers. If it falls behind, the
 *                           frame is logged with det_state=skipped_queue and
 *                           acquisition carries on at full rate.
 *   4. preview           -- last, and the only stage designed to lose data.
 *                           One-slot handoff: a frame the encoder has not
 *                           collected yet is simply overwritten.
 *
 * Nothing lower in that list can block anything higher in it. That is enforced
 * structurally, not by tuning: the acquisition thread's only shared-state
 * interaction is a short mutex around a bounded queue, and every branch out of
 * it ends in a QBUF.
 *
 * WHY THE DETECTOR COPIES THE FRAME. It memcpy's the DMA buffer into its own
 * memory and requeues the buffer immediately, before detecting anything. Two
 * reasons: the driver gets its buffer back in ~2 ms instead of ~20 ms, and the
 * several passes detection needs then run against cached memory. Reading a
 * V4L2 DMA buffer directly was measured at roughly 4x the cost of reading a
 * normal page, so the copy pays for itself many times over.
 *
 * DETECTION is the detect_blob algorithm: threshold at a fraction of the frame
 * peak, label 8-connected components, keep the largest, report its
 * intensity-weighted centroid. See detect_blob.c for the reasoning behind each
 * choice. -S decimates the detection grid, which is close to free on this
 * camera: the core's real sensor is 640x512 and the 1280x1024 stream is a 2x
 * upscale, so -S 2 discards interpolated pixels rather than information.
 *
 * WHAT THE CENTROID MEANS: the blob's position in the IMAGE. With the gimbal
 * moving, that is blob motion plus camera motion. The yaw/pitch columns beside
 * it are what let you separate the two afterwards -- which is the reason this
 * program logs both against one clock.
 *
 * OUTPUT (one timestamped run folder)
 *   imu.csv       ~1 kHz, every telemetry frame that passed CRC
 *   blobs.csv     one row per acquired frame
 *   summary.txt   the run report, identical to what is printed at exit
 *
 * All t_ columns are CLOCK_MONOTONIC, verified against the buffer flags rather
 * than assumed, so IMU and frame times are directly comparable.
 *
 * Build:  gcc -O2 -Wall -Wextra -o blob_log blob_log.c -lpthread -ljpeg -lm
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <jpeglib.h>   /* after stdio.h: jpeglib.h uses FILE without including it */
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define DEF_W        1280
#define DEF_H        1024
#define DEF_NBUF     12
#define MAX_NBUF     64
#define QDEPTH       4           /* buffers the detector may hold; < NBUF-4 */
#define RECQ         4096
#define TLM_LEN      36
#define TLM_FLOATS   8
#define RINGSZ       16384
#define SERBUF       8192
#define LATN         500000

#define DEF_MINAREA  20
#define DEF_RATIO    0.75
#define THR_FLOOR    100
#define DEF_STEP     1

#define DEF_PORT     8080
#define DEF_QUAL     70
#define DEF_DOWN     2
#define MAXCLIENT    8
#define CLIBUF       (1u << 20)
#define BOUNDARY     "thermalframe"

#define IMU_FLUSH    2000        /* rows between imu.csv flushes  (~2 s) */
#define BLOB_FLUSH   25          /* rows between blobs.csv flushes (~1 s) */

/* Pinned by USB serial, never by videoN/ttyUSBn -- those drift. See DEVICE_NODES.md */
static const char *THERMAL_LINK  = "/dev/thermal0";
static const char *VID_BYID_GLOB = "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";
static const char *TLM_LINK      = "/dev/local_dds";
static const char *TLM_BYID_GLOB = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK  = "/dev/ttyUSB0";
static const char *REC_ROOT      = "recordings";

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

static void fourcc_str(uint32_t f, char o[5])
{
    o[0]=f&0xff; o[1]=(f>>8)&0xff; o[2]=(f>>16)&0xff; o[3]=(f>>24)&0xff; o[4]=0;
    for (int i=0;i<4;i++) if (o[i]<32||o[i]>126) o[i]='?';
}

static int mkdir_p(const char *path)
{
    char tmp[600]; snprintf(tmp,sizeof tmp,"%s",path);
    for (char *q=tmp+1; *q; q++) {
        if (*q!='/') continue;
        *q='\0';
        if (mkdir(tmp,0755)==-1 && errno!=EEXIST) return -1;
        *q='/';
    }
    if (mkdir(tmp,0755)==-1 && errno!=EEXIST) return -1;
    return 0;
}

static char *resolve(const char *ex, const char *link, const char *pat, const char *fb)
{
    if (ex) return strdup(ex);
    if (link && access(link,F_OK)==0) return strdup(link);
    if (pat) {
        glob_t g;
        if (glob(pat,0,NULL,&g)==0 && g.gl_pathc>0) {
            char *p=strdup(g.gl_pathv[0]); globfree(&g); return p;
        }
        globfree(&g);
    }
    if (fb && access(fb,F_OK)==0) return strdup(fb);
    return NULL;
}

static int cmp_d(const void *a, const void *b)
{
    double x=*(const double*)a, y=*(const double*)b;
    return x<y?-1:x>y?1:0;
}

/* Pin a thread to one CPU. Unprivileged and cheap. Failure is never fatal. */
static int pin_to(int cpu)
{
    cpu_set_t s; CPU_ZERO(&s); CPU_SET(cpu, &s);
    return pthread_setaffinity_np(pthread_self(), sizeof s, &s) == 0;
}

/* ===================== IMU / gimbal telemetry ============================= */

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t c=0xFFFF;
    for (size_t i=0;i<n;i++) { c^=d[i]; for (int b=0;b<8;b++) c=(c&1)?(c>>1)^0x8408:c>>1; }
    return c;
}

struct imu { double t; float yaw, pitch; };
static struct imu ring[RINGSZ];
static unsigned ring_n = 0;
static pthread_mutex_t ring_mtx = PTHREAD_MUTEX_INITIALIZER;

static void ring_push(double t, float yaw, float pitch)
{
    pthread_mutex_lock(&ring_mtx);
    struct imu *a=&ring[ring_n % RINGSZ];
    a->t=t; a->yaw=yaw; a->pitch=pitch;
    ring_n++;
    pthread_mutex_unlock(&ring_mtx);
}

/* Copies the nearest sample out: returning a pointer would hand back memory the
 * telemetry thread may overwrite a millisecond later. */
static int ring_nearest(double t, struct imu *out, unsigned *total)
{
    int found=0;
    pthread_mutex_lock(&ring_mtx);
    if (total) *total=ring_n;
    if (ring_n) {
        unsigned have = ring_n<RINGSZ?ring_n:RINGSZ;
        double bestd=1e18;
        for (unsigned k=0;k<have;k++) {
            const struct imu *a=&ring[(ring_n-1-k)%RINGSZ];
            double d=t-a->t; if (d<0) d=-d;
            if (d<bestd) { bestd=d; *out=*a; found=1; }
            else if (a->t<t) break;
        }
    }
    pthread_mutex_unlock(&ring_mtx);
    return found;
}

static int g_tlm_fd = -1;
static FILE *g_imu_csv = NULL;
static unsigned long g_nimu = 0, g_ncrc = 0, g_nresync = 0;
static uint8_t g_sbuf[SERBUF];
static size_t g_slen = 0;
static int g_tlm_pin = -1;
static double g_imu_worst_gap = 0;
static unsigned long g_imu_batched = 0;

static void drain_tlm(void)
{
    if (g_slen >= sizeof g_sbuf) g_slen = 0;      /* pathological: resync */
    ssize_t n = read(g_tlm_fd, g_sbuf+g_slen, sizeof g_sbuf - g_slen);
    if (n <= 0) return;
    double t = now_mono();
    g_slen += (size_t)n;
    size_t i=0;
    static double last_t = 0;
    while (g_slen-i >= TLM_LEN) {
        if (g_sbuf[i]!=0xA5 || g_sbuf[i+1]!=0x5A) { i++; g_nresync++; continue; }
        uint16_t want=(uint16_t)(g_sbuf[i+34]|(g_sbuf[i+35]<<8));
        /* advance 1 on a CRC miss: a real frame can start one byte into a false
         * A5 5A, and skipping both magic bytes would step over it */
        if (crc16(&g_sbuf[i+2],32)!=want) { g_ncrc++; i++; continue; }
        float fl[TLM_FLOATS];
        memcpy(fl,&g_sbuf[i+2],sizeof fl);
        ring_push(t, fl[0], fl[1]);
        if (last_t>0) {
            double g=t-last_t;
            if (g>g_imu_worst_gap) g_imu_worst_gap=g;
            /* Two samples read out of the tty in one read() share a timestamp.
             * No sample is lost when that happens, but t_imu for the second
             * one is early by up to a millisecond -- worth knowing before
             * using these times for sub-millisecond latency work. */
            if (g < 50e-6) g_imu_batched++;
        }
        last_t=t;
        if (g_imu_csv) {
            fprintf(g_imu_csv, "%.6f", t);
            for (int k=0;k<TLM_FLOATS;k++) fprintf(g_imu_csv, ",%.4f", (double)fl[k]);
            fputc('\n', g_imu_csv);
        }
        g_nimu++;
        /* Bound how much is lost if this process is killed outright, without
         * writing on every sample: one flush per ~2 s of telemetry. */
        if (g_imu_csv && g_nimu % IMU_FLUSH == 0) fflush(g_imu_csv);
        i += TLM_LEN;
    }
    memmove(g_sbuf, g_sbuf+i, g_slen-i);
    g_slen -= i;
}

static void *tlm_thread(void *arg)
{
    (void)arg;
    if (g_tlm_pin >= 0) pin_to(g_tlm_pin);
    while (!g_stop) {
        struct pollfd p={.fd=g_tlm_fd,.events=POLLIN,.revents=0};
        int r=poll(&p,1,100);
        if (r<0) { if (errno==EINTR) continue; break; }
        if (r>0 && (p.revents&POLLIN)) drain_tlm();
    }
    return NULL;
}

static int open_serial(const char *dev, speed_t baud)
{
    /* O_RDONLY: this program must never transmit to the gimbal. */
    int fd=open(dev,O_RDONLY|O_NOCTTY|O_NONBLOCK);
    if (fd<0) return -1;
    struct termios t;
    if (tcgetattr(fd,&t)<0) { close(fd); return -1; }
    cfmakeraw(&t);
    cfsetispeed(&t,baud); cfsetospeed(&t,baud);
    t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CRTSCTS;
    t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
    if (tcsetattr(fd,TCSANOW,&t)<0) { close(fd); return -1; }
    tcflush(fd,TCIFLUSH);
    return fd;
}

/* ===================== blob detection ==================================== */

struct det {
    int    valid;
    double cx, cy;          /* intensity-weighted centroid, full-frame px */
    long   area;            /* full-frame px equivalent */
    int    peak;            /* brightest pixel in the blob */
    int    fpeak;           /* brightest pixel in the whole frame */
    int    thr;
    int    ncomp;           /* components above -a */
    long   second;          /* area of the runner-up, full-frame px equivalent */
    int    x0, y0, x1, y1;
};

/* Iterative 8-connected flood fill over a decimated grid. Recursion would blow
 * the stack on a large bright region, which a saturated blob against a warm
 * background easily is.
 *
 * Coordinates here are grid coordinates; the caller scales them back to
 * full-frame pixels so every number leaving this file means the same thing
 * regardless of -S. */
static long fill_v(const uint8_t *img, unsigned W, unsigned vw, unsigned vh,
                   unsigned step, int thr, uint8_t *seen, int32_t *stack,
                   long start, double *sx, double *sy, double *sw, int *peak,
                   int *x0, int *y0, int *x1, int *y1)
{
    long sp=0, area=0;
    stack[sp++]=(int32_t)start;
    seen[start]=1;
    *sx=*sy=*sw=0; *peak=0;
    *x0=(int)vw; *y0=(int)vh; *x1=-1; *y1=-1;

    while (sp>0) {
        long p=stack[--sp];
        int vx=(int)(p % vw), vy=(int)(p / vw);
        int v=img[(size_t)((unsigned)vy*step)*W + (size_t)(unsigned)vx*step];
        double w=(double)v;
        *sx+=w*vx; *sy+=w*vy; *sw+=w;
        if (v>*peak) *peak=v;
        if (vx<*x0) *x0=vx;
        if (vy<*y0) *y0=vy;
        if (vx>*x1) *x1=vx;
        if (vy>*y1) *y1=vy;
        area++;

        for (int dy=-1;dy<=1;dy++) {
            int ny=vy+dy; if (ny<0||ny>=(int)vh) continue;
            for (int dx=-1;dx<=1;dx++) {
                int nx=vx+dx; if (nx<0||nx>=(int)vw) continue;
                long q=(long)ny*vw+nx;
                if (seen[q]) continue;
                /* Mark dark neighbours seen too: they can never start a
                 * component, so this saves re-reading them from every
                 * neighbouring pixel and from the outer scan. */
                seen[q]=1;
                if (img[(size_t)((unsigned)ny*step)*W + (size_t)(unsigned)nx*step] < thr)
                    continue;
                stack[sp++]=(int32_t)q;
            }
        }
    }
    return area;
}

static struct det detect(const uint8_t *img, unsigned W, unsigned H,
                         unsigned step, int fixed_thr, double ratio, long minarea,
                         uint8_t *seen, int32_t *stack)
{
    struct det d; memset(&d,0,sizeof d);
    unsigned vw=W/step, vh=H/step;
    long n=(long)vw*vh;

    int peak=0;
    for (unsigned vy=0; vy<vh; vy++) {
        const uint8_t *r = img + (size_t)(vy*step)*W;
        for (unsigned vx=0; vx<vw; vx++) { int v=r[(size_t)vx*step]; if (v>peak) peak=v; }
    }
    d.fpeak=peak;

    int thr = fixed_thr>0 ? fixed_thr : (int)(ratio*peak);
    if (thr<THR_FLOOR) thr=THR_FLOOR;
    if (thr>254) thr=254;
    d.thr=thr;

    memset(seen,0,(size_t)n);
    /* -a is given in full-frame pixels so it means the same thing at any -S */
    long minv = minarea/((long)step*step); if (minv<1) minv=1;

    long best=0, second=0;
    for (long i=0;i<n;i++) {
        if (seen[i]) continue;
        unsigned vx=(unsigned)(i%vw), vy=(unsigned)(i/vw);
        if (img[(size_t)(vy*step)*W + (size_t)vx*step] < thr) { seen[i]=1; continue; }
        double sx,sy,sw; int pk,bx0,by0,bx1,by1;
        long a=fill_v(img,W,vw,vh,step,thr,seen,stack,i,
                      &sx,&sy,&sw,&pk,&bx0,&by0,&bx1,&by1);
        if (a>=minv) d.ncomp++;
        if (a>=minv && a<=best && a>second) second=a;
        if (a>best && a>=minv) {
            if (best>second) second=best;      /* the old winner is now the runner-up */
            best=a;
            d.valid=1;
            d.cx = sw>0 ? (sx/sw)*step : 0;
            d.cy = sw>0 ? (sy/sw)*step : 0;
            d.area = a*(long)step*step;
            d.peak = pk;
            d.x0=bx0*(int)step; d.y0=by0*(int)step;
            d.x1=bx1*(int)step; d.y1=by1*(int)step;
        }
    }
    d.second = second*(long)step*step;
    return d;
}

/* ---- annotation -------------------------------------------------------- */

static void px_set(uint8_t *img, unsigned W, unsigned H, int x, int y, uint8_t v)
{
    if (x>=0 && y>=0 && x<(int)W && y<(int)H) img[(size_t)y*W+x]=v;
}

static void hline(uint8_t *img, unsigned W, unsigned H, int x0, int x1, int y,
                  int t, uint8_t v)
{
    for (int dy=0;dy<t;dy++) for (int x=x0;x<=x1;x++) px_set(img,W,H,x,y+dy,v);
}

static void vline(uint8_t *img, unsigned W, unsigned H, int y0, int y1, int x,
                  int t, uint8_t v)
{
    for (int dx=0;dx<t;dx++) for (int y=y0;y<=y1;y++) px_set(img,W,H,x+dx,y,v);
}

/* The box is drawn PADDED outward and the crosshair arms stop short of it, so
 * nothing is painted over the blob itself. The reason to watch this preview is
 * to judge whether the box is on the right object, which is impossible if the
 * marker hides it. */
static void annotate(uint8_t *img, unsigned W, unsigned H, const struct det *d)
{
    if (!d->valid) return;
    const int pad=10, thick=2, arm=55;
    const uint8_t v=255;
    int bx0=d->x0-pad, by0=d->y0-pad, bx1=d->x1+pad, by1=d->y1+pad;

    hline(img,W,H,bx0,bx1,by0,thick,v);
    hline(img,W,H,bx0,bx1,by1,thick,v);
    vline(img,W,H,by0,by1,bx0,thick,v);
    vline(img,W,H,by0,by1,bx1,thick,v);

    int cx=(int)(d->cx+0.5), cy=(int)(d->cy+0.5);
    hline(img,W,H,bx1+4,bx1+arm,cy,1,v);
    hline(img,W,H,bx0-arm,bx0-4,cy,1,v);
    vline(img,W,H,by1+4,by1+arm,cx,1,v);
    vline(img,W,H,by0-arm,by0-4,cx,1,v);
}

/* ===================== preview: one-slot handoff ========================= */

static int      g_prev_on = 1;
static uint8_t *g_prev_pending = NULL;
static int      g_prev_ready = 0;
static pthread_mutex_t g_prev_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_prev_cv  = PTHREAD_COND_INITIALIZER;

static unsigned long s_prev_offered=0, s_prev_overwritten=0, s_prev_shown=0;
static unsigned long s_skip_cap=0, s_skip_busy=0;
static double s_enc_sum=0, s_enc_max=0;
static size_t s_jpeg_last=0;

/* ---- JPEG -------------------------------------------------------------- */

/* Downscale by an integer factor with a box average, then compress. Averaging
 * rather than point-sampling matters on a thermal image: nearest-neighbour makes
 * sensor noise look like structure, which is misleading when the point is to
 * judge what the camera is seeing. */
static size_t encode_jpeg(const uint8_t *src, unsigned W, unsigned H,
                          unsigned down, int quality,
                          uint8_t **out, unsigned long *out_cap)
{
    unsigned ow=W/down, oh=H/down;
    static uint8_t *row=NULL;
    static unsigned row_cap=0;
    if (row_cap<ow) { free(row); row=malloc(ow); row_cap=ow; }
    if (!row) return 0;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err=jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo,out,out_cap);
    cinfo.image_width=ow; cinfo.image_height=oh;
    cinfo.input_components=1; cinfo.in_color_space=JCS_GRAYSCALE;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo,quality,TRUE);
    jpeg_start_compress(&cinfo,TRUE);
    while (cinfo.next_scanline<oh) {
        unsigned y0=cinfo.next_scanline*down;
        for (unsigned x=0;x<ow;x++) {
            unsigned sum=0;
            for (unsigned dy=0;dy<down;dy++) {
                const uint8_t *s=src+(size_t)(y0+dy)*W+x*down;
                for (unsigned dx=0;dx<down;dx++) sum+=s[dx];
            }
            row[x]=(uint8_t)(sum/(down*down));
        }
        JSAMPROW r=row;
        jpeg_write_scanlines(&cinfo,&r,1);
    }
    jpeg_finish_compress(&cinfo);
    size_t len=*out_cap;
    jpeg_destroy_compress(&cinfo);
    return len;
}

/* ---- HTTP -------------------------------------------------------------- */

struct client {
    int fd, streaming;
    uint8_t *buf; size_t len, sent;
};
static struct client g_cli[MAXCLIENT];
static pthread_mutex_t g_cli_mtx = PTHREAD_MUTEX_INITIALIZER;
static int g_listen = -1;

static const char PAGE[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
    "<!doctype html><title>blob_log preview</title>"
    "<style>body{margin:0;background:#111;color:#ccc;font:13px system-ui;"
    "display:flex;flex-direction:column;align-items:center;gap:8px;padding:12px}"
    "img{max-width:100%;image-rendering:pixelated;border:1px solid #333}"
    "b{color:#eee}</style>"
    "<img src=\"/stream\">"
    "<div><b>blob_log</b> &mdash; annotated live preview. Logging runs at full "
    "rate regardless of this page; preview frames are dropped, logged frames "
    "are not.</div>";

static const char STREAM_HDR[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
    "Cache-Control: no-store\r\nConnection: close\r\n\r\n";

static void client_close(struct client *c)
{
    if (c->fd>=0) close(c->fd);
    c->fd=-1; c->streaming=0; c->len=c->sent=0;
}

/* Never blocks: a partial write leaves the rest for next time, and a client
 * still mid-frame is skipped rather than waited on. */
static void client_pump(struct client *c)
{
    while (c->sent<c->len) {
        ssize_t w=send(c->fd,c->buf+c->sent,c->len-c->sent,MSG_DONTWAIT);
        if (w>0) { c->sent+=(size_t)w; continue; }
        if (w<0 && (errno==EAGAIN||errno==EWOULDBLOCK)) return;
        if (w<0 && errno==EINTR) continue;
        client_close(c);
        return;
    }
}

static void broadcast(const uint8_t *jpg, size_t jlen)
{
    char head[256];
    int hl=snprintf(head,sizeof head,
                    "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
                    "Content-Length: %zu\r\n\r\n",jlen);
    pthread_mutex_lock(&g_cli_mtx);
    for (int i=0;i<MAXCLIENT;i++) {
        struct client *c=&g_cli[i];
        if (c->fd<0 || !c->streaming) continue;
        client_pump(c);
        if (c->fd<0) continue;
        if (c->sent<c->len) { s_skip_busy++; continue; }
        if ((size_t)hl+jlen+2 > CLIBUF) continue;
        memcpy(c->buf,head,hl);
        memcpy(c->buf+hl,jpg,jlen);
        memcpy(c->buf+hl+jlen,"\r\n",2);
        c->len=hl+jlen+2; c->sent=0;
        client_pump(c);
    }
    pthread_mutex_unlock(&g_cli_mtx);
}

static void *net_thread(void *arg)
{
    (void)arg;
    while (!g_stop) {
        struct pollfd p[1+MAXCLIENT];
        int n=0;
        p[n].fd=g_listen; p[n].events=POLLIN; p[n].revents=0; n++;
        pthread_mutex_lock(&g_cli_mtx);
        int map[MAXCLIENT];
        for (int i=0;i<MAXCLIENT;i++) {
            map[i]=-1;
            if (g_cli[i].fd<0) continue;
            map[i]=n;
            p[n].fd=g_cli[i].fd;
            p[n].events=POLLIN|(g_cli[i].sent<g_cli[i].len?POLLOUT:0);
            p[n].revents=0; n++;
        }
        pthread_mutex_unlock(&g_cli_mtx);

        if (poll(p,n,200)<=0) continue;

        if (p[0].revents & POLLIN) {
            struct sockaddr_in sa; socklen_t sl=sizeof sa;
            int fd=accept(g_listen,(struct sockaddr*)&sa,&sl);
            if (fd>=0) {
                int one=1;
                setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof one);
                fcntl(fd,F_SETFL,O_NONBLOCK);
                pthread_mutex_lock(&g_cli_mtx);
                int slot=-1;
                for (int i=0;i<MAXCLIENT;i++) if (g_cli[i].fd<0) { slot=i; break; }
                if (slot<0) close(fd);
                else { g_cli[slot].fd=fd; g_cli[slot].streaming=0;
                       g_cli[slot].len=g_cli[slot].sent=0; }
                pthread_mutex_unlock(&g_cli_mtx);
            }
        }

        pthread_mutex_lock(&g_cli_mtx);
        for (int i=0;i<MAXCLIENT;i++) {
            struct client *c=&g_cli[i];
            if (c->fd<0 || map[i]<0) continue;
            short re=p[map[i]].revents;
            if (re & (POLLHUP|POLLERR)) { client_close(c); continue; }
            if ((re & POLLIN) && !c->streaming) {
                char req[1024];
                ssize_t r=recv(c->fd,req,sizeof req-1,MSG_DONTWAIT);
                if (r<=0) { if (r==0) client_close(c); continue; }
                req[r]='\0';
                if (strstr(req,"GET /stream")) {
                    memcpy(c->buf,STREAM_HDR,sizeof STREAM_HDR-1);
                    c->len=sizeof STREAM_HDR-1; c->sent=0; c->streaming=1;
                    client_pump(c);
                } else {
                    memcpy(c->buf,PAGE,sizeof PAGE-1);
                    c->len=sizeof PAGE-1; c->sent=0;
                    client_pump(c);
                    if (c->sent>=c->len) client_close(c);   /* one-shot page */
                }
            } else if (re & POLLOUT) {
                client_pump(c);
                if (c->fd>=0 && !c->streaming && c->sent>=c->len) client_close(c);
            }
        }
        pthread_mutex_unlock(&g_cli_mtx);
    }
    return NULL;
}

/* ---- encoder ----------------------------------------------------------- */

static unsigned g_W=DEF_W, g_H=DEF_H;
static int g_down=DEF_DOWN, g_qual=DEF_QUAL, g_enc_pin=-1;
/* 0 means uncapped. Not written as 1.0/DEF_FPS: the default is 0 and that is a
 * compile-time division by zero. */
static double g_min_dt = 0.0;

static void *enc_thread(void *arg)
{
    (void)arg;
    if (g_enc_pin>=0) pin_to(g_enc_pin);
    setpriority(PRIO_PROCESS,(id_t)syscall(SYS_gettid),10);
    uint8_t *mine=malloc((size_t)g_W*g_H);
    if (!mine) die("encoder: out of memory");
    uint8_t *jpg=NULL; unsigned long jcap=0;
    double last=0;

    while (!g_stop) {
        pthread_mutex_lock(&g_prev_mtx);
        while (!g_prev_ready && !g_stop) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME,&ts);
            ts.tv_nsec += 100*1000*1000;
            if (ts.tv_nsec>=1000000000) { ts.tv_sec++; ts.tv_nsec-=1000000000; }
            pthread_cond_timedwait(&g_prev_cv,&g_prev_mtx,&ts);
        }
        if (g_stop) { pthread_mutex_unlock(&g_prev_mtx); break; }
        uint8_t *t=g_prev_pending; g_prev_pending=mine; mine=t;   /* swap, no copy */
        g_prev_ready=0;
        pthread_mutex_unlock(&g_prev_mtx);

        /* Rate-cap the preview; logging keeps running at full speed. The 0.9
         * tolerance matters: capping at exactly the camera rate beats against
         * frame-interval jitter and throws away frames arriving a few hundred
         * microseconds early -- measured as 15.6 fps shown out of 25 before
         * this factor existed. */
        double now=now_mono();
        if (g_min_dt>0 && now-last<g_min_dt*0.9) { s_skip_cap++; continue; }
        last=now;

        double t0=now_mono();
        size_t jlen=encode_jpeg(mine,g_W,g_H,(unsigned)g_down,g_qual,&jpg,&jcap);
        double cost=(now_mono()-t0)*1000.0;
        s_enc_sum+=cost; if (cost>s_enc_max) s_enc_max=cost;
        if (!jlen) continue;
        s_jpeg_last=jlen;
        broadcast(jpg,jlen);
        s_prev_shown++;
    }
    free(mine);
    if (jpg) free(jpg);
    return NULL;
}

/* ===================== record queue ====================================== */

enum { DS_OK=0, DS_SKIPQ, DS_BAD };
static const char *DS_NAME[] = { "ok","skipped_queue","bad_frame" };

struct frec {
    uint32_t seq, flags, bytesused;
    double t_fb, t_arr;
    int bufidx;              /* >=0: detector owns it and must QBUF; -1: already back */
    int usable;
    int have_imu;
    double imu_t;
    float yaw, pitch;
    unsigned n_imu_int;
};

static struct frec q[RECQ];
static unsigned q_head=0, q_tail=0;
static unsigned q_inflight=0;
static unsigned long q_dropped=0;
static pthread_mutex_t q_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  q_cv  = PTHREAD_COND_INITIALIZER;
static int q_done=0;

/* ===================== detector thread =================================== */

static int g_vfd=-1;
static void **g_bufs=NULL;
static size_t *g_blen=NULL;
static unsigned g_nbuf_granted=0;
static uint32_t g_frame_sz=0;
static FILE *g_csv=NULL;
static int g_det_pin=-1, g_det_nice=5;
static unsigned g_step=DEF_STEP;
static int g_fixed_thr=0;
static double g_ratio=DEF_RATIO;
static long g_minarea=DEF_MINAREA;

/* detector stats */
static unsigned long w_rows=0, w_det=0, w_found=0, w_skipq=0, w_bad=0, w_amb=0;
static double w_sum_cost=0, w_max_cost=0, w_sum_copy=0, w_max_copy=0;
static double w_sum_e2e=0, w_max_e2e=0, w_sum_q=0, w_max_q=0;
static double *w_e2e=NULL; static unsigned w_ne2e=0;
static double w_last_cx=0, w_last_cy=0;
static int    w_last_valid=0;

/* One row, written field by field rather than as one long format string. The
 * first version of this drifted by a column in two of the three branches
 * because the commas had to be counted by eye. */
static void emit_row(const struct frec *r, const struct det *d, int state,
                     double cost_ms, double queue_ms, double t_done)
{
    fprintf(g_csv,"%u,%.6f,%.6f,",r->seq,r->t_fb,r->t_arr);
    if (r->usable) fprintf(g_csv,"%.3f,",(r->t_arr-r->t_fb)*1000.0);
    else fputc(',',g_csv);

    if (r->have_imu)
        fprintf(g_csv,"%.4f,%.4f,%.6f,%.3f,%u,",
                (double)r->yaw,(double)r->pitch,r->imu_t,
                (r->t_fb-r->imu_t)*1000.0,r->n_imu_int);
    else
        fprintf(g_csv,",,,,%u,",r->n_imu_int);

    /* valid,cx,cy,area_px,blob_peak,frame_peak,thr,ncomp,second_area,
     * x0,y0,x1,y1,queue_ms,det_ms,t_detect_done,fb_to_det_ms
     * -- 17 fields, every branch. */
    if (state!=DS_OK) {
        fprintf(g_csv,",,,,,,,,,,,,,,,,,");               /* 17 empty */
    } else {
        if (d->valid) fprintf(g_csv,"1,%.3f,%.3f,%ld,%d,",d->cx,d->cy,d->area,d->peak);
        else          fprintf(g_csv,"0,,,,,");
        fprintf(g_csv,"%d,%d,%d,%ld,",d->fpeak,d->thr,d->ncomp,d->second);
        if (d->valid) fprintf(g_csv,"%d,%d,%d,%d,",d->x0,d->y0,d->x1,d->y1);
        else          fprintf(g_csv,",,,,");
        fprintf(g_csv,"%.3f,%.3f,%.6f,%.3f,",
                queue_ms,cost_ms,t_done,(t_done-r->t_fb)*1000.0);
    }
    fprintf(g_csv,"0x%08x,%s\n",r->flags,DS_NAME[state]);
}

static void *det_thread(void *arg)
{
    (void)arg;
    if (g_det_pin>=0) pin_to(g_det_pin);
    /* Nice the detector down so it cannot preempt acquisition or the IMU reader.
     * Raising the nice value needs no privilege; lowering it would. */
    setpriority(PRIO_PROCESS,(id_t)syscall(SYS_gettid),g_det_nice);

    w_e2e=malloc(sizeof(double)*LATN);
    unsigned vw=g_W/g_step, vh=g_H/g_step;
    uint8_t *work=malloc((size_t)g_W*g_H);
    uint8_t *seen=malloc((size_t)vw*vh);
    int32_t *stack=malloc(sizeof(int32_t)*(size_t)vw*vh);
    if (!work||!seen||!stack||!w_e2e) die("detector: out of memory");

    for (;;) {
        struct frec r;
        pthread_mutex_lock(&q_mtx);
        while (q_tail==q_head && !q_done) pthread_cond_wait(&q_cv,&q_mtx);
        if (q_tail==q_head && q_done) { pthread_mutex_unlock(&q_mtx); break; }
        r=q[q_tail % RECQ];
        q_tail++;
        pthread_mutex_unlock(&q_mtx);

        int state = DS_OK;
        struct det d; memset(&d,0,sizeof d);
        double cost=0, queue_ms=0, t_done=0;
        /* How long the frame sat between becoming available and this thread
         * picking it up. Charged to end-to-end latency just like compute is. */
        double t_pick = now_mono();

        if (r.bufidx>=0) {
            queue_ms = (t_pick - r.t_arr)*1000.0;
            /* Copy out and hand the buffer straight back. Detection then runs
             * against cached memory and the driver is never waiting on us. */
            double c0=now_mono();
            memcpy(work,g_bufs[r.bufidx],(size_t)g_W*g_H);
            double copy_ms=(now_mono()-c0)*1000.0;
            w_sum_copy+=copy_ms; if (copy_ms>w_max_copy) w_max_copy=copy_ms;

            struct v4l2_buffer b;
            memset(&b,0,sizeof b);
            b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
            b.index=(unsigned)r.bufidx;
            if (xioctl(g_vfd,VIDIOC_QBUF,&b)==-1)
                die("detector VIDIOC_QBUF %d: %s",r.bufidx,strerror(errno));
            pthread_mutex_lock(&q_mtx);
            if (q_inflight) q_inflight--;
            pthread_mutex_unlock(&q_mtx);

            double t0=now_mono();
            d=detect(work,g_W,g_H,g_step,g_fixed_thr,g_ratio,g_minarea,seen,stack);
            /* The answer exists HERE. Everything after this point -- the CSV
             * row, the annotation, the preview -- is bookkeeping, and charging
             * it to detection latency would overstate the number. */
            t_done=now_mono();
            cost=(t_done-t0)*1000.0 + copy_ms;
            w_sum_e2e += (t_done-r.t_fb)*1000.0;
            if ((t_done-r.t_fb)*1000.0 > w_max_e2e) w_max_e2e=(t_done-r.t_fb)*1000.0;
            w_sum_q += queue_ms; if (queue_ms>w_max_q) w_max_q=queue_ms;
            if (w_ne2e < LATN) w_e2e[w_ne2e++]=(t_done-r.t_fb)*1000.0;
            w_sum_cost+=cost; if (cost>w_max_cost) w_max_cost=cost;
            w_det++;
            if (d.valid) { w_found++; w_last_cx=d.cx; w_last_cy=d.cy; w_last_valid=1; }
            else w_last_valid=0;
            /* ncomp>1 is normal in any real scene -- a second warm object is
             * not ambiguity. It is only ambiguous when the runner-up is big
             * enough that the pick could plausibly flip between frames. */
            if (d.valid && d.second*2 >= d.area) w_amb++;

            /* Preview last, and only after the CSV row is safe to write.
             * Annotating in place is fine: `work` is overwritten next frame. */
            if (g_prev_on) {
                annotate(work,g_W,g_H,&d);
                pthread_mutex_lock(&g_prev_mtx);
                uint8_t *t=g_prev_pending; g_prev_pending=work; work=t;
                if (g_prev_ready) s_prev_overwritten++;   /* encoder missed one */
                g_prev_ready=1;
                s_prev_offered++;
                pthread_cond_signal(&g_prev_cv);
                pthread_mutex_unlock(&g_prev_mtx);
            }
        } else {
            state = r.usable ? DS_SKIPQ : DS_BAD;
            if (state==DS_SKIPQ) w_skipq++; else w_bad++;
        }

        emit_row(&r,&d,state,cost,queue_ms,t_done);
        w_rows++;
        if (w_rows % BLOB_FLUSH == 0) fflush(g_csv);
    }
    free(work); free(seen); free(stack);
    return NULL;
}

/* ===================== video ============================================= */

static int open_video(const char *dev, unsigned W, unsigned H, unsigned nbuf,
                      uint32_t *frame_sz, uint32_t *stride, char fcs[5],
                      struct v4l2_capability *cap)
{
    int fd=open(dev,O_RDWR|O_CLOEXEC);
    if (fd<0) die("cannot open %s: %s\n  in the video group? (id -nG)",dev,strerror(errno));
    memset(cap,0,sizeof *cap);
    if (xioctl(fd,VIDIOC_QUERYCAP,cap)==-1)
        die("VIDIOC_QUERYCAP on %s: %s",dev,strerror(errno));
    if (!(cap->capabilities & V4L2_CAP_VIDEO_CAPTURE))
        die("%s cannot capture video (caps 0x%08x).\n"
            "  The core's second node is metadata-only -- use -video-index0.",
            dev,cap->capabilities);
    if (!(cap->capabilities & V4L2_CAP_STREAMING))
        die("%s does not support streaming I/O",dev);

    struct v4l2_format f;
    memset(&f,0,sizeof f);
    f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width=W; f.fmt.pix.height=H;
    f.fmt.pix.pixelformat=V4L2_PIX_FMT_GREY;
    f.fmt.pix.field=V4L2_FIELD_NONE;
    if (xioctl(fd,VIDIOC_S_FMT,&f)==-1) die("VIDIOC_S_FMT: %s",strerror(errno));
    /* S_FMT is a negotiation, not a command. Detection indexes the buffer as
     * W*H bytes of GREY, so a substituted format would be measured wrong. */
    if (f.fmt.pix.width!=W || f.fmt.pix.height!=H ||
        f.fmt.pix.pixelformat!=V4L2_PIX_FMT_GREY) {
        char got[5]; fourcc_str(f.fmt.pix.pixelformat,got);
        die("driver refused the format.\n  asked %ux%u GREY, got %ux%u %s",
            W,H,f.fmt.pix.width,f.fmt.pix.height,got);
    }
    *frame_sz=f.fmt.pix.sizeimage;
    *stride=f.fmt.pix.bytesperline;
    fourcc_str(f.fmt.pix.pixelformat,fcs);
    if (*stride != W)
        die("driver uses %u B/line for a %u px wide GREY frame; this program\n"
            "  assumes a packed buffer",*stride,W);

    struct v4l2_requestbuffers rb;
    memset(&rb,0,sizeof rb);
    rb.count=nbuf; rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory=V4L2_MEMORY_MMAP;
    if (xioctl(fd,VIDIOC_REQBUFS,&rb)==-1)
        die("VIDIOC_REQBUFS (%u): %s",nbuf,strerror(errno));
    g_nbuf_granted=rb.count;
    if (rb.count < QDEPTH+4)
        die("driver gave only %u buffers; need at least %d so acquisition always\n"
            "  has spares while the detector holds up to %d",
            rb.count,QDEPTH+4,QDEPTH);
    g_bufs=calloc(rb.count,sizeof *g_bufs);
    g_blen=calloc(rb.count,sizeof *g_blen);
    if (!g_bufs||!g_blen) die("out of memory");
    for (unsigned i=0;i<rb.count;i++) {
        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP; b.index=i;
        if (xioctl(fd,VIDIOC_QUERYBUF,&b)==-1) die("VIDIOC_QUERYBUF %u: %s",i,strerror(errno));
        g_blen[i]=b.length;
        g_bufs[i]=mmap(NULL,b.length,PROT_READ,MAP_SHARED,fd,b.m.offset);
        if (g_bufs[i]==MAP_FAILED) die("mmap buffer %u: %s",i,strerror(errno));
        if (xioctl(fd,VIDIOC_QBUF,&b)==-1) die("VIDIOC_QBUF %u: %s",i,strerror(errno));
    }
    enum v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd,VIDIOC_STREAMON,&t)==-1)
        die("VIDIOC_STREAMON: %s\n  another process may hold %s"
            " (the core allows one streamer)",strerror(errno),dev);
    return fd;
}

static int open_listen(int port)
{
    int fd=socket(AF_INET,SOCK_STREAM,0);
    if (fd<0) die("socket: %s",strerror(errno));
    int one=1;
    setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
    struct sockaddr_in sa;
    memset(&sa,0,sizeof sa);
    sa.sin_family=AF_INET; sa.sin_addr.s_addr=htonl(INADDR_ANY);
    sa.sin_port=htons((uint16_t)port);
    if (bind(fd,(struct sockaddr*)&sa,sizeof sa)<0)
        die("bind port %d: %s\n  another viewer still running?",port,strerror(errno));
    if (listen(fd,8)<0) die("listen: %s",strerror(errno));
    fcntl(fd,F_SETFL,O_NONBLOCK);
    return fd;
}

static void usage(const char *me)
{
    printf(
"Log gimbal IMU at ~1 kHz and blob detections at camera rate, with an annotated\n"
"live preview. Logging has strict priority; only the preview drops frames.\n"
"\n"
"usage: %s [-t SECS] [-n FRAMES] [-D ROOT] [-O DIR] [-W W] [-H H] [-b BUFS]\n"
"          [-v VIDEODEV] [-g IMUDEV] [-B BAUD] [-C SECS] [-I SECS]\n"
"          [-S STEP] [-a MINAREA] [-T THR] [-r RATIO]\n"
"          [-p PORT] [-d DOWN] [-Q QUAL] [-f FPS] [-P] [-q]\n"
"\n"
"  stop condition\n"
"    -t SECS    stop after SECS of logging      (default: until Ctrl-C)\n"
"    -n FRAMES  stop after FRAMES frames\n"
"\n"
"  output\n"
"    -D ROOT    run-folder root                 (default %s/)\n"
"    -O DIR     use this folder exactly instead of a timestamped one\n"
"\n"
"  capture\n"
"    -W, -H     geometry                        (default %dx%d)\n"
"    -b BUFS    v4l2 buffers, %d..%d            (default %d)\n"
"    -v, -g     video / IMU device overrides\n"
"    -B BAUD    IMU baud, 115200 or 921600      (default 921600)\n"
"    -C SECS    camera settle before logging    (default 2)\n"
"    -I SECS    IMU settle before logging       (default 2)\n"
"\n"
"  detection\n"
"    -S STEP    detect on every STEP'th pixel, 1..8   (default %d)\n"
"               STEP 2 is nearly free on this core: the sensor is 640x512 and\n"
"               the 1280x1024 stream is a 2x upscale, so it drops interpolated\n"
"               pixels, not information. Coordinates stay in full-frame px.\n"
"    -a MINAREA smallest blob to accept, full-frame px (default %d)\n"
"    -T THR     fixed threshold 1..254 instead of adaptive\n"
"    -r RATIO   adaptive threshold = RATIO x frame peak (default %.2f,\n"
"               floored at %d)\n"
"\n"
"  preview\n"
"    -p PORT    HTTP port, 0 disables preview entirely (default %d)\n"
"    -d DOWN    preview downscale factor              (default %d)\n"
"    -Q QUAL    JPEG quality 1..100                   (default %d)\n"
"    -f FPS     cap preview rate, 0 = uncapped        (default 0)\n"
"\n"
"  -P           do not pin threads to CPUs\n"
"  -q           no live status line\n",
    me, REC_ROOT, DEF_W, DEF_H, QDEPTH+4, MAX_NBUF, DEF_NBUF,
    DEF_STEP, DEF_MINAREA, DEF_RATIO, THR_FLOOR, DEF_PORT, DEF_DOWN, DEF_QUAL);
}

int main(int argc, char **argv)
{
    const char *vdev_opt=NULL,*gdev_opt=NULL,*root=REC_ROOT,*dir_opt=NULL;
    unsigned W=DEF_W,H=DEF_H,nbuf=DEF_NBUF;
    int quiet=0, do_pin=1, port=DEF_PORT, c;
    double want_secs=0, cam_settle=2.0, imu_settle=2.0, cap_fps=0;
    unsigned long want_frames=0;
    long baud_n=921600; speed_t baud=B921600;

    while ((c=getopt(argc,argv,"t:n:D:O:W:H:b:v:g:B:C:I:S:a:T:r:p:d:Q:f:Pqh"))!=-1) {
        switch (c) {
        case 't': want_secs=atof(optarg); break;
        case 'n': want_frames=strtoul(optarg,NULL,10); break;
        case 'D': root=optarg; break;
        case 'O': dir_opt=optarg; break;
        case 'W': W=(unsigned)strtoul(optarg,NULL,10); break;
        case 'H': H=(unsigned)strtoul(optarg,NULL,10); break;
        case 'b': nbuf=(unsigned)strtoul(optarg,NULL,10); break;
        case 'v': vdev_opt=optarg; break;
        case 'g': gdev_opt=optarg; break;
        case 'C': cam_settle=atof(optarg); break;
        case 'I': imu_settle=atof(optarg); break;
        case 'S': g_step=(unsigned)strtoul(optarg,NULL,10); break;
        case 'a': g_minarea=strtol(optarg,NULL,10); break;
        case 'T': g_fixed_thr=atoi(optarg); break;
        case 'r': g_ratio=atof(optarg); break;
        case 'p': port=atoi(optarg); break;
        case 'd': g_down=atoi(optarg); break;
        case 'Q': g_qual=atoi(optarg); break;
        case 'f': cap_fps=atof(optarg); break;
        case 'P': do_pin=0; break;
        case 'q': quiet=1; break;
        case 'B':
            baud_n=strtol(optarg,NULL,10);
            if (baud_n==115200) baud=B115200;
            else if (baud_n==921600) baud=B921600;
            else die("-B must be 115200 or 921600");
            break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 2;
        }
    }

    if (nbuf<QDEPTH+4 || nbuf>MAX_NBUF)
        die("-b must be %d..%d (the detector may hold %d, the driver needs spares)",
            QDEPTH+4,MAX_NBUF,QDEPTH);
    if (W<8||H<8||W>8192||H>8192) die("geometry %ux%u out of range",W,H);
    if (g_step<1||g_step>8) die("-S must be 1..8");
    if (W/g_step<4 || H/g_step<4) die("-S %u leaves too small a grid for %ux%u",g_step,W,H);
    if (g_fixed_thr && (g_fixed_thr<1||g_fixed_thr>254)) die("-T must be 1..254");
    if (g_ratio<=0||g_ratio>=1) die("-r must be between 0 and 1");
    if (g_minarea<1) die("-a must be >= 1");
    if (g_down<1||g_down>8) die("-d must be 1..8");
    if (W%(unsigned)g_down || H%(unsigned)g_down)
        die("-d %d must divide the geometry %ux%u exactly",g_down,W,H);
    if (g_qual<1||g_qual>100) die("-Q must be 1..100");
    if (cap_fps<0) die("-f must be >= 0");
    if (port<0||port>65535) die("-p must be 0..65535");
    g_W=W; g_H=H;
    g_prev_on = port>0;
    g_min_dt = cap_fps>0 ? 1.0/cap_fps : 0.0;

    char *vdev=resolve(vdev_opt,THERMAL_LINK,VID_BYID_GLOB,NULL);
    if (!vdev) die("no Sirius capture node found (looked for %s, then %s)\n"
                   "  is the camera plugged in?  ls -l /dev/v4l/by-id/",
                   THERMAL_LINK,VID_BYID_GLOB);
    char *gdev=resolve(gdev_opt,TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);
    if (!gdev) die("no IMU/gimbal port found (looked for %s, then %s, then %s)\n"
                   "  is the USB-TTL converter plugged in?  ls -l /dev/serial/by-id/",
                   TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);

    /* ---- run folder ---- */
    char outdir[512], csvpath[640], imupath[640], sumpath[640];
    if (dir_opt) {
        snprintf(outdir,sizeof outdir,"%s",dir_opt);
    } else {
        time_t tt=time(NULL); struct tm tm; localtime_r(&tt,&tm);
        unsigned dup=0;
        for (;;) {
            int len;
            if (dup==0)
                len=snprintf(outdir,sizeof outdir,"%s/%04d-%02d-%02d/%02d%02d%02d",
                    root,tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,
                    tm.tm_hour,tm.tm_min,tm.tm_sec);
            else
                len=snprintf(outdir,sizeof outdir,"%s/%04d-%02d-%02d/%02d%02d%02d-%u",
                    root,tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,
                    tm.tm_hour,tm.tm_min,tm.tm_sec,dup);
            if (len<0||(size_t)len>=sizeof outdir) die("-D path too long");
            if (access(outdir,F_OK)!=0) break;
            if (++dup>999) die("cannot find an unused folder under %s",root);
        }
    }
    if (mkdir_p(outdir)==-1) die("cannot create folder %s: %s",outdir,strerror(errno));
    snprintf(csvpath,sizeof csvpath,"%s/blobs.csv",outdir);
    snprintf(imupath,sizeof imupath,"%s/imu.csv",outdir);
    snprintf(sumpath,sizeof sumpath,"%s/summary.txt",outdir);

    struct sigaction sa;
    memset(&sa,0,sizeof sa);
    sa.sa_handler=on_signal;
    sigaction(SIGINT,&sa,NULL); sigaction(SIGTERM,&sa,NULL);
    signal(SIGPIPE,SIG_IGN);          /* a viewer closing must not kill logging */

    if (do_pin) { g_tlm_pin=2; g_det_pin=3; g_enc_pin=4; }

    /* =================== PHASE 1: camera up and settled ================== */
    uint32_t stride=0; char fcs[5]; struct v4l2_capability cap;
    if (!quiet) fprintf(stderr,"phase 1     starting camera %s\n",vdev);
    g_vfd=open_video(vdev,W,H,nbuf,&g_frame_sz,&stride,fcs,&cap);
    unsigned nbuf_granted = g_nbuf_granted ? g_nbuf_granted : nbuf;
    if (!quiet)
        fprintf(stderr,"            %ux%u %s  %u B/frame  %u B/line  %u buffers\n",
                W,H,fcs,g_frame_sz,stride,nbuf_granted);
    if (do_pin && !pin_to(1) && !quiet)
        fprintf(stderr,"note        could not pin the acquisition thread\n");

    double p1_t0=now_mono();
    unsigned long p1_seen=0,p1_bad=0;
    int good_run=0, clock_checked=0;
    double p1_first_good=0;
    while (!g_stop) {
        struct pollfd pfd={.fd=g_vfd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,2000);
        if (pr<0) { if (errno==EINTR) continue; die("poll: %s",strerror(errno)); }
        if (pr==0) die("camera delivered no frame for 2 s during startup");
        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if (xioctl(g_vfd,VIDIOC_DQBUF,&b)==-1) { if (errno==EAGAIN) continue;
            die("VIDIOC_DQBUF: %s",strerror(errno)); }
        uint32_t fl=b.flags, by=b.bytesused;
        double tfb=b.timestamp.tv_sec+b.timestamp.tv_usec/1e6;
        if (!clock_checked) {
            clock_checked=1;
            if ((fl & V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                die("video timestamps are not CLOCK_MONOTONIC (flags 0x%08x) --\n"
                    "  every time column would be incomparable with the IMU",fl);
        }
        xioctl(g_vfd,VIDIOC_QBUF,&b);
        p1_seen++;
        int ok = !(fl & V4L2_BUF_FLAG_ERROR) && by>=(unsigned)W*H && tfb>0;
        if (ok) { if (!good_run) p1_first_good=now_mono(); good_run++; }
        else { p1_bad++; good_run=0; }
        if (good_run>=10 && now_mono()-p1_first_good>=cam_settle) break;
        if (now_mono()-p1_t0 > cam_settle+20.0)
            die("camera did not settle in %.0f s: %lu frames seen, %lu bad",
                cam_settle+20.0,p1_seen,p1_bad);
    }
    if (!quiet)
        fprintf(stderr,"            settled after %.2f s (%lu frames, %lu discarded"
                       " as bad)\n",now_mono()-p1_t0,p1_seen,p1_bad);

    /* =================== PHASE 2: IMU up and settled ===================== */
    if (!quiet) fprintf(stderr,"phase 2     starting IMU %s @ %ld\n",gdev,baud_n);
    g_tlm_fd=open_serial(gdev,baud);
    if (g_tlm_fd<0) die("IMU %s: %s\n  in the dialout group? (id -nG)",gdev,strerror(errno));
    g_imu_csv=fopen(imupath,"w");
    if (!g_imu_csv) die("cannot create %s: %s",imupath,strerror(errno));
    /* A big buffer keeps 1 kHz of rows from becoming 1 kHz of write() calls on
     * the thread that must not stall: the serial port is the one input with no
     * flow control behind it. */
    static char imubuf[1u<<20];
    setvbuf(g_imu_csv,imubuf,_IOFBF,sizeof imubuf);
    fprintf(g_imu_csv,"# blob_log: full-rate gimbal IMU, receive only\n");
    fprintf(g_imu_csv,"# port=%s baud=%ld  frame=A5 5A + 8 float32 LE + CRC16\n",gdev,baud_n);
    fprintf(g_imu_csv,"# t_imu = CLOCK_MONOTONIC at arrival, the same clock as blobs.csv\n");
    fprintf(g_imu_csv,"t_imu,yaw,pitch,f2,f3,f4,f5,f6,f7\n");

    pthread_t tid_tlm;
    if (pthread_create(&tid_tlm,NULL,tlm_thread,NULL)!=0)
        die("cannot start IMU thread: %s",strerror(errno));

    /* The video stream is already running, so this loop MUST keep draining it
     * while waiting for the IMU. Sleeping here instead lets every buffer fill:
     * the driver then has nowhere to put new frames, and the queued ones carry
     * timestamps seconds old, so the first logged latencies read ~2000 ms.
     * Discard-as-you-go keeps the pipeline shallow. */
    double p2_t0=now_mono();
    unsigned long imu_at_start=0,p2_discarded=0,batched_at_start=0;
    for (;;) {
        if (g_stop) break;
        struct pollfd pfd={.fd=g_vfd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,50);
        if (pr>0 && (pfd.revents&POLLIN)) {
            struct v4l2_buffer b;
            memset(&b,0,sizeof b);
            b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
            if (xioctl(g_vfd,VIDIOC_DQBUF,&b)==0) { xioctl(g_vfd,VIDIOC_QBUF,&b);
                                                    p2_discarded++; }
        }
        double el=now_mono()-p2_t0;
        pthread_mutex_lock(&ring_mtx);
        unsigned long n=g_nimu;
        pthread_mutex_unlock(&ring_mtx);
        double hz = el>0? n/el : 0;
        if (el>=imu_settle && n>=500 && hz>200.0) { imu_at_start=n;
                                                   batched_at_start=g_imu_batched;
                                                   break; }
        if (el > imu_settle+15.0) {
            enum v4l2_buf_type off=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(g_vfd,VIDIOC_STREAMOFF,&off);
            die("IMU did not settle in %.0f s: %lu samples (%.0f Hz), %lu crc errors.\n"
                "  Run ./check_gimbal.py to diagnose the link.",
                imu_settle+15.0,n,hz,g_ncrc);
        }
    }
    if (!quiet)
        fprintf(stderr,"            settled after %.2f s (%lu samples, %.0f Hz,"
                       " %lu crc errors; %lu frames discarded meanwhile)\n",
                now_mono()-p2_t0,imu_at_start,imu_at_start/(now_mono()-p2_t0),
                g_ncrc,p2_discarded);

    /* =================== PHASE 3: preview up ============================= */
    pthread_t tid_net,tid_enc;
    int have_prev=0;
    if (g_prev_on) {
        g_prev_pending=malloc((size_t)W*H);
        if (!g_prev_pending) die("out of memory for the preview slot");
        memset(g_prev_pending,0,(size_t)W*H);
        for (int i=0;i<MAXCLIENT;i++) {
            g_cli[i].fd=-1;
            g_cli[i].buf=malloc(CLIBUF);
            if (!g_cli[i].buf) die("out of memory for client buffers");
        }
        g_listen=open_listen(port);
        if (pthread_create(&tid_net,NULL,net_thread,NULL)!=0)
            die("cannot start net thread: %s",strerror(errno));
        if (pthread_create(&tid_enc,NULL,enc_thread,NULL)!=0)
            die("cannot start encoder thread: %s",strerror(errno));
        have_prev=1;
        if (!quiet) {
            char host[128]="";
            gethostname(host,sizeof host-1);
            fprintf(stderr,"phase 3     preview on http://%s:%d/  (%ux%u q%d)\n",
                    host[0]?host:"<jetson>",port,W/(unsigned)g_down,H/(unsigned)g_down,g_qual);
        }
    } else if (!quiet) {
        fprintf(stderr,"phase 3     preview disabled (-p 0)\n");
    }

    /* =================== PHASE 4: log ==================================== */
    g_csv=fopen(csvpath,"w");
    if (!g_csv) die("cannot create %s: %s",csvpath,strerror(errno));
    fprintf(g_csv,"# blob_log: one row per acquired frame, IMU paired, blob detected\n");
    fprintf(g_csv,"# video=%s imu=%s baud=%ld\n",vdev,gdev,baud_n);
    fprintf(g_csv,"# geometry=%ux%u %s  buffers=%u  detect_step=%u\n",
            W,H,fcs,nbuf_granted,g_step);
    if (g_fixed_thr) fprintf(g_csv,"# threshold=fixed %d\n",g_fixed_thr);
    else fprintf(g_csv,"# threshold=adaptive %.2f x frame peak, floor %d\n",g_ratio,THR_FLOOR);
    fprintf(g_csv,"# min_area=%ld full-frame px\n",g_minarea);
    fprintf(g_csv,"# clock=CLOCK_MONOTONIC for every t_ column\n");
    fprintf(g_csv,"# t_first_byte = kernel stamp, first USB payload of the frame on the host\n");
    fprintf(g_csv,"# t_available  = VIDIOC_DQBUF returned it; usable by any program\n");
    fprintf(g_csv,"# t_detect_done = cx,cy existed; the detection OUTPUT time\n");
    fprintf(g_csv,"# fb_to_det_ms = t_detect_done - t_first_byte, the host-side chain.\n");
    fprintf(g_csv,"#   This does NOT include the camera-internal part (real-world motion\n");
    fprintf(g_csv,"#   -> first byte on the host): integration plus the core's own\n");
    fprintf(g_csv,"#   pipeline. Measuring that needs an external reference -- the IMU.\n");
    fprintf(g_csv,"# cx,cy = intensity-weighted centroid in FULL-FRAME px, whatever -S is\n");
    fprintf(g_csv,"# cx,cy are positions in the IMAGE: blob motion plus camera motion.\n");
    fprintf(g_csv,"#   subtract the gimbal's own motion using yaw/pitch to get world motion\n");
    fprintf(g_csv,"# det_state: ok | skipped_queue (detector busy) | bad_frame\n");
    fprintf(g_csv,"seq,t_first_byte,t_available,latency_ms,"
                  "yaw,pitch,t_imu,imu_dt_ms,n_imu_interval,"
                  "valid,cx,cy,area_px,blob_peak,frame_peak,thr,ncomp,second_area,"
                  "x0,y0,x1,y1,queue_ms,det_ms,t_detect_done,fb_to_det_ms,"
                  "flags,det_state\n");

    pthread_t tid_det;
    if (pthread_create(&tid_det,NULL,det_thread,NULL)!=0)
        die("cannot start detector thread: %s",strerror(errno));

    if (!quiet) {
        fprintf(stderr,"phase 4     logging -> %s\n",outdir);
        fprintf(stderr,"            detector on its own thread (pin %d, nice +%d),"
                       " lease %d of %u buffers\n",
                g_det_pin,g_det_nice,QDEPTH,nbuf_granted);
        fprintf(stderr,"            Ctrl-C to stop\n");
    }

    const double t0=now_mono();
    unsigned long frames=0,dropped=0,seq_anom=0,timeouts=0,nomatch=0,n_usable=0;
    /* Start the interval count from where logging begins, not from program
     * start: otherwise the first row claims every settle-phase sample. */
    unsigned prev_ring; uint32_t last_seq=0; int have_seq=0;
    pthread_mutex_lock(&ring_mtx); prev_ring=ring_n; pthread_mutex_unlock(&ring_mtx);
    double prev_fb=0,gap_min=1e9,gap_max=0,t_status=0;
    double sum_lat=0,min_lat=1e9,max_lat=0,sum_idt=0,worst_idt=0;
    double *lat=malloc(sizeof(double)*LATN); unsigned nlat=0;
    /* Is acquisition itself prompt? poll() blocking for most of the frame
     * interval means this thread was already parked waiting when the frame
     * landed. poll() returning instantly means the frame was ALREADY sitting in
     * the driver when we got round to asking -- i.e. we were late, and part of
     * the measured latency is us, not the camera. */
    double a_sum_pw=0, a_max_pw=0, a_min_pw=1e9;
    double a_sum_work=0, a_max_work=0;
    unsigned long a_ready_already=0, a_backlog=0;
    double *pw=malloc(sizeof(double)*LATN); unsigned npw=0;
    if (!pw) die("out of memory");
    if (!lat) die("out of memory");
    int isatty_err=isatty(STDERR_FILENO);
    const char *why="signal";

    while (!g_stop) {
        if (want_frames && frames>=want_frames) { why="frame count"; break; }
        if (want_secs>0 && now_mono()-t0>=want_secs) { why="duration"; break; }

        struct pollfd pfd={.fd=g_vfd,.events=POLLIN,.revents=0};
        double t_poll0=now_mono();
        int pr=poll(&pfd,1,2000);
        double t_poll1=now_mono();
        if (pr<0) { if (errno==EINTR) continue; die("poll: %s",strerror(errno)); }
        if (pr==0) {
            timeouts++;
            fprintf(stderr,"\nwarning: no frame for 2 s (timeout %lu)\n",timeouts);
            if (timeouts>=3) { why="camera stopped delivering frames"; break; }
            continue;
        }
        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if (xioctl(g_vfd,VIDIOC_DQBUF,&b)==-1) { if (errno==EAGAIN) continue;
            die("VIDIOC_DQBUF: %s",strerror(errno)); }

        /* First thing, before any work: the moment the frame became usable.
         * Anything done before reading it would be charged to latency. */
        double t_arr=now_mono();
        /* QBUF overwrites this struct and the buffer may go to the detector, so
         * snapshot every field needed later right now. */
        struct frec r;
        memset(&r,0,sizeof r);
        r.seq=b.sequence; r.flags=b.flags; r.bytesused=b.bytesused;
        r.t_fb=b.timestamp.tv_sec+b.timestamp.tv_usec/1e6;
        r.t_arr=t_arr;
        unsigned bidx=b.index;
        timeouts=0;
        frames++;

        double pw_ms=(t_poll1-t_poll0)*1000.0;
        a_sum_pw+=pw_ms;
        if (pw_ms>a_max_pw) a_max_pw=pw_ms;
        if (pw_ms<a_min_pw) a_min_pw=pw_ms;
        if (npw<LATN) pw[npw++]=pw_ms;
        if (pw_ms<1.0) a_ready_already++;
        /* One non-blocking poll: if another complete frame is ALREADY ready the
         * instant we took this one, a backlog exists and we are behind. Costs a
         * single syscall on a 40 ms budget. */
        {
            struct pollfd pb={.fd=g_vfd,.events=POLLIN,.revents=0};
            if (poll(&pb,1,0)>0 && (pb.revents&POLLIN)) a_backlog++;
        }

        if (have_seq) {
            /* Guard the arithmetic: sequences can repeat when a stream restarts,
             * and unsigned subtraction there wraps to ~4.29e9. */
            if (r.seq>last_seq+1) dropped += r.seq-last_seq-1;
            else if (r.seq<=last_seq) seq_anom++;
        }
        last_seq=r.seq; have_seq=1;

        r.usable = !(r.flags & V4L2_BUF_FLAG_ERROR)
                   && r.bytesused>=(unsigned)W*H && r.t_fb>0;
        if (r.usable) {
            n_usable++;
            double l=(t_arr-r.t_fb)*1000.0;
            sum_lat+=l; if (l>max_lat) max_lat=l; if (l<min_lat) min_lat=l;
            if (nlat<LATN) lat[nlat++]=l;
            if (prev_fb>0) {
                double g=(r.t_fb-prev_fb)*1000.0;
                if (g<gap_min) gap_min=g;
                if (g>gap_max) gap_max=g;
            }
            prev_fb=r.t_fb;
        }

        struct imu a; unsigned rt=0;
        r.have_imu = r.usable ? ring_nearest(r.t_fb,&a,&rt) : 0;
        if (!r.have_imu) { pthread_mutex_lock(&ring_mtx); rt=ring_n;
                           pthread_mutex_unlock(&ring_mtx);
                           if (r.usable) nomatch++; }
        else { r.imu_t=a.t; r.yaw=a.yaw; r.pitch=a.pitch;
               double d=(r.t_fb-a.t)*1000.0, ad=d<0?-d:d;
               sum_idt+=ad; if (ad>worst_idt) worst_idt=ad; }
        r.n_imu_int = rt-prev_ring;
        prev_ring = rt;

        /* ---- hand off, or keep priority and requeue ---- */
        int handed=0;
        pthread_mutex_lock(&q_mtx);
        int qroom = (q_head-q_tail) < RECQ;
        int leaseroom = q_inflight < QDEPTH;
        if (r.usable && qroom && leaseroom) { r.bufidx=(int)bidx; q_inflight++; handed=1; }
        else r.bufidx=-1;
        if (qroom) { q[q_head % RECQ]=r; q_head++; pthread_cond_signal(&q_cv); }
        else q_dropped++;
        pthread_mutex_unlock(&q_mtx);

        /* If the detector did not take the buffer it goes straight back. This is
         * the line that guarantees detection can never stall acquisition. */
        if (!handed && xioctl(g_vfd,VIDIOC_QBUF,&b)==-1)
            die("VIDIOC_QBUF %u: %s",bidx,strerror(errno));

        double now=now_mono();
        if (!quiet && now-t_status>=0.1 && now-t0>=0.3) {
            t_status=now;
            double el=now-t0;
            pthread_mutex_lock(&q_mtx);
            unsigned depth=q_head-q_tail, infl=q_inflight;
            pthread_mutex_unlock(&q_mtx);
            char blob[48];
            if (w_last_valid) snprintf(blob,sizeof blob,"%6.1f,%6.1f",w_last_cx,w_last_cy);
            else snprintf(blob,sizeof blob,"   --,    --");
            fprintf(stderr,"\r  %6.1fs %6lu fr %5.2ffps | imu %6.1fHz | blob %s"
                           " | det %lu skip %lu | prev %lu | drop %lu q%u/%u%s",
                    el,frames,el>0?frames/el:0.0,
                    el>0?(g_nimu-imu_at_start)/el:0.0, blob,
                    w_det,w_skipq,s_prev_shown,dropped,depth,infl,
                    isatty_err?"  ":"\n");
            fflush(stderr);
        }
        double w_ms=(now_mono()-t_poll1)*1000.0;
        a_sum_work+=w_ms;
        if (w_ms>a_max_work) a_max_work=w_ms;
    }

    double el=now_mono()-t0;

    /* ---- shut down in order: stop acquiring, drain the detector, then the
     * rest. The detector must finish before the buffers are unmapped. ---- */
    pthread_mutex_lock(&q_mtx); q_done=1; pthread_cond_broadcast(&q_cv);
    pthread_mutex_unlock(&q_mtx);
    pthread_join(tid_det,NULL);
    g_stop=1;
    pthread_join(tid_tlm,NULL);
    if (have_prev) {
        pthread_mutex_lock(&g_prev_mtx);
        pthread_cond_broadcast(&g_prev_cv);
        pthread_mutex_unlock(&g_prev_mtx);
        pthread_join(tid_enc,NULL);
        pthread_join(tid_net,NULL);
        for (int i=0;i<MAXCLIENT;i++) { if (g_cli[i].fd>=0) close(g_cli[i].fd);
                                        free(g_cli[i].buf); }
        if (g_listen>=0) close(g_listen);
        free(g_prev_pending);
    }

    enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(g_vfd,VIDIOC_STREAMOFF,&type)==-1)
        fprintf(stderr,"\nwarning: VIDIOC_STREAMOFF: %s\n",strerror(errno));
    for (unsigned i=0;i<nbuf_granted;i++) if (g_bufs[i]) munmap(g_bufs[i],g_blen[i]);
    close(g_vfd); close(g_tlm_fd);
    fclose(g_csv);
    fclose(g_imu_csv);
    if (!quiet) fputc('\n',stderr);

    /* ---- summary ---- */
    double mean_fps = el>0?frames/el:0.0;
    unsigned long imu_n = g_nimu-imu_at_start;
    double p50=0,p95=0,pmax=0;
    if (nlat) {
        qsort(lat,nlat,sizeof(double),cmp_d);
        p50=lat[nlat/2]; p95=lat[(int)(nlat*0.95)]; pmax=lat[nlat-1];
    }
    FILE *S=fopen(sumpath,"w");
    for (int pass=0;pass<2;pass++) {
        FILE *o = pass==0 ? stderr : S;
        if (!o) continue;
        fprintf(o,"=== blob_log run summary ===\n");
        fprintf(o,"stopped          %s\n",
                g_stop&&!want_secs&&!want_frames?"signal (Ctrl-C)":why);
        fprintf(o,"logged           %.2f s\n",el);
        fprintf(o,"video device     %s\n",vdev);
        fprintf(o,"imu device       %s @ %ld\n",gdev,baud_n);
        fprintf(o,"card / driver    %s / %s\n",cap.card,cap.driver);
        fprintf(o,"geometry         %ux%u %s  %u B/frame\n",W,H,fcs,g_frame_sz);
        fprintf(o,"buffers          %u   detector lease %d\n",nbuf_granted,QDEPTH);
        fprintf(o,"thread pinning   %s (acq 1, imu %d, det %d, enc %d), det nice +%d\n",
                do_pin?"on":"off",g_tlm_pin,g_det_pin,g_enc_pin,g_det_nice);

        fprintf(o,"\n-- frames (priority 1) --\n");
        fprintf(o,"acquired         %lu  (%.2f fps)\n",frames,mean_fps);
        fprintf(o,"usable           %lu   unusable %lu\n",n_usable,frames-n_usable);
        fprintf(o,"interval         %.2f .. %.2f ms\n",n_usable>1?gap_min:0.0,gap_max);
        fprintf(o,"dropped upstream %lu%s\n",dropped,
                dropped?"  (kernel sequence gaps -- lost before this program saw them)":"");
        if (seq_anom) fprintf(o,"seq anomalies    %lu  (sequence did not advance)\n",seq_anom);
        fprintf(o,"first byte -> available (latency)\n");
        fprintf(o,"  mean %.3f  min %.3f  p50 %.3f  p95 %.3f  max %.3f ms  (n=%u)\n",
                n_usable?sum_lat/n_usable:0.0,n_usable?min_lat:0.0,p50,p95,pmax,nlat);

        fprintf(o,"\n-- imu (priority 2) --\n");
        fprintf(o,"samples          %lu  (%.1f Hz)\n",imu_n, el>0?imu_n/el:0.0);
        fprintf(o,"crc errors       %lu   resync bytes %lu\n",g_ncrc,g_nresync);
        {
            unsigned long nb = g_imu_batched-batched_at_start;
            fprintf(o,"arrival spacing  worst gap %.2f ms;  %lu sample(s) (%.1f%%) shared a\n"
                      "                 timestamp with the one before (read batching)\n",
                    g_imu_worst_gap*1000.0, nb, imu_n?100.0*nb/imu_n:0.0);
        }
        fprintf(o,"frame pairing    mean |dt| %.3f ms   worst %.3f ms   unmatched %lu\n",
                (n_usable-nomatch)?sum_idt/(double)(n_usable-nomatch):0.0,worst_idt,nomatch);

        fprintf(o,"\n-- blob detection (priority 3) --\n");
        fprintf(o,"csv rows         %lu  (one per acquired frame)\n",w_rows);
        fprintf(o,"detected on      %lu frames  (%.1f%% of acquired, %.2f fps)\n",
                w_det, frames?100.0*w_det/frames:0.0, el>0?w_det/el:0.0);
        fprintf(o,"  blob found     %lu  (%.1f%% of detected)\n",
                w_found, w_det?100.0*w_found/w_det:0.0);
        fprintf(o,"  ambiguous      %lu  (runner-up at least half the winner's area:\n"
            "                 the pick could flip between frames)\n",w_amb);
        fprintf(o,"  skipped_queue  %lu  (detector busy; acquisition kept priority)\n",w_skipq);
        fprintf(o,"  bad_frame      %lu  (error/short frame, pixels not trusted)\n",w_bad);
        fprintf(o,"detect step      %u  (grid %ux%u)\n",g_step,W/g_step,H/g_step);
        fprintf(o,"cost             mean %.2f ms/frame  max %.2f ms  (interval %.1f ms)\n",
                w_det?w_sum_cost/w_det:0.0, w_max_cost, frames>1?el/frames*1000.0:0.0);
        fprintf(o,"  of which copy  mean %.2f ms  max %.2f ms\n",
                w_det?w_sum_copy/w_det:0.0, w_max_copy);
        if (q_dropped)
            fprintf(o,"CSV ROWS LOST    %lu  (record queue overflowed -- raise RECQ)\n",q_dropped);

        fprintf(o,"\n-- is acquisition prompt? --\n");
        {
            double w50=0,w95=0;
            if (npw) {
                qsort(pw,npw,sizeof(double),cmp_d);
                w50=pw[npw/2]; w95=pw[(int)(npw*0.95)];
            }
            fprintf(o,"blocked in poll  mean %.2f  p50 %.2f  p95 %.2f  min %.2f  max %.2f ms\n",
                    frames?a_sum_pw/frames:0.0,w50,w95,npw?a_min_pw:0.0,a_max_pw);
            fprintf(o,"                 (of a %.1f ms frame interval -- time spent parked,\n"
                      "                  ready to take the frame the moment it lands)\n",
                    frames>1?el/frames*1000.0:0.0);
            fprintf(o,"own work/frame   mean %.2f  max %.2f ms  (everything this thread\n"
                      "                 does between frames: pairing, queueing, status)\n",
                    frames?a_sum_work/frames:0.0,a_max_work);
            fprintf(o,"frame was ALREADY waiting when we polled:  %lu of %lu (%.2f%%)\n",
                    a_ready_already,frames,frames?100.0*a_ready_already/frames:0.0);
            fprintf(o,"another frame ready the instant we took one: %lu (%.2f%%)\n",
                    a_backlog,frames?100.0*a_backlog/frames:0.0);
            fprintf(o,"verdict          %s\n",
                    (frames && 100.0*a_ready_already/frames < 2.0 && a_backlog==0)
                    ? "prompt -- this thread was waiting on the camera, not the\n"
                      "                 other way round, so t_available is as early as the\n"
                      "                 driver allows"
                    : "NOT prompt -- frames were waiting in the driver before\n"
                      "                 this thread asked for them; some of the measured\n"
                      "                 latency is ours");
        }

        fprintf(o,"\n-- latency chain: motion in the world -> cx,cy exist --\n");
        {
            double q50=0,q95=0,qmx=0;
            if (w_ne2e) {
                qsort(w_e2e,w_ne2e,sizeof(double),cmp_d);
                q50=w_e2e[w_ne2e/2]; q95=w_e2e[(int)(w_ne2e*0.95)]; qmx=w_e2e[w_ne2e-1];
            }
            fprintf(o,"  [A] world motion -> first byte on host   NOT MEASURED HERE\n");
            fprintf(o,"      sensor integration + the core's internal pipeline. No host\n");
            fprintf(o,"      clock can see it; it needs an external reference. Use the IMU:\n");
            fprintf(o,"      move the gimbal and run ./det_latency on this folder.\n");
            fprintf(o,"  [B] first byte -> frame available        mean %7.3f  p95 %7.3f ms\n",
                    n_usable?sum_lat/n_usable:0.0,p95);
            fprintf(o,"      the frame arriving over USB, ~one frame interval at 25 fps\n");
            fprintf(o,"  [C] available -> detector picked it up   mean %7.3f  max %7.3f ms\n",
                    w_det?w_sum_q/w_det:0.0,w_max_q);
            fprintf(o,"  [D] detector work (copy + detect)        mean %7.3f  max %7.3f ms\n",
                    w_det?w_sum_cost/w_det:0.0,w_max_cost);
            fprintf(o,"      -----------------------------------------------------------\n");
            fprintf(o,"  B+C+D  first byte -> cx,cy exist         mean %7.3f  p50 %7.3f\n",
                    w_det?w_sum_e2e/w_det:0.0,q50);
            fprintf(o,"                                           p95  %7.3f  max %7.3f ms\n",
                    q95,qmx);
            fprintf(o,"  Total real-world latency = [A] + %.1f ms.\n",
                    w_det?w_sum_e2e/w_det:0.0);
        }

        fprintf(o,"\n-- preview (priority 4, droppable by design) --\n");
        if (!g_prev_on) fprintf(o,"disabled         (-p 0)\n");
        else {
            fprintf(o,"port             %d   %ux%u jpeg q%d   last frame %zu B\n",
                    port,W/(unsigned)g_down,H/(unsigned)g_down,g_qual,s_jpeg_last);
            fprintf(o,"offered          %lu   sent %lu  (%.2f fps)\n",
                    s_prev_offered,s_prev_shown, el>0?s_prev_shown/el:0.0);
            fprintf(o,"dropped          %lu overwritten before encode, %lu rate-capped,"
                      " %lu client busy\n",s_prev_overwritten,s_skip_cap,s_skip_busy);
            fprintf(o,"encode cost      mean %.2f ms   max %.2f ms\n",
                    s_prev_shown?s_enc_sum/s_prev_shown:0.0,s_enc_max);
        }

        fprintf(o,"\n-- verdict --\n");
        if (dropped==0 && w_skipq==0 && q_dropped==0)
            fprintf(o,"no frames lost and every frame was detected on: neither detection\n"
                      "nor the preview interfered with acquisition.\n");
        else {
            if (dropped)
                fprintf(o,"%lu frame(s) dropped upstream. 'skipped_queue' above says whether\n"
                          "this program was involved: if it is 0, the loss was the USB link\n"
                          "or the driver, not us.\n",dropped);
            if (w_skipq)
                fprintf(o,"%lu frame(s) logged without detection because the detector was\n"
                          "busy. No frame was lost -- every one has a CSV row. To detect on\n"
                          "all of them, run with -S 2 (about 4x cheaper).\n",w_skipq);
        }
        fprintf(o,"\nfiles\n  blobs    %s\n  imu      %s\n  summary  %s\n",
                csvpath,imupath,sumpath);
    }
    if (S) fclose(S);

    free(lat); free(g_bufs); free(g_blen); free(vdev); free(gdev);
    return (frames==0 || w_rows==0) ? 1 : 0;
}
