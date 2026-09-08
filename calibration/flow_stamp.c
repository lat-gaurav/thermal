/* flow_stamp.c -- staged start, dual-timestamp every frame, per-frame optical
 * flow, one CSV, plus a run summary. Pure C, no Python anywhere in the pipeline.
 *
 *   ./flow_stamp                 run until Ctrl-C
 *   ./flow_stamp -t 120          run 120 s
 *
 * ---------------------------------------------------------------------------
 * WHAT IT DOES, IN ORDER
 *
 *   Phase 1  start the camera, discard frames until the stream has settled
 *   Phase 2  start reading the gimbal IMU, wait until its rate is steady
 *   Phase 3  record: stamp, pair and flow every frame into the CSV
 *   Exit     write a summary file (on -t expiry, frame count, or Ctrl-C)
 *
 * Phase 1 exists because the core returns its first buffers flagged ERROR with
 * bytesused 0 -- 8 of them, typically, some carrying timestamp 0. Recording those
 * would put meaningless rows at the head of every run. Phase 2 exists because the
 * IMU link needs a moment before its rate is representative, and pairing a frame
 * against a half-filled ring gives a worse match than waiting.
 *
 * ---------------------------------------------------------------------------
 * THE TWO TIMESTAMPS, per frame
 *
 *   t_first_byte  v4l2_buffer.timestamp -- the kernel's own stamp, taken when the
 *                 FIRST USB payload of that frame lands on the Jetson. This is the
 *                 earliest moment the host knows anything about the frame.
 *
 *   t_available   CLOCK_MONOTONIC read the instant VIDIOC_DQBUF returns the buffer
 *                 to this process: the frame is complete, reassembled, and usable
 *                 by any program.
 *
 *   latency_ms    t_available - t_first_byte. Transfer + reassembly + queueing.
 *
 * Both are CLOCK_MONOTONIC, the same clock the IMU samples are stamped on, so
 * every time column in the CSV is directly comparable with no conversion. This is
 * verified rather than assumed: each buffer is checked for
 * V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC and the run aborts if it is ever absent.
 *
 * Stated plainly because it matters: the buffer flag claims TSTAMP_SRC_SOE
 * ("start of exposure"), but uvcvideo runs here with hwtimestamps=0 and this
 * core's UVC payload headers carry neither PTS nor SCR. So t_first_byte is
 * derived from payload arrival on the host -- it is genuinely "first byte hit the
 * Jetson", which is what it is labelled, but it is NOT the instant the sensor
 * exposed the image. That sits an unknown fixed offset earlier.
 *
 * ---------------------------------------------------------------------------
 * WHY FLOW CANNOT HINDER ACQUISITION
 *
 * Three separate threads, and the acquisition thread is deliberately the dumbest:
 *
 *   acquisition   poll -> DQBUF -> snapshot 5 scalars -> hand the buffer index to
 *                 the flow queue -> loop. It never reads a pixel, never writes a
 *                 file, never computes anything. Microseconds per frame.
 *
 *   flow worker   takes frames from that queue in order, reads the pixels,
 *                 requeues the buffer to the driver as soon as the pixel pass is
 *                 done, then does the alignment search and writes the CSV row.
 *
 *   telemetry     does nothing but poll the IMU serial port, so arrival stamps
 *                 stay at the ~1 ms the link actually delivers. Draining that
 *                 port from a loop that also does per-frame work blurs the stamps
 *                 by milliseconds -- measured, on this hardware, as a pairing
 *                 error of 2.9 ms instead of 0.29 ms.
 *
 * The queue is the guarantee. If the flow worker falls behind and the queue is
 * full, the acquisition thread requeues the buffer immediately and marks that
 * frame flow_state=skipped_queue. It never waits, never blocks, and never lets a
 * buffer go unreturned. The failure mode under load is losing FLOW, never frames.
 * QDEPTH is capped below the buffer count so the driver always keeps spares.
 *
 * Two more measures, both available without privileges (RLIMIT_RTPRIO is 0 here,
 * so SCHED_FIFO is not an option): each thread is pinned to its own CPU so the
 * flow search cannot preempt acquisition, and the flow worker is niced down.
 *
 * ---------------------------------------------------------------------------
 * FLOW METHOD
 *
 * Each frame collapses to two 1-D profiles, row means and column means, and
 * consecutive profiles are aligned by the shift minimising mean absolute
 * difference. Collapsing first turns an O(W*H*shifts) 2-D search into
 * O((W+H)*shifts). Profiles are mean-removed so auto-exposure hunting cannot
 * masquerade as motion, and high-passed with a phase-neutral forward+backward
 * running mean, because row means are dominated by lens vignetting and sensor
 * shading fixed to the SENSOR -- leave that in and the matcher locks onto zero
 * shift however far the image travelled. The minimum is refined sub-pixel with a
 * parabola. Validated against synthetic known shifts to within 0.06 px.
 *
 * SIGN: dy_px > 0 = image content moved DOWN the sensor; dx_px > 0 = moved RIGHT.
 * Both axes are measured because the mounting orientation decides which image
 * axis a given rotation appears on.
 *
 * t_flow is the MIDPOINT of the interval the displacement spans, not the later
 * frame's stamp: a between-frames measurement attributed to the later frame is
 * half a frame interval late. Correlate against t_flow, identify frames by
 * t_first_byte.
 *
 * The flow search has a ceiling: a shift beyond -s cannot be found at all, and a
 * measurement pinned at the span is reported as flow_state=clipped rather than as
 * a real displacement.
 *
 * Build:  gcc -O2 -Wall -Wextra -o flow_stamp flow_stamp.c -lpthread -lm
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define DEF_W        1280
#define DEF_H        1024
#define DEF_NBUF     16          /* deep: the worker may hold some while computing */
#define MAX_NBUF     64
#define DEF_SPAN     300
#define QDEPTH       8           /* frames the worker may own at once; < NBUF-4 */
#define RECQ         4096        /* pending CSV records; tiny, so make it generous */
#define TLM_LEN      36
#define TLM_FLOATS   8
#define RINGSZ       16384
#define SERBUF       8192
#define MAXPROF      8192
#define MAXSPAN      1000
#define HP_WIN       101
#define LATN         500000

/* Pinned by USB serial, never by videoN/ttyUSBn -- those drift. See DEVICE_NODES.md */
static const char *THERMAL_LINK   = "/dev/thermal0";
static const char *VID_BYID_GLOB  = "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";
static const char *TLM_LINK       = "/dev/local_dds";
static const char *TLM_BYID_GLOB  = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK   = "/dev/ttyUSB0";
static const char *REC_ROOT       = "recordings";

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
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
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

static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

static int cmp_d(const void *a, const void *b)
{
    double x=*(const double*)a, y=*(const double*)b;
    return x<y?-1:x>y?1:0;
}

/* Pin a thread to one CPU. Unprivileged and cheap; keeps the flow search off the
 * core the acquisition loop runs on. Failure is reported, never fatal. */
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
static FILE *g_tlm_csv = NULL;
static unsigned long g_nimu = 0, g_ncrc = 0;
static uint8_t g_sbuf[SERBUF];
static size_t g_slen = 0;
static int g_tlm_pin = -1;

static void drain_tlm(void)
{
    if (g_slen >= sizeof g_sbuf) g_slen = 0;      /* pathological: resync */
    ssize_t n = read(g_tlm_fd, g_sbuf+g_slen, sizeof g_sbuf - g_slen);
    if (n <= 0) return;
    double t = now_mono();
    g_slen += (size_t)n;
    size_t i=0;
    while (g_slen-i >= TLM_LEN) {
        if (g_sbuf[i]!=0xA5 || g_sbuf[i+1]!=0x5A) { i++; continue; }
        uint16_t want=(uint16_t)(g_sbuf[i+34]|(g_sbuf[i+35]<<8));
        /* advance 1 on a CRC miss: a real frame can start one byte into a
         * false A5 5A, and skipping both magic bytes would step over it */
        if (crc16(&g_sbuf[i+2],32)!=want) { g_ncrc++; i++; continue; }
        float fl[TLM_FLOATS];
        memcpy(fl,&g_sbuf[i+2],sizeof fl);
        ring_push(t, fl[0], fl[1]);
        if (g_tlm_csv) {
            fprintf(g_tlm_csv, "%.6f", t);
            for (int k=0;k<TLM_FLOATS;k++) fprintf(g_tlm_csv, ",%.4f", (double)fl[k]);
            fputc('\n', g_tlm_csv);
        }
        g_nimu++;
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

/* ===================== flow ============================================== */

static double align_prof(const double *a, const double *b, int n, int span,
                         double *quality, int *at_limit)
{
    static __thread double sm[MAXPROF], ha[MAXPROF], hb[MAXPROF];
    static __thread double cost[2*MAXSPAN+1];
    if (n>MAXPROF) n=MAXPROF;
    if (span>MAXSPAN) span=MAXSPAN;
    if (span>n/2) span=n/2;

    const double *src[2]={a,b};
    double *dst[2]={ha,hb};
    for (int p=0;p<2;p++) {
        double run=0; int cnt=0;
        for (int i=0;i<n;i++) {
            run+=src[p][i]; cnt++;
            if (cnt>HP_WIN) { run-=src[p][i-HP_WIN]; cnt--; }
            sm[i]=run/cnt;
        }
        /* backward pass keeps the smoothing phase-neutral, so the high-passed
         * profile is not itself shifted */
        run=0; cnt=0;
        for (int i=n-1;i>=0;i--) {
            run+=sm[i]; cnt++;
            if (cnt>HP_WIN) { run-=sm[i+HP_WIN]; cnt--; }
            dst[p][i]=src[p][i]-run/cnt;
        }
    }
    double best=1e300,sum=0; int bestd=0,nd=0;
    for (int d=-span;d<=span;d++) {
        int i0=d<0?-d:0, i1=d<0?n:n-d;
        double acc=0; int m=0;
        for (int i=i0;i<i1;i++) {
            double diff=ha[i]-hb[i+d];
            acc += diff<0?-diff:diff;
            m++;
        }
        double cv=m?acc/m:1e300;
        cost[d+span]=cv; sum+=cv; nd++;
        if (cv<best) { best=cv; bestd=d; }
    }
    double mean=nd?sum/nd:0;
    *quality = mean>0?(mean-best)/mean:0;
    /* A best match sitting on the edge of the search means the true shift is at
     * least this large and possibly larger -- not a measurement. */
    if (at_limit) *at_limit = (bestd<=-span+1 || bestd>=span-1);

    double dy=bestd; int bi=bestd+span;
    if (bi>0 && bi<2*span) {
        double lo=cost[bi-1], hi=cost[bi+1];
        double den=lo-2*best+hi;
        if (den!=0) {
            double adj=0.5*(lo-hi)/den;
            if (adj>-1&&adj<1) dy=bestd+adj;
        }
    }
    return dy;
}

/* ===================== record queue ====================================== */

enum { FS_OK=0, FS_NOPREV, FS_SKIP_QUEUE, FS_BADFRAME, FS_CLIPPED };
static const char *FS_NAME[] = { "ok","no_prev","skipped_queue","bad_frame","clipped" };

struct frec {
    uint32_t seq, flags, bytesused;
    double t_fb, t_arr;
    int bufidx;              /* >=0: worker owns it and must QBUF; -1: already back */
    int usable;
    int have_imu;
    double imu_t, yaw, pitch;
    unsigned n_imu_int;
};

static struct frec q[RECQ];
static unsigned q_head=0, q_tail=0;          /* head = next to write, tail = next to read */
static unsigned q_inflight=0;                /* records whose bufidx >= 0 */
static unsigned long q_dropped=0;
static pthread_mutex_t q_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  q_cv  = PTHREAD_COND_INITIALIZER;
static int q_done = 0;

/* ===================== globals shared with the worker ==================== */

static int g_vfd = -1;
static void **g_bufs = NULL;
static size_t *g_blen = NULL;
static unsigned g_W=DEF_W, g_H=DEF_H, g_span=DEF_SPAN;
static uint32_t g_frame_sz=0;
static FILE *g_csv = NULL;
static int g_flow_pin = -1, g_flow_nice = 5;

/* worker stats */
static unsigned long w_rows=0, w_flow=0, w_lowq=0, w_clip=0, w_noprev=0, w_skipq=0, w_bad=0;
static double w_sum_absdy=0, w_sum_absdx=0, w_sum_qy=0, w_sum_qx=0;
static double w_sum_cost=0, w_max_cost=0;

static void *flow_thread(void *arg)
{
    (void)arg;
    if (g_flow_pin >= 0) pin_to(g_flow_pin);
    /* Nice the worker down so it cannot preempt acquisition. Raising the nice
     * value needs no privilege; lowering it would (RLIMIT_NICE is 0 here). */
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), g_flow_nice);

    double *rprof=malloc(sizeof(double)*g_H), *rprev=malloc(sizeof(double)*g_H);
    double *cprof=malloc(sizeof(double)*g_W), *cprev=malloc(sizeof(double)*g_W);
    unsigned long *rsum=malloc(sizeof(unsigned long)*g_H);
    unsigned long *csum=malloc(sizeof(unsigned long)*g_W);
    if (!rprof||!rprev||!cprof||!cprev||!rsum||!csum) die("worker: out of memory");

    int have_prev=0;
    double prev_fb=0, prev_yaw=0, prev_pitch=0;
    int have_prev_imu=0;

    for (;;) {
        struct frec r;
        pthread_mutex_lock(&q_mtx);
        while (q_tail==q_head && !q_done) pthread_cond_wait(&q_cv,&q_mtx);
        if (q_tail==q_head && q_done) { pthread_mutex_unlock(&q_mtx); break; }
        r = q[q_tail % RECQ];
        q_tail++;
        pthread_mutex_unlock(&q_mtx);

        int state = FS_OK;
        double dy=0,dx=0,qy=0,qx=0,tf=0,dtf=0;
        int flow_ok=0;
        double tc0 = now_mono();

        if (r.bufidx >= 0) {
            /* One sequential pass accumulating BOTH row and column sums. A
             * separate y-inner loop for columns would stride the cache and cost
             * several times more. */
            const uint8_t *img=(const uint8_t*)g_bufs[r.bufidx];
            memset(csum,0,sizeof(unsigned long)*g_W);
            for (unsigned y=0;y<g_H;y++) {
                const uint8_t *row=img+(size_t)y*g_W;
                unsigned long rs=0;
                for (unsigned x=0;x<g_W;x++) { unsigned v=row[x]; rs+=v; csum[x]+=v; }
                rsum[y]=rs;
            }
            /* Buffer back to the driver the instant the pixels are read, before
             * the alignment search -- it is out of the driver's hands for as
             * little time as possible. */
            struct v4l2_buffer b;
            memset(&b,0,sizeof b);
            b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
            b.index=(unsigned)r.bufidx;
            xioctl(g_vfd,VIDIOC_QBUF,&b);
            pthread_mutex_lock(&q_mtx); q_inflight--; pthread_mutex_unlock(&q_mtx);

            double gs=0;
            for (unsigned y=0;y<g_H;y++) { rprof[y]=(double)rsum[y]/g_W; gs+=rprof[y]; }
            double gm=gs/g_H;
            for (unsigned y=0;y<g_H;y++) rprof[y]-=gm;
            gs=0;
            for (unsigned x=0;x<g_W;x++) { cprof[x]=(double)csum[x]/g_H; gs+=cprof[x]; }
            gm=gs/g_W;
            for (unsigned x=0;x<g_W;x++) cprof[x]-=gm;

            if (have_prev && prev_fb>0) {
                int lim_y=0, lim_x=0;
                dy=align_prof(rprev,rprof,(int)g_H,g_span,&qy,&lim_y);
                dx=align_prof(cprev,cprof,(int)g_W,g_span,&qx,&lim_x);
                dtf=r.t_fb-prev_fb;
                tf=prev_fb+dtf/2.0;
                flow_ok=1;
                if (lim_y||lim_x) { state=FS_CLIPPED; w_clip++; }
                w_flow++;
                w_sum_absdy += dy<0?-dy:dy;
                w_sum_absdx += dx<0?-dx:dx;
                w_sum_qy += qy; w_sum_qx += qx;
                if (qy<0.05||qx<0.05) w_lowq++;
            } else {
                state=FS_NOPREV; w_noprev++;
            }
            memcpy(rprev,rprof,sizeof(double)*g_H);
            memcpy(cprev,cprof,sizeof(double)*g_W);
            have_prev=1;
            prev_fb=r.t_fb;
        } else {
            /* No pixels were made available for this frame: either the queue was
             * full (acquisition kept priority, as designed) or the frame itself
             * was unusable. Either way the profile chain is broken, so the NEXT
             * frame cannot produce flow against a stale predecessor. */
            state = r.usable ? FS_SKIP_QUEUE : FS_BADFRAME;
            if (r.usable) w_skipq++; else w_bad++;
            have_prev=0;
        }
        double cost=(now_mono()-tc0)*1000.0;
        w_sum_cost+=cost; if (cost>w_max_cost) w_max_cost=cost;

        /* ---- CSV row. Single writer, consuming the queue in order, so rows
         * come out in frame order without any sorting. ---- */
        fprintf(g_csv,"%u,%.6f,%.6f,%.3f,", r.seq, r.t_fb, r.t_arr,
                (r.t_arr-r.t_fb)*1000.0);
        if (r.have_imu)
            fprintf(g_csv,"%.4f,%.4f,%.6f,%.3f,%u,",
                    r.yaw, r.pitch, r.imu_t, (r.t_fb-r.imu_t)*1000.0, r.n_imu_int);
        else
            fprintf(g_csv,",,,,%u,", r.n_imu_int);
        if (flow_ok)
            fprintf(g_csv,"%.6f,%.3f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,",
                    tf, dtf*1000.0, dy, dtf>0?dy/dtf:0.0, qy,
                    dx, dtf>0?dx/dtf:0.0, qx);
        else
            fprintf(g_csv,",,,,,,,,");
        if (flow_ok && r.have_imu && have_prev_imu) {
            double dyaw=wrap180(r.yaw-prev_yaw), dpit=wrap180(r.pitch-prev_pitch);
            fprintf(g_csv,"%.4f,%.4f,%.3f,%.3f,", dyaw, dpit,
                    dtf>0?dyaw/dtf:0.0, dtf>0?dpit/dtf:0.0);
        } else {
            fprintf(g_csv,",,,,");
        }
        fprintf(g_csv,"0x%08x,%s\n", r.flags, FS_NAME[state]);
        w_rows++;
        if (r.have_imu) { prev_yaw=r.yaw; prev_pitch=r.pitch; have_prev_imu=1; }
    }
    free(rprof); free(rprev); free(cprof); free(cprev); free(rsum); free(csum);
    return NULL;
}

/* ===================== video ============================================= */

static unsigned g_nbuf_granted = 0;

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
    /* S_FMT is a negotiation, not a command. Flow indexes the buffer as W*H bytes
     * of GREY, so a substituted format would be measured wrong -- refuse. */
    if (f.fmt.pix.width!=W || f.fmt.pix.height!=H ||
        f.fmt.pix.pixelformat!=V4L2_PIX_FMT_GREY) {
        char got[5]; fourcc_str(f.fmt.pix.pixelformat,got);
        die("driver refused the format.\n  asked %ux%u GREY, got %ux%u %s",
            W,H,f.fmt.pix.width,f.fmt.pix.height,got);
    }
    *frame_sz=f.fmt.pix.sizeimage;
    *stride=f.fmt.pix.bytesperline;
    fourcc_str(f.fmt.pix.pixelformat,fcs);

    struct v4l2_requestbuffers rb;
    memset(&rb,0,sizeof rb);
    rb.count=nbuf; rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory=V4L2_MEMORY_MMAP;
    if (xioctl(fd,VIDIOC_REQBUFS,&rb)==-1)
        die("VIDIOC_REQBUFS (%u): %s",nbuf,strerror(errno));
    g_nbuf_granted = rb.count;
    if (rb.count < QDEPTH+4)
        die("driver gave only %u buffers; need at least %d so acquisition always\n"
            "  has spares while the flow worker holds up to %d",
            rb.count, QDEPTH+4, QDEPTH);
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

int main(int argc, char **argv)
{
    const char *vdev_opt=NULL,*gdev_opt=NULL,*root=REC_ROOT,*out_opt=NULL;
    unsigned W=DEF_W,H=DEF_H,nbuf=DEF_NBUF;
    int span=DEF_SPAN, quiet=0, want_tlm_csv=0, do_pin=1, c;
    double want_secs=0, cam_settle=2.0, imu_settle=2.0;
    unsigned long want_frames=0;
    long baud_n=921600; speed_t baud=B921600;

    while ((c=getopt(argc,argv,"t:n:o:D:s:b:W:H:v:g:B:C:I:PqTih"))!=-1) {
        switch (c) {
        case 't': want_secs=atof(optarg); break;
        case 'n': want_frames=strtoul(optarg,NULL,10); break;
        case 'o': out_opt=optarg; break;
        case 'D': root=optarg; break;
        case 's': span=atoi(optarg); break;
        case 'b': nbuf=(unsigned)strtoul(optarg,NULL,10); break;
        case 'W': W=(unsigned)strtoul(optarg,NULL,10); break;
        case 'H': H=(unsigned)strtoul(optarg,NULL,10); break;
        case 'v': vdev_opt=optarg; break;
        case 'g': gdev_opt=optarg; break;
        case 'C': cam_settle=atof(optarg); break;
        case 'I': imu_settle=atof(optarg); break;
        case 'P': do_pin=0; break;
        case 'q': quiet=1; break;
        case 'i': want_tlm_csv=1; break;
        /* -T used to suppress the full-rate IMU CSV. That is now the default,
         * so it is accepted and ignored rather than erroring on old commands. */
        case 'T': break;
        case 'B':
            baud_n=strtol(optarg,NULL,10);
            if (baud_n==115200) baud=B115200;
            else if (baud_n==921600) baud=B921600;
            else die("-B must be 115200 or 921600");
            break;
        case 'h':
        default:
            printf(
"Staged start, dual-timestamped frames, per-frame optical flow, CSV + summary.\n"
"Flow runs on its own thread and can never delay frame acquisition.\n"
"\n"
"usage: %s [-t SECS] [-n FRAMES] [-o CSV] [-D ROOT] [-s SPAN] [-b BUFS]\n"
"          [-C SECS] [-I SECS] [-W W] [-H H] [-v VIDEODEV] [-g IMUDEV]\n"
"          [-B BAUD] [-P] [-T] [-q]\n"
"\n"
"  -t SECS    stop after SECS of recording (default: until Ctrl-C)\n"
"  -n FRAMES  stop after FRAMES recorded frames\n"
"  -o CSV     per-frame CSV path (default: a timestamped run folder)\n"
"  -D ROOT    run-folder root (default %s/)\n"
"  -s SPAN    max flow shift searched, px (default %d)\n"
"  -b BUFS    v4l2 buffers, %d..%d (default %d)\n"
"  -C SECS    camera settle time before recording (default 2)\n"
"  -I SECS    IMU settle time before recording (default 2)\n"
"  -W, -H     geometry (default %dx%d)\n"
"  -v, -g     device overrides; -B telemetry baud (115200 or 921600)\n"
"  -P         do not pin threads to CPUs\n"
"  -i         also write the full-rate (~1 kHz) IMU CSV as a second file\n"
"             (off by default: the per-frame CSV already carries the\n"
"             IMU sample paired to each frame)\n"
"  -q         no live status line\n", argv[0], REC_ROOT, DEF_SPAN,
                QDEPTH+4, MAX_NBUF, DEF_NBUF, DEF_W, DEF_H);
            return c=='h'?0:2;
        }
    }
    if (nbuf < QDEPTH+4 || nbuf > MAX_NBUF)
        die("-b must be %d..%d (the worker may hold %d, the driver needs spares)",
            QDEPTH+4, MAX_NBUF, QDEPTH);
    if (span<1) die("-s must be >= 1");
    if (W<2||H<2||W>MAXPROF||H>MAXPROF) die("geometry %ux%u out of range",W,H);
    g_W=W; g_H=H; g_span=span;

    char *vdev=resolve(vdev_opt,THERMAL_LINK,VID_BYID_GLOB,NULL);
    if (!vdev) die("no Sirius capture node found (looked for %s, then %s)\n"
                   "  is the camera plugged in?  ls -l /dev/v4l/by-id/",
                   THERMAL_LINK,VID_BYID_GLOB);
    char *gdev=resolve(gdev_opt,TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);
    if (!gdev) die("no IMU/gimbal port found (looked for %s, then %s, then %s)\n"
                   "  is the USB-TTL converter plugged in?  ls -l /dev/serial/by-id/",
                   TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);

    /* ---- run folder ---- */
    char outdir[512]="", csvpath[640], tlmpath[700], sumpath[700];
    if (out_opt) {
        snprintf(csvpath,sizeof csvpath,"%s",out_opt);
        const char *sl=strrchr(csvpath,'/');
        if (sl&&sl!=csvpath) snprintf(outdir,sizeof outdir,"%.*s",(int)(sl-csvpath),csvpath);
        snprintf(tlmpath,sizeof tlmpath,"%s.imu.csv",csvpath);
        snprintf(sumpath,sizeof sumpath,"%s.summary.txt",csvpath);
    } else {
        time_t tt=time(NULL); struct tm tm; localtime_r(&tt,&tm);
        unsigned dup=0;
        for (;;) {
            int len;
            if (dup==0)
                len=snprintf(outdir,sizeof outdir,"%s/%04d-%02d-%02d/%02d%02d%02d",
                    root,tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
            else
                len=snprintf(outdir,sizeof outdir,"%s/%04d-%02d-%02d/%02d%02d%02d-%u",
                    root,tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,dup);
            if (len<0||(size_t)len>=sizeof outdir) die("-D path too long");
            if (access(outdir,F_OK)!=0) break;
            if (++dup>999) die("cannot find an unused folder under %s",root);
        }
        snprintf(csvpath,sizeof csvpath,"%s/frames.csv",outdir);
        snprintf(tlmpath,sizeof tlmpath,"%s/imu.csv",outdir);
        snprintf(sumpath,sizeof sumpath,"%s/summary.txt",outdir);
    }
    if (outdir[0] && mkdir_p(outdir)==-1)
        die("cannot create folder %s: %s",outdir,strerror(errno));

    struct sigaction sa;
    memset(&sa,0,sizeof sa);
    sa.sa_handler=on_signal;
    sigaction(SIGINT,&sa,NULL); sigaction(SIGTERM,&sa,NULL);

    if (do_pin) { g_tlm_pin=2; g_flow_pin=3; }

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
    unsigned long p1_seen=0, p1_bad=0;
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
        /* Settled = a run of clean frames AND the settle time elapsed. The run
         * requirement is what skips the stream-start transient; the timer is what
         * lets AGC and the core's internal pipeline reach steady state. */
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
    if (want_tlm_csv) {
        g_tlm_csv=fopen(tlmpath,"w");
        if (!g_tlm_csv) die("cannot create %s: %s",tlmpath,strerror(errno));
        fprintf(g_tlm_csv,"# full-rate IMU, receive only. port=%s baud=%ld\n",gdev,baud_n);
        fprintf(g_tlm_csv,"# t_imu = CLOCK_MONOTONIC at arrival, same clock as the frames\n");
        fprintf(g_tlm_csv,"t_imu,yaw,pitch,f2,f3,f4,f5,f6,f7\n");
    }
    pthread_t tid_tlm;
    if (pthread_create(&tid_tlm,NULL,tlm_thread,NULL)!=0)
        die("cannot start IMU thread: %s",strerror(errno));

    /* The video stream is already running, so this loop MUST keep draining it
     * while waiting for the IMU. Sleeping here instead let all 16 buffers fill:
     * the driver then had nowhere to put new frames (40 lost to sequence gaps),
     * and the buffers that were waiting carried timestamps up to 2 s old, so the
     * first recorded latencies read ~2000 ms. Discard-as-you-go keeps the
     * pipeline shallow and phase 3 starts with genuinely fresh frames. */
    double p2_t0=now_mono();
    unsigned long imu_at_start=0, p2_discarded=0;
    for (;;) {
        if (g_stop) break;
        struct pollfd pfd={.fd=g_vfd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,50);
        if (pr>0 && (pfd.revents&POLLIN)) {
            struct v4l2_buffer b;
            memset(&b,0,sizeof b);
            b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
            if (xioctl(g_vfd,VIDIOC_DQBUF,&b)==0) {
                xioctl(g_vfd,VIDIOC_QBUF,&b);
                p2_discarded++;
            }
        }
        double el=now_mono()-p2_t0;
        pthread_mutex_lock(&ring_mtx);
        unsigned long n=g_nimu;
        pthread_mutex_unlock(&ring_mtx);
        double hz = el>0? n/el : 0;
        /* Settled = enough samples to fill a frame interval many times over, the
         * settle time elapsed, and a plausible rate. */
        if (el>=imu_settle && n>=500 && hz>200.0) { imu_at_start=n; break; }
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

    /* =================== PHASE 3: record ================================= */
    g_csv=fopen(csvpath,"w");
    if (!g_csv) die("cannot create %s: %s",csvpath,strerror(errno));
    fprintf(g_csv,"# flow_stamp: per-frame timestamps, IMU pairing and optical flow\n");
    fprintf(g_csv,"# video=%s imu=%s baud=%ld\n",vdev,gdev,baud_n);
    fprintf(g_csv,"# geometry=%ux%u %s  flow_span=%d px  buffers=%u\n",W,H,fcs,span,nbuf_granted);
    fprintf(g_csv,"# clock=CLOCK_MONOTONIC for every t_ column\n");
    fprintf(g_csv,"# t_first_byte = kernel stamp, first USB payload of the frame on the host\n");
    fprintf(g_csv,"# t_available  = VIDIOC_DQBUF returned it; usable by any program\n");
    fprintf(g_csv,"# t_flow       = midpoint of the interval the flow spans; correlate on this\n");
    fprintf(g_csv,"# dy_px>0 = content moved DOWN; dx_px>0 = moved RIGHT\n");
    fprintf(g_csv,"# flow_state: ok | no_prev | skipped_queue | bad_frame | clipped\n");
    fprintf(g_csv,"seq,t_first_byte,t_available,latency_ms,"
                  "yaw,pitch,t_imu,imu_dt_ms,n_imu_interval,"
                  "t_flow,interval_ms,dy_px,dy_rate_px_s,qual_y,"
                  "dx_px,dx_rate_px_s,qual_x,"
                  "d_yaw_deg,d_pitch_deg,yaw_rate_dps,pitch_rate_dps,"
                  "flags,flow_state\n");

    pthread_t tid_flow;
    if (pthread_create(&tid_flow,NULL,flow_thread,NULL)!=0)
        die("cannot start flow thread: %s",strerror(errno));

    if (!quiet) {
        fprintf(stderr,"phase 3     recording -> %s\n",csvpath);
        fprintf(stderr,"            flow on its own thread (pin %d, nice +%d),"
                       " queue depth %d of %u buffers\n",
                g_flow_pin,g_flow_nice,QDEPTH,nbuf_granted);
        fprintf(stderr,"            Ctrl-C to stop\n");
    }

    const double t0=now_mono();
    unsigned long frames=0, dropped=0, seq_anom=0, timeouts=0, nomatch=0, n_usable=0;
    unsigned prev_ring=0; uint32_t last_seq=0; int have_seq=0;
    double prev_fb=0, gap_min=1e9, gap_max=0, t_status=0;
    double sum_lat=0, min_lat=1e9, max_lat=0, sum_idt=0, worst_idt=0;
    double *lat=malloc(sizeof(double)*LATN); unsigned nlat=0;
    if (!lat) die("out of memory");
    int isatty_err=isatty(STDERR_FILENO);
    const char *why="signal";

    while (!g_stop) {
        if (want_frames && frames>=want_frames) { why="frame count"; break; }
        if (want_secs>0 && now_mono()-t0>=want_secs) { why="duration"; break; }

        struct pollfd pfd={.fd=g_vfd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,2000);
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

        /* First thing, before any work: this is the moment the frame became
         * usable. Anything done before reading it would be charged to latency. */
        double t_arr=now_mono();
        /* QBUF overwrites this struct, and the buffer may be handed to the worker,
         * so snapshot every field needed later right now. */
        struct frec r;
        memset(&r,0,sizeof r);
        r.seq=b.sequence; r.flags=b.flags; r.bytesused=b.bytesused;
        r.t_fb=b.timestamp.tv_sec+b.timestamp.tv_usec/1e6;
        r.t_arr=t_arr;
        unsigned bidx=b.index;
        timeouts=0;
        frames++;

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
        if (r.usable && qroom && leaseroom) {
            r.bufidx=(int)bidx;
            q_inflight++;
            handed=1;
        } else {
            r.bufidx=-1;
        }
        if (qroom) { q[q_head % RECQ]=r; q_head++; pthread_cond_signal(&q_cv); }
        else q_dropped++;
        pthread_mutex_unlock(&q_mtx);

        /* If the worker did not take the buffer, it goes straight back. This is
         * the line that guarantees flow can never stall acquisition. */
        if (!handed && xioctl(g_vfd,VIDIOC_QBUF,&b)==-1)
            die("VIDIOC_QBUF %u: %s",bidx,strerror(errno));

        double now=now_mono();
        if (!quiet && now-t_status>=0.1 && now-t0>=0.3) {
            t_status=now;
            double el=now-t0;
            pthread_mutex_lock(&q_mtx);
            unsigned depth=q_head-q_tail, infl=q_inflight;
            pthread_mutex_unlock(&q_mtx);
            fprintf(stderr,"\r  %6.1fs %6lu fr %5.2ffps | imu %6.1fHz | lat %5.1fms"
                           " | q %u/%d infl %u | flow %lu skip %lu | drop %lu%s",
                    el,frames,el>0?frames/el:0.0, el>0?(g_nimu-imu_at_start)/el:0.0,
                    n_usable?sum_lat/n_usable:0.0, depth,RECQ, infl,
                    w_flow, w_skipq, dropped, isatty_err?"  ":"\n");
            fflush(stderr);
        }
    }

    double el=now_mono()-t0;

    /* ---- shut down in order: stop acquiring, drain the worker, stop IMU ---- */
    pthread_mutex_lock(&q_mtx); q_done=1; pthread_cond_broadcast(&q_cv);
    pthread_mutex_unlock(&q_mtx);
    pthread_join(tid_flow,NULL);
    g_stop=1;
    pthread_join(tid_tlm,NULL);

    enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(g_vfd,VIDIOC_STREAMOFF,&type)==-1)
        fprintf(stderr,"\nwarning: VIDIOC_STREAMOFF: %s\n",strerror(errno));
    for (unsigned i=0;i<nbuf_granted;i++) if (g_bufs[i]) munmap(g_bufs[i],g_blen[i]);
    close(g_vfd); close(g_tlm_fd);
    fclose(g_csv);
    if (g_tlm_csv) fclose(g_tlm_csv);
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
        fprintf(o,"=== flow_stamp run summary ===\n");
        fprintf(o,"stopped          %s\n", g_stop&&!want_secs&&!want_frames?"signal (Ctrl-C)":why);
        fprintf(o,"recorded         %.2f s\n",el);
        fprintf(o,"video device     %s\n",vdev);
        fprintf(o,"imu device       %s @ %ld\n",gdev,baud_n);
        fprintf(o,"card / driver    %s / %s\n",cap.card,cap.driver);
        fprintf(o,"geometry         %ux%u %s  %u B/frame  %u B/line\n",W,H,fcs,g_frame_sz,stride);
        fprintf(o,"buffers          %u   flow queue depth %d\n",nbuf_granted,QDEPTH);
        fprintf(o,"flow span        %d px\n",span);
        fprintf(o,"thread pinning   %s (acq 1, imu %d, flow %d), flow nice +%d\n",
                do_pin?"on":"off",g_tlm_pin,g_flow_pin,g_flow_nice);
        fprintf(o,"\n-- frames --\n");
        fprintf(o,"acquired         %lu  (%.2f fps)\n",frames,mean_fps);
        fprintf(o,"usable           %lu   unusable %lu\n",n_usable,frames-n_usable);
        fprintf(o,"interval         %.2f .. %.2f ms\n",n_usable>1?gap_min:0.0,gap_max);
        fprintf(o,"dropped upstream %lu%s\n",dropped,
                dropped?"  (kernel sequence gaps -- lost before this program saw them)":"");
        if (seq_anom) fprintf(o,"seq anomalies    %lu  (sequence did not advance)\n",seq_anom);
        fprintf(o,"\n-- timestamps --\n");
        fprintf(o,"first byte -> available (latency)\n");
        fprintf(o,"  mean %.3f  min %.3f  p50 %.3f  p95 %.3f  max %.3f ms  (n=%u)\n",
                n_usable?sum_lat/n_usable:0.0,n_usable?min_lat:0.0,p50,p95,pmax,nlat);
        fprintf(o,"\n-- imu --\n");
        fprintf(o,"samples          %lu  (%.1f Hz)   crc errors %lu\n",
                imu_n, el>0?imu_n/el:0.0, g_ncrc);
        fprintf(o,"frame pairing    mean |dt| %.3f ms   worst %.3f ms   unmatched %lu\n",
                (n_usable-nomatch)?sum_idt/(double)(n_usable-nomatch):0.0,worst_idt,nomatch);
        fprintf(o,"\n-- flow --\n");
        fprintf(o,"csv rows         %lu\n",w_rows);
        fprintf(o,"computed         %lu\n",w_flow);
        fprintf(o,"  no_prev        %lu  (chain broken by the frame before)\n",w_noprev);
        fprintf(o,"  skipped_queue  %lu  (worker busy; acquisition kept priority)\n",w_skipq);
        fprintf(o,"  bad_frame      %lu  (error/short frame, pixels not trusted)\n",w_bad);
        fprintf(o,"  clipped        %lu  (shift reached the %d px search limit)\n",w_clip,span);
        if (w_flow) {
            fprintf(o,"mean |dy|        %.3f px    mean |dx| %.3f px\n",
                    w_sum_absdy/w_flow, w_sum_absdx/w_flow);
            fprintf(o,"mean quality     y %.3f   x %.3f   low-quality rows %lu\n",
                    w_sum_qy/w_flow, w_sum_qx/w_flow, w_lowq);
        }
        fprintf(o,"worker cost      mean %.2f ms/frame   max %.2f ms   (interval %.1f ms)\n",
                w_rows?w_sum_cost/w_rows:0.0, w_max_cost, frames>1?el/frames*1000.0:0.0);
        if (q_dropped)
            fprintf(o,"CSV ROWS LOST    %lu  (record queue overflowed -- raise RECQ)\n",q_dropped);
        fprintf(o,"\n-- verdict --\n");
        if (dropped==0)
            fprintf(o,"no frames were dropped: flow did not interfere with acquisition\n");
        else
            fprintf(o,"%lu frame(s) dropped upstream. Compare 'skipped_queue' above: if it\n"
                      "is 0, the loss was the USB link or driver, not this program.\n",dropped);
        fprintf(o,"\nfiles\n  frames   %s\n",csvpath);
        if (g_tlm_csv) fprintf(o,"  imu      %s\n",tlmpath);
        fprintf(o,"  summary  %s\n",sumpath);
    }
    if (S) fclose(S);

    free(lat); free(g_bufs); free(g_blen); free(vdev); free(gdev);
    return (frames==0||w_rows==0) ? 1 : 0;
}
