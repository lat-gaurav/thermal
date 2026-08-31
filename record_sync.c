/* record_sync.c -- record raw frames AND the IMU/gimbal telemetry against them,
 * in one process, on one clock. Nothing else: no flow, no analysis.
 *
 * Two streams, guaranteed to come from the same run:
 *
 *   raw frames    Exactly the bytes the kernel hands back, frame after frame, to
 *                 one flat file. No compression, no conversion, no container, no
 *                 padding -- same as record_raw.c.
 *
 *   telemetry     Gimbal/IMU @ 921600: A5 5A | 8 x float32 LE | CRC16 (MCRF4XX
 *                 over bytes[2..33]), 36 B, ~1010 Hz. float[0]=yaw,
 *                 float[1]=pitch in degrees; floats 2..7 are undocumented but
 *                 logged verbatim, since they cost nothing and cannot be
 *                 recovered later. Opened READ-ONLY: nothing is ever sent to
 *                 the gimbal.
 *
 * WHY THE TELEMETRY IS ON ITS OWN THREAD. Arrival stamps are only as good as how
 * often the port is read: samples land ~1 ms apart, so a read() returning k of
 * them stamps all k identically. Draining the port from the video loop lets the
 * per-frame disk write set the cadence, which blurs the stamps by milliseconds.
 * A thread that does nothing but poll the port holds stamps at the ~1 ms the link
 * actually delivers, whatever the video path is doing. It also means the
 * telemetry cannot cost a frame: it never touches the camera, and 36 kB/s is
 * 0.08% of the video's 43 MB/s.
 *
 * ONE CLOCK, VERIFIED NOT ASSUMED. Frames carry the kernel's own
 * v4l2_buffer.timestamp, taken when the frame's first USB payload lands on the
 * host -- far steadier than anything userspace can measure. Telemetry is stamped
 * CLOCK_MONOTONIC on arrival. uvcvideo here runs clock=CLOCK_MONOTONIC and every
 * buffer is checked for V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, so the two files join
 * with no conversion.
 *
 * Caveat worth stating: the buffer flag says TSTAMP_SRC_SOE ("start of
 * exposure"), but uvcvideo runs hwtimestamps=0 here and the core's UVC payload
 * headers carry neither PTS nor SCR, so that stamp is really derived from
 * payload arrival on the host -- not from a camera-side clock. Frames and
 * telemetry are consistently aligned relative to each other, but jointly offset
 * from true exposure by an unknown fixed delay.
 *
 * FILES WRITTEN, all into one timestamped run folder:
 *   raw-<W>x<H>-<FOURCC>.gray       frames, back to back, no header
 *   raw-<W>x<H>-<FOURCC>.gray.idx   per-frame seq / timestamp / bytes / flags
 *   raw-<W>x<H>-<FOURCC>.gray.meta  geometry -- raw video is unplayable without it
 *   frames.csv                      per frame: both timestamps + the nearest
 *                                   telemetry sample (this is the pairing)
 *   telemetry.csv                   every telemetry sample at full ~1010 Hz
 *
 * frames.csv is the "IMU against each frame" view; telemetry.csv keeps the full
 * rate, which is ~40x richer than one sample per frame and is what you want for
 * anything interpolated. The .idx is kept in record_raw's exact format so tools
 * written against that still work on this capture.
 *
 * Build:  gcc -O2 -Wall -Wextra -o record_sync record_sync.c -lpthread
 * Run:    ./record_sync -t 60            (Ctrl-C to stop)
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
#include <sys/statvfs.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define DEF_W      1280
#define DEF_H      1024
#define DEF_NBUF   8
#define MAX_NBUF   64
#define TLM_LEN    36
#define TLM_FLOATS 8
#define RINGSZ     8192
#define SERBUF     8192
#define STATUS_HZ  10
#define FLUSH_MB   64            /* push this much to disk, then drop from cache */
#define KEEP_MB    16

/* Pinned by USB serial, never by videoN/ttyUSBn -- those numbers drift across
 * re-enumeration. Same rule as record_raw.c and sync_log.c. */
static const char *THERMAL_LINK = "/dev/thermal0";
static const char *VID_BYID_GLOB =
    "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0";
static const char *TLM_LINK = "/dev/local_dds";
static const char *TLM_BYID_GLOB =
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK = "/dev/ttyUSB0";
static const char *REC_ROOT = "recordings";

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

static void fourcc_str(uint32_t f, char out[5])
{
    out[0]=f&0xff; out[1]=(f>>8)&0xff; out[2]=(f>>16)&0xff; out[3]=(f>>24)&0xff; out[4]=0;
    for (int i=0;i<4;i++) if (out[i]<32||out[i]>126) out[i]='?';
}

static int mkdir_p(const char *path)
{
    char tmp[600]; snprintf(tmp, sizeof tmp, "%s", path);
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

/* write() may return short. Anything less than the whole frame on disk silently
 * shifts every following frame, so loop until it is all out. */
static int write_all(int fd, const void *buf, size_t n)
{
    const char *p=buf;
    while (n) {
        ssize_t w=write(fd,p,n);
        if (w<0) { if (errno==EINTR) continue; return -1; }
        p+=w; n-=(size_t)w;
    }
    return 0;
}

/* -- telemetry ------------------------------------------------------------ */

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t c=0xFFFF;
    for (size_t i=0;i<n;i++) {
        c^=d[i];
        for (int b=0;b<8;b++) c=(c&1)?(c>>1)^0x8408:c>>1;
    }
    return c;
}

struct angle { double t; float yaw, pitch; };
static struct angle ring[RINGSZ];
static unsigned ring_n = 0;
/* Written by the telemetry thread, read by the video loop, so guarded.
 * Contention is negligible: ~1010 short writes/s against ~25 reads/s. */
static pthread_mutex_t ring_mtx = PTHREAD_MUTEX_INITIALIZER;

static void ring_push(double t, float yaw, float pitch)
{
    pthread_mutex_lock(&ring_mtx);
    struct angle *a=&ring[ring_n % RINGSZ];
    a->t=t; a->yaw=yaw; a->pitch=pitch;
    ring_n++;
    pthread_mutex_unlock(&ring_mtx);
}

/* Copies out the nearest sample: returning a pointer would hand back memory the
 * telemetry thread may overwrite a millisecond later. */
static int ring_nearest(double t, struct angle *out, unsigned *total)
{
    int found=0;
    pthread_mutex_lock(&ring_mtx);
    if (total) *total=ring_n;
    if (ring_n) {
        unsigned have = ring_n<RINGSZ?ring_n:RINGSZ;
        double bestd=1e18;
        for (unsigned k=0;k<have;k++) {
            const struct angle *a=&ring[(ring_n-1-k)%RINGSZ];
            double d=t-a->t; if (d<0) d=-d;
            if (d<bestd) { bestd=d; *out=*a; found=1; }
            else if (a->t<t) break;      /* moving away, older only gets worse */
        }
    }
    pthread_mutex_unlock(&ring_mtx);
    return found;
}

/* Page-cache housekeeping runs on its own thread. Doing it inline stalled the
 * capture loop for ~40 ms every 64 MB (~52 frames), which showed up as periodic
 * latency spikes in the frames.csv latency column -- measured at seq 55, 107,
 * 159, ... on an early run. No frames were lost (the buffer ring absorbed it),
 * but the latency figure is one of the things this recorder exists to measure,
 * so the housekeeping must not contaminate it. */
static int g_out_fd = -1;
static unsigned long long g_written = 0;      /* guarded by g_written_mtx */
static pthread_mutex_t g_written_mtx = PTHREAD_MUTEX_INITIALIZER;

static void *flush_thread(void *arg)
{
    (void)arg;
    unsigned long long done = 0;
    while (!g_stop) {
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 200 * 1000 * 1000 };
        nanosleep(&ts, NULL);
        pthread_mutex_lock(&g_written_mtx);
        unsigned long long w = g_written;
        pthread_mutex_unlock(&g_written_mtx);
        if (w - done < ((unsigned long long)FLUSH_MB << 20)) continue;
        sync_file_range(g_out_fd, (off_t)done, (off_t)(w - done),
                        SYNC_FILE_RANGE_WRITE);
        if (w > ((unsigned long long)KEEP_MB << 20))
            posix_fadvise(g_out_fd, 0,
                          (off_t)(w - ((unsigned long long)KEEP_MB << 20)),
                          POSIX_FADV_DONTNEED);
        done = w;
    }
    return NULL;
}

static int g_tlm_fd = -1;
static FILE *g_tlm_csv = NULL;
static unsigned long g_nang = 0, g_ncrc = 0;
static uint8_t g_sbuf[SERBUF];
static size_t g_slen = 0;

static void drain_tlm(void)
{
    if (g_slen >= sizeof g_sbuf) g_slen = 0;        /* pathological: resync */
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
        g_nang++;
        i += TLM_LEN;
    }
    memmove(g_sbuf, g_sbuf+i, g_slen-i);
    g_slen -= i;
}

static void *tlm_thread(void *arg)
{
    (void)arg;
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
    struct termios tio;
    if (tcgetattr(fd,&tio)<0) { close(fd); return -1; }
    cfmakeraw(&tio);
    cfsetispeed(&tio,baud); cfsetospeed(&tio,baud);
    tio.c_cflag|=CLOCAL|CREAD; tio.c_cflag&=~CRTSCTS;
    tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=0;
    if (tcsetattr(fd,TCSANOW,&tio)<0) { close(fd); return -1; }
    tcflush(fd,TCIFLUSH);
    return fd;
}

static void usage(const char *me)
{
    printf(
"Record raw frames and the IMU/gimbal telemetry against them. Nothing else.\n"
"\n"
"usage: %s [-t SECS] [-n FRAMES] [-D ROOT] [-b BUFS] [-W W] [-H H]\n"
"          [-v VIDEODEV] [-g TLMDEV] [-B BAUD] [-q] [-h]\n"
"\n"
"  -t SECS    stop after SECS seconds (default: until Ctrl-C)\n"
"  -n FRAMES  stop after FRAMES frames\n"
"  -D ROOT    run-folder root (default %s/)\n"
"  -b BUFS    mmap buffers, 2..%d (default %d)\n"
"  -W, -H     capture size (default %dx%d, the only size the core offers)\n"
"  -v DEV     video device   (default %s, else the by-id path)\n"
"  -g DEV     telemetry port (default %s, else the FTDI by-id path)\n"
"  -B BAUD    telemetry baud, 115200 or 921600 (default 921600)\n"
"  -q         no status line\n"
"\n"
"Writes into one timestamped run folder:\n"
"  raw-<W>x<H>-<FOURCC>.gray[.idx|.meta]   frames, index, geometry\n"
"  frames.csv      per frame: both timestamps + the nearest telemetry sample\n"
"  telemetry.csv   every telemetry sample, full ~1010 Hz\n"
"\n"
"Disk cost is ~43 MB/s (~155 GB/hour) for the frames; the telemetry adds 36 kB/s.\n",
        me, REC_ROOT, MAX_NBUF, DEF_NBUF, DEF_W, DEF_H, THERMAL_LINK, TLM_LINK);
}

int main(int argc, char **argv)
{
    const char *vdev_opt=NULL, *gdev_opt=NULL, *root=REC_ROOT;
    unsigned long want_frames=0;
    double want_secs=0;
    unsigned nbuf=DEF_NBUF, W=DEF_W, H=DEF_H;
    speed_t baud=B921600; long baud_n=921600;
    int quiet=0, c;

    while ((c=getopt(argc,argv,"t:n:D:b:W:H:v:g:B:qh"))!=-1) {
        switch (c) {
        case 't': want_secs=atof(optarg); break;
        case 'n': want_frames=strtoul(optarg,NULL,10); break;
        case 'D': root=optarg; break;
        case 'b': nbuf=(unsigned)strtoul(optarg,NULL,10); break;
        case 'W': W=(unsigned)strtoul(optarg,NULL,10); break;
        case 'H': H=(unsigned)strtoul(optarg,NULL,10); break;
        case 'v': vdev_opt=optarg; break;
        case 'g': gdev_opt=optarg; break;
        case 'B':
            baud_n=strtol(optarg,NULL,10);
            if (baud_n==115200) baud=B115200;
            else if (baud_n==921600) baud=B921600;
            else die("-B must be 115200 or 921600 (got %s)", optarg);
            break;
        case 'q': quiet=1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (nbuf<2 || nbuf>MAX_NBUF) die("-b must be 2..%d", MAX_NBUF);

    char *vdev=resolve(vdev_opt,THERMAL_LINK,VID_BYID_GLOB,NULL);
    if (!vdev) die("no Sirius capture node found (looked for %s, then %s)\n"
                   "  is the camera plugged in?  ls -l /dev/v4l/by-id/",
                   THERMAL_LINK, VID_BYID_GLOB);
    char *gdev=resolve(gdev_opt,TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);
    if (!gdev) die("no telemetry port found (looked for %s, then %s, then %s)\n"
                   "  is the FTDI cable plugged in?  ls -l /dev/serial/by-id/",
                   TLM_LINK, TLM_BYID_GLOB, TLM_FALLBACK);

    /* ---- device + format ------------------------------------------------ */
    int vfd=open(vdev,O_RDWR|O_CLOEXEC);
    if (vfd<0) die("cannot open %s: %s\n  in the video group? (id -nG)", vdev, strerror(errno));
    struct v4l2_capability cap;
    memset(&cap,0,sizeof cap);
    if (xioctl(vfd,VIDIOC_QUERYCAP,&cap)==-1)
        die("VIDIOC_QUERYCAP on %s: %s", vdev, strerror(errno));
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        die("%s cannot capture video (caps 0x%08x).\n"
            "  The core's second node is metadata-only -- use -video-index0.",
            vdev, cap.capabilities);
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        die("%s does not support streaming I/O", vdev);

    struct v4l2_format fmt;
    memset(&fmt,0,sizeof fmt);
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width=W; fmt.fmt.pix.height=H;
    fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field=V4L2_FIELD_NONE;
    if (xioctl(vfd,VIDIOC_S_FMT,&fmt)==-1) die("VIDIOC_S_FMT: %s", strerror(errno));
    /* S_FMT is a negotiation, not a command. Accepting a substituted format
     * silently is how a raw file ends up unreadable with nothing to say why. */
    if (fmt.fmt.pix.width!=W || fmt.fmt.pix.height!=H ||
        fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_GREY) {
        char got[5]; fourcc_str(fmt.fmt.pix.pixelformat,got);
        die("driver refused the format.\n  asked %ux%u GREY, got %ux%u %s\n"
            "  run './record_raw -L' to see what this core offers",
            W,H,fmt.fmt.pix.width,fmt.fmt.pix.height,got);
    }
    const uint32_t frame_sz=fmt.fmt.pix.sizeimage;
    const uint32_t stride=fmt.fmt.pix.bytesperline;
    char fcs[5]; fourcc_str(fmt.fmt.pix.pixelformat,fcs);

    struct v4l2_requestbuffers rb;
    memset(&rb,0,sizeof rb);
    rb.count=nbuf; rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory=V4L2_MEMORY_MMAP;
    if (xioctl(vfd,VIDIOC_REQBUFS,&rb)==-1)
        die("VIDIOC_REQBUFS (%u): %s", nbuf, strerror(errno));
    if (rb.count<2) die("driver gave only %u buffers, need >= 2", rb.count);
    if (rb.count!=nbuf) fprintf(stderr,"note: asked %u buffers, got %u\n",nbuf,rb.count);
    nbuf=rb.count;

    void **bufs=calloc(nbuf,sizeof *bufs);
    size_t *blen=calloc(nbuf,sizeof *blen);
    if (!bufs||!blen) die("out of memory");
    for (unsigned i=0;i<nbuf;i++) {
        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP; b.index=i;
        if (xioctl(vfd,VIDIOC_QUERYBUF,&b)==-1) die("VIDIOC_QUERYBUF %u: %s",i,strerror(errno));
        blen[i]=b.length;
        bufs[i]=mmap(NULL,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,vfd,b.m.offset);
        if (bufs[i]==MAP_FAILED) die("mmap buffer %u: %s",i,strerror(errno));
    }

    /* ---- run folder ----------------------------------------------------- */
    char outdir[512], graypath[640], idxpath[700], metapath[700];
    char framespath[700], tlmpath[700];
    {
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
        snprintf(graypath,sizeof graypath,"%s/raw-%ux%u-%s.gray",outdir,W,H,fcs);
        snprintf(idxpath,sizeof idxpath,"%s.idx",graypath);
        snprintf(metapath,sizeof metapath,"%s.meta",graypath);
        snprintf(framespath,sizeof framespath,"%s/frames.csv",outdir);
        snprintf(tlmpath,sizeof tlmpath,"%s/telemetry.csv",outdir);
    }

    /* Space check when the run length is known up front. */
    if (want_frames||want_secs>0) {
        double est=want_frames?(double)want_frames:want_secs*33.0;
        double need=est*frame_sz;
        struct statvfs vfs;
        if (mkdir_p(outdir)==-1) die("cannot create folder %s: %s",outdir,strerror(errno));
        if (statvfs(outdir,&vfs)==0) {
            double avail=(double)vfs.f_bavail*vfs.f_frsize;
            if (need>avail) die("not enough space: need ~%.1f GB, %.1f GB free",need/1e9,avail/1e9);
            if (!quiet) fprintf(stderr,"estimate    %.1f GB of %.1f GB free\n",need/1e9,avail/1e9);
        }
    }
    if (mkdir_p(outdir)==-1) die("cannot create folder %s: %s",outdir,strerror(errno));

    int gfd=open_serial(gdev,baud);
    if (gfd<0) die("telemetry %s: %s\n  in the dialout group? (id -nG)",gdev,strerror(errno));
    g_tlm_fd=gfd;

    int ofd=open(graypath,O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0644);
    if (ofd<0) die("cannot create %s: %s",graypath,strerror(errno));
    FILE *idx=fopen(idxpath,"w");
    if (!idx) die("cannot create %s: %s",idxpath,strerror(errno));
    fprintf(idx,"# frame kernel_seq buf_timestamp_s bytes flags\n");
    FILE *fcsv=fopen(framespath,"w");
    if (!fcsv) die("cannot create %s: %s",framespath,strerror(errno));
    g_tlm_csv=fopen(tlmpath,"w");
    if (!g_tlm_csv) die("cannot create %s: %s",tlmpath,strerror(errno));

    fprintf(fcsv,"# record_sync: one row per frame, with the telemetry sample nearest it\n");
    fprintf(fcsv,"# video=%s telemetry=%s baud=%ld\n",vdev,gdev,baud_n);
    fprintf(fcsv,"# geometry=%ux%u %s  %u B/frame\n",W,H,fcs,frame_sz);
    fprintf(fcsv,"# clock=CLOCK_MONOTONIC for t_first_byte, t_arrival, t_angle\n");
    fprintf(fcsv,"# t_first_byte = kernel stamp, first USB payload of the frame on the host\n");
    fprintf(fcsv,"# t_arrival    = DQBUF handed the finished frame to userspace\n");
    fprintf(fcsv,"# byte_offset  = seek here in the .gray to read this frame\n");
    fprintf(fcsv,"# flags: bit 0x40 = V4L2_BUF_FLAG_ERROR. A frame with that set, or\n");
    fprintf(fcsv,"# with bytes != %u, holds no usable image; its timing cells are left\n",frame_sz);
    fprintf(fcsv,"# empty rather than filled with a figure derived from a zero timestamp.\n");
    fprintf(fcsv,"seq,t_first_byte,t_arrival,latency_ms,byte_offset,bytes,flags,"
                 "yaw,pitch,t_angle,angle_dt_ms,n_ang_interval\n");
    fprintf(g_tlm_csv,"# record_sync telemetry, receive only\n");
    fprintf(g_tlm_csv,"# port=%s baud=%ld\n",gdev,baud_n);
    fprintf(g_tlm_csv,"# t_angle = CLOCK_MONOTONIC seconds at arrival\n");
    fprintf(g_tlm_csv,"# float[0]=yaw float[1]=pitch degrees; f2..f7 undocumented\n");
    fprintf(g_tlm_csv,"t_angle,yaw,pitch,f2,f3,f4,f5,f6,f7\n");

    struct sigaction sa;
    memset(&sa,0,sizeof sa);
    sa.sa_handler=on_signal;
    sigaction(SIGINT,&sa,NULL); sigaction(SIGTERM,&sa,NULL);

    for (unsigned i=0;i<nbuf;i++) {
        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP; b.index=i;
        if (xioctl(vfd,VIDIOC_QBUF,&b)==-1) die("VIDIOC_QBUF %u: %s",i,strerror(errno));
    }
    enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(vfd,VIDIOC_STREAMON,&type)==-1)
        die("VIDIOC_STREAMON: %s\n  another process may hold %s"
            " (the core allows one streamer)",strerror(errno),vdev);

    g_out_fd=ofd;
    pthread_t tid, fid;
    if (pthread_create(&tid,NULL,tlm_thread,NULL)!=0)
        die("cannot start telemetry thread: %s",strerror(errno));
    if (pthread_create(&fid,NULL,flush_thread,NULL)!=0)
        die("cannot start flush thread: %s",strerror(errno));

    if (!quiet) {
        fprintf(stderr,"video       %s\n",vdev);
        fprintf(stderr,"format      %ux%u %s  %u B/frame  %u B/line (uncompressed)\n",
                W,H,fcs,frame_sz,stride);
        fprintf(stderr,"telemetry   %s @ %ld, receive only\n",gdev,baud_n);
        fprintf(stderr,"buffers     %u mmap (%.1f MB ring)\n",nbuf,(double)nbuf*frame_sz/1e6);
        fprintf(stderr,"folder      %s/\n",outdir);
        fprintf(stderr,"recording   Ctrl-C to stop\n");
    }

    /* ---- capture loop --------------------------------------------------- */
    const double t0=now_mono();
    double t_status=0, prev_fb=0;
    unsigned long frames=0, dropped=0, seq_anom=0, err_frames=0, short_frames=0,
                  timeouts=0, nomatch=0, n_usable=0;
    unsigned long long bytes=0;
    unsigned prev_ring=0;
    uint32_t last_seq=0;
    int have_seq=0, clock_checked=0;
    double sum_lat=0, min_lat=1e9, max_lat=0, sum_adt=0, worst_adt=0;
    double gap_min=1e9, gap_max=0;
    int isatty_err=isatty(STDERR_FILENO);
    const char *why="signal";

    while (!g_stop) {
        if (want_frames && frames>=want_frames) { why="frame count"; break; }
        if (want_secs>0 && now_mono()-t0>=want_secs) { why="duration"; break; }

        struct pollfd pfd={.fd=vfd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,2000);
        if (pr==-1) { if (errno==EINTR) continue; die("poll: %s",strerror(errno)); }
        if (pr==0) {
            timeouts++;
            fprintf(stderr,"\nwarning: no frame for 2s (timeout %lu)\n",timeouts);
            if (timeouts>=3) { why="device stopped delivering frames"; break; }
            continue;
        }

        struct v4l2_buffer b;
        memset(&b,0,sizeof b);
        b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if (xioctl(vfd,VIDIOC_DQBUF,&b)==-1) {
            if (errno==EAGAIN) continue;
            die("VIDIOC_DQBUF: %s",strerror(errno));
        }
        /* First thing, before any work: this is the arrival instant. */
        double t_arr=now_mono();
        /* VIDIOC_QBUF overwrites this struct, so snapshot everything now. */
        uint32_t fseq=b.sequence, fflags=b.flags, fbytes=b.bytesused;
        unsigned fidx=b.index;
        double t_fb=b.timestamp.tv_sec + b.timestamp.tv_usec/1e6;
        timeouts=0;

        if (!clock_checked) {
            clock_checked=1;
            if ((fflags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                die("video timestamps are not CLOCK_MONOTONIC (flags 0x%08x)"
                    " -- pairing with the telemetry would be meaningless",fflags);
        }

        /* Sequence gaps mean frames were lost upstream. Guard the arithmetic:
         * sequences can repeat when a stream never really starts, and unsigned
         * subtraction there wraps to ~4.29e9. */
        if (have_seq) {
            if (fseq>last_seq+1) dropped += fseq-last_seq-1;
            else if (fseq<=last_seq) seq_anom++;
        }
        last_seq=fseq; have_seq=1;

        if (fflags & V4L2_BUF_FLAG_ERROR) err_frames++;
        if (fbytes != frame_sz) short_frames++;
        /* The stream-start transient returns a few buffers flagged ERROR with
         * bytesused 0, and most of those carry timestamp 0. A latency computed
         * against a zero timestamp is not a large latency, it is meaningless --
         * letting it into the summary produced a reported max of 5598232 ms. So
         * such frames are still written and indexed, but excluded from every
         * timing statistic. */
        int usable = !(fflags & V4L2_BUF_FLAG_ERROR)
                     && fbytes == frame_sz && t_fb > 0.0;
        if (usable) n_usable++;

        off_t this_off=(off_t)bytes;
        /* Exactly bytesused bytes, nothing added, nothing dropped. */
        if (fbytes && write_all(ofd,bufs[fidx],fbytes)==-1)
            die("write to %s: %s",graypath,strerror(errno));
        bytes+=fbytes;
        frames++;

        if (xioctl(vfd,VIDIOC_QBUF,&b)==-1)
            die("VIDIOC_QBUF %u: %s",fidx,strerror(errno));

        /* pair the frame with the nearest telemetry sample -- only meaningful if
         * the frame has a real timestamp to pair against */
        struct angle a; unsigned ring_total=0;
        int have_a = usable ? ring_nearest(t_fb,&a,&ring_total) : 0;
        if (!usable) { pthread_mutex_lock(&ring_mtx); ring_total=ring_n;
                       pthread_mutex_unlock(&ring_mtx); }
        double adt=0;
        if (have_a) {
            adt=(t_fb-a.t)*1000.0;
            double ad=adt<0?-adt:adt;
            sum_adt+=ad; if (ad>worst_adt) worst_adt=ad;
        } else if (usable) nomatch++;
        unsigned n_int=ring_total-prev_ring;
        prev_ring=ring_total;

        double lat=(t_arr-t_fb)*1000.0;
        if (usable) {
            sum_lat+=lat; if (lat>max_lat) max_lat=lat; if (lat<min_lat) min_lat=lat;
            if (prev_fb>0) {
                double g=(t_fb-prev_fb)*1000.0;
                if (g<gap_min) gap_min=g;
                if (g>gap_max) gap_max=g;
            }
            prev_fb=t_fb;
        }

        fprintf(idx,"%lu %u %.6f %u 0x%08x\n",frames-1,fseq,t_fb,fbytes,fflags);
        if (usable)
            fprintf(fcsv,"%u,%.6f,%.6f,%.3f,%lld,%u,0x%08x,",
                    fseq,t_fb,t_arr,lat,(long long)this_off,fbytes,fflags);
        else
            fprintf(fcsv,"%u,,%.6f,,%lld,%u,0x%08x,",
                    fseq,t_arr,(long long)this_off,fbytes,fflags);
        if (have_a) fprintf(fcsv,"%.4f,%.4f,%.6f,%.3f,%u\n",a.yaw,a.pitch,a.t,adt,n_int);
        else        fprintf(fcsv,",,,,%u\n",n_int);

        /* Publish the write offset for the flusher thread; the syscalls themselves
         * happen there, off this loop. */
        pthread_mutex_lock(&g_written_mtx);
        g_written = bytes;
        pthread_mutex_unlock(&g_written_mtx);

        double now=now_mono();
        if (!quiet && now-t_status>=1.0/STATUS_HZ) {
            t_status=now;
            double el=now-t0;
            fprintf(stderr,"\r  %6.1fs %7lu fr %6.2ffps %7.2fGB %5.1fMB/s | "
                           "imu %6.1fHz %7lu | lat %5.1fms | drop %lu%s",
                    el,frames,el>0?frames/el:0.0,bytes/1e9,el>0?bytes/el/1e6:0.0,
                    el>0?g_nang/el:0.0,g_nang,frames?sum_lat/frames:0.0,dropped,
                    isatty_err?"  ":"\n");
            fflush(stderr);
        }
    }

    double el=now_mono()-t0;
    g_stop=1;
    pthread_join(tid,NULL);
    pthread_join(fid,NULL);
    if (xioctl(vfd,VIDIOC_STREAMOFF,&type)==-1)
        fprintf(stderr,"\nwarning: VIDIOC_STREAMOFF: %s\n",strerror(errno));
    for (unsigned i=0;i<nbuf;i++) munmap(bufs[i],blen[i]);
    free(bufs); free(blen);
    close(vfd); close(gfd);
    fclose(idx); fclose(fcsv);
    if (g_tlm_csv) fclose(g_tlm_csv);
    if (fdatasync(ofd)==-1) fprintf(stderr,"\nwarning: fdatasync: %s\n",strerror(errno));
    close(ofd);
    if (!quiet) fputc('\n',stderr);

    double mean=el>0?frames/el:0.0;
    FILE *m=fopen(metapath,"w");
    if (m) {
        fprintf(m,
            "# raw capture by record_sync.c -- no compression, no conversion\n"
            "file            %s\ndevice          %s\ncard            %s\n"
            "driver          %s\ntelemetry_port  %s\ntelemetry_baud  %ld\n"
            "width           %u\nheight          %u\nfourcc          %s\n"
            "bytes_per_line  %u\nbytes_per_frame %u\nframes          %lu\n"
            "bytes           %llu\nduration_s      %.3f\nmean_fps        %.4f\n"
            "dropped_frames  %lu\nerror_frames    %lu\nshort_frames    %lu\n"
            "seq_anomalies   %lu\ntelemetry_samples %lu\ntelemetry_crc_err %lu\n"
            "finished_unix   %ld\n"
            "layout          frames stored back to back, no header, no padding\n",
            graypath,vdev,cap.card,cap.driver,gdev,baud_n,W,H,fcs,stride,frame_sz,
            frames,bytes,el,mean,dropped,err_frames,short_frames,seq_anom,
            g_nang,g_ncrc,(long)time(NULL));
        fclose(m);
    }

    fprintf(stderr,"stopped     %s\n",g_stop&&!want_secs&&!want_frames?"signal":why);
    fprintf(stderr,"frames      %lu in %.2f s  =  %.2f fps  (interval %.1f..%.1f ms)\n",
            frames,el,mean,n_usable>1?gap_min:0.0,gap_max);
    fprintf(stderr,"written     %llu bytes (%.2f GB) at %.1f MB/s\n",
            bytes,bytes/1e9,el>0?bytes/el/1e6:0.0);
    fprintf(stderr,"telemetry   %lu samples (%.1f Hz)  crc errors %lu\n",
            g_nang,el>0?g_nang/el:0.0,g_ncrc);
    if (n_usable) {
        fprintf(stderr,"latency     first byte -> usable: mean %.2f  min %.2f  max %.2f ms"
                       "  (over %lu valid frames)\n",
                sum_lat/n_usable,min_lat,max_lat,n_usable);
        fprintf(stderr,"pairing     frame <-> telemetry: mean |dt| %.3f ms, worst %.3f, unmatched %lu\n",
                sum_adt/(double)(n_usable-nomatch?n_usable-nomatch:1),worst_adt,nomatch);
    }
    if (frames-n_usable)
        fprintf(stderr,"excluded    %lu frame(s) had no usable image (error/short/no"
                       " timestamp) -- written and indexed, but kept out of the stats\n",
                frames-n_usable);
    if (dropped)
        fprintf(stderr,"DROPPED     %lu frame(s) lost upstream -- %.2f%% of %lu produced\n",
                dropped,100.0*dropped/(double)(frames+dropped),frames+dropped);
    else
        fprintf(stderr,"dropped     0 -- kernel sequence numbers are contiguous\n");
    if (err_frames)
        fprintf(stderr,"ERRORS      %lu frame(s) flagged V4L2_BUF_FLAG_ERROR"
                       " (content unreliable, still written)\n",err_frames);
    if (short_frames)
        fprintf(stderr,"SHORT       %lu frame(s) were not %u bytes -- see the .idx\n",
                short_frames,frame_sz);
    if (seq_anom)
        fprintf(stderr,"SEQUENCE    %lu frame(s) did not advance the sequence"
                       " -- stream start transient\n",seq_anom);
    fprintf(stderr,"folder      %s/\n",outdir);
    fprintf(stderr,"  frames    %s\n  index     %s\n  meta      %s\n"
                   "  per-frame %s\n  telemetry %s\n",
            graypath,idxpath,metapath,framespath,tlmpath);
    fprintf(stderr,"\nplay it back:\n"
                   "  ffplay -f rawvideo -pixel_format gray -video_size %ux%u"
                   " -framerate %.2f %s\n",W,H,mean>0?mean:25.0,graypath);

    int bad=(frames==0||bytes==0||g_nang==0);
    if (bad) fprintf(stderr,"\nFAILED      nothing usable (%lu frames, %llu bytes,"
                            " %lu telemetry samples)\n",frames,bytes,g_nang);
    free(vdev); free(gdev);
    return bad?1:0;
}
