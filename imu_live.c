/* imu_live.c -- live yaw/pitch and, more usefully, live ANGULAR RATE with a
 * verdict on whether you are panning at a speed the tracker can follow.
 *
 *   ./imu_live
 *
 * The raw angles are the easy part. The number that decides whether a recording
 * will be usable is the RATE, because it sets how far the scene moves between
 * frames:
 *
 *     px_per_frame = rate_deg_s x px_per_deg / fps
 *
 * Too slow and the motion drowns in centroid noise; too fast and the tracker
 * matches the wrong feature -- and on this rig it does that while still
 * reporting high quality, which is the failure that made every earlier
 * jerk-based latency estimate irreproducible. So this shows px/frame next to the
 * rate and names the band you are in, which is what you actually steer by.
 *
 * Defaults are this rig's measured values: 27 px/deg (46.8 deg HFOV, fitted from
 * two independent runs) and 25 fps. Override with -k and -f.
 *
 * DO NOT run this while blob_log is recording. Both open the same tty, and every
 * byte one process reads is a byte the other never sees, so the two corrupt each
 * other's frame sync. This program watches for that and says so if it sees the
 * signature: plenty of bytes arriving but few frames passing CRC.
 *
 * Receive only -- the port is opened O_RDONLY and nothing is ever transmitted to
 * the gimbal.
 *
 * Build:  gcc -O2 -Wall -Wextra -o imu_live imu_live.c -lm
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define TLM_LEN    36
#define TLM_FLOATS 8
#define SERBUF     8192
#define HIST       4096          /* ~4 s at 1 kHz */

static const char *TLM_LINK      = "/dev/local_dds";
static const char *TLM_BYID_GLOB = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0";
static const char *TLM_FALLBACK  = "/dev/ttyUSB0";

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap,fmt);
    fputc('\n',stderr); vfprintf(stderr,fmt,ap); va_end(ap); fputc('\n',stderr);
    exit(1);
}

static double now_mono(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return t.tv_sec + t.tv_nsec/1e9;
}

static double wrap180(double d)
{
    while (d>180.0) d-=360.0;
    while (d<-180.0) d+=360.0;
    return d;
}

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t c=0xFFFF;
    for (size_t i=0;i<n;i++) { c^=d[i]; for (int b=0;b<8;b++) c=(c&1)?(c>>1)^0x8408:c>>1; }
    return c;
}

static char *resolve(const char *ex)
{
    if (ex) return strdup(ex);
    if (access(TLM_LINK,F_OK)==0) return strdup(TLM_LINK);
    glob_t g;
    if (glob(TLM_BYID_GLOB,0,NULL,&g)==0 && g.gl_pathc>0) {
        char *p=strdup(g.gl_pathv[0]); globfree(&g); return p;
    }
    globfree(&g);
    if (access(TLM_FALLBACK,F_OK)==0) return strdup(TLM_FALLBACK);
    return NULL;
}

static int open_serial(const char *dev, speed_t baud)
{
    /* O_RDONLY: never transmit to the gimbal. */
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

/* ---- history, for rate and excursion --------------------------------- */

struct samp { double t, yaw, pitch; };
static struct samp hist[HIST];
static unsigned hn=0;

static void push(double t, double y, double p)
{
    hist[hn % HIST].t=t; hist[hn % HIST].yaw=y; hist[hn % HIST].pitch=p;
    hn++;
}

/* Rate over the trailing `win` seconds. Live, so this is a backward difference:
 * it lags the true rate by about win/2, which is irrelevant for a human steering
 * a pan and keeps the reading steady. wrap180 on the difference is essential --
 * this rig's pitch reads -180..+180 and rolls over mid-swing. */
static int rate_now(double win, double *ry, double *rp)
{
    if (hn<2) return 0;
    unsigned have = hn<HIST?hn:HIST;
    const struct samp *cur=&hist[(hn-1)%HIST];
    const struct samp *old=NULL;
    for (unsigned k=1;k<have;k++) {
        const struct samp *s=&hist[(hn-1-k)%HIST];
        old=s;
        if (cur->t - s->t >= win) break;
    }
    if (!old) return 0;
    double dt=cur->t-old->t;
    if (dt<=1e-6) return 0;
    *ry=wrap180(cur->yaw-old->yaw)/dt;
    *rp=wrap180(cur->pitch-old->pitch)/dt;
    return 1;
}

/* Peak |rate| over the trailing `win` seconds, so a swing's peak stays visible
 * long enough to read it off a scrolling line. */
static void peak_over(double win, double dwin, double *py, double *pp)
{
    *py=*pp=0;
    if (hn<3) return;
    unsigned have = hn<HIST?hn:HIST;
    double tnow=hist[(hn-1)%HIST].t;
    for (unsigned k=0;k+1<have;k++) {
        const struct samp *a=&hist[(hn-1-k)%HIST];
        if (tnow-a->t > win) break;
        /* find a partner dwin earlier for a stable local difference */
        for (unsigned j=k+1;j<have;j++) {
            const struct samp *b=&hist[(hn-1-j)%HIST];
            if (a->t-b->t < dwin) continue;
            double dt=a->t-b->t;
            if (dt>1e-6) {
                double y=fabs(wrap180(a->yaw-b->yaw)/dt);
                double p=fabs(wrap180(a->pitch-b->pitch)/dt);
                if (y>*py) *py=y;
                if (p>*pp) *pp=p;
            }
            break;
        }
    }
}

/* ---- band verdict ---------------------------------------------------- */

/* Bands in px/frame, which is the quantity that actually matters. The upper
 * limit is where consecutive frames stop overlapping enough to track; the lower
 * is where the signal stops beating centroid noise. */
static const char *band(double pxf, const char **colour)
{
    if (pxf < 11.0)  { *colour="\033[90m"; return "too slow";    }
    if (pxf < 25.0)  { *colour="\033[36m"; return "ok";          }
    if (pxf <= 45.0) { *colour="\033[32m"; return "IDEAL";       }
    if (pxf <= 54.0) { *colour="\033[36m"; return "ok";          }
    if (pxf <= 150.0){ *colour="\033[33m"; return "too fast";    }
    { *colour="\033[31m"; return "UNTRACKABLE"; }
}

static void usage(const char *me)
{
    printf(
"Live yaw/pitch and angular rate, with a verdict on whether the pan speed is\n"
"one the blob tracker can follow.\n"
"\n"
"usage: %s [-g PORT] [-B BAUD] [-w MS] [-k PXDEG] [-f FPS] [-a] [-p] [-h]\n"
"\n"
"  -g PORT   serial port override (default: /dev/local_dds, then by-id)\n"
"  -B BAUD   115200 or 921600           (default 921600)\n"
"  -w MS     rate window, ms            (default 30, matching det_latency -w)\n"
"  -k PXDEG  pixels per degree          (default 27.0, measured on this rig)\n"
"  -f FPS    camera frame rate          (default 25)\n"
"  -a        also show all 8 telemetry floats\n"
"  -p        plain output: one line per update, no cursor tricks (for logs)\n"
"\n"
"Aim for the IDEAL band while panning for a det_latency recording. Do NOT run\n"
"this at the same time as blob_log: both read the same tty and each steals the\n"
"other's bytes.\n", me);
}

int main(int argc, char **argv)
{
    const char *port_opt=NULL;
    double win_ms=30.0, pxdeg=27.0, fps=25.0;
    int show_all=0, plain=0, c;
    long baud_n=921600; speed_t baud=B921600;

    while ((c=getopt(argc,argv,"g:B:w:k:f:aph"))!=-1) {
        switch (c) {
        case 'g': port_opt=optarg; break;
        case 'w': win_ms=atof(optarg); break;
        case 'k': pxdeg=atof(optarg); break;
        case 'f': fps=atof(optarg); break;
        case 'a': show_all=1; break;
        case 'p': plain=1; break;
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
    if (win_ms<1||win_ms>1000) die("-w must be 1..1000 ms");
    if (pxdeg<=0) die("-k must be > 0");
    if (fps<=0) die("-f must be > 0");

    char *port=resolve(port_opt);
    if (!port) die("no gimbal port found (looked for %s, then %s, then %s)\n"
                   "  is the USB-TTL converter plugged in?  ls -l /dev/serial/by-id/",
                   TLM_LINK,TLM_BYID_GLOB,TLM_FALLBACK);

    int fd=open_serial(port,baud);
    if (fd<0) die("cannot open %s: %s\n  in the dialout group? (id -nG)",
                  port,strerror(errno));

    struct sigaction sa;
    memset(&sa,0,sizeof sa);
    sa.sa_handler=on_signal;
    sigaction(SIGINT,&sa,NULL); sigaction(SIGTERM,&sa,NULL);

    fprintf(stderr,"imu_live  %s @ %ld, receive only\n",port,baud_n);
    fprintf(stderr,"rate window %.0f ms   scale %.1f px/deg @ %.0f fps"
                   "  -> %.2f px/frame per deg/s\n",
            win_ms,pxdeg,fps,pxdeg/fps);
    fprintf(stderr,"target band 11-54 px/frame, best 25-45.  Ctrl-C to stop\n\n");

    uint8_t buf[SERBUF];
    size_t blen=0;
    unsigned long nframe=0, ncrc=0, nresync=0, nbytes=0;
    double t0=now_mono(), tprint=0;
    double ex_ymin=0, ex_ymax=0, ex_pmin=0, ex_pmax=0, ex_yacc=0, ex_pacc=0;
    int have_prev=0; double prev_y=0, prev_p=0;
    float last[TLM_FLOATS]; memset(last,0,sizeof last);
    int warned_reader=0, waiting=0;
    int tty=isatty(STDERR_FILENO) && !plain;

    while (!g_stop) {
        struct pollfd pfd={.fd=fd,.events=POLLIN,.revents=0};
        int pr=poll(&pfd,1,100);
        if (pr<0) { if (errno==EINTR) continue; break; }
        if (pr>0 && (pfd.revents&POLLIN)) {
            if (blen>=sizeof buf) blen=0;
            ssize_t n=read(fd,buf+blen,sizeof buf-blen);
            if (n>0) {
                nbytes+=(size_t)n;
                blen+=(size_t)n;
                double t=now_mono();
                size_t i=0;
                while (blen-i>=TLM_LEN) {
                    if (buf[i]!=0xA5||buf[i+1]!=0x5A) { i++; nresync++; continue; }
                    uint16_t want=(uint16_t)(buf[i+34]|(buf[i+35]<<8));
                    if (crc16(&buf[i+2],32)!=want) { ncrc++; i++; continue; }
                    memcpy(last,&buf[i+2],sizeof last);
                    double y=last[0], p=last[1];
                    push(t,y,p);
                    if (have_prev) {
                        ex_yacc+=wrap180(y-prev_y);
                        ex_pacc+=wrap180(p-prev_p);
                        if (ex_yacc<ex_ymin) ex_ymin=ex_yacc;
                        if (ex_yacc>ex_ymax) ex_ymax=ex_yacc;
                        if (ex_pacc<ex_pmin) ex_pmin=ex_pacc;
                        if (ex_pacc>ex_pmax) ex_pmax=ex_pacc;
                    }
                    prev_y=y; prev_p=p; have_prev=1;
                    nframe++;
                    i+=TLM_LEN;
                }
                memmove(buf,buf+i,blen-i);
                blen-=i;
            }
        }

        double now=now_mono();
        if (now-tprint < 0.05) continue;
        tprint=now;
        double el=now-t0;

        /* Bytes flowing but frames not passing CRC is the signature of a second
         * reader on the same tty stealing every other chunk. Say so once. */
        if (!warned_reader && el>3.0 && nframe>500) {
            double hz=nframe/el;
            double resync_frac = nbytes? (double)nresync/nbytes : 0.0;
            /* The gimbal sends a steady ~1010 Hz. A second reader on the same tty
             * steals whole chunks, so the rate collapses and the leftovers arrive
             * mid-frame -- rate and resync fraction both move, and either alone
             * is enough to say something is wrong. */
            if (hz < 700.0 || resync_frac > 0.10) {
                fprintf(stderr,"\nwarning: %.0f Hz (expected ~1010) with %lu resync"
                               " bytes of %lu (%.0f%%), %lu crc errors.\n"
                               "  Another process is probably reading %s -- is"
                               " blob_log running?\n\n",
                        hz,nresync,nbytes,100.0*resync_frac,ncrc,port);
                warned_reader=1;
            }
        }

        if (!nframe) {
            fprintf(stderr,"\r  waiting for telemetry...  %.0f s, %lu bytes,"
                           " %lu resync   ",el,nbytes,nresync);
            fflush(stderr);
            waiting=1;
            continue;
        }
        /* close off the waiting line so the first data line starts clean */
        if (waiting) { fputc('\n',stderr); waiting=0; }

        double ry=0,rp=0;
        rate_now(win_ms/1000.0,&ry,&rp);
        double pky=0,pkp=0;
        peak_over(3.0,win_ms/1000.0,&pky,&pkp);

        /* the dominant axis is what the tracker will see */
        double dom = fabs(ry)>fabs(rp)?fabs(ry):fabs(rp);
        double pxf = dom*pxdeg/fps;
        double pk  = pky>pkp?pky:pkp;
        double pxf_pk = pk*pxdeg/fps;
        const char *col; const char *verdict=band(pxf,&col);
        const char *colp; const char *vpk=band(pxf_pk,&colp);
        if (!tty) { col=""; colp=""; }
        const char *rst = tty?"\033[0m":"";

        fprintf(stderr,
            "\r yaw %+8.2f  %+7.1f dps | pitch %+8.2f  %+7.1f dps"
            " | now %s%5.1f px/fr %-11s%s | peak3s %s%5.1f px/fr %-11s%s"
            " | %.0fHz crc %lu  ",
            last[0],ry,last[1],rp,
            col,pxf,verdict,rst, colp,pxf_pk,vpk,rst,
            el>0?nframe/el:0.0,ncrc);
        if (plain) fputc('\n',stderr);
        fflush(stderr);
    }

    double el=now_mono()-t0;
    fputc('\n',stderr);
    fprintf(stderr,"\n-- summary --\n");
    fprintf(stderr,"listened         %.1f s at %ld baud, receive only\n",el,baud_n);
    fprintf(stderr,"frames           %lu valid  (%.1f Hz)   crc %lu   resync %lu\n",
            nframe, el>0?nframe/el:0.0, ncrc, nresync);
    if (nframe) {
        fprintf(stderr,"yaw              last %+.3f deg   excursion %.2f deg\n",
                last[0], ex_ymax-ex_ymin);
        fprintf(stderr,"pitch            last %+.3f deg   excursion %.2f deg\n",
                last[1], ex_pmax-ex_pmin);
        fprintf(stderr,"                 (excursion is unwrapped, so it is real motion\n"
                       "                  even across the +-180 rollover)\n");
        if (show_all) {
            fprintf(stderr,"all 8 floats     ");
            for (int i=0;i<TLM_FLOATS;i++) fprintf(stderr,"%.4f%s",(double)last[i],
                                                   i<TLM_FLOATS-1?", ":"\n");
        }
    }
    close(fd); free(port);
    return nframe?0:1;
}
