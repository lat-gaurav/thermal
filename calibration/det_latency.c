/* det_latency.c -- how long after the world moves does the detection show it?
 *
 *   ./det_latency -d recordings/2026-08-31/191500        (a blob_log run)
 *   ./det_latency -d recordings/... -v                   (show the loss curve)
 *
 * THE METHOD. The gimbal IMU is ground truth: it reports its own angle at ~1 kHz
 * on the same CLOCK_MONOTONIC as the frames, and that report is effectively
 * instantaneous next to the video path. The camera is rigidly bolted to the
 * gimbal, so for a world-fixed scene the image MUST move as a fixed linear
 * function of gimbal angle. Sweep a time shift tau backwards, fit that relation
 * at each tau, and the tau that minimises the residual is the latency.
 *
 * WHY A JOINT 2x2 FIT, not two separate correlations. The mount is rigid, so
 * image x and y are not independent observations of two unrelated things -- they
 * are one rotation. This fits the whole map
 *
 *     [ vx ]   [ b00  b01 ] [ yaw_rate   ]   [ a0 ]
 *     [ vy ] = [ b10  b11 ] [ pitch_rate ] + [ a1 ]
 *
 * at every candidate tau, and the loss is the fraction of image motion that map
 * leaves unexplained:
 *
 *     loss(tau) = (SSres_x + SSres_y) / (SStot_x + SStot_y)  =  1 - R^2
 *
 * Fitting both axes together uses all the motion instead of half of it, and it
 * tolerates any roll misalignment between camera and gimbal axes -- a per-axis
 * correlation silently assumes there is none.
 *
 * THE FREE PHYSICAL CHECK, which is the real reason for the joint form. A rigid
 * mount can only produce B = scale x rotation, and that forces b00 = b11 and
 * b01 = -b10. Nothing in the fit imposes it, so how nearly it holds is an
 * independent test that the minimum is real. Likewise sqrt(|det B|) must come out
 * at the lens's pixels-per-degree -- a property of the optics, not of the fit.
 * Measured on this rig at 27.3 px/deg (46.8 deg HFOV) from two independent runs;
 * an earlier assumed 18.6 was simply wrong. A deep minimum with a non-conformal
 * matrix or an absurd scale is coincidence, and this program says so instead of
 * reporting the number.
 *
 * RATES, NOT POSITIONS. Both signals carry slow drift: thermal drift in the
 * centroid, bias in the gimbal, and the target's own motion if it is not
 * world-fixed. Drift is large and shared, so fitting positions would let it
 * dominate and drag the minimum toward tau = 0. Differentiating removes it.
 *
 * REPRODUCIBILITY IS REPORTED, NOT ASSUMED. The record is also split into blocks
 * and fitted independently, so the answer arrives with a spread rather than as a
 * bare number. Earlier onset-based attempts on this rig gave 1.91, 3.05, 5.86,
 * 5.84 and 2.72 frames -- a between-run spread far larger than the within-run
 * scatter, which is exactly what a single number hides.
 *
 * MOTION MUST BE TRACKABLE. Too little and the fit sees only noise; too much and
 * the tracker matches the wrong feature -- on this rig it does that while still
 * reporting high quality, which puts confident wrong numbers into the fit. Both
 * failure modes are detected and named before any answer is printed.
 *
 * Build:  gcc -O2 -Wall -Wextra -o det_latency det_latency.c -lm
 */
#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAXCOL   64
#define LINEMAX  8192
#define MAXCURVE 80

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap,fmt);
    fputs("error: ",stderr); vfprintf(stderr,fmt,ap); va_end(ap); fputc('\n',stderr);
    exit(1);
}

/* ===================== generic CSV ====================================== */

struct csv {
    char  *name[MAXCOL];
    int    ncol;
    char **cell;
    long   nrow, cap;
};

static int split(char *line, char **out, int max)
{
    int n=0; char *p=line;
    while (n<max) {
        out[n++]=p;
        char *q=strchr(p,',');
        if (!q) break;
        *q='\0'; p=q+1;
    }
    return n;
}

/* A CR left on the last field makes every numeric parse of that column silently
 * return 0, so strip both line endings. */
static void chomp(char *s)
{
    size_t n=strlen(s);
    while (n && (s[n-1]=='\n'||s[n-1]=='\r')) s[--n]='\0';
}

static void csv_read(const char *path, struct csv *c)
{
    FILE *f=fopen(path,"r");
    if (!f) die("cannot open %s: %s",path,strerror(errno));
    memset(c,0,sizeof *c);
    char line[LINEMAX], *fld[MAXCOL];

    while (fgets(line,sizeof line,f)) {          /* header = first non-# line */
        if (line[0]=='#') continue;
        chomp(line);
        c->ncol=split(line,fld,MAXCOL);
        for (int i=0;i<c->ncol;i++) c->name[i]=strdup(fld[i]);
        break;
    }
    if (!c->ncol) die("%s: no header row found",path);

    c->cap=4096;
    c->cell=malloc(sizeof(char*)*(size_t)c->cap*c->ncol);
    if (!c->cell) die("out of memory");
    while (fgets(line,sizeof line,f)) {
        if (line[0]=='#') continue;
        chomp(line);
        if (!line[0]) continue;
        if (c->nrow==c->cap) {
            c->cap*=2;
            c->cell=realloc(c->cell,sizeof(char*)*(size_t)c->cap*c->ncol);
            if (!c->cell) die("out of memory");
        }
        int n=split(line,fld,MAXCOL);
        for (int i=0;i<c->ncol;i++)
            c->cell[c->nrow*c->ncol+i]=strdup(i<n?fld[i]:"");
        c->nrow++;
    }
    fclose(f);
    if (!c->nrow) die("%s: no data rows",path);
}

/* Columns are looked up BY NAME, never by position: these CSVs have gained
 * columns twice already, and an index would have quietly read the wrong one. */
static int col(const struct csv *c, const char *want)
{
    for (int i=0;i<c->ncol;i++) if (strcmp(c->name[i],want)==0) return i;
    return -1;
}
static int col_req(const struct csv *c, const char *want, const char *path)
{
    int i=col(c,want);
    if (i<0) die("%s has no column '%s'",path,want);
    return i;
}
static const char *cell(const struct csv *c, long r, int i)
{
    return c->cell[r*c->ncol+i];
}
static int empty(const struct csv *c, long r, int i)
{
    const char *s=cell(c,r,i);
    return !s || !s[0];
}

/* ===================== series =========================================== */

static double wrap180(double d)
{
    while (d>180.0) d-=360.0;
    while (d<-180.0) d+=360.0;
    return d;
}

struct series { double *t, *v; long n; };

static void ser_alloc(struct series *s, long n)
{
    if (n<1) n=1;
    s->t=malloc(sizeof(double)*(size_t)n);
    s->v=malloc(sizeof(double)*(size_t)n);
    if (!s->t||!s->v) die("out of memory");
    s->n=0;
}

/* Linear interpolation; outside the covered span it reports failure rather than
 * clamping. Clamping would invent a constant rate at the ends and bias every lag
 * whose shift reaches past the data. */
static double ser_at(const struct series *s, double t, int *ok)
{
    *ok=0;
    if (s->n<2 || t<s->t[0] || t>s->t[s->n-1]) return 0;
    long lo=0, hi=s->n-1;
    while (hi-lo>1) { long m=(lo+hi)/2; if (s->t[m]<=t) lo=m; else hi=m; }
    double dt=s->t[hi]-s->t[lo];
    *ok=1;
    if (dt<=0) return s->v[lo];
    return s->v[lo]+((t-s->t[lo])/dt)*(s->v[hi]-s->v[lo]);
}

/* Central-difference rate of an angle series over +-win seconds. The smoothing
 * matters: differentiating 1 kHz angle data raw amplifies its quantisation into
 * something that correlates with nothing. */
static void rate_of(const struct series *ang, struct series *out, double win)
{
    ser_alloc(out,ang->n);
    for (long i=0;i<ang->n;i++) {
        long a=i,b=i;
        while (a>0        && ang->t[i]-ang->t[a] < win) a--;
        while (b<ang->n-1 && ang->t[b]-ang->t[i] < win) b++;
        double dt=ang->t[b]-ang->t[a];
        if (dt<=0) continue;
        out->t[out->n]=ang->t[i];
        out->v[out->n]=wrap180(ang->v[b]-ang->v[a])/dt;      /* deg/s */
        out->n++;
    }
}

/* True excursion of an ANGLE series. A raw min-max is wrong across the +-180
 * wrap: this rig's pitch reads -180..+180 and a plain range called that 359.98
 * deg of motion when the gimbal actually swept 45. Accumulate wrapped deltas
 * instead, which is also what the rate calculation does. */
static double range_of(const struct series *s)
{
    if (s->n<2) return 0;
    double acc=0,mn=0,mx=0;
    for (long i=1;i<s->n;i++) {
        acc+=wrap180(s->v[i]-s->v[i-1]);
        if (acc<mn) mn=acc;
        if (acc>mx) mx=acc;
    }
    return mx-mn;
}

/* Spread of a rate series -- how hard this axis was actually excited. Measured
 * on the SMOOTHED rates the fit uses, not raw differences: differentiating 1 kHz
 * angle data raw turns 0.05 deg of jitter into 50 deg/s of phantom rate, which
 * would make a motionless axis look well excited. */
static double sd_of(const struct series *s)
{
    if (s->n<2) return 0;
    double m=0;
    for (long i=0;i<s->n;i++) m+=s->v[i];
    m/=s->n;
    double v=0;
    for (long i=0;i<s->n;i++) v+=(s->v[i]-m)*(s->v[i]-m);
    return sqrt(v/s->n);
}

static double peak_abs(const struct series *s)
{
    double p=0;
    for (long i=0;i<s->n;i++) if (fabs(s->v[i])>p) p=fabs(s->v[i]);
    return p;
}

static int cmp_d(const void *a,const void *b)
{
    double x=*(const double*)a,y=*(const double*)b;
    return x<y?-1:x>y?1:0;
}

/* ===================== the fit ========================================== */

/* Frames whose angular rate exceeds this are dropped. Fast pans are not merely
 * noisy, they are actively misleading: once the between-frame shift approaches
 * the tracker's search range the match locks onto the wrong feature, and on this
 * rig it does so while still reporting high quality. A confident wrong number
 * hurts least squares far more than a missing row. 0 = no limit. */
static double g_maxdps = 0;
static long   g_kept_at_limit = -1;

/* Frames excluded as track breaks: the detector jumped to a DIFFERENT object,
 * so the image moved far more than the gimbal can account for. These are not
 * noise, they are a different signal entirely, and least squares chases them
 * hard. NULL = keep everything. */
static unsigned char *g_excl = NULL;

/* Restrict the fit to blobs in a band of image rows. A progressive (rolling)
 * readout captures the bottom of the frame later than the top, so a frame's
 * content epoch depends on WHERE in the frame you measure it -- and the lag
 * fitted from a blob is the lag at that blob's height, not a frame-wide
 * constant. Banding by row is what turns that into a measurement.
 * g_cyhi <= g_cylo disables the filter. */
static double *g_cy = NULL;
static double g_cylo = 0, g_cyhi = 0;

struct fit {
    double loss;              /* 1 - R^2 over both axes jointly: the objective */
    double R2, R2x, R2y;
    double a[2], b[2][2];     /* vx = a0 + b00*wy + b01*wp ; vy = a1 + b10*wy + b11*wp */
    long   n;
    int    ok;
};

/* Solve a 3x3 normal system by Gaussian elimination with partial pivoting.
 * Cholesky would be marginally faster and would fail confusingly on a
 * near-singular system; pivoting just reports it. */
static int solve3(double A[3][3], double rhs[3], double out[3])
{
    double M[3][4];
    for (int i=0;i<3;i++) { for (int j=0;j<3;j++) M[i][j]=A[i][j]; M[i][3]=rhs[i]; }
    for (int c=0;c<3;c++) {
        int p=c;
        for (int r=c+1;r<3;r++) if (fabs(M[r][c])>fabs(M[p][c])) p=r;
        if (fabs(M[p][c])<1e-12) return 0;
        if (p!=c) for (int j=0;j<4;j++) { double t=M[c][j]; M[c][j]=M[p][j]; M[p][j]=t; }
        for (int r=0;r<3;r++) {
            if (r==c) continue;
            double f=M[r][c]/M[c][c];
            for (int j=c;j<4;j++) M[r][j]-=f*M[c][j];
        }
    }
    for (int i=0;i<3;i++) out[i]=M[i][3]/M[i][i];
    return 1;
}

/* Fit the rigid map at one lag over frames in [t_lo,t_hi]; t_hi<=0 means all. */
static struct fit fit_at(const struct series *ivx, const struct series *ivy,
                         const struct series *ry, const struct series *rp,
                         double lag, double t_lo, double t_hi)
{
    struct fit F; memset(&F,0,sizeof F);
    double A[3][3]={{0,0,0},{0,0,0},{0,0,0}}, rx[3]={0,0,0}, rz[3]={0,0,0};
    double svx=0,svy=0,svxx=0,svyy=0;
    long n=0;
    long nmax = ivx->n < ivy->n ? ivx->n : ivy->n;   /* built in lockstep */

    /* one pass accumulating the normal equations for both targets: the design
     * matrix is shared, so there is no reason to walk the data twice */
    for (long i=0;i<nmax;i++) {
        double t=ivx->t[i];
        if (t_hi>0 && (t<t_lo || t>t_hi)) continue;
        int ok1,ok2;
        double wy=ser_at(ry,t-lag,&ok1);
        double wp=ser_at(rp,t-lag,&ok2);
        if (!ok1||!ok2) continue;
        if (g_maxdps>0 && (fabs(wy)>g_maxdps || fabs(wp)>g_maxdps)) continue;
        if (g_excl && g_excl[i]) continue;
        if (g_cy && g_cyhi>g_cylo && (g_cy[i]<g_cylo||g_cy[i]>g_cyhi)) continue;
        double vx=ivx->v[i], vy=ivy->v[i];
        double u[3]={1.0,wy,wp};
        for (int a=0;a<3;a++) {
            for (int b=0;b<3;b++) A[a][b]+=u[a]*u[b];
            rx[a]+=u[a]*vx; rz[a]+=u[a]*vy;
        }
        svx+=vx; svy+=vy; svxx+=vx*vx; svyy+=vy*vy;
        n++;
    }
    if (n<20) return F;

    double px[3],py[3];
    if (!solve3(A,rx,px) || !solve3(A,rz,py)) return F;

    F.a[0]=px[0]; F.b[0][0]=px[1]; F.b[0][1]=px[2];
    F.a[1]=py[0]; F.b[1][0]=py[1]; F.b[1][1]=py[2];

    double ssx=0,ssy=0;
    for (long i=0;i<nmax;i++) {
        double t=ivx->t[i];
        if (t_hi>0 && (t<t_lo || t>t_hi)) continue;
        int ok1,ok2;
        double wy=ser_at(ry,t-lag,&ok1);
        double wp=ser_at(rp,t-lag,&ok2);
        if (!ok1||!ok2) continue;
        if (g_maxdps>0 && (fabs(wy)>g_maxdps || fabs(wp)>g_maxdps)) continue;
        if (g_excl && g_excl[i]) continue;
        if (g_cy && g_cyhi>g_cylo && (g_cy[i]<g_cylo||g_cy[i]>g_cyhi)) continue;
        double ex=ivx->v[i]-(px[0]+px[1]*wy+px[2]*wp);
        double ey=ivy->v[i]-(py[0]+py[1]*wy+py[2]*wp);
        ssx+=ex*ex; ssy+=ey*ey;
    }
    double totx=svxx-svx*svx/n, toty=svyy-svy*svy/n;
    if (totx<=0||toty<=0) return F;

    F.n=n;
    F.loss=(ssx+ssy)/(totx+toty);
    F.R2=1.0-F.loss;
    F.R2x=1.0-ssx/totx;
    F.R2y=1.0-ssy/toty;
    F.ok=1;
    return F;
}

struct best { double lag; struct fit f; int at_edge; };

/* Sweep for the minimum of loss(tau), then refine with a parabola through the
 * three samples around it -- the same sub-sample trick the flow code uses, and
 * the reason the answer resolves finer than the 1 ms step. */
static struct best sweep(const struct series *ivx, const struct series *ivy,
                         const struct series *ry, const struct series *rp,
                         double lo, double hi, double step,
                         double t_lo, double t_hi,
                         double *cl, double *cv, int *ncv)
{
    struct best B; memset(&B,0,sizeof B);
    long n=0;
    for (double L=lo; L<=hi+1e-9; L+=step) n++;
    double *ls=malloc(sizeof(double)*(size_t)n);
    double *lv=malloc(sizeof(double)*(size_t)n);
    struct fit *fs=malloc(sizeof(struct fit)*(size_t)n);
    if (!ls||!lv||!fs) die("out of memory");

    long k=0, bi=-1;
    for (double L=lo; L<=hi+1e-9 && k<n; L+=step, k++) {
        fs[k]=fit_at(ivx,ivy,ry,rp,L/1000.0,t_lo,t_hi);
        ls[k]=L;
        lv[k]=fs[k].ok?fs[k].loss:2.0;
        if (fs[k].ok && (bi<0 || lv[k]<lv[bi])) bi=k;
    }
    if (bi<0) { free(ls); free(lv); free(fs); return B; }

    B.lag=ls[bi]; B.f=fs[bi];
    B.at_edge=(bi==0 || bi==k-1);
    if (bi>0 && bi<k-1) {
        double y0=lv[bi-1], y1=lv[bi], y2=lv[bi+1];
        double den=y0-2*y1+y2;
        if (fabs(den)>1e-15) {
            double d=0.5*(y0-y2)/den;
            if (d>-1 && d<1) B.lag=ls[bi]+d*step;
        }
    }
    if (cl && cv && ncv) {
        *ncv=0;
        long stride = k>MAXCURVE ? k/MAXCURVE : 1;
        for (long i=0;i<k && *ncv<MAXCURVE;i+=stride) {
            cl[*ncv]=ls[i]; cv[*ncv]=lv[i]; (*ncv)++;
        }
    }
    free(ls); free(lv); free(fs);
    return B;
}

static void usage(const char *me)
{
    printf(
"Find the lag between real-world movement and the detection output by fitting\n"
"the rigid camera-on-gimbal relation at every candidate time shift and taking\n"
"the shift that minimises the residual. The IMU is the ground truth.\n"
"\n"
"usage: %s [-d RUNDIR | -b CSV] [-i IMUCSV] [-L LO_MS] [-U HI_MS] [-s STEP_MS]\n"
"          [-w SMOOTH_MS] [-m MAXDPS] [-k PXDEG] [-j BLOCKS] [-v]\n"
"\n"
"  -d RUNDIR   a blob_log run folder (blobs.csv + imu.csv), or a flow_stamp\n"
"              folder (frames.csv). The normal way to call this.\n"
"  -b CSV      the per-frame CSV directly\n"
"  -i IMUCSV   the ~1 kHz IMU CSV; without it the per-frame yaw/pitch columns\n"
"              are used and the time resolution is much coarser\n"
"  -L, -U      lag search range, ms          (default -100 .. 300)\n"
"  -s STEP_MS  lag step                      (default 1)\n"
"  -w MS       rate smoothing half-window    (default 8)\n"
"  -m MAXDPS   ignore frames whose angular rate exceeds this. Use it when a\n"
"              record mixes gentle and violent motion: the violent part is\n"
"              untrackable and pollutes the fit. 0 = keep all (default)\n"
"  -k PXDEG    expected px/deg for the scale check (default 27.0, MEASURED on\n"
"              this rig from two independent runs: 27.34 and 26.54, i.e. a\n"
"              46.8 deg horizontal FOV. The old 18.6 was an assumption and was\n"
"              wrong. Halve it if you ever stream the native 640 sensor.)\n"
"  -j BLOCKS   independent blocks for the reproducibility spread (default 4)\n"
"  -W PX       frame width, for the field-of-view check (default 1280)\n"
"  -R          do not reject track breaks (keep every frame in the fit)\n"
"  -T          index frames by t_detect_done instead of t_first_byte, so the\n"
"              lag reported IS the full world -> detection-output latency. The\n"
"              two must differ by exactly the mean fb_to_det_ms; that identity\n"
"              is what makes the endpoint of the measurement checkable.\n"
"  -Z NBANDS   split the frames into NBANDS bands by the blob's IMAGE ROW and\n"
"              fit each separately, then fit lag against row. This measures the\n"
"              progressive-readout gradient: on a rolling shutter the bottom of\n"
"              the frame is captured later than the top, so the lag depends on\n"
"              where in the frame you look. Needs the blob to actually TRAVERSE\n"
"              vertically -- a few hundred rows of spread, not a few tens.\n"
"  -v          print the loss curve\n"
"\n"
"RECORDING FOR THIS: pan the gimbal smoothly back and forth a few degrees each\n"
"way at roughly 0.3-1 Hz for 20-30 s, with a warm object in view. Keep the peak\n"
"rate in the 10-50 deg/s band: below it the motion is lost in centroid noise,\n"
"above it the tracker loses lock and reports confident nonsense. A violent jerk\n"
"is the worst case for this measurement, not the best.\n",
    me);
}

int main(int argc, char **argv)
{
    const char *dir=NULL,*bpath=NULL,*ipath=NULL;
    double lo=-100,hi=300,step=1.0,win_ms=8.0,pxdeg=27.0,framew=1280.0;
    int verbose=0,nblock=4,no_robust=0,nrowband=0,tbase_det=0,c;

    while ((c=getopt(argc,argv,"d:b:i:L:U:s:w:m:k:j:W:Z:RTvh"))!=-1) {
        switch (c) {
        case 'd': dir=optarg; break;
        case 'b': bpath=optarg; break;
        case 'i': ipath=optarg; break;
        case 'L': lo=atof(optarg); break;
        case 'U': hi=atof(optarg); break;
        case 's': step=atof(optarg); break;
        case 'w': win_ms=atof(optarg); break;
        case 'm': g_maxdps=atof(optarg); break;
        case 'k': pxdeg=atof(optarg); break;
        case 'j': nblock=atoi(optarg); break;
        case 'R': no_robust=1; break;
        case 'T': tbase_det=1; break;
        case 'Z': nrowband=atoi(optarg); break;
        case 'W': framew=atof(optarg); break;
        case 'v': verbose=1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 2;
        }
    }
    if (!dir && !bpath) { usage(argv[0]); return 2; }
    if (hi<=lo) die("-U must exceed -L");
    if (step<=0) die("-s must be > 0");
    if (nblock<1||nblock>32) die("-j must be 1..32");
    if (pxdeg<=0) die("-k must be > 0");

    char bbuf[1024],ibuf[1024];
    if (dir) {
        snprintf(bbuf,sizeof bbuf,"%s/blobs.csv",dir);
        if (access(bbuf,R_OK)!=0) snprintf(bbuf,sizeof bbuf,"%s/frames.csv",dir);
        bpath=bbuf;
        snprintf(ibuf,sizeof ibuf,"%s/imu.csv",dir);
        if (!ipath && access(ibuf,R_OK)==0) ipath=ibuf;
    }

    struct csv B;
    csv_read(bpath,&B);

    int c_cx=col(&B,"cx"), c_dxp=col(&B,"dx_px");
    int blobmode = c_cx>=0;
    if (!blobmode && c_dxp<0)
        die("%s is neither a blob_log CSV (needs cx,cy) nor a flow_stamp CSV\n"
            "  (needs dx_px,dy_px)",bpath);

    int c_tfb=col_req(&B,"t_first_byte",bpath);
    int c_tdet=col(&B,"t_detect_done");
    int c_state=blobmode?col(&B,"det_state"):col(&B,"flow_state");
    int c_valid=col(&B,"valid");
    int c_cy=col(&B,"cy"), c_dyp=col(&B,"dy_px");
    int c_tflow=col(&B,"t_flow"), c_int=col(&B,"interval_ms");
    int c_yaw=col(&B,"yaw"), c_pitch=col(&B,"pitch"), c_timu=col(&B,"t_imu");
    if (blobmode && c_cy<0) die("%s has cx but no cy",bpath);

    /* ---- image velocity, px/s, on the frame's own clock ---- */
    struct series ivx,ivy;
    ser_alloc(&ivx,B.nrow); ser_alloc(&ivy,B.nrow);
    double *cyv=malloc(sizeof(double)*(size_t)B.nrow);
    long ncy=0;
    if (!cyv) die("out of memory");
    double sum_det=0; long n_det=0;
    double t_first=0,t_last=0; long n_frames=0;
    double sum_int=0; long n_int=0;

    for (long r=0;r<B.nrow;r++) {
        if (empty(&B,r,c_tfb)) continue;
        double t=atof(cell(&B,r,c_tfb));
        if (!n_frames) t_first=t;
        t_last=t; n_frames++;
        if (c_tdet>=0 && !empty(&B,r,c_tdet)) {
            sum_det+=(atof(cell(&B,r,c_tdet))-t)*1000.0; n_det++;
        }
    }

    if (blobmode) {
        /* Central difference on the centroid. A row is used only if it and both
         * neighbours carry a valid detection: otherwise the difference straddles
         * a gap and produces a spike the fit would chase. */
        for (long r=1;r+1<B.nrow;r++) {
            int ok=1;
            for (long k=r-1;k<=r+1;k++) {
                if (empty(&B,k,c_cx)||empty(&B,k,c_cy)||empty(&B,k,c_tfb)) { ok=0; break; }
                if (c_valid>=0 && strcmp(cell(&B,k,c_valid),"1")!=0) { ok=0; break; }
                if (c_state>=0 && strcmp(cell(&B,k,c_state),"ok")!=0) { ok=0; break; }
            }
            if (!ok) continue;
            double t0=atof(cell(&B,r-1,c_tfb)), t2=atof(cell(&B,r+1,c_tfb));
            double dt=t2-t0;
            if (dt<=0) continue;
            double t=atof(cell(&B,r,c_tfb));
            if (tbase_det && c_tdet>=0 && !empty(&B,r,c_tdet))
                t=atof(cell(&B,r,c_tdet));
            ivx.t[ivx.n]=t;
            ivx.v[ivx.n]=(atof(cell(&B,r+1,c_cx))-atof(cell(&B,r-1,c_cx)))/dt;
            ivx.n++;
            ivy.t[ivy.n]=t;
            ivy.v[ivy.n]=(atof(cell(&B,r+1,c_cy))-atof(cell(&B,r-1,c_cy)))/dt;
            ivy.n++;
            cyv[ncy++]=atof(cell(&B,r,c_cy));      /* this frame's blob row */
            sum_int+=dt/2.0; n_int++;
        }
    } else {
        /* flow_stamp: dx_px is already a between-frame displacement, and t_flow
         * is documented as the midpoint of the interval it spans -- exactly the
         * right timestamp for a velocity. */
        for (long r=0;r<B.nrow;r++) {
            if (c_state>=0 && strcmp(cell(&B,r,c_state),"ok")!=0) continue;
            if (empty(&B,r,c_dxp)||empty(&B,r,c_dyp)||empty(&B,r,c_int)) continue;
            double iv=atof(cell(&B,r,c_int))/1000.0;
            if (iv<=0) continue;
            double t=(c_tflow>=0&&!empty(&B,r,c_tflow))?atof(cell(&B,r,c_tflow))
                                                       :atof(cell(&B,r,c_tfb));
            ivx.t[ivx.n]=t; ivx.v[ivx.n]=atof(cell(&B,r,c_dxp))/iv; ivx.n++;
            ivy.t[ivy.n]=t; ivy.v[ivy.n]=atof(cell(&B,r,c_dyp))/iv; ivy.n++;
            sum_int+=iv; n_int++;
        }
    }
    double mean_int = n_int? sum_int/n_int : 0.04;
    if (blobmode && ncy==ivx.n) g_cy=cyv;

    /* ---- IMU angles ---- */
    struct series ayaw,apitch;
    struct csv I;
    const char *imu_src;
    int have_imu_csv = ipath && access(ipath,R_OK)==0;
    if (have_imu_csv) {
        csv_read(ipath,&I);
        int t_i=col_req(&I,"t_imu",ipath);
        int y_i=col_req(&I,"yaw",ipath);
        int p_i=col_req(&I,"pitch",ipath);
        ser_alloc(&ayaw,I.nrow); ser_alloc(&apitch,I.nrow);
        for (long r=0;r<I.nrow;r++) {
            if (empty(&I,r,t_i)||empty(&I,r,y_i)||empty(&I,r,p_i)) continue;
            double t=atof(cell(&I,r,t_i));
            ayaw.t[ayaw.n]=t;     ayaw.v[ayaw.n]=atof(cell(&I,r,y_i));     ayaw.n++;
            apitch.t[apitch.n]=t; apitch.v[apitch.n]=atof(cell(&I,r,p_i)); apitch.n++;
        }
        imu_src="imu.csv (~1 kHz)";
    } else {
        if (c_yaw<0) die("no imu.csv found and %s has no yaw column",bpath);
        ser_alloc(&ayaw,B.nrow); ser_alloc(&apitch,B.nrow);
        for (long r=0;r<B.nrow;r++) {
            if (empty(&B,r,c_yaw)||empty(&B,r,c_pitch)) continue;
            double t=(c_timu>=0&&!empty(&B,r,c_timu))?atof(cell(&B,r,c_timu))
                                                     :atof(cell(&B,r,c_tfb));
            ayaw.t[ayaw.n]=t;     ayaw.v[ayaw.n]=atof(cell(&B,r,c_yaw));     ayaw.n++;
            apitch.t[apitch.n]=t; apitch.v[apitch.n]=atof(cell(&B,r,c_pitch)); apitch.n++;
        }
        imu_src="per-frame yaw/pitch columns (frame rate only -- coarse)";
    }
    if (ayaw.n<20) die("only %ld usable IMU samples",ayaw.n);

    struct series ry,rp;
    rate_of(&ayaw,&ry,win_ms/1000.0);
    rate_of(&apitch,&rp,win_ms/1000.0);

    /* ---- header ---- */
    double dur=t_last-t_first;
    printf("=== det_latency: world movement -> detection output ===\n");
    printf("per-frame csv    %s  (%s)\n",bpath,blobmode?"blob_log":"flow_stamp");
    printf("imu source       %s\n",imu_src);
    printf("record           %.2f s, %ld frames, interval %.2f ms (%.2f fps)\n",
           dur,n_frames,mean_int*1000.0,mean_int>0?1.0/mean_int:0.0);
    printf("usable frames    %ld (central differences with a valid detection)\n",ivx.n);
    printf("lag sweep        %.0f .. %.0f ms step %.1f;  rate smoothing +-%.0f ms\n",
           lo,hi,step,win_ms);
    if (g_maxdps>0) {
        /* Report what the limit actually leaves. A violently panned record has
         * slow frames only at the turnarounds, so a rate limit can silently cut
         * the data down to nothing -- worth saying before the fit is attempted. */
        long kept=0;
        for (long i=0;i<ivx.n;i++) {
            int o1,o2;
            double wy=ser_at(&ry,ivx.t[i],&o1), wp=ser_at(&rp,ivx.t[i],&o2);
            if (o1&&o2&&fabs(wy)<=g_maxdps&&fabs(wp)<=g_maxdps) kept++;
        }
        printf("rate limit       above %.0f deg/s excluded -> %ld of %ld frames kept\n",
               g_maxdps,kept,ivx.n);
        g_kept_at_limit=kept;
    }

    /* ---- can this record answer at all? ---- */
    double yaw_range=range_of(&ayaw), pitch_range=range_of(&apitch);
    double ypk=peak_abs(&ry), ppk=peak_abs(&rp);
    double implied_pk=(ypk>ppk?ypk:ppk)*pxdeg*mean_int;
    long n_fast=0,n_tot=0;
    for (long i=0;i<ry.n && i<rp.n;i++) {
        double a=fabs(ry.v[i]), b=fabs(rp.v[i]);
        double px=(a>b?a:b)*pxdeg*mean_int;
        n_tot++; if (px>150.0) n_fast++;
    }
    printf("\ngimbal motion    yaw %.2f deg range (peak %.1f deg/s)\n",yaw_range,ypk);
    printf("                 pitch %.2f deg range (peak %.1f deg/s)\n",pitch_range,ppk);
    printf("trackability     peak implied shift %.0f px/frame;  %.1f%% of the record\n"
           "                 above 150 px/frame   -> %s\n",
           implied_pk,n_tot?100.0*n_fast/n_tot:0.0,
           implied_pk>150.0?"TOO FAST, the tracker will lose lock":"trackable");
    int too_fast = implied_pk>150.0;

    /* Can the target even stay in view for the whole sweep? A swing wider than
     * the field of view guarantees it leaves the frame, and then the detector
     * tracks whatever else is brightest. This cost a whole recording before it
     * was checked: 51.8 deg of sweep at 27 px/deg demands 1399 px of travel
     * through a 1280 px frame. */
    double demand_hi = (yaw_range>pitch_range?yaw_range:pitch_range)*pxdeg;
    printf("field of view    sweep demands %.0f px of travel across a %.0f px frame"
           " -> %s\n",demand_hi,framew,
           demand_hi>framew*0.95
             ? "TARGET CANNOT STAY IN VIEW"
             : demand_hi>framew*0.8 ? "very little margin" : "fits");
    int fov_over = demand_hi>framew*0.95;

    if (yaw_range<0.5 && pitch_range<0.5) {
        printf("\nCANNOT ANSWER: the gimbal did not move. Both axes stayed inside\n"
               "0.5 deg for the whole record, which is sensor noise, not movement.\n"
               "There is no shared event in the two signals to line up.\n"
               "\n  Re-record while panning a few degrees back and forth.\n");
        return 2;
    }
    if (ivx.n<40) {
        printf("\nCANNOT ANSWER: only %ld usable frames. Record for longer.\n",ivx.n);
        return 2;
    }

    /* ---- the sweep ---- */
    double cl[MAXCURVE],cv[MAXCURVE]; int ncv=0;
    struct best W=sweep(&ivx,&ivy,&ry,&rp,lo,hi,step,0,0,cl,cv,&ncv);
    if (!W.f.ok) {
        printf("\nCANNOT ANSWER: the fit did not converge anywhere in the sweep.\n");
        if (g_kept_at_limit>=0 && g_kept_at_limit<20)
            printf("  The -m %.0f rate limit left only %ld usable frames (20 needed).\n"
                   "  In a violently panned record the only frames below the limit are\n"
                   "  the turnaround instants, so filtering cannot rescue it -- there is\n"
                   "  no sustained slow motion to fit. Re-record with a gentle pan.\n",
                   g_maxdps,g_kept_at_limit);
        return 2;
    }

    /* ---- reject track breaks and re-sweep -------------------------------
     * A first pass gives a lag good enough to predict where the image SHOULD
     * have moved. Frames that moved wildly more than that are the detector
     * having switched to a different object -- most often because the target
     * left the frame. Dropping them and re-sweeping is what separates "the
     * target wandered off" from "the latency is different". */
    long n_break=0;
    if (!no_robust) {
        long nmax = ivx.n<ivy.n?ivx.n:ivy.n;
        g_excl=calloc((size_t)nmax,1);
        if (!g_excl) die("out of memory");
        for (int pass=0;pass<2;pass++) {
            struct fit F0=fit_at(&ivx,&ivy,&ry,&rp,W.lag/1000.0,0,0);
            if (!F0.ok) break;
            /* RMS residual over the frames still included */
            double ss=0; long nn=0;
            for (long i=0;i<nmax;i++) {
                if (g_excl[i]) continue;
                int o1,o2;
                double wy=ser_at(&ry,ivx.t[i]-W.lag/1000.0,&o1);
                double wp=ser_at(&rp,ivx.t[i]-W.lag/1000.0,&o2);
                if (!o1||!o2) continue;
                double ex=ivx.v[i]-(F0.a[0]+F0.b[0][0]*wy+F0.b[0][1]*wp);
                double ey=ivy.v[i]-(F0.a[1]+F0.b[1][0]*wy+F0.b[1][1]*wp);
                ss+=ex*ex+ey*ey; nn++;
            }
            if (nn<20) break;
            double rms=sqrt(ss/nn);
            long marked=0;
            for (long i=0;i<nmax;i++) {
                if (g_excl[i]) continue;
                int o1,o2;
                double wy=ser_at(&ry,ivx.t[i]-W.lag/1000.0,&o1);
                double wp=ser_at(&rp,ivx.t[i]-W.lag/1000.0,&o2);
                if (!o1||!o2) continue;
                double ex=ivx.v[i]-(F0.a[0]+F0.b[0][0]*wy+F0.b[0][1]*wp);
                double ey=ivy.v[i]-(F0.a[1]+F0.b[1][0]*wy+F0.b[1][1]*wp);
                if (sqrt(ex*ex+ey*ey) > 4.0*rms) { g_excl[i]=1; marked++; }
            }
            n_break+=marked;
            if (!marked) break;
            struct best W2=sweep(&ivx,&ivy,&ry,&rp,lo,hi,step,0,0,cl,cv,&ncv);
            if (W2.f.ok) W=W2;
        }
        if (n_break)
            printf("\ntrack breaks     %ld of %ld frames rejected (%.1f%%): the image\n"
                   "                 moved far more than the gimbal can explain, so the\n"
                   "                 detector had switched to a different object\n",
                   n_break,ivx.n,100.0*n_break/ivx.n);
    }

    if (verbose && ncv) {
        printf("\n-- loss vs lag (lower is better; * marks the minimum) --\n");
        double mn=cv[0],mx=cv[0];
        for (int i=1;i<ncv;i++) { if (cv[i]<mn) mn=cv[i]; if (cv[i]>mx) mx=cv[i]; }
        for (int i=0;i<ncv;i++) {
            int w=(int)(52*(cv[i]-mn)/(mx-mn>1e-12?mx-mn:1));
            printf("  %7.1f ms  %6.4f  %.*s%s\n",cl[i],cv[i],w,
                   "####################################################",
                   cv[i]<=mn+1e-12?" *":"");
        }
    }

    /* ---- the fitted map, and what physics demands of it ---- */
    struct fit F=W.f;

    /* Which axes were actually driven? If only one was, the 2x2 map is
     * RANK-DEFICIENT: the column belonging to the idle axis is unconstrained, so
     * det B is meaningless and with it both sqrt|det| and the conformality test.
     * The LAG is still perfectly well determined -- it comes from the axis that
     * did move -- so the fix is to narrow the physical checks, not to refuse.
     * Getting this wrong made this program reject a good measurement whose lag
     * was stable to 6.6 ms across four independent blocks. */
    double exc_y=sd_of(&ry), exc_p=sd_of(&rp);
    double exc_hi = exc_y>exc_p?exc_y:exc_p;
    double exc_lo = exc_y>exc_p?exc_p:exc_y;
    int single_axis = (exc_hi<=0) || (exc_lo/exc_hi < 0.15);
    int dom = (exc_y>=exc_p)?0:1;          /* 0 = yaw, 1 = pitch */

    double det=F.b[0][0]*F.b[1][1]-F.b[0][1]*F.b[1][0];
    double scale;
    if (single_axis) {
        /* the driven axis's column of B is the image response to it, in px/deg;
         * its length is the scale, and that IS identifiable from one axis */
        double rx=F.b[0][dom], rz=F.b[1][dom];
        scale=sqrt(rx*rx+rz*rz);
    } else {
        scale=sqrt(fabs(det));
    }
    /* A rigid mount can only give scale x rotation, forcing b00=b11 and
     * b01=-b10. Nothing in the fit imposes that, so the departure is a genuine
     * independent test of whether this minimum is camera motion at all. */
    double conf = scale>1e-9 ? (fabs(F.b[0][0]-F.b[1][1])+fabs(F.b[0][1]+F.b[1][0]))
                               /(2.0*scale) : 9.9;
    /* For a world-fixed scene the map INVERTS (pan right, scene moves left), so
     * B = -scale x R(roll) and atan2 lands 180 deg away from the physical roll.
     * Report the roll folded into (-90,90] and state the inversion separately,
     * rather than printing a misleading -173 deg for a 7 deg misalignment. */
    double roll = atan2(F.b[1][0],F.b[0][0])*180.0/M_PI;
    int inverts = (F.b[0][0]+F.b[1][1]) < 0;
    while (roll >  90.0) roll -= 180.0;
    while (roll <= -90.0) roll += 180.0;

    printf("\n-- best fit: world movement -> %s = %.1f ms --\n",
           tbase_det ? "DETECTION OUTPUT" : "frame's first byte",
           W.lag);
    if (!tbase_det)
        printf("   (this lag ENDS AT THE FIRST BYTE. Detection is not in it --\n"
               "    add [B+C+D] below, or re-run with -T for the full chain.)\n");
    printf("  loss (1-R^2)   %.4f      R^2 %.4f   (x %.4f, y %.4f)\n",
           F.loss,F.R2,F.R2x,F.R2y);
    printf("  fitted map     [ vx ]   [ %8.2f %8.2f ] [ yaw_rate   ]\n",
           F.b[0][0],F.b[0][1]);
    printf("                 [ vy ] = [ %8.2f %8.2f ] [ pitch_rate ]  px/deg\n",
           F.b[1][0],F.b[1][1]);
    printf("  excitation     yaw %.1f deg/s rms, pitch %.1f deg/s rms  -> %s\n",
           exc_y,exc_p,single_axis?"ONE axis driven":"both axes driven");
    if (single_axis)
        printf("  scale          %.2f px/deg   (|response to %s|; -k says ~%.1f)\n",
               scale,dom?"pitch":"yaw",pxdeg);
    else
        printf("  scale          %.2f px/deg   (sqrt|det|; -k says ~%.1f)\n",scale,pxdeg);
    if (single_axis)
        printf("  conformality   n/a           (needs both axes driven to be defined)\n");
    else
        printf("  conformality   %.3f          (0 = a perfect scale+rotation)\n",conf);
    printf("  implied roll   %+.1f deg between camera and gimbal axes\n",roll);
    if (!single_axis)
        printf("  orientation    image %s relative to gimbal axes  %s\n",
               inverts?"inverts":"follows",
               inverts?"(expected: a world-fixed scene moves opposite to the pan)"
                      :"(unexpected -- check the target is not itself moving)");
    printf("  frames in fit  %ld\n",F.n);

    /* ---- reproducibility across independent blocks ---- */
    double blk[32]; int nb=0;
    double bmean=0,bsd=0;
    if (nblock>1 && dur>0) {
        double seg=dur/nblock;
        for (int i=0;i<nblock;i++) {
            struct best b=sweep(&ivx,&ivy,&ry,&rp,lo,hi,step,
                                t_first+i*seg,t_first+(i+1)*seg,NULL,NULL,NULL);
            if (b.f.ok && !b.at_edge && b.f.R2>0.3) blk[nb++]=b.lag;
        }
    }
    if (nb>1) {
        for (int i=0;i<nb;i++) bmean+=blk[i];
        bmean/=nb;
        for (int i=0;i<nb;i++) bsd+=(blk[i]-bmean)*(blk[i]-bmean);
        bsd=sqrt(bsd/(nb-1));
        qsort(blk,nb,sizeof(double),cmp_d);
        printf("\n-- reproducibility: %d of %d blocks fitted, %.1f s each --\n",
               nb,nblock,dur/nblock);
        printf("  per-block lag  ");
        for (int i=0;i<nb;i++) printf("%.1f%s",blk[i],i+1<nb?", ":"");
        printf(" ms\n");
        printf("  mean %.1f ms, sd %.1f ms\n",bmean,bsd);
    }

    /* ---- lag versus image row: the progressive-readout gradient --------
     * A microbolometer is read out row by row, so the bottom of the frame is
     * captured later than the top. Relative to a fixed t_first_byte that makes
     * the TOP look older -- a higher lag -- than the bottom. Fitting lag against
     * row recovers the scan time directly, and it also explains why a lag
     * measured from one blob is the lag AT THAT ROW, not a frame constant. */
    double grad_ms_row=0, scan_ms=0;

    if (nrowband>1 && g_cy) {
        long nmax=ivx.n<ivy.n?ivx.n:ivy.n;
        double cymin=1e18,cymax=-1e18;
        for (long i=0;i<nmax;i++) {
            if (g_excl&&g_excl[i]) continue;
            if (g_cy[i]<cymin) cymin=g_cy[i];
            if (g_cy[i]>cymax) cymax=g_cy[i];
        }
        double spread=cymax-cymin;
        printf("\n-- lag versus image row (progressive-readout gradient) --\n");
        printf("  blob row spread  %.0f .. %.0f  = %.0f rows\n",cymin,cymax,spread);
        double expect_ms = 20.0*spread/1024.0;
        if (spread < 150.0) {
            printf("  CANNOT MEASURE: %.0f rows of spread is not enough. If the whole\n"
                   "  frame scans in ~20 ms, %.0f rows moves the lag by only %.2f ms,\n"
                   "  against a per-band precision of roughly 1-2 ms. The blob must\n"
                   "  TRAVERSE the frame vertically: drive the gimbal axis that moves the\n"
                   "  image up and down, not the one that pans it sideways.\n",
                   spread,spread,expect_ms);
        } else {
            /* Equal-WIDTH bands across the row range, with a population floor.
             * Equal-population banding is wrong here: the blob's row histogram
             * is a spike, so equal counts produced bands 1-3 spanning rows
             * 494-499 -- three fits at effectively the same row, which measures
             * nothing about a gradient. */
            printf("  band       rows        lag_ms      R^2   frames\n");
            double sx=0,sy=0,sxx=0,sxy=0; int nfit=0;
            double same_row_lo=1e18, same_row_hi=-1e18;
            double bw=spread/nrowband;
            for (int bi=0;bi<nrowband;bi++) {
                double lo_c=cymin+bi*bw, hi_c=cymin+(bi+1)*bw;
                if (bi==nrowband-1) hi_c=cymax+1.0;
                long pop=0;
                for (long i=0;i<nmax;i++) {
                    if (g_excl&&g_excl[i]) continue;
                    if (g_cy[i]>=lo_c&&g_cy[i]<=hi_c) pop++;
                }
                if (pop<40) { printf("  %-4d  %4.0f-%4.0f    (only %ld frames, skipped)\n",
                                     bi+1,lo_c,hi_c,pop); continue; }
                g_cylo=lo_c; g_cyhi=hi_c;
                struct best bb=sweep(&ivx,&ivy,&ry,&rp,lo,hi,step,0,0,NULL,NULL,NULL);
                g_cylo=g_cyhi=0;
                if (!bb.f.ok) { printf("  %-4d  %4.0f-%4.0f    (no fit)\n",bi+1,lo_c,hi_c); continue; }
                double mid=0.5*(lo_c+hi_c);
                printf("  %-4d  %4.0f-%4.0f  %10.1f  %7.4f  %6ld%s\n",
                       bi+1,lo_c,hi_c,bb.lag,bb.f.R2,bb.f.n,
                       (bb.at_edge||bb.f.R2<0.5)?"  (unreliable)":"");
                if (!bb.at_edge && bb.f.R2>=0.5) {
                    sx+=mid; sy+=bb.lag; sxx+=mid*mid; sxy+=mid*bb.lag; nfit++;
                    if (bb.lag<same_row_lo) same_row_lo=bb.lag;
                    if (bb.lag>same_row_hi) same_row_hi=bb.lag;
                }
            }
            if (nfit>=3) {
                double vxx=sxx-sx*sx/nfit, vxy=sxy-sx*sy/nfit;
                if (fabs(vxx)>1e-9) {
                    grad_ms_row=vxy/vxx;
                    scan_ms=grad_ms_row*1024.0;
                    printf("  gradient         %+.4f ms per row\n",grad_ms_row);
                    printf("  implied scan     %.1f ms across 1024 rows\n",fabs(scan_ms));
                    /* Row increases downward. A top-to-bottom readout captures the
                     * bottom LATER, so the bottom's content sits closer to
                     * t_first_byte and its lag is SMALLER -- a negative gradient. */
                    printf("  direction        %s\n", scan_ms<-2.0
                        ? "lag FALLS down the frame: the bottom is captured\n"
                          "                   LATER than the top. A normal top-to-bottom readout."
                        : scan_ms>2.0
                        ? "lag RISES down the frame: the bottom is captured\n"
                          "                   EARLIER than the top -- readout runs bottom-to-top."
                        : "flat: no measurable readout gradient");
                    if (nfit>=2 && same_row_hi>same_row_lo)
                        printf("  precision floor  bands scatter %.1f ms; a gradient smaller\n"
                               "                   than that is not distinguishable from noise\n",
                               same_row_hi-same_row_lo);
                }
            } else {
                printf("  too few usable bands (%d) to fit a gradient\n",nfit);
            }
        }
    }

    /* ---- checks, before any number is believed ---- */
    printf("\n-- checks --\n");
    int fatal=0;
    printf("  fit quality    R^2 %.3f      %s\n",F.R2,
           F.R2>=0.8?"strong":F.R2>=0.5?"usable":"TOO WEAK");
    if (F.R2<0.5) fatal=1;
    printf("  minimum        %s\n",W.at_edge?"AT THE EDGE of the sweep -- widen -L/-U"
                                            :"inside the sweep");
    if (W.at_edge) fatal=1;
    /* The scale check exists to catch a coincidental minimum. It is a WARNING,
     * not a veto: -k is an assumption about the lens, and a measured scale that
     * is self-consistent at R^2 0.98 with a stable block spread is much better
     * evidence than the assumption it disagrees with. Vetoing on it would let a
     * wrong default silently discard a good measurement. */
    printf("  scale          %.2f px/deg  %s\n",scale,
           (scale>pxdeg*0.5&&scale<pxdeg*2.0)
             ? "consistent with -k"
             : "differs from -k by more than 2x -- if the lens is unknown,\n"
               "                 trust this measurement and pass it as -k next time");
    if (single_axis)
        printf("  conformality   skipped -- only one gimbal axis was driven, so the\n"
               "                 2x2 map is rank-deficient and this test is undefined.\n"
               "                 Drive BOTH axes to get it back.\n");
    else {
        printf("  conformality   %.3f         %s\n",conf,
               conf<0.35?"a rigid scale+rotation, as the mount requires"
                        :"NOT a rigid rotation -- the fit is not explaining camera motion");
        if (conf>=0.35) fatal=1;
    }
    if (nb>1)
        printf("  block spread   sd %.1f ms  %s\n",bsd,
               bsd<10.0?"stable across the record"
                       :"LARGE -- the lag is not stable within one recording");
    if (!have_imu_csv)
        printf("  resolution     IMU only at frame rate; the lag is interpolated\n"
               "                 between %.0f ms samples, so treat it as coarse\n",
               mean_int*1000.0);

    if (fatal) {
        printf("\nCANNOT ANSWER with confidence: a check above failed.\n");
        if (too_fast)
            printf("  The cause here is too MUCH motion. At %.0f px of implied shift per\n"
                   "  frame, consecutive images barely overlap and are motion-blurred, so\n"
                   "  the tracker matches the wrong feature -- confidently, with high\n"
                   "  reported quality. Retry with -m %.0f to keep only comfortably\n"
                   "  trackable frames (50 px/frame), or re-record with a slower pan.\n",
                   implied_pk,50.0/(pxdeg*mean_int));
        else
            printf("  The usual causes are too little motion relative to centroid noise,\n"
                   "  or a target that is itself moving in the world. A moving target\n"
                   "  shows up as low R^2 with the scale and conformality still GOOD:\n"
                   "  the rigid map explains the camera's share of the motion correctly\n"
                   "  and simply cannot account for the rest. If that is the pattern\n"
                   "  above, aim at something stationary and warm instead.\n");
        if (fov_over)
            printf("  Also: the sweep is WIDER THAN THE FIELD OF VIEW (%.0f px demanded,\n"
                   "  %.0f px frame). The target must leave the frame, after which the\n"
                   "  detector tracks something else entirely. Keep the total sweep under\n"
                   "  about %.0f deg.\n",demand_hi,framew,framew*0.7/pxdeg);
        printf("\n  IDEAL RECORDING: smooth pan back and forth, total sweep under %.0f deg\n"
               "  so the target stays in view, 0.3-1 Hz, 20-30 s, peak rate 10-50 deg/s\n"
               "  (%.0f-%.0f px/frame here).\n",
               framew*0.7/pxdeg,10*pxdeg*mean_int,50*pxdeg*mean_int);
        return 2;
    }

    /* ---- the answer ---- */
    double A=W.lag;
    printf("\n-- answer --\n");
    if (tbase_det) {
        printf("  world movement -> DETECTION OUTPUT (-T: the full chain)\n");
        printf("        %.1f ms   (%.2f frames at %.2f fps)\n",
               A,mean_int>0?A/1000.0/mean_int:0.0,mean_int>0?1.0/mean_int:0.0);
        if (nb>1) printf("        block spread +-%.1f ms\n",bsd);
    } else {
        printf("  [A] world movement -> the frame's first byte on the host\n");
        printf("        %.1f ms   (%.2f frames at %.2f fps)\n",
               A,mean_int>0?A/1000.0/mean_int:0.0,mean_int>0?1.0/mean_int:0.0);
        if (nb>1) printf("        block spread +-%.1f ms\n",bsd);
        printf("        the camera's own hidden latency: integration plus the core's\n");
        printf("        internal pipeline. No host clock can see this part.\n");
    }
    if (n_det) {
        double BCD=sum_det/n_det;
        if (tbase_det) {
            /* With -T the fitted lag already ENDS at the detection output, so
             * adding the host-side chain again would count it twice. Decompose
             * instead, which also shows the two routes agreeing. */
            printf("  This lag already spans the whole chain. Decomposed:\n");
            printf("        %.1f ms  world -> first byte   (this fit minus the chain below)\n",
                   A-BCD);
            printf("      + %.1f ms  first byte -> cx,cy exist  (measured, from the CSV)\n",BCD);
            printf("      = %.1f ms  TOTAL, world movement -> detection output  (%.2f frames)\n",
                   A, mean_int>0?A/1000.0/mean_int:0.0);
        } else {
            printf("  [B+C+D] first byte -> cx,cy exist (measured, from the CSV)\n");
            printf("        %.1f ms\n",BCD);
            printf("\n  TOTAL, world movement -> detection output:  %.1f ms  (%.2f frames)\n",
                   A+BCD, mean_int>0?(A+BCD)/1000.0/mean_int:0.0);
        }
    } else {
        printf("  [B+C+D] absent: this CSV has no t_detect_done column (that is a\n");
        printf("        blob_log column; flow_stamp stops at t_available). Add the\n");
        printf("        host-side figure from that run's summary for the total.\n");
    }
    return 0;
}
