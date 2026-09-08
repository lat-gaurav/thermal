/* detect_blob.c -- find the bright blob in a raw thermal capture, track it frame
 * by frame, and measure its oscillation.
 *
 *   ./detect_blob -i recordings/.../raw-1280x1024-GREY.gray
 *
 * Offline, over a finished .gray written by record_raw or record_sync. Nothing
 * touches the camera, so no amount of compute here can cost a frame.
 *
 * WHAT IT DOES PER FRAME
 *   1. threshold to a binary mask of "bright"
 *   2. label connected components (8-connected flood fill), keep the largest
 *      above -a pixels
 *   3. report its intensity-weighted centroid, area, peak value and bounding box
 *
 * The centroid is intensity-weighted rather than a plain pixel mean, which gives
 * sub-pixel position and is far steadier on a saturated blob whose edge pixels
 * flicker between frames.
 *
 * THRESHOLD. Default is adaptive: 0.75 x the frame's peak value, with a floor, so
 * a blob that dims or the gain changing between frames does not silently lose it.
 * A fixed absolute threshold is available with -t when you want every frame
 * treated identically.
 *
 * OSCILLATION. The centroid series is linearly detrended, then a direct DFT over
 * the whole series gives the dominant frequency in each axis. Detrending matters
 * here: the camera itself pans during these recordings, which puts a ramp in the
 * blob's image position, and a ramp has large low-frequency content that would
 * otherwise be read as the oscillation. Cycle count is taken from mean crossings,
 * which is independent of the DFT and so acts as a cross-check.
 *
 * WHAT THIS MEASURES, precisely: the blob's position in the IMAGE. If the camera
 * is moving, that is blob motion plus camera motion, and the two cannot be
 * separated from pixels alone. To get the blob's motion in the world, subtract
 * the camera's own motion -- which is exactly what flow_stamp's dy_px/dx_px
 * columns record for a paired capture.
 *
 * Build:  gcc -O2 -Wall -Wextra -o detect_blob detect_blob.c -lm
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define DEF_W       1280
#define DEF_H       1024
#define DEF_MINAREA 20
#define DEF_RATIO   0.75      /* threshold = RATIO * frame peak */
#define THR_FLOOR   100       /* never threshold below this */

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fputs("error: ", stderr); vfprintf(stderr, fmt, ap); va_end(ap); fputc('\n', stderr);
    exit(1);
}

struct det {
    int    valid;
    double cx, cy;        /* intensity-weighted centroid, px */
    long   area;          /* pixels above threshold in the blob */
    int    peak;          /* brightest pixel in the blob */
    int    x0, y0, x1, y1;
    int    thr;
    int    ncomp;         /* components above -a; >1 means the choice is ambiguous */
    double t;             /* timestamp from the .idx, if available */
};

/* ---- connected components ---------------------------------------------- */

/* Iterative 8-connected flood fill. Recursion would blow the stack on a large
 * bright region, which a saturated blob plus a bright background easily is. */
static long fill(const uint8_t *img, unsigned W, unsigned H, int thr,
                 int32_t *lab, int id, long start, int32_t *stack,
                 double *sx, double *sy, double *sw, int *peak,
                 int *x0, int *y0, int *x1, int *y1)
{
    long sp = 0, area = 0;
    stack[sp++] = start;
    lab[start] = id;
    *sx = *sy = *sw = 0; *peak = 0;
    *x0 = W; *y0 = H; *x1 = -1; *y1 = -1;

    while (sp > 0) {
        long p = stack[--sp];
        int x = p % W, y = p / W;
        int v = img[p];
        double w = (double)v;
        *sx += w * x; *sy += w * y; *sw += w;
        if (v > *peak) *peak = v;
        if (x < *x0) *x0 = x;
        if (y < *y0) *y0 = y;
        if (x > *x1) *x1 = x;
        if (y > *y1) *y1 = y;
        area++;

        for (int dy = -1; dy <= 1; dy++) {
            int ny = y + dy;
            if (ny < 0 || ny >= (int)H) continue;
            for (int dx = -1; dx <= 1; dx++) {
                int nx = x + dx;
                if (nx < 0 || nx >= (int)W) continue;
                long q = (long)ny * W + nx;
                if (lab[q] || img[q] < thr) continue;
                lab[q] = id;
                stack[sp++] = q;
            }
        }
    }
    return area;
}

static struct det detect(const uint8_t *img, unsigned W, unsigned H,
                         int fixed_thr, double ratio, long minarea,
                         int32_t *lab, int32_t *stack)
{
    struct det d;
    memset(&d, 0, sizeof d);
    long n = (long)W * H;

    int peak = 0;
    for (long i = 0; i < n; i++) if (img[i] > peak) peak = img[i];

    int thr = fixed_thr > 0 ? fixed_thr : (int)(ratio * peak);
    if (thr < THR_FLOOR) thr = THR_FLOOR;
    if (thr > 254) thr = 254;
    d.thr = thr;

    memset(lab, 0, sizeof(int32_t) * n);
    int id = 0;
    long best_area = 0;
    for (long i = 0; i < n; i++) {
        if (lab[i] || img[i] < thr) continue;
        double sx, sy, sw; int pk, bx0, by0, bx1, by1;
        long a = fill(img, W, H, thr, lab, ++id, i, stack,
                      &sx, &sy, &sw, &pk, &bx0, &by0, &bx1, &by1);
        if (a >= minarea) d.ncomp++;
        if (a > best_area && a >= minarea) {
            best_area = a;
            d.valid = 1;
            d.cx = sw > 0 ? sx / sw : 0;
            d.cy = sw > 0 ? sy / sw : 0;
            d.area = a; d.peak = pk;
            d.x0 = bx0; d.y0 = by0; d.x1 = bx1; d.y1 = by1;
        }
    }
    return d;
}

/* ---- annotation -------------------------------------------------------- */

static void px_set(uint8_t *img, unsigned W, unsigned H, int x, int y, uint8_t v)
{
    if (x >= 0 && y >= 0 && x < (int)W && y < (int)H) img[(size_t)y * W + x] = v;
}

static void hline(uint8_t *img, unsigned W, unsigned H, int x0, int x1, int y,
                  int t, uint8_t v)
{
    for (int dy = 0; dy < t; dy++)
        for (int x = x0; x <= x1; x++) px_set(img, W, H, x, y + dy, v);
}

static void vline(uint8_t *img, unsigned W, unsigned H, int y0, int y1, int x,
                  int t, uint8_t v)
{
    for (int dx = 0; dx < t; dx++)
        for (int y = y0; y <= y1; y++) px_set(img, W, H, x + dx, y, v);
}

/* Mark the detection on a copy of the frame. The box is drawn PADDED outward and
 * the crosshair arms stop short of it, so nothing is painted over the blob
 * itself -- the point of looking at this video is to judge whether the box is on
 * the right object, which is impossible if the marker hides it. */
static void annotate(uint8_t *img, unsigned W, unsigned H, const struct det *d)
{
    if (!d->valid) return;
    const int pad = 10, thick = 2, arm = 55;
    const uint8_t v = 255;
    int bx0 = d->x0 - pad, by0 = d->y0 - pad;
    int bx1 = d->x1 + pad, by1 = d->y1 + pad;

    hline(img, W, H, bx0, bx1, by0, thick, v);
    hline(img, W, H, bx0, bx1, by1, thick, v);
    vline(img, W, H, by0, by1, bx0, thick, v);
    vline(img, W, H, by0, by1, bx1, thick, v);

    int cx = (int)(d->cx + 0.5), cy = (int)(d->cy + 0.5);
    hline(img, W, H, bx1 + 4, bx1 + arm, cy, 1, v);
    hline(img, W, H, bx0 - arm, bx0 - 4, cy, 1, v);
    vline(img, W, H, by1 + 4, by1 + arm, cx, 1, v);
    vline(img, W, H, by0 - arm, by0 - 4, cx, 1, v);
}

/* ---- oscillation ------------------------------------------------------- */

/* Direct DFT. N is a few hundred here, so O(N^2) is microseconds and avoids
 * pulling in a FFT dependency for no gain. */
static void dominant(const double *v, int n, double fps,
                     double *f_hz, double *amp, double *frac_power)
{
    double mean = 0;
    for (int i = 0; i < n; i++) mean += v[i];
    mean /= n;

    /* linear detrend: the camera pans, and that ramp is not the oscillation */
    double sx = 0, sxx = 0, sxy = 0, sy = 0;
    for (int i = 0; i < n; i++) { sx += i; sxx += (double)i*i; sxy += (double)i*v[i]; sy += v[i]; }
    double den = n * sxx - sx * sx;
    double slope = den != 0 ? (n * sxy - sx * sy) / den : 0;
    double icpt  = (sy - slope * sx) / n;

    double *d = malloc(sizeof(double) * n);
    if (!d) die("out of memory");
    for (int i = 0; i < n; i++) d[i] = v[i] - (slope * i + icpt);

    double total = 0, best = 0; int bk = 0;
    int kmax = n / 2;
    double *pw = malloc(sizeof(double) * (kmax + 1));
    if (!pw) die("out of memory");
    for (int k = 1; k <= kmax; k++) {
        double re = 0, im = 0;
        for (int i = 0; i < n; i++) {
            double a = -2.0 * M_PI * k * i / n;
            re += d[i] * cos(a); im += d[i] * sin(a);
        }
        pw[k] = re * re + im * im;
        total += pw[k];
        if (pw[k] > best) { best = pw[k]; bk = k; }
    }
    *f_hz = bk * fps / n;
    /* amplitude of a sinusoid from its one-sided DFT magnitude */
    *amp = bk ? 2.0 * sqrt(best) / n : 0;
    *frac_power = total > 0 ? best / total : 0;
    free(d); free(pw);
}

/* mean crossings -> half-cycles; independent of the DFT, so a useful check */
static int mean_crossings(const double *v, int n)
{
    double mean = 0;
    for (int i = 0; i < n; i++) mean += v[i];
    mean /= n;
    /* hysteresis at 10% of the swing keeps noise from inflating the count */
    double lo = v[0], hi = v[0];
    for (int i = 1; i < n; i++) { if (v[i] < lo) lo = v[i]; if (v[i] > hi) hi = v[i]; }
    double h = 0.10 * (hi - lo);
    int cross = 0, state = v[0] > mean ? 1 : -1;
    for (int i = 1; i < n; i++) {
        if (state > 0 && v[i] < mean - h) { state = -1; cross++; }
        else if (state < 0 && v[i] > mean + h) { state = 1; cross++; }
    }
    return cross;
}

static void usage(const char *me)
{
    printf(
"Track the bright blob in a raw thermal capture and measure its oscillation.\n"
"\n"
"usage: %s -i CAPTURE.gray [-o CSV] [-t THR] [-a MINAREA]\n"
"          [-r RATIO] [-W W] [-H H] [-x IDX] [-F FPS] [-q]\n"
"\n"
"  -i FILE     the .gray to analyse (required)\n"
"  -o CSV      per-frame output (default CAPTURE.gray.blob.csv; '-' for stdout)\n"
"  -O RAW      also write annotated GREY frames here ('-' for stdout), for\n"
"              piping straight into ffmpeg -- see the note printed at the end\n"
"  -t THR      fixed threshold 1..254 (default: adaptive, %.2f x frame peak)\n"
"  -r RATIO    adaptive threshold ratio (default %.2f)\n"
"  -a MINAREA  ignore components smaller than this many px (default %d)\n"
"  -x IDX      index file for timestamps (default CAPTURE.gray.idx)\n"
"  -F FPS      frame rate for the frequency axis (default: from the .idx/.meta)\n"
"  -W, -H      geometry (default: from the .meta, else %dx%d)\n"
"  -q          summary only, no per-frame progress\n",
        me, DEF_RATIO, DEF_RATIO, DEF_MINAREA, DEF_W, DEF_H);
}

int main(int argc, char **argv)
{
    const char *in = NULL, *out = NULL, *idxp = NULL, *vidout = NULL;
    unsigned W = DEF_W, H = DEF_H;
    int fixed_thr = 0, quiet = 0, wh_set = 0, c;
    long minarea = DEF_MINAREA;
    double ratio = DEF_RATIO, fps_opt = 0;

    while ((c = getopt(argc, argv, "i:o:O:t:r:a:x:F:W:H:qh")) != -1) {
        switch (c) {
        case 'i': in = optarg; break;
        case 'o': out = optarg; break;
        case 'O': vidout = optarg; break;
        case 't': fixed_thr = atoi(optarg); break;
        case 'r': ratio = atof(optarg); break;
        case 'a': minarea = atol(optarg); break;
        case 'x': idxp = optarg; break;
        case 'F': fps_opt = atof(optarg); break;
        case 'W': W = (unsigned)strtoul(optarg, NULL, 10); wh_set = 1; break;
        case 'H': H = (unsigned)strtoul(optarg, NULL, 10); wh_set = 1; break;
        case 'q': quiet = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (!in) { usage(argv[0]); return 2; }
    if (fixed_thr && (fixed_thr < 1 || fixed_thr > 254)) die("-t must be 1..254");
    if (ratio <= 0 || ratio >= 1) die("-r must be between 0 and 1");

    char buf_idx[700], buf_meta[700], buf_out[700];
    if (!idxp) { snprintf(buf_idx, sizeof buf_idx, "%s.idx", in); idxp = buf_idx; }
    snprintf(buf_meta, sizeof buf_meta, "%s.meta", in);
    if (!out) { snprintf(buf_out, sizeof buf_out, "%s.blob.csv", in); out = buf_out; }
    int to_stdout = strcmp(out, "-") == 0;

    /* geometry from the .meta unless overridden */
    if (!wh_set) {
        FILE *m = fopen(buf_meta, "r");
        if (m) {
            char l[512]; unsigned w = 0, h = 0;
            while (fgets(l, sizeof l, m)) {
                sscanf(l, "width %u", &w);
                sscanf(l, "height %u", &h);
            }
            fclose(m);
            if (w && h) { W = w; H = h;
                if (!quiet) fprintf(stderr, "meta        %ux%u (from %s)\n", W, H, buf_meta); }
        }
    }
    if (W < 4 || H < 4 || (double)W * H > 5e8) die("geometry %ux%u out of range", W, H);

    /* timestamps, if the index is there */
    long ntime = 0, cap_t = 4096;
    double *tt = malloc(sizeof(double) * cap_t);
    if (!tt) die("out of memory");
    FILE *fi = fopen(idxp, "r");
    if (fi) {
        char l[512];
        while (fgets(l, sizeof l, fi)) {
            if (l[0] == '#') continue;
            unsigned long fr; unsigned sq, by, fl; double ts;
            if (sscanf(l, "%lu %u %lf %u %x", &fr, &sq, &ts, &by, &fl) != 5) continue;
            if (ntime == cap_t) { cap_t *= 2; tt = realloc(tt, sizeof(double) * cap_t);
                                  if (!tt) die("out of memory"); }
            tt[ntime++] = ts;
        }
        fclose(fi);
        if (!quiet) fprintf(stderr, "index       %ld timestamps (from %s)\n", ntime, idxp);
    } else if (!quiet) {
        fprintf(stderr, "index       %s not found -- times will come from -F/assumed fps\n", idxp);
    }

    int fd = open(in, O_RDONLY);
    if (fd < 0) die("cannot open %s: %s", in, strerror(errno));
    struct stat st;
    if (fstat(fd, &st) != 0) die("stat %s: %s", in, strerror(errno));
    size_t fsz = (size_t)W * H;
    long nframes = st.st_size / fsz;
    if (nframes < 2) die("%s holds only %ld frame(s) at %ux%u", in, nframes, W, H);
    if (!quiet)
        fprintf(stderr, "capture     %s  %.2f GB  %ld frames of %ux%u\n",
                in, st.st_size / 1e9, nframes, W, H);

    /* frame rate for the frequency axis */
    double fps = fps_opt;
    if (fps <= 0 && ntime >= 2) {
        double span = tt[ntime - 1] - tt[0];
        if (span > 0) fps = (ntime - 1) / span;
    }
    if (fps <= 0) { fps = 25.0;
        if (!quiet) fprintf(stderr, "note        no timestamps; assuming %.1f fps for frequency\n", fps); }

    uint8_t  *img   = malloc(fsz);
    int32_t  *lab   = malloc(sizeof(int32_t) * fsz);
    int32_t  *stack = malloc(sizeof(int32_t) * fsz);
    double   *cxs   = malloc(sizeof(double) * nframes);
    double   *cys   = malloc(sizeof(double) * nframes);
    if (!img || !lab || !stack || !cxs || !cys) die("out of memory");

    FILE *fo = to_stdout ? stdout : fopen(out, "w");
    if (!fo) die("cannot create %s: %s", out, strerror(errno));
    fprintf(fo, "# detect_blob: brightest connected component per frame\n");
    fprintf(fo, "# capture=%s  geometry=%ux%u\n", in, W, H);
    fprintf(fo, "# threshold=%s  min_area=%ld px\n",
            fixed_thr ? "fixed" : "adaptive (ratio x frame peak)", minarea);
    fprintf(fo, "# cx,cy are the intensity-weighted centroid in pixels\n");
    fprintf(fo, "# ncomp>1 means several components qualified; the largest was taken\n");
    fprintf(fo, "frame,t,cx,cy,area,peak,thr,ncomp,x0,y0,x1,y1\n");

    FILE *fv = NULL;
    /* When the annotated video goes to stdout, the report must not: appending
     * text to the raw stream corrupts it, and ffmpeg reports it as a truncated
     * final frame. Send the report to stderr in that case. */
    FILE *rep = stdout;
    if (vidout) {
        fv = (strcmp(vidout, "-") == 0) ? stdout : fopen(vidout, "wb");
        if (!fv) die("cannot create %s: %s", vidout, strerror(errno));
        if (fv == stdout) rep = stderr;
    }

    long nvalid = 0, nmulti = 0;
    double t0 = ntime ? tt[0] : 0;

    for (long f = 0; f < nframes; f++) {
        if (pread(fd, img, fsz, (off_t)f * fsz) != (ssize_t)fsz) {
            fprintf(stderr, "\nshort read at frame %ld -- truncated?\n", f);
            nframes = f;
            break;
        }
        struct det d = detect(img, W, H, fixed_thr, ratio, minarea, lab, stack);
        d.t = (f < ntime) ? tt[f] - t0 : f / fps;

        if (d.valid) {
            cxs[nvalid] = d.cx; cys[nvalid] = d.cy; nvalid++;
            if (d.ncomp > 1) nmulti++;
            fprintf(fo, "%ld,%.6f,%.3f,%.3f,%ld,%d,%d,%d,%d,%d,%d,%d\n",
                    f, d.t, d.cx, d.cy, d.area, d.peak, d.thr, d.ncomp,
                    d.x0, d.y0, d.x1, d.y1);
        } else {
            fprintf(fo, "%ld,%.6f,,,,,%d,0,,,,\n", f, d.t, d.thr);
        }
        if (fv) {
            annotate(img, W, H, &d);
            if (fwrite(img, 1, fsz, fv) != fsz)
                die("writing %s: %s", vidout, strerror(errno));
        }
        if (!quiet && (f % 25 == 0 || f == nframes - 1)) {
            fprintf(stderr, "\r  %ld/%ld frames, %ld with a blob", f + 1, nframes, nvalid);
            fflush(stderr);
        }
    }
    close(fd);
    if (fo != stdout) fclose(fo);
    if (fv && fv != stdout) fclose(fv);
    if (!quiet) fputc('\n', stderr);

    fprintf(rep, "\n=== detection ===\n");
    fprintf(rep, "  frames            %ld\n", nframes);
    fprintf(rep, "  blob found in     %ld  (%.1f%%)\n", nvalid, 100.0 * nvalid / nframes);
    if (nmulti)
        fprintf(rep, "  multiple comps    %ld frame(s) had >1 qualifying component;\n"
               "                    the largest was taken. Raise -a or -r if that\n"
               "                    is picking up something other than the blob.\n", nmulti);
    if (nvalid < 8) {
        fprintf(rep, "\n  too few detections to analyse oscillation.\n"
               "  Try a lower -r (currently %.2f) or a smaller -a (currently %ld).\n",
               ratio, minarea);
        return 1;
    }

    /* extent */
    double xmin = cxs[0], xmax = cxs[0], ymin = cys[0], ymax = cys[0];
    for (long i = 1; i < nvalid; i++) {
        if (cxs[i] < xmin) xmin = cxs[i];
        if (cxs[i] > xmax) xmax = cxs[i];
        if (cys[i] < ymin) ymin = cys[i];
        if (cys[i] > ymax) ymax = cys[i];
    }
    fprintf(rep, "\n=== position ===\n");
    fprintf(rep, "  x  %.1f .. %.1f px   (swing %.1f)\n", xmin, xmax, xmax - xmin);
    fprintf(rep, "  y  %.1f .. %.1f px   (swing %.1f)\n", ymin, ymax, ymax - ymin);

    double fx, ax, px, fy, ay, py;
    dominant(cxs, (int)nvalid, fps, &fx, &ax, &px);
    dominant(cys, (int)nvalid, fps, &fy, &ay, &py);
    int cx_cross = mean_crossings(cxs, (int)nvalid);
    int cy_cross = mean_crossings(cys, (int)nvalid);
    double dur = nvalid / fps;

    fprintf(rep, "\n=== oscillation (detrended, %.2f fps, %.2f s) ===\n", fps, dur);
    fprintf(rep, "  %-5s %-10s %-11s %-12s %-9s %s\n",
           "axis", "freq Hz", "period s", "amplitude", "power", "cycles (crossings/2)");
    fprintf(rep, "  %-5s %-10.3f %-11.3f %-12.2f %-9.2f %.1f\n",
           "x", fx, fx > 0 ? 1 / fx : 0, ax, px, cx_cross / 2.0);
    fprintf(rep, "  %-5s %-10.3f %-11.3f %-12.2f %-9.2f %.1f\n",
           "y", fy, fy > 0 ? 1 / fy : 0, ay, py, cy_cross / 2.0);

    const char *axis = (ax >= ay) ? "x" : "y";
    double fdom = (ax >= ay) ? fx : fy, adom = (ax >= ay) ? ax : ay;
    double pdom = (ax >= ay) ? px : py;
    int cdom = (ax >= ay) ? cx_cross : cy_cross;
    fprintf(rep, "\n  dominant: %s axis, %.3f Hz (%.2f s period), amplitude %.1f px\n",
           axis, fdom, fdom > 0 ? 1 / fdom : 0, adom);
    fprintf(rep, "  DFT expects %.1f cycles over %.2f s; crossings give %.1f\n",
           fdom * dur, dur, cdom / 2.0);

    /* the two estimates disagreeing is the signal that this is not a clean
     * single-frequency oscillation, and is worth saying rather than hiding */
    double dft_cycles = fdom * dur, cr_cycles = cdom / 2.0;
    fprintf(rep, "\n=== caveats ===\n");
    int warned = 0;
    if (dft_cycles > 0.5 && fabs(dft_cycles - cr_cycles) > 0.35 * dft_cycles) {
        fprintf(rep, "  * the DFT and the crossing count disagree (%.1f vs %.1f cycles),\n"
               "    so the motion is probably not a clean single frequency.\n",
               dft_cycles, cr_cycles);
        warned = 1;
    }
    if (pdom < 0.25) {
        fprintf(rep, "  * the dominant bin holds only %.0f%% of the detrended power --\n"
               "    the motion is broadband, not a single tone.\n", pdom * 100);
        warned = 1;
    }
    if (nvalid < nframes) {
        fprintf(rep, "  * the blob was missing in %ld of %ld frames; gaps were dropped\n"
               "    from the series rather than interpolated, which shifts the\n"
               "    frequency slightly.\n", nframes - nvalid, nframes);
        warned = 1;
    }
    fprintf(rep, "  * position is measured in the IMAGE. If the camera was also moving,\n"
           "    this is blob motion plus camera motion. flow_stamp's dy_px/dx_px\n"
           "    columns measure the camera's own motion, to separate the two.\n");
    if (!warned) fprintf(rep, "  (nothing else -- the oscillation looks clean)\n");

    if (!to_stdout) fprintf(rep, "\n  csv  %s\n", out);
    if (vidout && strcmp(vidout, "-") != 0) {
        fprintf(rep, "  raw  %s  (annotated %ux%u GREY)\n", vidout, W, H);
        fprintf(rep, "\n  to mp4:\n"
               "    ffmpeg -f rawvideo -pixel_format gray -video_size %ux%u \\\n"
               "           -framerate %.4f -i %s \\\n"
               "           -c:v libx264 -crf 20 -pix_fmt yuv420p out.mp4\n",
               W, H, fps, vidout);
    }
    free(img); free(lab); free(stack); free(cxs); free(cys); free(tt);
    return 0;
}
