/* flow_post.c -- OFFLINE optical flow over a record_raw capture, merged with
 * logged gimbal telemetry, into one CSV.
 *
 * Nothing here touches the camera. It reads a finished .gray file, so no amount
 * of compute can cost a frame -- which is the whole point of doing it this way
 * rather than during capture.
 *
 *   ./record_raw -t 60 &                          # frames + .idx timestamps
 *   ./tlm_log -t 60 -o rec.tlm.csv                # angles, no camera contact
 *   ./flow_post -i recordings/.../raw-1280x1024-GREY.gray -a rec.tlm.csv
 *
 * Inputs:
 *   <capture>.gray   raw frames back to back, exactly as record_raw wrote them
 *   <capture>.idx    per-frame kernel sequence, timestamp, byte count. This is
 *                    what makes post-processing exact: frame offsets come from
 *                    the recorded byte counts, so a short frame cannot shift
 *                    every later frame, and the timestamps are the kernel's own.
 *   <capture>.meta   geometry (read automatically; -W/-H override)
 *   -a <tlm.csv>     optional tlm_log output; without it, flow columns are
 *                    still produced and the angle columns are left empty.
 *
 * FLOW METHOD (from ~/drone_detection/flow_calc.c). Each frame collapses to two
 * 1-D profiles, row means and column means, and consecutive profiles are aligned
 * by the shift minimising mean absolute difference. Collapsing first turns an
 * O(W*H*shifts) 2-D search into O((W+H)*shifts). Each profile has its mean
 * removed so auto-exposure hunting cannot masquerade as motion, and is
 * high-passed with a phase-neutral forward+backward running mean, because row
 * means are dominated by lens vignetting and sensor shading that are fixed to
 * the SENSOR and do not move when the scene does -- leave that in and the
 * matcher locks onto zero shift no matter how far the image travelled. The cost
 * minimum is refined to sub-pixel with a parabola.
 *
 * SIGN: dy_px is the displacement of image CONTENT from the previous frame to
 * this one, positive = content moved DOWN the sensor. dx_px positive = moved
 * RIGHT. Both axes are measured because the camera's mounting orientation
 * decides which image axis a given rotation appears on.
 *
 * t_flow: a displacement is measured BETWEEN two frames, so it belongs to the
 * MIDPOINT of that interval, not to the later frame. Stamping it with t_frame
 * pushes every sample dt/2 (~15-20 ms here) late and biases any latency derived
 * from it by exactly that much. t_frame is kept so a row still maps to its
 * frame in the .gray; t_flow is the one to correlate against.
 *
 * dy_px is per frame-interval and the intervals are not uniform, so dy_rate
 * (px/s) is emitted too. The gimbal rate columns span the SAME interval, so
 * dy_rate_px_s / pitch_rate_dps is px-per-degree directly.
 *
 * quality is how far the best match sits below the mean cost, 0..1. Near 0 means
 * the profile had nothing to lock onto (blank wall, motion blur) and that row's
 * flow should not be trusted. Rows are still emitted -- filtering is the
 * analyst's call, not this program's.
 *
 * DEVIATION FROM flow_calc.c: that file high-passes the ROW profile inline and
 * then again inside align(), while the COLUMN profile is high-passed only once
 * inside align(). Filtering the two axes differently is not defensible, so here
 * both are high-passed exactly once. dy_px may therefore differ slightly from
 * flow_calc's on identical input; dx_px is unchanged.
 *
 * Build:  gcc -O2 -Wall -Wextra -o flow_post flow_post.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DEF_W    1280
#define DEF_H    1024
#define DEF_SPAN 300
#define MAXPROF  8192
#define MAXSPAN  1000
#define HP_WIN   101

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fputc('\n', stderr); vfprintf(stderr, fmt, ap); va_end(ap); fputc('\n', stderr);
    exit(1);
}

static double now_mono(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* Pitch wraps at +/-180, so a plain subtraction across the wrap yields a
 * spurious 360 deg/interval. Fold into (-180,180]. */
static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

/* -- 1-D alignment -------------------------------------------------------- */

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
        /* backward pass keeps the smoothing phase-neutral, so the high-passed
         * profile is not itself shifted */
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

/* -- telemetry table ------------------------------------------------------ */

struct angle { double t; double yaw, pitch; };
static struct angle *angles = NULL;
static size_t nang = 0, cap_ang = 0;

static void ang_push(double t, double yaw, double pitch)
{
    if (nang == cap_ang) {
        cap_ang = cap_ang ? cap_ang * 2 : 65536;
        angles = realloc(angles, cap_ang * sizeof *angles);
        if (!angles) die("out of memory holding %zu telemetry samples", cap_ang);
    }
    angles[nang].t = t; angles[nang].yaw = yaw; angles[nang].pitch = pitch;
    nang++;
}

/* Binary search for the sample nearest t. The table is time-ordered because
 * tlm_log appends in arrival order; verified on load. */
static const struct angle *ang_nearest(double t)
{
    if (!nang) return NULL;
    size_t lo = 0, hi = nang - 1;
    if (t <= angles[0].t) return &angles[0];
    if (t >= angles[hi].t) return &angles[hi];
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (angles[mid].t <= t) lo = mid; else hi = mid;
    }
    double dl = t - angles[lo].t, dh = angles[hi].t - t;
    return (dl <= dh) ? &angles[lo] : &angles[hi];
}

static void load_tlm(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) die("cannot open telemetry %s: %s", path, strerror(errno));
    char line[1024];
    unsigned long bad = 0, unordered = 0;
    double prev = -1;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        double t, yaw, pitch;
        if (sscanf(line, "%lf,%lf,%lf", &t, &yaw, &pitch) != 3) { bad++; continue; }
        if (prev >= 0 && t < prev) unordered++;
        prev = t;
        ang_push(t, yaw, pitch);
    }
    fclose(f);
    if (!nang) die("no telemetry rows parsed from %s", path);
    if (unordered)
        die("telemetry %s is not time-ordered (%lu backward steps) -- the\n"
            "  nearest-sample search assumes order; sort it first",
            path, unordered);
    fprintf(stderr, "telemetry   %zu samples, %.3f..%.3f s (%.1f Hz)\n",
            nang, angles[0].t, angles[nang-1].t,
            nang > 1 ? (nang - 1) / (angles[nang-1].t - angles[0].t) : 0.0);
    if (bad > 1)   /* 1 is the expected column-header line */
        fprintf(stderr, "            %lu unparsable line(s) skipped\n", bad);
}

/* -- meta / idx ----------------------------------------------------------- */

static void load_meta(const char *path, unsigned *W, unsigned *H, int *found)
{
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[512];
    unsigned w = 0, h = 0;
    char fourcc[32] = "";
    while (fgets(line, sizeof line, f)) {
        if (sscanf(line, "width %u", &w) == 1) continue;
        if (sscanf(line, "height %u", &h) == 1) continue;
        sscanf(line, "fourcc %31s", fourcc);
    }
    fclose(f);
    if (w && h) {
        *W = w; *H = h; *found = 1;
        fprintf(stderr, "meta        %ux%u %s (from %s)\n", w, h,
                fourcc[0] ? fourcc : "?", path);
        if (fourcc[0] && strcmp(fourcc, "GREY") != 0)
            die("capture is %s, not GREY -- flow here assumes 1 byte per pixel",
                fourcc);
    }
}

static void usage(const char *me)
{
    printf(
"Offline optical flow over a record_raw capture, merged with tlm_log telemetry.\n"
"Reads a finished file -- cannot drop frames.\n"
"\n"
"usage: %s -i CAPTURE.gray [-a TLM.CSV] [-o OUT.CSV] [-s SPAN]\n"
"          [-W W] [-H H] [-x IDX] [-h]\n"
"\n"
"  -i FILE    the .gray written by record_raw (required)\n"
"  -a CSV     tlm_log telemetry to merge (optional; angle columns empty without)\n"
"  -o CSV     output (default: CAPTURE.gray.flow.csv; '-' for stdout)\n"
"  -s SPAN    max shift searched, px (default %d)\n"
"  -x IDX     index file (default: CAPTURE.gray.idx)\n"
"  -W, -H     geometry override (default: read from CAPTURE.gray.meta, else %dx%d)\n"
"\n"
"Output columns -- the first 7 match sync_log's:\n"
"  seq,t_frame,yaw,pitch,t_angle,dt_ms,n_angles,\n"
"  t_flow,dy_px,dy_rate_px_s,qual_y,dx_px,dx_rate_px_s,qual_x,\n"
"  d_yaw_deg,d_pitch_deg,yaw_rate_dps,pitch_rate_dps\n"
"\n"
"dy_px>0 = content moved DOWN; dx_px>0 = moved RIGHT.\n"
"t_flow is the interval midpoint -- correlate against that, not t_frame.\n",
        me, DEF_SPAN, DEF_W, DEF_H);
}

int main(int argc, char **argv)
{
    const char *in = NULL, *tlm = NULL, *out = NULL, *idxp = NULL;
    unsigned W = DEF_W, H = DEF_H;
    int span = DEF_SPAN, meta_found = 0, wh_override = 0, c;

    while ((c = getopt(argc, argv, "i:a:o:s:x:W:H:h")) != -1) {
        switch (c) {
        case 'i': in = optarg; break;
        case 'a': tlm = optarg; break;
        case 'o': out = optarg; break;
        case 's': span = atoi(optarg); break;
        case 'x': idxp = optarg; break;
        case 'W': W = (unsigned)strtoul(optarg, NULL, 10); wh_override = 1; break;
        case 'H': H = (unsigned)strtoul(optarg, NULL, 10); wh_override = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (!in) { usage(argv[0]); return 2; }
    if (span < 1) die("-s must be >= 1");

    char buf_idx[700], buf_meta[700], buf_out[700];
    if (!idxp) { snprintf(buf_idx, sizeof buf_idx, "%s.idx", in); idxp = buf_idx; }
    snprintf(buf_meta, sizeof buf_meta, "%s.meta", in);
    if (!out) { snprintf(buf_out, sizeof buf_out, "%s.flow.csv", in); out = buf_out; }
    int to_stdout = (strcmp(out, "-") == 0);

    if (!wh_override) load_meta(buf_meta, &W, &H, &meta_found);
    if (!meta_found && !wh_override)
        fprintf(stderr, "meta        %s unreadable -- assuming %ux%u\n",
                buf_meta, W, H);
    if (W < 2 || H < 2 || W > MAXPROF || H > MAXPROF)
        die("geometry %ux%u out of range", W, H);

    if (tlm) load_tlm(tlm);
    else fprintf(stderr, "telemetry   none given -- angle columns will be empty\n");

    FILE *fi = fopen(idxp, "r");
    if (!fi) die("cannot open index %s: %s\n"
                 "  record_raw writes it unless -I was used; without it there\n"
                 "  are no per-frame timestamps to pair against.",
                 idxp, strerror(errno));
    int rawfd = open(in, O_RDONLY);
    if (rawfd < 0) die("cannot open %s: %s", in, strerror(errno));
    struct stat st;
    if (fstat(rawfd, &st) == 0)
        fprintf(stderr, "capture     %s  %.2f GB\n", in, st.st_size / 1e9);

    FILE *fo = to_stdout ? stdout : fopen(out, "w");
    if (!fo) die("cannot create %s: %s", out, strerror(errno));

    size_t fsz = (size_t)W * H;
    uint8_t *img = malloc(fsz);
    double *rprof = malloc(sizeof(double) * H), *rprev = malloc(sizeof(double) * H);
    double *cprof = malloc(sizeof(double) * W), *cprev = malloc(sizeof(double) * W);
    unsigned long *rsum = malloc(sizeof(unsigned long) * H);
    unsigned long *csum = malloc(sizeof(unsigned long) * W);
    if (!img || !rprof || !rprev || !cprof || !cprev || !rsum || !csum)
        die("out of memory");

    fprintf(fo, "# flow_post: optical flow + gimbal angle, one row per frame\n");
    fprintf(fo, "# capture=%s idx=%s\n", in, idxp);
    fprintf(fo, "# telemetry=%s\n", tlm ? tlm : "(none)");
    fprintf(fo, "# geometry=%ux%u GREY  flow_span=%d px\n", W, H, span);
    fprintf(fo, "# clock=CLOCK_MONOTONIC for t_frame, t_angle, t_flow\n");
    fprintf(fo, "# dy_px>0 = content moved DOWN; dx_px>0 = moved RIGHT\n");
    fprintf(fo, "# t_flow = interval midpoint; correlate against t_flow\n");
    fprintf(fo, "seq,t_frame,yaw,pitch,t_angle,dt_ms,n_angles,"
                "t_flow,dy_px,dy_rate_px_s,qual_y,"
                "dx_px,dx_rate_px_s,qual_x,"
                "d_yaw_deg,d_pitch_deg,yaw_rate_dps,pitch_rate_dps\n");

    char line[512];
    unsigned long nrow = 0, nflow = 0, lowq = 0, nomatch = 0, skipped = 0;
    unsigned long dropped = 0;
    double sum_absdy = 0, sum_qy = 0, sum_qx = 0, sum_dt = 0, worst_dt = 0;
    double prev_ft = 0, prev_yaw = 0, prev_pitch = 0;
    int have_prev = 0, have_prev_ang = 0, have_seq = 0;
    uint32_t last_seq = 0;
    /* record_raw writes frames back to back using bytesused, so an offset is
     * the running sum of every earlier frame's byte count -- taken from the
     * index, never assumed to be a fixed stride. */
    off_t offset = 0;
    double t_start = now_mono();

    while (fgets(line, sizeof line, fi)) {
        if (line[0] == '#') continue;
        unsigned long fidx; unsigned seq, bytes, flags;
        double ts;
        if (sscanf(line, "%lu %u %lf %u %x", &fidx, &seq, &ts, &bytes, &flags) != 5)
            continue;

        off_t this_off = offset;
        offset += bytes;

        if (bytes != fsz) {
            /* Short or empty frame: profiling it would inject a bogus
             * displacement, and it must still advance the offset (done above). */
            skipped++;
            have_prev = 0;      /* the chain is broken; do not pair across it */
            continue;
        }
        if (pread(rawfd, img, fsz, this_off) != (ssize_t)fsz) {
            fprintf(stderr, "\nshort read at frame %lu (offset %lld) -- "
                            ".gray truncated?\n", fidx, (long long)this_off);
            break;
        }

        if (have_seq) {
            if (seq > last_seq + 1) dropped += seq - last_seq - 1;
        }
        last_seq = seq; have_seq = 1;

        /* one sequential pass accumulating BOTH row and column sums; a separate
         * y-inner loop for columns would stride the cache and cost far more */
        memset(csum, 0, sizeof(unsigned long) * W);
        for (unsigned y = 0; y < H; y++) {
            const uint8_t *row = img + (size_t)y * W;
            unsigned long rs = 0;
            for (unsigned x = 0; x < W; x++) {
                unsigned v = row[x];
                rs += v;
                csum[x] += v;
            }
            rsum[y] = rs;
        }
        /* mean-remove: a global brightness change must not look like motion */
        double gs = 0;
        for (unsigned y = 0; y < H; y++) { rprof[y] = (double)rsum[y] / W; gs += rprof[y]; }
        double gm = gs / H;
        for (unsigned y = 0; y < H; y++) rprof[y] -= gm;
        gs = 0;
        for (unsigned x = 0; x < W; x++) { cprof[x] = (double)csum[x] / H; gs += cprof[x]; }
        gm = gs / W;
        for (unsigned x = 0; x < W; x++) cprof[x] -= gm;

        const struct angle *a = ang_nearest(ts);
        double dt_ang = 0;
        if (a) {
            dt_ang = (ts - a->t) * 1000.0;
            double ad = dt_ang < 0 ? -dt_ang : dt_ang;
            if (ad > worst_dt) worst_dt = ad;
            sum_dt += ad;
        } else if (tlm) {
            nomatch++;
        }

        int flow_ok = 0;
        double dy = 0, dx = 0, qy = 0, qx = 0, tf = 0, dtf = 0;
        if (have_prev && prev_ft > 0) {
            dy = align_prof(rprev, rprof, (int)H, span, &qy);
            dx = align_prof(cprev, cprof, (int)W, span, &qx);
            dtf = ts - prev_ft;
            tf = prev_ft + dtf / 2.0;     /* midpoint, not the later frame */
            flow_ok = 1; nflow++;
            sum_absdy += dy < 0 ? -dy : dy;
            sum_qy += qy; sum_qx += qx;
            if (qy < 0.05 || qx < 0.05) lowq++;
        }

        if (a)
            fprintf(fo, "%u,%.6f,%.4f,%.4f,%.6f,%.3f,%zu",
                    seq, ts, a->yaw, a->pitch, a->t, dt_ang, nang);
        else
            fprintf(fo, "%u,%.6f,,,,,0", seq, ts);

        if (flow_ok)
            fprintf(fo, ",%.6f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f",
                    tf, dy, dtf > 0 ? dy / dtf : 0.0, qy,
                    dx, dtf > 0 ? dx / dtf : 0.0, qx);
        else
            fprintf(fo, ",,,,,,,");

        if (flow_ok && a && have_prev_ang) {
            double dyaw = wrap180(a->yaw - prev_yaw);
            double dpit = wrap180(a->pitch - prev_pitch);
            fprintf(fo, ",%.4f,%.4f,%.3f,%.3f", dyaw, dpit,
                    dtf > 0 ? dyaw / dtf : 0.0, dtf > 0 ? dpit / dtf : 0.0);
        } else {
            fprintf(fo, ",,,,");
        }
        fputc('\n', fo);

        memcpy(rprev, rprof, sizeof(double) * H);
        memcpy(cprev, cprof, sizeof(double) * W);
        prev_ft = ts;
        have_prev = 1;
        if (a) { prev_yaw = a->yaw; prev_pitch = a->pitch; have_prev_ang = 1; }
        nrow++;
        if (nrow % 100 == 0) {
            fprintf(stderr, "\r  %lu frames, %lu flow values", nrow, nflow);
            fflush(stderr);
        }
    }

    double el = now_mono() - t_start;
    fclose(fi);
    close(rawfd);
    if (fo != stdout) fclose(fo);

    fprintf(stderr, "\r                                        \r");
    fprintf(stderr, "frames     %lu read, %lu flow values in %.2f s (%.0f fps)\n",
            nrow, nflow, el, el > 0 ? nrow / el : 0.0);
    if (skipped)
        fprintf(stderr, "skipped    %lu frame(s) not %zu bytes (short/error at"
                        " capture); flow not paired across them\n", skipped, fsz);
    if (dropped)
        fprintf(stderr, "gaps       %lu frame(s) were already missing at capture"
                        " (sequence gaps) -- intervals across them are longer\n",
                dropped);
    if (nflow)
        fprintf(stderr, "flow       mean |dy| %.2f px, mean quality y=%.3f x=%.3f,"
                        " low-quality %lu (%.1f%%)\n",
                sum_absdy / nflow, sum_qy / nflow, sum_qx / nflow, lowq,
                100.0 * lowq / nflow);
    if (tlm && nrow)
        fprintf(stderr, "match      mean |dt| %.3f ms, worst %.3f ms, unmatched %lu\n",
                sum_dt / (double)(nrow - nomatch ? nrow - nomatch : 1),
                worst_dt, nomatch);
    if (!to_stdout) fprintf(stderr, "csv        %s\n", out);

    int bad = (nrow == 0 || nflow == 0);
    if (bad)
        fprintf(stderr, "\nFAILED     produced nothing usable (%lu frames,"
                        " %lu flow values)\n", nrow, nflow);
    free(img); free(rprof); free(rprev); free(cprof); free(cprev);
    free(rsum); free(csum); free(angles);
    return bad ? 1 : 0;
}
