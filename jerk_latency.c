/* jerk_latency.c -- count jerks in a flow_stamp frames.csv, two independent ways,
 * and measure the lag between them.
 *
 *   ./jerk_latency recordings/2026-08-31/172848/frames.csv
 *
 * The measurement: a jerk is applied to the camera platform. The IMU sees it
 * immediately; the camera sees it some frames later, because the thermal core
 * buffers internally before sending. Detect the onset in each signal
 * independently, pair them, and the difference is the camera's pipeline delay.
 *
 * Onsets are used rather than peaks or cross-correlation on purpose. At the start
 * of a jerk the motion is small, so the flow measurement is still inside its
 * search span and its quality is still high. At the peak, fast motion clips
 * against the span limit and motion blur collapses the match quality -- which
 * biases any peak- or correlation-based estimate toward the surviving tail of
 * each jerk. Onset detection is immune to that.
 *
 * TWO SIGNALS, both magnitudes so the result does not depend on which axis the
 * platform was jerked along:
 *   IMU  sqrt(yaw_rate_dps^2 + pitch_rate_dps^2)
 *   flow sqrt(dy_rate_px_s^2 + dx_rate_px_s^2), rows with flow_state == ok only
 *
 * THRESHOLDS are set from each run's own quiet baseline, not hardcoded: the mean
 * of every sample below the overall mean approximates the resting noise level,
 * and the threshold sits above that. Two runs with different scene contrast or
 * gimbal noise therefore get comparable treatment. Both terms are adjustable.
 *
 * COLUMNS ARE FOUND BY NAME from the header line, not by position. Reading
 * yaw_rate from a fixed column index breaks silently the moment the CSV layout
 * changes, and a silently wrong column produces a plausible-looking wrong answer.
 *
 * WHAT THIS DOES NOT DO: convert the answer into a single system latency. Across
 * runs the frame-domain figure has ranged from 1.9 to 5.9 frames, far more than
 * the within-run spread, so something between runs dominates -- camera fps mode
 * and USB bus contention are the known candidates. This tool reports each run
 * honestly and flags the conditions that make a run untrustworthy; deciding
 * which runs are comparable is a judgement it deliberately leaves to you.
 *
 * Build:  gcc -O2 -Wall -Wextra -o jerk_latency jerk_latency.c -lm
 */
#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAXROW   2000000
#define MAXCOL   64
#define LINEBUF  8192

/* defaults */
#define DEF_IMU_ADD   30.0    /* deg/s above the quiet baseline           */
#define DEF_FLOW_MULT 4.0     /* flow threshold = baseline*MULT + ADD     */
#define DEF_FLOW_ADD  200.0   /* px/s                                     */
#define DEF_WINDOW    15      /* frames to search forward for a flow onset */
#define DEF_RELEASE   4       /* frames below threshold that end a jerk    */

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fputs("error: ", stderr); vfprintf(stderr, fmt, ap); va_end(ap);
    fputc('\n', stderr);
    exit(1);
}

/* ---- CSV helpers -------------------------------------------------------- */

/* Split a line on commas in place. Returns the field count. Empty fields stay
 * empty strings, which is how "not applicable" is encoded in these files. */
static int split(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < max; p++)
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    /* strip trailing newline from the last field */
    char *e = f[n-1] + strlen(f[n-1]);
    while (e > f[n-1] && (e[-1] == '\n' || e[-1] == '\r')) *--e = '\0';
    return n;
}

static int col_index(char **hdr, int nh, const char *name)
{
    for (int i = 0; i < nh; i++) if (strcmp(hdr[i], name) == 0) return i;
    return -1;
}

static int have(char **f, int n, int idx)
{
    return idx >= 0 && idx < n && f[idx][0] != '\0';
}

static int cmp_int(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return x < y ? -1 : x > y ? 1 : 0;
}

/* ---- onset detection ---------------------------------------------------- */

/* Mean of every valid sample strictly below the overall mean. A run where the
 * platform is mostly still is dominated by resting noise, so this lands near
 * that noise level without needing a sort or a hardcoded constant. */
static double quiet_baseline(const double *v, int n)
{
    double sum = 0; int cnt = 0;
    for (int i = 0; i < n; i++) if (v[i] >= 0) { sum += v[i]; cnt++; }
    if (!cnt) return 0;
    double mean = sum / cnt;
    double qs = 0; int qn = 0;
    for (int i = 0; i < n; i++) if (v[i] >= 0 && v[i] < mean) { qs += v[i]; qn++; }
    return qn ? qs / qn : mean;
}

/* Fill onset[] with the first frame index of each excursion above thr. An
 * excursion ends only after `release` consecutive frames back below thr, so one
 * jerk that dips momentarily is not counted twice. */
static int find_onsets(const double *v, int n, double thr, int release,
                       int *onset, int maxon)
{
    int k = 0, in = 0, gap = 0;
    for (int i = 0; i < n; i++) {
        if (v[i] >= 0 && v[i] > thr) {
            if (!in) { in = 1; if (k < maxon) onset[k++] = i; }
            gap = 0;
        } else if (in) {
            if (++gap >= release) in = 0;
        }
    }
    return k;
}

static void usage(const char *me)
{
    printf(
"Count jerks in a flow_stamp frames.csv two ways, then measure the lag.\n"
"\n"
"usage: %s [options] FRAMES.CSV\n"
"\n"
"  -i DPS     IMU threshold above its quiet baseline (default %.0f deg/s)\n"
"  -m MULT    flow threshold = baseline*MULT + ADD  (default MULT %.0f)\n"
"  -a PXS     flow threshold additive term          (default %.0f px/s)\n"
"  -w FRAMES  how far forward to look for the flow onset (default %d)\n"
"  -r FRAMES  frames below threshold that end a jerk (default %d)\n"
"  -q         summary only, no per-jerk table\n"
"  -c         emit the per-jerk table as CSV\n"
"\n"
"Reports: jerk count from the IMU, jerk count from the flow, then per-jerk lag\n"
"in frames and milliseconds with mean/median/sd and a histogram.\n",
        me, DEF_IMU_ADD, DEF_FLOW_MULT, DEF_FLOW_ADD, DEF_WINDOW, DEF_RELEASE);
}

int main(int argc, char **argv)
{
    double imu_add = DEF_IMU_ADD, flow_mult = DEF_FLOW_MULT, flow_add = DEF_FLOW_ADD;
    int window = DEF_WINDOW, release = DEF_RELEASE, quiet = 0, as_csv = 0, c;

    while ((c = getopt(argc, argv, "i:m:a:w:r:qch")) != -1) {
        switch (c) {
        case 'i': imu_add   = atof(optarg); break;
        case 'm': flow_mult = atof(optarg); break;
        case 'a': flow_add  = atof(optarg); break;
        case 'w': window    = atoi(optarg); break;
        case 'r': release   = atoi(optarg); break;
        case 'q': quiet = 1; break;
        case 'c': as_csv = 1; break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 2;
        }
    }
    if (optind >= argc) { usage(argv[0]); return 2; }
    const char *path = argv[optind];
    if (window < 1) die("-w must be >= 1");
    if (release < 1) die("-r must be >= 1");

    FILE *f = fopen(path, "r");
    if (!f) die("cannot open %s: %s", path, strerror(errno));

    char line[LINEBUF];
    char *fld[MAXCOL], *hdr[MAXCOL];
    char hdrline[LINEBUF];
    int nh = 0;

    /* find the header: the first non-comment line, which starts with "seq," */
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        snprintf(hdrline, sizeof hdrline, "%s", line);
        nh = split(hdrline, hdr, MAXCOL);
        break;
    }
    if (!nh) die("%s: no header line found", path);

    /* columns by name -- a fixed index would break silently on a layout change */
    struct { const char *name; int idx; } need[] = {
        { "t_first_byte",  -1 }, { "yaw_rate_dps",  -1 }, { "pitch_rate_dps", -1 },
        { "dy_rate_px_s",  -1 }, { "dx_rate_px_s",  -1 }, { "flow_state",     -1 },
    };
    for (size_t i = 0; i < sizeof need / sizeof *need; i++) {
        need[i].idx = col_index(hdr, nh, need[i].name);
        if (need[i].idx < 0)
            die("%s: header has no column '%s'\n"
                "  is this a flow_stamp frames.csv?", path, need[i].name);
    }
    const int C_T = need[0].idx, C_YR = need[1].idx, C_PR = need[2].idx,
              C_DY = need[3].idx, C_DX = need[4].idx, C_FS = need[5].idx;

    double *imu = malloc(sizeof(double) * MAXROW);
    double *flo = malloc(sizeof(double) * MAXROW);
    double *tfb = malloc(sizeof(double) * MAXROW);
    if (!imu || !flo || !tfb) die("out of memory");

    int n = 0;
    long n_flow_rows = 0, n_imu_rows = 0;
    while (fgets(line, sizeof line, f) && n < MAXROW) {
        if (line[0] == '#') continue;
        int nf = split(line, fld, MAXCOL);
        if (nf < 2 || fld[0][0] == '\0') continue;
        /* a data row starts with a numeric seq */
        if (!(fld[0][0] >= '0' && fld[0][0] <= '9')) continue;

        tfb[n] = have(fld, nf, C_T) ? atof(fld[C_T]) : -1;

        if (have(fld, nf, C_YR) && have(fld, nf, C_PR)) {
            double y = atof(fld[C_YR]), p = atof(fld[C_PR]);
            imu[n] = sqrt(y*y + p*p);
            n_imu_rows++;
        } else imu[n] = -1;

        int ok = have(fld, nf, C_FS) && strcmp(fld[C_FS], "ok") == 0;
        if (ok && have(fld, nf, C_DY) && have(fld, nf, C_DX)) {
            double dy = atof(fld[C_DY]), dx = atof(fld[C_DX]);
            flo[n] = sqrt(dy*dy + dx*dx);
            n_flow_rows++;
        } else flo[n] = -1;

        n++;
    }
    fclose(f);
    if (n < 2) die("%s: only %d data row(s)", path, n);

    /* mean frame interval, from the kernel's own first-byte stamps */
    double sg = 0; long ng = 0;
    for (int i = 1; i < n; i++)
        if (tfb[i] > 0 && tfb[i-1] > 0) { sg += (tfb[i] - tfb[i-1]) * 1000.0; ng++; }
    double interval = ng ? sg / ng : 0;

    double ibase = quiet_baseline(imu, n), fbase = quiet_baseline(flo, n);
    double ithr = ibase + imu_add, fthr = fbase * flow_mult + flow_add;

    int *ion = malloc(sizeof(int) * n), *fon = malloc(sizeof(int) * n);
    if (!ion || !fon) die("out of memory");
    int n_ion = find_onsets(imu, n, ithr, release, ion, n);
    int n_fon = find_onsets(flo, n, fthr, release, fon, n);

    /* ---- pair each IMU onset with the next flow onset ------------------- */
    int *gapf = malloc(sizeof(int) * (n_ion ? n_ion : 1));
    if (!gapf) die("out of memory");
    int m = 0, missed = 0, missed_at_eof = 0;

    if (!quiet) {
        if (as_csv) printf("jerk,imu_frame,flow_frame,gap_frames,gap_ms\n");
        else printf("%-5s %-9s %-10s %-8s %-9s\n",
                    "jerk", "imu_frm", "flow_frm", "gap_fr", "gap_ms");
    }
    for (int k = 0; k < n_ion; k++) {
        int i = ion[k], fj = -1;
        for (int j = i; j <= i + window && j < n; j++)
            if (flo[j] >= 0 && flo[j] > fthr) { fj = j; break; }
        if (fj < 0) {
            missed++;
            /* An onset too close to the end of file cannot match by construction;
             * that is a truncated recording, not a detection failure. */
            if (i + window >= n) missed_at_eof++;
            if (!quiet) {
                if (as_csv) printf("%d,%d,,,\n", k + 1, i);
                else printf("%-5d %-9d %-10s %-8s %-9s%s\n", k + 1, i, "none", "-", "-",
                            (i + window >= n) ? "  (near end of file)" : "");
            }
            continue;
        }
        int d = fj - i;
        gapf[m++] = d;
        if (!quiet) {
            if (as_csv) printf("%d,%d,%d,%d,%.1f\n", k + 1, i, fj, d, d * interval);
            else printf("%-5d %-9d %-10d %-8d %-9.1f\n", k + 1, i, fj, d, d * interval);
        }
    }

    /* ---- stats ---------------------------------------------------------- */
    printf("\n=== jerk counts ===\n");
    printf("  by IMU   %d\n", n_ion);
    printf("  by flow  %d\n", n_fon);
    printf("  rows     %d   (imu present %ld, flow ok %ld)\n", n, n_imu_rows, n_flow_rows);
    printf("  interval %.2f ms  (%.2f fps)\n", interval, interval > 0 ? 1000.0 / interval : 0);
    printf("  thresholds  imu %.1f dps (baseline %.2f)   flow %.0f px/s (baseline %.1f)\n",
           ithr, ibase, fthr, fbase);

    if (!m) {
        printf("\n=== latency ===\n  no jerk could be paired -- nothing to report\n");
        return 1;
    }

    int *srt = malloc(sizeof(int) * m);
    memcpy(srt, gapf, sizeof(int) * m);
    qsort(srt, m, sizeof(int), cmp_int);
    double sum = 0;
    for (int i = 0; i < m; i++) sum += gapf[i];
    double mean = sum / m, ss = 0;
    for (int i = 0; i < m; i++) { double d = gapf[i] - mean; ss += d * d; }
    double sd = m > 1 ? sqrt(ss / (m - 1)) : 0;
    double med = (m % 2) ? srt[m/2] : (srt[m/2 - 1] + srt[m/2]) / 2.0;

    printf("\n=== latency ===\n");
    printf("  matched   %d of %d IMU jerks", m, n_ion);
    if (missed) printf("  (unmatched %d%s)", missed,
                       missed_at_eof ? ", some at end of file" : "");
    printf("\n");
    printf("  frames    mean %.2f   median %.1f   sd %.2f   sem %.2f   range %d..%d\n",
           mean, med, sd, sd / sqrt(m), srt[0], srt[m-1]);
    printf("  ms        mean %.1f   median %.1f   sd %.1f   sem %.1f\n",
           mean * interval, med * interval, sd * interval, sd * interval / sqrt(m));

    printf("\n  histogram (frames)\n");
    for (int v = srt[0]; v <= srt[m-1]; v++) {
        int cnt = 0;
        for (int i = 0; i < m; i++) if (gapf[i] == v) cnt++;
        if (!cnt) continue;
        printf("    %2d frame%s %4d  ", v, v == 1 ? " " : "s", cnt);
        int bars = (int)(40.0 * cnt / m);
        for (int b = 0; b < bars; b++) putchar('#');
        putchar('\n');
    }

    /* ---- conditions that make a run untrustworthy ----------------------- */
    int warned = 0;
    printf("\n=== caveats ===\n");
    if (n_fon > n_ion) {
        printf("  * flow found MORE jerks than the IMU (%d vs %d). Extra flow\n"
               "    crossings mean the pairing can latch onto an unrelated one and\n"
               "    report a spuriously short gap. Raise -a or -m and re-run.\n",
               n_fon, n_ion);
        warned = 1;
    }
    if (srt[0] <= 1) {
        printf("  * a gap of %d frame(s) appeared. Physically the camera cannot see\n"
               "    a jerk before its own pipeline delay, so this is likely a false\n"
               "    flow crossing rather than a real measurement.\n", srt[0]);
        warned = 1;
    }
    if (sd > 1.5) {
        printf("  * sd is %.2f frames, wide enough that the mean is not a tight\n"
               "    estimate. Check whether the run had a stable frame rate.\n", sd);
        warned = 1;
    }
    if (interval > 0) {
        double fps = 1000.0 / interval;
        if (fabs(fps - 25.04) > 0.6 && fabs(fps - 32.80) > 0.6) {
            printf("  * %.2f fps is neither of this core's clean modes (25.04 / 32.80).\n"
                   "    A starved rate means the core is dropping frames internally and\n"
                   "    buffering more, which changes the very delay being measured.\n"
                   "    Runs at different delivered rates are not comparable.\n", fps);
            warned = 1;
        }
    }
    if (!warned) printf("  none -- the run looks internally consistent\n");

    free(imu); free(flo); free(tfb); free(ion); free(fon); free(gapf); free(srt);
    return 0;
}
