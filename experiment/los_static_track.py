#!/usr/bin/env python3
"""EXPERIMENT: track a single clicked point using only gimbal attitude, no detector.

    los_static_track.py FILE.rawrec [--los LOS.csv]

Click anywhere on the image to mark a real point on the target, assumed to be
static in the world. From then on, for every other frame, this predicts where
that same point should appear -- using only the change in the gimbal's own
attitude (the qw,qx,qy,qz quaternion logged in the matching los-*.csv) and a
pinhole reprojection. No detection algorithm runs here at all: the marker is a
pure "if the target never moved in the world, and only the camera rotated,
here is where it would be" prediction, drawn as a check against real footage.

KEYS
    click           set/replace the reference point at the current frame
    right / n / .   next frame
    left / p / ,    previous frame
    space           play / pause
    [ / ]           latency estimate: -5 ms / +5 ms (live re-tune)
    q / esc         quit

ASSUMPTIONS (read before trusting the marker)
  - R_BC below is the measured camera-to-body extrinsic rotation (roll 90,
    pitch 25 up, AND the axis reordering from image axes into the body's
    real FRD convention) -- taken from a companion rig's extrinsics.json,
    where cam_roll_deg was itself validated by correlating predicted vs.
    observed image motion over 524 frame pairs. Not re-derived or
    re-verified against this specific airframe.
  - Pinhole projection using the header's "focal" (px) with the principal
    point assumed to be the image centre -- no lens distortion, no calibrated
    principal point.
  - The los-*.csv is matched to each frame by SLERP between the two
    bracketing t_mono samples, not nearest-neighbour -- a stale nearest
    sample was observed elsewhere to go wrong on ~30% of frames.
  - A frame's t_mono marks when its bytes reached the host, not when the
    world was actually imaged. The latency estimate subtracts that before
    the attitude lookup; see INITIAL_LATENCY_S's comment for where the
    starting number comes from. It's a starting guess, not a fixed truth --
    tune it live with '[' / ']' while watching a static reference track: if
    the marker lags behind the real target's motion, latency is set too
    high (looking too far into the past); if it leads, too low.
  - The target is assumed genuinely static in the world. Any drift between
    the marker and the real target is either that assumption breaking down
    (the target moved) or one of the simplifications above.
"""
import argparse
import csv
import importlib.util
import pathlib
import re
import sys
import time

import cv2
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
import config


def _load_rawrec_viewer():
    """Reuse the .rawrec parsing/display helpers from tools/rawrec_viewer.py."""
    spec = importlib.util.spec_from_file_location(
        "rawrec_viewer", REPO_ROOT / "tools" / "rawrec_viewer.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rv = _load_rawrec_viewer()

WIN = "los static-point experiment"
REF_COLOR = (0, 255, 255)     # the clicked point, at its own frame
PRED_COLOR = (0, 0, 255)      # the predicted point, on every other frame

LATENCY_DOWN_KEYS = (ord('['),)
LATENCY_UP_KEYS = (ord(']'),)


def find_los_csv(rawrec_path, override=None):
    """Locate the los-*.csv matching a flight-*.rawrec.

    Same stamp, different prefix and no episode suffix: flight-<stamp>-NN.rawrec
    pairs with los-<stamp>.csv. Checked in order, cheapest and most specific
    first:

      1. same directory as the .rawrec (the old, flat layout).
      2. a sibling 'telemetry/' directory next to a 'rawrec/' one -- this
         repo's on-disk layout (see logs/, docs/REPOSITORY_GUIDE.md): captures
         live in logs/rawrec/, their matching CSVs in logs/telemetry/.
      3. a recursive search, first under the .rawrec's own parent and then
         under its parent's parent -- covers a dataset split into arbitrary
         subfolders (a no_drone/ or drone/ split, say) without hardcoding
         their names, and still finds (2) even if the two type-folders are
         nested one level deeper than expected.
    """
    if override:
        return pathlib.Path(override)
    rawrec_path = pathlib.Path(rawrec_path)
    stem = re.sub(r"-\d+$", "", rawrec_path.stem)
    los_name = stem.replace("flight-", "los-", 1) + ".csv"

    same_dir = rawrec_path.parent / los_name
    if same_dir.is_file():
        return same_dir

    sibling = rawrec_path.parent.parent / "telemetry" / los_name
    if sibling.is_file():
        return sibling

    for ancestor in (rawrec_path.parent, rawrec_path.parent.parent):
        for hit in sorted(ancestor.glob(f"**/{los_name}")):
            return hit
    return None


def load_quaternions(los_path):
    """Sorted (t_mono, qw, qx, qy, qz) for every row with a usable quaternion."""
    rows = []
    with open(los_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["t_mono"])
                q = tuple(float(row[k]) for k in ("qw", "qx", "qy", "qz"))
            except (KeyError, ValueError, TypeError):
                continue
            rows.append((t,) + q)
    rows.sort(key=lambda r: r[0])
    return rows


def slerp(q0, q1, t):
    """Spherical linear interpolation between two (w, x, y, z) quaternions."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        return out / np.linalg.norm(out)
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    sin_theta_0 = np.sin(theta_0)
    s0 = np.cos(theta) - dot * np.sin(theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return s0 * q0 + s1 * q1


def interp_quat(t_target, quat_rows, ts):
    """Attitude at t_target, SLERPed between the two bracketing samples.

    Nearest-neighbour matching was tried first and dropped: a companion rig's
    own notes report an index-based join going stale on ~30% of frames,
    producing a horizon that's "sometimes perfect and sometimes very
    misaligned". SLERPing by the exact time fraction avoids that.
    """
    j = int(np.searchsorted(ts, t_target))
    if j <= 0:
        return quat_rows[0][1:]
    if j >= len(quat_rows):
        return quat_rows[-1][1:]
    t0, t1 = ts[j - 1], ts[j]
    frac = 0.0 if t1 <= t0 else (t_target - t0) / (t1 - t0)
    return tuple(slerp(quat_rows[j - 1][1:], quat_rows[j][1:], frac))


# Tunables live in config.py -- see it for the reasoning (the phase-
# correlation measurements the latency starting point came from, the R_bc
# extrinsics measurement, etc).
INITIAL_LATENCY_S = config.LOS_LATENCY_S
LATENCY_STEP_S = config.LOS_LATENCY_STEP_S
MIN_LATENCY_S = config.LOS_MIN_LATENCY_S
MAX_LATENCY_S = config.LOS_MAX_LATENCY_S

R_BC = np.array(config.MOUNT_R_BC)

# Permutation from this file's image-ray axes (x=right, y=down, z=forward)
# into the sensor-frame axis order R_BC expects (X=forward, Y=right, Z=down).
_P_IMG_TO_SENSOR = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

MOUNT = R_BC @ _P_IMG_TO_SENSOR  # image-ray axes -> quaternion's body (FRD) axes


def quat_to_matrix(q):
    """Standard Hamilton (w, x, y, z) quaternion to rotation matrix."""
    qw, qx, qy, qz = q
    n = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    if n == 0:
        return np.eye(3)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def project(ref_uv, ref_q, cur_q, focal, cx, cy):
    """Where a world-static point at ref_uv (seen under ref_q) appears under cur_q.

    ref_uv is cast to an image-frame ray, rotated by MOUNT into the
    quaternion's body axes (undoing the camera's fixed mounting roll), then
    into the (arbitrary, but fixed) reference frame by ref_q, then back into
    the current body frame by cur_q's inverse, then back to image axes by
    MOUNT's inverse, and reprojected. Returns None if the point has rotated
    behind the camera.
    """
    u0, v0 = ref_uv
    r0_img = np.array([(u0 - cx) / focal, (v0 - cy) / focal, 1.0])
    r0_img /= np.linalg.norm(r0_img)
    r0_body = MOUNT @ r0_img
    world = quat_to_matrix(ref_q) @ r0_body
    r1_body = quat_to_matrix(cur_q).T @ world
    r1_img = MOUNT.T @ r1_body
    if r1_img[2] <= 1e-6:
        return None
    return cx + focal * r1_img[0] / r1_img[2], cy + focal * r1_img[1] / r1_img[2]


class SmoothTracker:
    """Fuses the LOS (attitude-only) prediction with detector output, blended
    smoothly frame to frame -- replaces a periodic hard re-anchor, which
    either trusts one old click forever or teleports the reference straight
    onto a raw detection the moment it fires.

    Every update(): PREDICT the reference forward by attitude alone
    (project()), then look for a detector candidate within a gate sized by
    how long it's been since the last good fix AND by how fast the camera is
    actually rotating (angular rate widens the gate and discounts detector
    confidence -- both LOS reprojection and the detector's own confidence are
    known to degrade at high angular rate: interpolation/timing error grows
    with rotation rate, and a fast-moving target smears across the detector's
    top-hat kernel). If a candidate is accepted, the reference is pulled
    toward it by a gain capped at ALPHA_MAX -- never fully replaced -- so a
    single frame can only ever partially correct the estimate. That cap is
    what makes a visible jump structurally impossible, not just unlikely.

    boxes passed to update() must be (x, y, w, h, score) -- score is
    detector confidence, higher meaning more trustworthy.
    """

    # All tunables live in config.py -- see it for why each is set the way
    # it is (CONF_REF in particular: it's what keeps a strong-but-farther
    # detection from automatically beating a weak-but-closer one, by
    # folding distance and confidence into [0, 1] before comparing them).
    BASE_GATE_PX = config.TRACKER_BASE_GATE_PX
    GATE_PX_PER_FRAME = config.TRACKER_GATE_PX_PER_FRAME
    GATE_PX_PER_DEG_S = config.TRACKER_GATE_PX_PER_DEG_S
    MAX_GATE_PX = config.TRACKER_MAX_GATE_PX
    OMEGA_REF_DEG_S = config.TRACKER_OMEGA_REF_DEG_S
    CONF_REF = config.TRACKER_CONF_REF
    ALPHA_MAX = config.TRACKER_ALPHA_MAX
    MISS_GAIN = config.TRACKER_MISS_GAIN

    def __init__(self, quats, frame_times, focal, cx, cy, project_fn):
        self.quats = quats
        self.frame_times = frame_times
        self.focal = focal
        self.cx = cx
        self.cy = cy
        self.project = project_fn
        self.ref = None  # {"frame_idx": int, "uv": (x, y)}
        self.misses = 0
        self.status = "none"  # "none" | "tracking" | "coasting"
        # Why this frame went the way it did. update() computes all of this to
        # make its decision and used to throw it away, which made a bad lock
        # impossible to diagnose after the fact: a flight log that says
        # "coasting" without saying whether anything was even in the gate, or
        # how good the thing it fused was, cannot tell you which of the two
        # failed. Read-only for callers; overwritten every update().
        self.last = {"score": None, "dist_px": None, "gate_px": None,
                     "omega_deg_s": None, "alpha": None, "n_in_gate": 0}
        self.drops = 0            # how many locks have been given up
        self.last_release = None  # why the most recent one was

    def set_click(self, frame_idx, uv):
        self.ref = {"frame_idx": frame_idx, "uv": uv}
        self.misses = 0
        self.status = "tracking"
        self.drops = getattr(self, "drops", 0)

    def release(self, reason="released"):
        """Give up the current reference, so the next frame can acquire afresh.

        WITHOUT THIS THE TRACKER IS A ONE-SHOT DEVICE. `ref` was only ever
        assigned, never cleared, so whatever it latched onto first it held for
        the life of the process -- coasting on attitude alone forever if the
        lock broke. `valid` correctly drops to 0 while coasting, so guidance
        times out and holds, which is safe but terminal: the seeker was done for
        the rest of the flight even if the target came back into plain view.

        Releasing is deliberately NOT automatic on a long coast. Coasting is the
        designed behaviour through an occlusion, and a timeout would be a second
        threshold to guess at. The caller decides, on evidence -- see
        tools/flight_pipeline.py, which releases when the radar cue and the
        tracked LOS disagree persistently, i.e. when something other than the
        tracker says the tracker is wrong.
        """
        self.ref = None
        self.misses = 0
        self.status = "none"
        self.drops = getattr(self, "drops", 0) + 1
        self.last_release = reason
        self.last = {"score": None, "dist_px": None, "gate_px": None,
                     "omega_deg_s": None, "alpha": None, "n_in_gate": 0}

    def _omega_deg_s(self, i0, i1):
        if self.quats is None or i0 == i1:
            return 0.0
        dot = abs(sum(a * b for a, b in zip(self.quats[i0], self.quats[i1])))
        dot = min(1.0, max(-1.0, dot))
        theta_deg = np.degrees(2 * np.arccos(dot))
        dt = abs(self.frame_times[i1] - self.frame_times[i0])
        return theta_deg / dt if dt > 0 else 0.0

    def point(self, frame_idx):
        """Current predicted image position for frame_idx, or None."""
        if self.ref is None or self.quats is None:
            return None
        if frame_idx == self.ref["frame_idx"]:
            return self.ref["uv"]
        return self.project(self.ref["uv"], self.quats[self.ref["frame_idx"]],
                             self.quats[frame_idx], self.focal, self.cx, self.cy)

    def gate_px(self, frame_idx):
        """Current acceptance-gate radius (px) for frame_idx, or None with no
        reference yet. Public so a caller can size a search crop around
        point(frame_idx) before running a detector, rather than searching
        the whole frame once the tracker actually knows roughly where to
        look."""
        if self.ref is None or self.quats is None:
            return None
        omega = self._omega_deg_s(self.ref["frame_idx"], frame_idx)
        elapsed = abs(frame_idx - self.ref["frame_idx"])
        return min(self.MAX_GATE_PX, self.BASE_GATE_PX
                   + self.GATE_PX_PER_FRAME * elapsed + self.GATE_PX_PER_DEG_S * omega)

    def update(self, frame_idx, boxes):
        """Advance the tracker to frame_idx: predict, gate against boxes, blend."""
        if self.ref is None or self.quats is None or frame_idx == self.ref["frame_idx"]:
            return

        predicted = self.point(frame_idx)
        if predicted is None:
            self.status = "coasting"
            self.last = {"score": None, "dist_px": None, "gate_px": None,
                         "omega_deg_s": None, "alpha": None, "n_in_gate": 0}
            return  # can't propagate (behind camera / off-frame) -- try again next frame
        px, py = predicted

        omega = self._omega_deg_s(self.ref["frame_idx"], frame_idx)
        gate = self.gate_px(frame_idx)
        conf_scale = 1.0 / (1.0 + omega / self.OMEGA_REF_DEG_S)

        best, best_cost, best_score, best_d, n_in_gate = None, None, None, None, 0
        for box in boxes:
            x, y, w, h, score = box
            bx, by = x + w / 2.0, y + h / 2.0
            d = ((bx - px) ** 2 + (by - py) ** 2) ** 0.5
            if d > gate:
                continue
            n_in_gate += 1
            eff_conf = max(score * conf_scale, 0.0)
            dist_term = d / gate                                   # in [0, 1]
            conf_term = 1.0 - min(eff_conf / self.CONF_REF, 1.0)   # in [0, 1]
            cost = dist_term + conf_term                            # in [0, 2]
            if best is None or cost < best_cost:
                best, best_cost, best_score, best_d = (bx, by), cost, score, d

        self.last = {"score": best_score, "dist_px": best_d, "gate_px": gate,
                     "omega_deg_s": omega, "alpha": None, "n_in_gate": n_in_gate}

        if best is None:
            self.ref = {"frame_idx": frame_idx, "uv": (px, py)}
            self.misses += 1
            self.status = "coasting"
            return

        match_quality = 1.0 - best_cost / 2.0  # cost in [0, 2] -> quality in [0, 1]
        alpha = min(self.ALPHA_MAX, max(0.0, match_quality) * (1.0 + self.MISS_GAIN * self.misses))
        self.last["alpha"] = alpha
        blended = (px + alpha * (best[0] - px), py + alpha * (best[1] - py))
        self.ref = {"frame_idx": frame_idx, "uv": blended}
        self.misses = 0
        self.status = "tracking"


# Tunable lives in config.py -- see it for why it's set the way it is.
CROP_MARGIN_PX = config.TRACKER_CROP_MARGIN_PX


def crop_bounds(cx, cy, radius, w, h):
    """Clamped (x0, y0, x1, y1) crop window of the given radius around (cx, cy),
    or None if the point has drifted entirely outside the frame -- clamping
    x0 and x1 independently doesn't stop x0 from landing past x1 (or beyond w
    entirely) when cx is more than radius past the right/bottom edge, which
    would otherwise hand a detector an empty array and crash it."""
    x0 = max(0, int(cx - radius))
    y0 = max(0, int(cy - radius))
    x1 = min(w, int(cx + radius))
    y1 = min(h, int(cy + radius))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the .rawrec capture to open")
    ap.add_argument("--los", help="override: path to the matching los-*.csv")
    args = ap.parse_args()

    meta = rv.read_header(args.file)
    w, h, bpp = meta["width"], meta["height"], meta["bpp"]
    dtype = np.uint8 if bpp == 1 else np.dtype("<u2")
    focal = meta.get("focal")
    if not focal:
        sys.exit(f"{args.file}: header has no 'focal', can't reproject")
    cx, cy = w / 2.0, h / 2.0

    los_path = find_los_csv(args.file, args.los)
    if los_path is None:
        sys.exit(f"{args.file}: no matching los-*.csv found (pass --los to point at one)")
    quat_rows = load_quaternions(los_path)
    if not quat_rows:
        sys.exit(f"{los_path}: no rows with a usable quaternion")
    ts = np.array([r[0] for r in quat_rows])

    frames = rv.build_index(args.file, meta)
    offsets = [off for off, _ in frames]
    total = len(offsets)
    frame_times = [t for _, t in frames]
    base_interval = 1.0 / rv.measured_fps(frames)

    latency = INITIAL_LATENCY_S
    quats = [interp_quat(t - latency, quat_rows, ts) for t in frame_times]

    def rebuild_quats():
        nonlocal quats
        quats = [interp_quat(t - latency, quat_rows, ts) for t in frame_times]

    f = open(args.file, "rb")

    def read_frame(idx):
        f.seek(offsets[idx])
        buf = f.read(meta["frame_bytes"])
        return np.frombuffer(buf, dtype=dtype).reshape(h, w)

    frame_idx = 0
    playing = False
    last_tick = time.monotonic()
    raw = read_frame(frame_idx)
    disp = rv.to_display(raw, bpp)

    ref = None  # {"frame_idx": int, "uv": (x, y)} -- attitude looked up live,
                # from the current `quats`, so a latency retune after the
                # click still applies to the reference frame too.
    mouse = {"x": None, "y": None}

    def on_mouse(event, x, y, flags, param):
        nonlocal ref
        mouse["x"], mouse["y"] = x, y - rv.HUD_H
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= mouse["x"] < w and 0 <= mouse["y"] < h:
            ref = {"frame_idx": frame_idx, "uv": (mouse["x"], mouse["y"])}

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, on_mouse)

    def goto_frame(idx):
        nonlocal frame_idx, raw, disp
        idx = max(0, min(total - 1, idx))
        if idx == frame_idx:
            return
        frame_idx = idx
        raw = read_frame(frame_idx)
        disp = rv.to_display(raw, bpp)

    while True:
        now = time.monotonic()
        if playing and now - last_tick >= base_interval:
            last_tick = now
            if frame_idx < total - 1:
                goto_frame(frame_idx + 1)
            else:
                playing = False

        header = np.zeros((rv.HUD_H, w, 3), dtype=np.uint8)
        text = f"frame {frame_idx + 1}/{total}  {'PLAY' if playing else 'PAUSE'}"
        mx, my = mouse["x"], mouse["y"]
        if mx is not None and 0 <= mx < w and 0 <= my < h:
            text += f"    x={mx} y={my}  val={int(raw[my, mx])}"
        text += "    ref=SET" if ref else "    ref=none (click the target)"
        text += f"    latency={latency * 1000:.0f}ms"
        cv2.putText(header, text, (10, rv.HUD_H - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

        view = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        if ref is not None:
            if frame_idx == ref["frame_idx"]:
                cv2.drawMarker(view, tuple(int(v) for v in ref["uv"]), REF_COLOR,
                                cv2.MARKER_CROSS, 14, 2)
            else:
                pred = project(ref["uv"], quats[ref["frame_idx"]], quats[frame_idx], focal, cx, cy)
                if pred is not None:
                    u1, v1 = pred
                    if 0 <= u1 < w and 0 <= v1 < h:
                        cv2.drawMarker(view, (int(u1), int(v1)), PRED_COLOR,
                                        cv2.MARKER_CROSS, 14, 2)
        canvas = np.vstack([header, view])
        cv2.imshow(WIN, canvas)

        k = cv2.waitKeyEx(30)
        if k == -1:
            continue
        c = k & 0xFF

        if k in rv.QUIT_KEYS or c in rv.QUIT_KEYS:
            break
        elif k in rv.PLAY_KEYS or c in rv.PLAY_KEYS:
            playing = not playing
            last_tick = time.monotonic()
        elif k in rv.NEXT_KEYS or c in rv.NEXT_KEYS:
            goto_frame(frame_idx + 1)
        elif k in rv.PREV_KEYS or c in rv.PREV_KEYS:
            goto_frame(frame_idx - 1)
        elif k in LATENCY_DOWN_KEYS or c in LATENCY_DOWN_KEYS:
            latency = max(MIN_LATENCY_S, latency - LATENCY_STEP_S)
            rebuild_quats()
        elif k in LATENCY_UP_KEYS or c in LATENCY_UP_KEYS:
            latency = min(MAX_LATENCY_S, latency + LATENCY_STEP_S)
            rebuild_quats()

    f.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
