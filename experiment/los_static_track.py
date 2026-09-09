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
    pairs with los-<stamp>.csv. It isn't always in the same directory (some
    live under a no_drone/ or drone/ subfolder), so fall back to a recursive
    search under the .rawrec's parent.
    """
    if override:
        return pathlib.Path(override)
    rawrec_path = pathlib.Path(rawrec_path)
    stem = re.sub(r"-\d+$", "", rawrec_path.stem)
    los_name = stem.replace("flight-", "los-", 1) + ".csv"
    same_dir = rawrec_path.parent / los_name
    if same_dir.is_file():
        return same_dir
    for hit in sorted(rawrec_path.parent.glob(f"**/{los_name}")):
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


INITIAL_LATENCY_S = -0.070  # world moves -> attitude lookup point. A companion
                           # rig's own two credible phase-correlation runs gave
                           # +150 ms (corr 0.545, scale 0.857) and +200 ms
                           # (corr 0.620, scale 0.826); this is their midpoint.
                           # A third run on that rig gave -78.5 ms but with
                           # corr -0.048 and scale -0.064 -- by its own
                           # validity check (scale near +/-1.0) that run was
                           # noise, not a measurement, and is disregarded.
                           # Not measured on this airframe -- just a starting
                           # point for the live '[' / ']' tuning below.
LATENCY_STEP_S = 0.010     # 5 ms per key press
MIN_LATENCY_S = -0.5
MAX_LATENCY_S = 0.5

# Measured camera-to-body extrinsic rotation, from a companion rig's
# extrinsics.json (R_bc): rotates a vector in the SENSOR frame (X=boresight,
# Y=image-right, Z=image-down) into BODY FRD (X=forward, Y=right, Z=down) --
# the quaternion's own convention. This is roll (90, validated by correlating
# predicted vs. observed image motion over 524 frame pairs: +90 gave corr
# +0.735, -90 gave -0.735) and pitch (25 up) TOGETHER with the axis reorder
# from (right, down, forward) to (forward, right, down) -- not a pure roll
# composed with a pure pitch in image-axis order, which is what an earlier
# version of this file used and which a direct numeric check against R_bc
# below showed was wrong by up to 2.0 in matrix entries, not just a sign.
R_BC = np.array([
    [0.906307787037, -0.422618261741, 0.0],
    [0.0, 0.0, 1.0],
    [-0.422618261741, -0.906307787037, 0.0],
])

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
