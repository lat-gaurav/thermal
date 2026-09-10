#!/usr/bin/env python3
"""Headless web viewer for .rawrec captures -- same detector/filter/LOS
pipeline as rawrec_viewer.py, served over HTTP for a machine with no display
(the rpi5 flight recorder runs headless, so cv2.imshow's GUI window there is
a no-op at best and a crash at worst).

    web_viewer.py FILE.rawrec [--host 0.0.0.0] [--port 8000]
    web_viewer.py --live /dev/thermal0 [--width 1280] [--height 1024]

Open http://<host>:<port>/ in a browser on any machine on the same network.
Same controls as rawrec_viewer.py: drag the frame slider, click the image to
set a LOS reference, cycle detectors, toggle annotation, play/pause with
adjustable speed, hover for the pixel readout.

--live reads straight from a V4L2 camera instead of a recorded file, for
watching the detector run against the real feed. No slider/play controls in
that mode (there is no frame count, just "now") -- the page polls for the
latest processed frame instead. There is no matching los-*.csv for a live
feed either, so LOS/filtering is unavailable there; everything else (detector
selection, annotation) works the same. The device only allows one reader, so
this can't run alongside anything else already holding it (e.g. `sudo
systemctl stop ir-tracker` first if that's what's running).

Reuses rawrec_viewer.py's parsing/detector/filter/LOS functions directly
(same folder, plain import) rather than re-deriving any of that here -- this
file is only the HTTP/rendering layer rawrec_viewer.py's cv2 window doesn't
have a headless equivalent for.
"""
import argparse
import base64
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

import rawrec_viewer as rv  # also bootstraps repo root onto sys.path
import config

INDEX_HTML = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>rawrec web viewer</title>
<style>
  body { background:#111; color:#0f0; font-family:monospace; margin:0; padding:10px; }
  #hud { margin-bottom:6px; white-space:pre; }
  #img { max-width:100%; image-rendering:pixelated; cursor:crosshair; display:block; }
  input[type=range] { width:100%; }
  button { margin:6px 6px 0 0; }
</style></head>
<body>
  <div id="hud">loading...</div>
  <img id="img">
  <div id="fileControls">
    <input type="range" id="slider" min="0" max="0" value="0">
    <div>
      <button id="prev">prev</button>
      <button id="play">play</button>
      <button id="next">next</button>
      <button id="slower">slower</button>
      <button id="faster">faster</button>
    </div>
  </div>
  <div>
    <button id="det">cycle detector</button>
    <button id="ann">toggle annotate</button>
    <button id="autoinit">auto-init LOS here</button>
  </div>
<script>
let total = 1, idx = 0, speed = 1.0, playing = false, timer = null, fps = 25, isLive = false;
let hoverTxt = "";
const img = document.getElementById('img');
const hud = document.getElementById('hud');
const slider = document.getElementById('slider');
const playBtn = document.getElementById('play');

async function loadMeta() {
  const m = await (await fetch('/api/meta')).json();
  total = m.total; fps = m.fps; isLive = m.is_live;
  if (isLive) {
    document.getElementById('fileControls').style.display = 'none';
    setInterval(() => goto(0), 150);  // continuous poll -- there is no "frame count" to step through
  } else {
    slider.max = total - 1;
  }
  goto(0);
}

async function goto(i) {
  idx = isLive ? 0 : Math.max(0, Math.min(total - 1, i));
  if (!isLive) slider.value = idx;
  const d = await (await fetch('/api/frame/' + idx)).json();
  img.src = d.image;
  const status = isLive ? 'LIVE' : (playing ? 'PLAY' : 'PAUSE');
  const frameTxt = isLive ? '' : `frame ${d.frame}/${d.total}  `;
  hud.textContent = `${frameTxt}${status} ` +
    `${isLive ? '' : '(' + speed.toFixed(2) + 'x)  '}detector=${d.detector} ` +
    `[${d.annotate ? 'ON' : 'OFF'}]  roi=${d.roi}  los=${d.los_status}${hoverTxt}`;
}

function restartTimer() {
  if (isLive) return;
  if (timer) clearInterval(timer);
  if (playing) {
    timer = setInterval(() => {
      if (idx >= total - 1) {
        playing = false; playBtn.textContent = 'play'; clearInterval(timer); return;
      }
      goto(idx + 1);
    }, 1000 / (fps * speed));
  }
}

slider.addEventListener('input', e => goto(parseInt(e.target.value)));
document.getElementById('prev').onclick = () => goto(idx - 1);
document.getElementById('next').onclick = () => goto(idx + 1);
document.getElementById('slower').onclick = () => { speed = Math.max(1/16, speed/1.5); restartTimer(); };
document.getElementById('faster').onclick = () => { speed = Math.min(16, speed*1.5); restartTimer(); };
playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? 'pause' : 'play'; restartTimer(); };
document.getElementById('det').onclick = async () => { await fetch('/api/detector/cycle', {method:'POST'}); goto(idx); };
document.getElementById('ann').onclick = async () => { await fetch('/api/annotate/toggle', {method:'POST'}); goto(idx); };
document.getElementById('autoinit').onclick = async () => {
  const r = await fetch('/api/autoinit', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({frame: idx})});
  const d = await r.json();
  if (!d.ok) { hoverTxt = `    auto-init REJECTED: ${d.reason}`; }
  goto(idx);
};

function imgCoords(e) {
  const rect = img.getBoundingClientRect();
  const sx = img.naturalWidth / rect.width, sy = img.naturalHeight / rect.height;
  return [Math.round((e.clientX - rect.left) * sx), Math.round((e.clientY - rect.top) * sy)];
}

img.addEventListener('click', async e => {
  const [x, y] = imgCoords(e);
  const r = await fetch('/api/click', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({frame: idx, x, y})});
  const d = await r.json();
  if (!d.ok) {
    hoverTxt = `    click at (${x},${y}) REJECTED: ${d.reason}`;
  }
  goto(idx);
});

let hoverPending = false;
img.addEventListener('mousemove', e => {
  if (hoverPending) return;
  hoverPending = true;
  setTimeout(() => { hoverPending = false; }, 80);
  const [x, y] = imgCoords(e);
  fetch(`/api/pixel/${idx}?x=${x}&y=${y}`).then(r => r.json()).then(d => {
    hoverTxt = d.value === null ? "" : `    x=${x} y=${y}  val=${d.value}`;
  });
});

loadMeta();
</script>
</body></html>
"""


class LiveSource:
    """Continuously grabs frames from a V4L2 camera in a background thread.

    Decoupled from processing speed on purpose: the detector (~0.2s/frame for
    tophat_scr) is far slower than the camera's true throughput (~25fps this
    bus can sustain), so serving "whatever's latest" rather than queueing
    every captured frame is the only way to stay live instead of falling
    further and further behind.
    """

    def __init__(self, device, width, height):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            sys.exit(f"{device}: could not open (in use by another process? "
                      f"e.g. 'sudo systemctl stop ir-tracker' first if that's running)")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # keep raw single-channel GREY

        self.lock = threading.Lock()
        self.latest = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        for _ in range(100):  # wait up to ~2s for the first real frame
            if self.latest is not None:
                break
            time.sleep(0.02)
        else:
            sys.exit(f"{device}: opened but no frame arrived -- check --width/--height "
                      f"match what the device actually delivers")

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.latest = frame
            else:
                time.sleep(0.01)

    def get(self):
        with self.lock:
            return self.latest


class Session:
    """All server-side mutable state for one open frame source, one process."""

    def __init__(self, path=None, live_device=None, live_width=None, live_height=None):
        self.live = None
        if path is not None:
            self.meta = rv.read_header(path)
            self.w, self.h, self.bpp = self.meta["width"], self.meta["height"], self.meta["bpp"]
            self.dtype = np.uint8 if self.bpp == 1 else np.dtype("<u2")
            self.frames = rv.build_index(path, self.meta)
            self.offsets = [off for off, _ in self.frames]
            self.total = len(self.offsets)
            self.fps = rv.measured_fps(self.frames)
            self._fh = open(path, "rb")
        else:
            live_width = config.CAMERA_WIDTH if live_width is None else live_width
            live_height = config.CAMERA_HEIGHT if live_height is None else live_height
            self.live = LiveSource(live_device, live_width, live_height)
            self.w, self.h, self.bpp = live_width, live_height, 1
            self.dtype = np.uint8
            self.total = None  # no fixed length -- this is a live stream
            self.fps = None

        self.lock = threading.Lock()

        self.detectors = rv.load_detectors()
        self.detectors.append(("none", lambda raw: []))
        self.det_idx = 0
        self.annotate = True
        self.filters = rv.load_filters()
        self.initialisers = rv.load_initialisers()

        # LOS needs a matching los-*.csv, which only exists for a recorded file.
        self.los_track = rv._load_los_track()
        self.focal = self.meta.get("focal") if path is not None else None
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        self.quats = None
        if path is not None:
            los_path = self.los_track.find_los_csv(path)
            quat_rows = self.los_track.load_quaternions(los_path) if los_path else []
            ts = np.array([r[0] for r in quat_rows]) if quat_rows else None
            self.quats = ([self.los_track.interp_quat(t - config.LOS_LATENCY_S, quat_rows, ts)
                           for _, t in self.frames] if (self.focal and quat_rows) else None)
        self.tracker = (self.los_track.SmoothTracker(
            self.quats, [t for _, t in self.frames], self.focal, self.cx, self.cy,
            self.los_track.project) if self.quats is not None else None)
        self.roi_desc = "FULL"

        self._cache_idx = None
        self._cache_raw = None

    def _clamp(self, idx):
        return 0 if self.total is None else max(0, min(self.total - 1, idx))

    def read_frame(self, idx):
        if self.live is not None:
            return self.live.get()
        if idx == self._cache_idx:
            return self._cache_raw
        self._fh.seek(self.offsets[idx])
        buf = self._fh.read(self.meta["frame_bytes"])
        raw = np.frombuffer(buf, dtype=self.dtype).reshape(self.h, self.w)
        self._cache_idx, self._cache_raw = idx, raw
        return raw

    def run_detector(self, raw, idx):
        """Runs the detector full-frame, unless the tracker has a reference
        (tracking OR coasting) -- then it's cheaper to search only a window
        around the LOS prediction, sized to safely contain the tracker's own
        gate. Cropping stays on through a coast: the gate itself grows the
        longer a fix has been missed (see gate_px), so it's still a search
        window, not a frozen one, and dropping straight back to a full-frame
        search on the first missed detection would waste exactly the
        speedup we want most while something is briefly out of view."""
        if not self.annotate:
            self.roi_desc = "FULL"
            return []
        if self.tracker is not None and self.tracker.ref is not None:
            predicted = self.tracker.point(idx)
            gate = self.tracker.gate_px(idx)
            bounds = (self.los_track.crop_bounds(predicted[0], predicted[1],
                                                  gate + self.los_track.CROP_MARGIN_PX,
                                                  self.w, self.h)
                      if predicted is not None and gate is not None else None)
            if bounds is not None:
                x0, y0, x1, y1 = bounds
                self.roi_desc = f"{x1 - x0}x{y1 - y0}"
                crop = raw[y0:y1, x0:x1]
                boxes = self.detectors[self.det_idx][1](crop)
                return [(x + x0, y + y0, bw, bh, score) for (x, y, bw, bh, score) in boxes]
        self.roi_desc = "FULL"
        return self.detectors[self.det_idx][1](raw)

    def render(self, idx):
        """JPEG bytes + metadata dict for one frame, fully rendered (boxes, LOS marker)."""
        idx = self._clamp(idx)
        with self.lock:
            raw = self.read_frame(idx)
            boxes = self.run_detector(raw, idx)
            if self.tracker is not None:
                self.tracker.update(idx, boxes)
            los_point = self.tracker.point(idx) if self.tracker is not None else None

            draw_boxes = boxes
            for _, fn in self.filters:
                draw_boxes = fn(draw_boxes, {"los_point": los_point,
                                              "frame_w": self.w, "frame_h": self.h})

            disp = rv.to_display(raw, self.bpp)
            view = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
            if self.annotate:
                for box in draw_boxes:
                    x, y, bw, bh = box[:4]
                    cv2.rectangle(view, (x, y), (x + bw, y + bh), rv.BOX_COLOR, 1)
            if los_point is not None:
                color = (rv.LOS_TRACK_COLOR if self.tracker.status == "tracking"
                         else rv.LOS_COAST_COLOR)
                pt = (int(los_point[0]), int(los_point[1]))
                cv2.drawMarker(view, pt, color, cv2.MARKER_CROSS, 14, 2)

            if self.quats is None:
                los_status = "n/a"
            elif self.tracker.ref is None:
                los_status = "none"
            else:
                los_status = self.tracker.status
            meta = {
                "frame": idx + 1, "total": self.total,
                "detector": self.detectors[self.det_idx][0], "annotate": self.annotate,
                "los_status": los_status, "roi": self.roi_desc,
            }

        ok, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, config.WEB_JPEG_QUALITY])
        return jpg.tobytes(), meta

    def pixel_value(self, idx, x, y):
        with self.lock:
            raw = self.read_frame(self._clamp(idx))
        if 0 <= x < self.w and 0 <= y < self.h:
            return int(raw[y, x])
        return None

    def set_click(self, idx, x, y):
        """Returns (ok, reason) -- reason explains a rejection either way."""
        if self.quats is None:
            reason = ("no attitude data for a live feed" if self.live is not None
                      else "no matching los-*.csv found for this file")
            return False, reason
        if not (0 <= x < self.w and 0 <= y < self.h):
            return False, f"({x},{y}) is outside the {self.w}x{self.h} frame"
        with self.lock:
            self.tracker.set_click(self._clamp(idx), (float(x), float(y)))
        return True, None

    def auto_init(self, idx):
        """No-click LOS init: runs whichever strategy is loaded from
        initialisation/ (only the first one found is used for now).
        Returns (ok, reason)."""
        if self.quats is None:
            reason = ("no attitude data for a live feed" if self.live is not None
                      else "no matching los-*.csv found for this file")
            return False, reason
        if not self.initialisers:
            return False, "no initialisation strategy loaded from initialisation/"
        idx = self._clamp(idx)
        with self.lock:
            raw = self.read_frame(idx)
            boxes = self.run_detector(raw, idx)
            uv = rv.pick_initialiser(config.VIEWER_INITIALISER, self.initialisers)[1](
                boxes, {"frame_w": self.w, "frame_h": self.h})
            if uv is None:
                return False, "no isolated detection on this frame"
            self.tracker.set_click(idx, uv)
        return True, None

    def cycle_detector(self):
        with self.lock:
            self.det_idx = (self.det_idx + 1) % len(self.detectors)
            return self.detectors[self.det_idx][0]

    def toggle_annotate(self):
        with self.lock:
            self.annotate = not self.annotate
            return self.annotate


def make_handler(session):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # keep stdout quiet; this runs headless on the rpi

        def _json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            parts = [p for p in parsed.path.split("/") if p]

            if parsed.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(INDEX_HTML)))
                self.end_headers()
                self.wfile.write(INDEX_HTML)

            elif parsed.path == "/api/meta":
                self._json({
                    "total": session.total, "width": session.w, "height": session.h,
                    "fps": session.fps, "is_live": session.live is not None,
                    "detectors": [n for n, _ in session.detectors],
                    "filters": [n for n, _ in session.filters],
                    "initialisers": [n for n, _ in session.initialisers],
                    "has_los": session.quats is not None,
                })

            elif len(parts) == 3 and parts[0] == "api" and parts[1] == "frame":
                idx = int(parts[2])
                jpg, meta = session.render(idx)
                meta["image"] = "data:image/jpeg;base64," + base64.b64encode(jpg).decode("ascii")
                self._json(meta)

            elif len(parts) == 3 and parts[0] == "api" and parts[1] == "pixel":
                idx = int(parts[2])
                q = parse_qs(parsed.query)
                x, y = int(q["x"][0]), int(q["y"][0])
                self._json({"value": session.pixel_value(idx, x, y)})

            else:
                self._json({"error": "not found"}, status=404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body) if body else {}
            except json.JSONDecodeError:
                payload = {}

            if self.path == "/api/click":
                ok, reason = session.set_click(int(payload.get("frame", 0)),
                                                int(payload.get("x", -1)), int(payload.get("y", -1)))
                self._json({"ok": ok, "reason": reason})
            elif self.path == "/api/autoinit":
                ok, reason = session.auto_init(int(payload.get("frame", 0)))
                self._json({"ok": ok, "reason": reason})
            elif self.path == "/api/detector/cycle":
                self._json({"detector": session.cycle_detector()})
            elif self.path == "/api/annotate/toggle":
                self._json({"annotate": session.toggle_annotate()})
            else:
                self._json({"error": "not found"}, status=404)

    return Handler


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="the .rawrec capture to open")
    ap.add_argument("--live", metavar="DEVICE",
                     help="read live frames from a V4L2 camera instead, e.g. /dev/thermal0 "
                          "(mutually exclusive with FILE; the device only allows one reader, "
                          "so stop any service that already holds it first)")
    ap.add_argument("--width", type=int, default=config.CAMERA_WIDTH,
                     help="live camera frame width")
    ap.add_argument("--height", type=int, default=config.CAMERA_HEIGHT,
                     help="live camera frame height")
    ap.add_argument("--host", default=config.WEB_HOST,
                     help="bind address (default: all interfaces)")
    ap.add_argument("--port", type=int, default=config.WEB_PORT)
    args = ap.parse_args()

    if bool(args.file) == bool(args.live):
        sys.exit("pass exactly one of FILE or --live")

    if args.live:
        session = Session(live_device=args.live, live_width=args.width, live_height=args.height)
        label = f"live {args.live} ({args.width}x{args.height})"
    else:
        session = Session(path=args.file)
        label = f"{args.file} ({session.total} frames)"

    server = ThreadingHTTPServer((args.host, args.port), make_handler(session))
    print(f"serving {label} on http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
