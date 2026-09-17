"""The annotated frame, served over HTTP, without costing the tracking loop.

    pv = Preview().start()
    pv.offer(frame, overlay)      # from the main loop: cheap, never blocks
    ...
    open http://<pi>:8002/

WHY A SEPARATE THREAD IS NOT OPTIONAL HERE. A full-frame search costs 32.6 ms of
a 40 ms budget, leaving 7.4 ms. Annotating and JPEG-encoding a 1280x1024 frame
does not fit in 7.4 ms, so doing it inline would push the pipeline over budget
and the camera would start outrunning it -- trading the thing that matters for a
picture of it. offer() therefore stores a reference and returns; this thread
annotates and encodes at its own pace and silently drops whatever it could not
keep up with.

MJPEG, not a page that polls. One `multipart/x-mixed-replace` response streams
frames as they are produced, so a browser shows a moving image with no
JavaScript and no reload, and a viewer that disconnects costs nothing.

WHAT IS DRAWN, and why each thing is there:
    detections      every box the detector returned, so you can see whether the
                    scene is quiet or a clutter field
    LOS marker      green while fusing detections, orange while coasting on
                    attitude alone -- the distinction that says whether the
                    tracker still has evidence or is guessing
    cue marker      where the radar says the target is, with the acquisition
                    radius around it, so a lock can be judged by eye against
                    the thing that authorises it
    HUD             the numbers you would otherwise have to read out of the
                    journal: rate, status, cue residual, drops, arm state
"""
import json
import pathlib
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

_INDEX = b"""<!doctype html><html><head><meta charset="utf-8">
<title>thermal flight</title>
<style>
 body{background:#0b0b0b;color:#cfc;font-family:ui-monospace,monospace;margin:0;padding:10px}
 #wrap{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-start}
 img{max-width:100%;image-rendering:pixelated;display:block;border:1px solid #333}
 #panel{min-width:340px;font-size:13px;line-height:1.5}
 h2{font-size:12px;color:#6f6;margin:10px 0 3px;text-transform:uppercase;letter-spacing:.08em;
    border-bottom:1px solid #2a2a2a;padding-bottom:2px}
 table{border-collapse:collapse;width:100%}
 td{padding:1px 6px 1px 0;vertical-align:top}
 td.k{color:#7a8;width:47%}
 td.v{color:#dfd;font-weight:600}
 .ok{color:#5f5}.warn{color:#fc4}.bad{color:#f55}.off{color:#777}
</style></head><body>
<div id="wrap">
  <img id="img" src="/stream.mjpg">
  <div id="panel">connecting...</div>
</div>
<script>
const G = [
 ["pipeline",   ["frame","fps","det_ms","loop_ms","roi","skipped","grabbed","uptime_s"]],
 ["tracker",    ["status","los_x","los_y","gate_px","n_in_gate","det_score","match_dist_px",
                 "alpha","omega_deg_s","misses","drops","last_release"]],
 ["detector",   ["detector","n_boxes","n_kept","initialiser","acq_via","acq_cueless_n"]],
 ["bearing sent",["az_deg","el_deg","det_valid","valid_hold","seq_sent","body_az_deg","body_el_deg",
                 "los_n","los_e","los_d"]],
 ["radar cue",  ["cue_valid","cue_az_deg","cue_el_deg","cue_range_m","cue_age_ms",
                 "cue_u","cue_v","cue_resid_deg","cue_bad_frames","cue_seen"]],
 ["cube link",  ["sysid","cube_armed","att_hz","att_age_ms","hb_age_ms","lat_cam_pitch_deg",
                 "uplink","n_sent"]],
 ["rc switches",["algo_armed","arm_reason","rc_arm_us","rec_armed","rec_reason","rc_rec_us"]],
 ["recording",  ["rec_on","episode","rec_frames","raw_dropped","rec_path","rec_stop_reason"]],
 ["system",     ["cpu_temp_c","throttled","git_sha","dirty"]],
 ["settings",   ["lookback_s","focal_px","cue_acquire_px","cue_drop_deg","cue_drop_frames",
                 "min_scr","preview_fps"]],
];
function cls(k,v){
  if(v===null||v===undefined||v==="") return "off";
  if(k==="status") return v==="tracking"?"ok":(v==="dropped"?"bad":"warn");
  if(k==="throttled") return v==="0x0"?"ok":"bad";
  if(k==="dirty") return v?"warn":"ok";
  if(k==="valid_hold") return v?"warn":"off";   // valid, but dead-reckoned
  if(k==="cue_valid"||k==="det_valid"||k==="algo_armed"||k==="rec_on") return v?"ok":"off";
  if(k==="skipped"||k==="raw_dropped"||k==="drops") return Number(v)>0?"warn":"ok";
  if(k==="cpu_temp_c") return Number(v)>75?"bad":(Number(v)>60?"warn":"ok");
  return "";
}
function fmt(v){
  if(v===null||v===undefined) return "-";
  if(typeof v==="boolean") return v?"yes":"no";
  if(typeof v==="number") return Number.isInteger(v)?v:v.toFixed(3);
  return String(v);
}
async function tick(){
  try{
    const s = await (await fetch("/api/state")).json();
    let h="";
    for(const [title,keys] of G){
      h+="<h2>"+title+"</h2><table>";
      for(const k of keys){
        if(!(k in s)) continue;
        h+="<tr><td class=k>"+k+"</td><td class='v "+cls(k,s[k])+"'>"+fmt(s[k])+"</td></tr>";
      }
      h+="</table>";
    }
    document.getElementById("panel").innerHTML=h;
  }catch(e){ document.getElementById("panel").textContent="no data: "+e; }
}
tick(); setInterval(tick, 400);
</script></body></html>"""


class Preview:
    """Latest-frame HTTP preview. One producer (the main loop), N viewers."""

    def __init__(self, port=None, max_fps=None, scale=None, quality=None):
        self.port = int(config.PREVIEW_PORT if port is None else port)
        self.max_fps = float(config.PREVIEW_MAX_FPS if max_fps is None else max_fps)
        self.scale = float(config.PREVIEW_SCALE if scale is None else scale)
        self.quality = int(config.PREVIEW_JPEG_QUALITY if quality is None else quality)
        self._pending = None          # (frame, overlay) handed over by the loop
        self._state = {}              # everything the page shows, latest wins
        self._jpeg = None             # the most recently encoded frame
        self._seq = 0
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._run = False
        self._encoder = None
        self._server = None
        self._server_thread = None
        self.encoded = 0
        self.dropped = 0

    # ---- producer side ------------------------------------------------------
    def offer(self, frame, overlay, state=None):
        """Hand over the latest frame and telemetry. Never encodes here.

        `state` is published separately from the image and is NOT rate-limited
        with it: the numbers stay current at the loop's own rate even though the
        picture is encoded at PREVIEW_MAX_FPS. Copying a small dict per frame is
        microseconds; encoding a JPEG is not, which is the whole reason these
        two are decoupled.
        """
        if not self._run:
            return
        with self._lock:
            if self._pending is not None:
                self.dropped += 1     # the encoder never got to the previous one
            self._pending = (frame, dict(overlay))
            if state is not None:
                self._state = state

    def state(self):
        with self._lock:
            d = dict(self._state)
        d["preview_fps"] = self.max_fps
        d["preview_encoded"] = self.encoded
        d["preview_dropped"] = self.dropped
        return d

    # ---- lifecycle ----------------------------------------------------------
    def start(self):
        self._run = True
        self._encoder = threading.Thread(target=self._encode_loop, name="preview",
                                         daemon=True)
        self._encoder.start()
        self._server = ThreadingHTTPServer(("0.0.0.0", self.port), _make_handler(self))
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever,
                                               name="preview-http", daemon=True)
        self._server_thread.start()
        return self

    def stop(self):
        self._run = False
        with self._cv:
            self._cv.notify_all()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._encoder is not None:
            self._encoder.join(timeout=2.0)

    # ---- consumer side ------------------------------------------------------
    def latest(self, after_seq, timeout=5.0):
        """(seq, jpeg) once something newer than after_seq exists, else (seq, None)."""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._run and self._seq <= after_seq:
                if not self._cv.wait(max(0.01, deadline - time.monotonic())):
                    if time.monotonic() >= deadline:
                        return self._seq, None
            return self._seq, self._jpeg

    # ---- the encoder thread -------------------------------------------------
    def _encode_loop(self):
        period = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        while self._run:
            t0 = time.monotonic()
            with self._lock:
                item, self._pending = self._pending, None
            if item is None:
                time.sleep(0.02)
                continue
            frame, ov = item
            try:
                jpg = self._render(frame, ov)
            except Exception:
                # A preview must never be able to take the pipeline down.
                time.sleep(period)
                continue
            with self._cv:
                self._jpeg = jpg
                self._seq += 1
                self.encoded += 1
                self._cv.notify_all()
            lag = period - (time.monotonic() - t0)
            if lag > 0:
                time.sleep(lag)

    def _render(self, frame, ov):
        view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        for (x, y, w, h, *_rest) in ov.get("boxes", ()):
            cv2.rectangle(view, (int(x), int(y)), (int(x + w), int(y + h)),
                          config.VIEWER_BOX_COLOR, 1)

        cue = ov.get("cue_point")
        if cue is not None:
            cx, cy = int(cue[0]), int(cue[1])
            cv2.circle(view, (cx, cy), int(config.CUE_ACQUIRE_MAX_PX),
                       config.PREVIEW_CUE_COLOR, 1)
            cv2.drawMarker(view, (cx, cy), config.PREVIEW_CUE_COLOR,
                           cv2.MARKER_TILTED_CROSS, 22, 2)

        los = ov.get("los_point")
        if los is not None:
            status = ov.get("status", "none")
            colour = (config.VIEWER_LOS_TRACK_COLOR if status == "tracking"
                      else config.PREVIEW_DROP_COLOR if status == "dropped"
                      else config.VIEWER_LOS_COAST_COLOR)
            cv2.drawMarker(view, (int(los[0]), int(los[1])), colour,
                           cv2.MARKER_CROSS, 26, 2)
            gate = ov.get("gate_px")
            if gate:
                cv2.circle(view, (int(los[0]), int(los[1])), int(gate), colour, 1)

        if self.scale != 1.0:
            view = cv2.resize(view, None, fx=self.scale, fy=self.scale,
                              interpolation=cv2.INTER_AREA)

        for i, line in enumerate(ov.get("hud", ())):
            cv2.putText(view, line, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)

        ok, jpg = cv2.imencode(".jpg", view,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return jpg.tobytes() if ok else None


def _make_handler(preview):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_a):
            pass                      # this runs headless; keep the journal clean

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(_INDEX)))
                self.end_headers()
                self.wfile.write(_INDEX)
                return
            if self.path == "/api/state":
                body = json.dumps(preview.state(), default=str).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/frame.jpg":
                seq, jpg = preview.latest(-1, timeout=5.0)
                if jpg is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpg)
                return
            if self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        seq, jpg = preview.latest(seq, timeout=10.0)
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode()
                                         + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass              # a viewer closed the tab; not an error
                return
            self.send_error(404)

    return Handler
