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

MANUAL ACQUISITION. The page also accepts a click on the image: the browser
posts the click's pixel in the JPEG's own (post-scale) coordinate space to
/api/click, and click_point() below divides that back by PREVIEW_SCALE so
every consumer downstream works in the camera's native frame, same as every
box and the cue point. That point only DOES something if the pipeline is
running with --initialiser manual_click (initialisation/manual_click.py); with
any other initialiser loaded, a click is accepted, stored, and simply never
read by anything -- harmless, not wired to the tracker.

RUNTIME CONTROLS. The page can also arm the algorithm and hot-swap the
detector/initialiser, over POST /api/control:
    {"algo": true|false}     ORed with the RC switch in flight_pipeline.py --
                              see the note there on why this can only ADD an
                              ON, never force an OFF the RC switch didn't ask
                              for.
    {"detector": "<name>"}   queued here, applied by the main loop at the top
    {"initialiser": "<name>"} of its next frame -- see pop_swaps().
    {"filter": "<name>", "enabled": true|false}
                              takes effect on the very next frame -- unlike
                              the detector/initialiser, a filter chain has no
                              "call in progress" to race, so this is a plain
                              toggle, not a queued swap. See filters_enabled().
GET /api/options lists the valid names for all three (set once at startup via
set_options(), from whatever tools/flight_pipeline.py actually loaded).
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
 /* EVERYTHING FITS IN ONE VIEWPORT, NO PAGE SCROLL: html/body are pinned to
    100% height with overflow hidden, #wrap fills that exactly, and the two
    columns size themselves to it. If a very short/narrow window still can't
    fit both columns' content, #panel and #ctrl scroll INTERNALLY (see their
    own overflow:auto) rather than the page growing -- clipping data with
    overflow:hidden there would hide telemetry silently, which is worse than
    an occasional inner scrollbar. */
 html,body{height:100%}
 body{background:#0b0b0b;color:#cfc;font-family:ui-monospace,monospace;margin:0;
      padding:8px;box-sizing:border-box;overflow:hidden}
 #wrap{height:calc(100% - 16px);display:flex;gap:10px;align-items:stretch}
 #left{flex:1 1 56%;min-width:0;display:flex;flex-direction:column;height:100%}
 #imgbox{flex:1 1 auto;min-height:0;display:flex;align-items:center;justify-content:center;
        overflow:hidden}
 #img{max-width:100%;max-height:100%;width:auto;height:auto;display:block;
     image-rendering:pixelated;border:1px solid #333;cursor:crosshair}
 #panel{flex:1 1 44%;min-width:280px;height:100%;overflow:auto;font-size:12px;line-height:1.4;
       columns:2;column-gap:16px}
 .grp{break-inside:avoid-column;break-inside:avoid}
 h2{font-size:11px;color:#6f6;margin:6px 0 2px;text-transform:uppercase;letter-spacing:.07em;
    border-bottom:1px solid #2a2a2a;padding-bottom:1px}
 table{border-collapse:collapse;width:100%}
 td{padding:0 6px 0 0;vertical-align:top}
 td.k{color:#7a8;width:47%}
 td.v{color:#dfd;font-weight:600}
 .ok{color:#5f5}.warn{color:#fc4}.bad{color:#f55}.off{color:#777}
 #clickmsg{flex:0 0 auto;font-size:11px;color:#7a8;min-height:1.3em;margin:4px 0 0}
 #ctrl{flex:0 0 auto;overflow:auto;margin-top:6px;padding:6px 8px;border:1px solid #2a2a2a;
      border-radius:4px}
 #ctrl label{display:block;font-size:10px;color:#7a8;text-transform:uppercase;
             letter-spacing:.06em;margin:5px 0 1px}
 #ctrl label:first-child{margin-top:0}
 #ctrl select,#ctrl button{width:100%;background:#151515;color:#cfc;border:1px solid #333;
                           padding:4px 6px;font:inherit;font-size:12px;border-radius:3px;
                           cursor:pointer}
 #algoBtn{font-weight:600;letter-spacing:.04em}
 #algoBtn.on{background:#163016;border-color:#3a6;color:#9f9}
 #algoBtn.off{background:#301616;border-color:#a55;color:#f99}
 #filterList{display:flex;flex-wrap:wrap;column-gap:12px;row-gap:1px}
 #filterList label{display:flex;align-items:center;gap:4px;font-size:12px;color:#dfd;
                   text-transform:none;letter-spacing:normal;margin:0;font-weight:normal}
 #filterList input{width:auto;cursor:pointer}
 #ctrlmsg{font-size:11px;color:#7a8;min-height:1.3em;margin-top:4px}
 /* Below ~700px tall or ~760px wide, one screen genuinely cannot hold a
    640x512 image AND ~10 telemetry groups AND the control panel at a
    legible size -- fall back to a normal scrolling page rather than
    shrinking everything into unreadability. */
 @media (max-height:700px),(max-width:760px){
   body{overflow:auto}
   #wrap{height:auto;flex-wrap:wrap}
   #left{height:auto}
   #imgbox{min-height:240px}
   #panel{height:auto;columns:1}
 }
</style></head><body>
<div id="wrap">
  <div id="left">
    <div id="imgbox"><img id="img" src="/stream.mjpg"></div>
    <div id="clickmsg">click the target to acquire manually (needs --initialiser manual_click)</div>
    <div id="ctrl">
      <label>algorithm</label>
      <button id="algoBtn" onclick="toggleAlgo()">...</button>
      <label>detector</label>
      <select id="detSel" onchange="postControl({detector:this.value})"></select>
      <label>initialiser</label>
      <select id="initSel" onchange="postControl({initialiser:this.value})"></select>
      <label>filters</label>
      <div id="filterList"></div>
      <div id="ctrlmsg"></div>
    </div>
  </div>
  <div id="panel">connecting...</div>
</div>
<script>
// Click-to-acquire: send the click in the JPEG's OWN pixel space (its natural
// width/height, i.e. already at PREVIEW_SCALE) -- the browser may be
// stretching or shrinking the displayed <img> to fit the layout, so the click
// position has to be rescaled from "where on screen" to "which JPEG pixel"
// before it means anything, and the server maps that back to the camera's
// native frame from there (see Preview.set_click).
(function(){
  const img = document.getElementById("img");
  const msg = document.getElementById("clickmsg");
  img.addEventListener("click", async (ev) => {
    const r = img.getBoundingClientRect();
    const nx = (ev.clientX - r.left) / r.width;
    const ny = (ev.clientY - r.top) / r.height;
    const x = Math.round(nx * (img.naturalWidth || r.width));
    const y = Math.round(ny * (img.naturalHeight || r.height));
    try {
      const resp = await fetch("/api/click", {
        method: "POST", headers: {"Content-Type": "application/json"},
        body: JSON.stringify({x, y}),
      });
      msg.textContent = resp.ok
        ? "acquiring near (" + x + ", " + y + ") ..."
        : "click rejected: " + (await resp.text());
    } catch (e) { msg.textContent = "click failed: " + e; }
    setTimeout(() => { msg.textContent =
      "click the target to acquire manually (needs --initialiser manual_click)"; },
      2500);
  });
})();

// Runtime controls: algo arm toggle, detector/initialiser hot-swap. All three
// go through one endpoint, /api/control; the outcome (accepted or an unknown
// name) comes back later via state().control_msg, not the POST response --
// the POST only confirms the request was QUEUED, applying it is the main
// loop's job (see Preview.pop_swaps in flight/preview.py).
let webArmed = false;   // last state THIS PAGE asked for; toggles independently
                        // of algo_armed, which is the RC-OR-web result
async function postControl(body){
  try{
    const r = await fetch("/api/control", {method:"POST",
      headers:{"Content-Type":"application/json"}, body: JSON.stringify(body)});
    if(!r.ok) document.getElementById("ctrlmsg").textContent = "rejected: "+(await r.text());
  }catch(e){ document.getElementById("ctrlmsg").textContent = "failed: "+e; }
}
function toggleAlgo(){
  webArmed = !webArmed;
  postControl({algo: webArmed});
}
function toggleFilter(name, on){
  postControl({filter: name, enabled: on});
}
async function loadOptions(){
  try{
    const o = await (await fetch("/api/options")).json();
    for(const [id,list] of [["detSel",o.detectors||[]],["initSel",o.initialisers||[]]]){
      document.getElementById(id).innerHTML =
        list.map(n => "<option value=\\""+n+"\\">"+n+"</option>").join("");
    }
    // Checkboxes, not a select: any subset of filters can run at once, so
    // this is independent toggles, not a single choice like the two above.
    // Starts checked -- every filter is ENABLED by default (see
    // Preview.set_options) -- and tick() below reconciles it with reality.
    document.getElementById("filterList").innerHTML = (o.filters||[]).map(n =>
      "<label><input type=checkbox checked data-filter=\\""+n+"\\" "
      + "onchange=\\"toggleFilter('"+n+"', this.checked)\\">"+n+"</label>").join("");
  }catch(e){}
}
loadOptions();

const G = [
 ["pipeline",   ["frame","fps","det_ms","loop_ms","roi","skipped","grabbed","uptime_s"]],
 ["tracker",    ["status","los_x","los_y","gate_px","n_in_gate","det_score","match_dist_px",
                 "alpha","omega_deg_s","misses","drops","last_release"]],
 ["detector",   ["detector","n_boxes","n_kept","initialiser","acq_via","acq_cueless_n"]],
 ["bearing sent",["az_deg","el_deg","det_valid","valid_hold","cap_latency_ms","seq_sent",
                 "body_az_deg","body_el_deg","los_n","los_e","los_d"]],
 ["radar cue",  ["cue_valid","cue_az_deg","cue_el_deg","cue_range_m","cue_age_ms",
                 "cue_u","cue_v","cue_resid_deg","cue_bad_frames","cue_seen"]],
 ["cube link",  ["sysid","cube_armed","att_hz","att_age_ms","hb_age_ms","lat_cam_pitch_deg",
                 "uplink","n_sent"]],
 ["rc switches",["algo_armed","algo_web_armed","arm_reason","rc_arm_us",
                 "rec_armed","rec_reason","rc_rec_us"]],
 ["recording",  ["rec_on","episode","rec_frames","raw_dropped","rec_path","rec_stop_reason"]],
 ["system",     ["cpu_temp_c","throttled","git_sha","dirty"]],
 ["power",      ["in_volt_v","core_amp_a","rail_watt_w"]],
 ["settings",   ["lookback_s","focal_px","cue_acquire_px","manual_click_px","cue_drop_deg",
                 "cue_drop_frames","min_scr","preview_fps"]],
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
  // 4.75 V is the 5V spec floor -- under it a brownout is a question of when.
  // Between 4.75 and 4.85 there is little headroom left for the droop that
  // arrives with load. Same lines tools/disk_soak.py checks against.
  if(k==="in_volt_v") return Number(v)<4.75?"bad":(Number(v)<4.85?"warn":"ok");
  // config.LAT_DET_VALID_TIMEOUT (500ms) is the point past which the firmware
  // itself calls a detection stale -- bad at that line, warn at half of it.
  if(k==="cap_latency_ms") return Number(v)>500?"bad":(Number(v)>250?"warn":"ok");
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
      // .grp keeps a group's heading glued to its own table across the
      // panel's CSS columns -- without it, a column break can split a
      // heading from the rows it labels.
      h+="<div class=grp><h2>"+title+"</h2><table>";
      for(const k of keys){
        if(!(k in s)) continue;
        h+="<tr><td class=k>"+k+"</td><td class='v "+cls(k,s[k])+"'>"+fmt(s[k])+"</td></tr>";
      }
      h+="</table></div>";
    }
    document.getElementById("panel").innerHTML=h;

    const btn = document.getElementById("algoBtn");
    webArmed = !!s.algo_web_armed;         // reflect what another tab requested too
    btn.textContent = "ALGO: " + (s.algo_armed ? "ON" : "OFF")
      + (webArmed ? "  (web armed)" : "");
    btn.className = s.algo_armed ? "on" : "off";

    // Keep a select showing what is actually loaded, but never fight someone
    // mid-click: a user with the dropdown open should not have it reset from
    // under them by the next 400 ms poll.
    for(const [id,key] of [["detSel","detector"],["initSel","initialiser"]]){
      const sel = document.getElementById(id);
      if(sel && s[key] && sel.value !== s[key] && document.activeElement !== sel)
        sel.value = s[key];
    }
    // Same "don't fight the mouse" rule as the selects above: never re-check a
    // box the user has focused, so a click and the next poll can't race.
    if(s.filters_enabled){
      const on = new Set(s.filters_enabled);
      for(const cb of document.querySelectorAll("#filterList input")){
        if(document.activeElement !== cb) cb.checked = on.has(cb.dataset.filter);
      }
    }
    if(s.control_msg) document.getElementById("ctrlmsg").textContent = s.control_msg;
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
        self._click = None            # (x, y, expire_monotonic) in native-frame px
        self._web_armed = False       # operator's requested algo state, from the page
        self._swap = {}               # pending {"detector"|"initialiser": name}
        self._options = {"detectors": [], "initialisers": [], "filters": []}
        self._filters_enabled = None  # None until set_options() runs = "not known yet"
        self._control_msg = ""        # last swap/click outcome, shown on the page
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
            d["algo_web_armed"] = self._web_armed
            d["control_msg"] = self._control_msg
            d["filters_enabled"] = (sorted(self._filters_enabled)
                                    if self._filters_enabled is not None else None)
        d["preview_fps"] = self.max_fps
        d["preview_encoded"] = self.encoded
        d["preview_dropped"] = self.dropped
        return d

    # ---- manual acquisition (operator clicks the image) ----------------------
    def set_click(self, x, y):
        """Record a click, given in the JPEG's OWN pixel space (post-scale).

        Divided back by self.scale here, once, so every consumer -- the
        manual_click initialiser included -- works in the camera's native
        frame, same as every detector box and the cue point.
        """
        with self._lock:
            ox = x / self.scale if self.scale else float(x)
            oy = y / self.scale if self.scale else float(y)
            self._click = (ox, oy, time.monotonic() + config.MANUAL_CLICK_TIMEOUT_S)

    def click_point(self):
        """(x, y) of a pending click in native-frame pixels, or None.

        Auto-expires: a click that never landed near a real detection must not
        silently reappear and cause a surprise lock long after the operator
        moved on. Does NOT clear itself on a merely unsuccessful match -- the
        caller (an initialiser) clears it explicitly once it actually acquires,
        via clear_click(), so a click gets a few frames of detector noise to
        land in before it expires.
        """
        with self._lock:
            if self._click is None:
                return None
            x, y, expire = self._click
            if time.monotonic() > expire:
                self._click = None
                return None
            return (x, y)

    def clear_click(self):
        """Consume the pending click so it cannot re-acquire after a later drop."""
        with self._lock:
            self._click = None

    # ---- runtime controls (algo arm, detector/initialiser swap, filters) -----
    def set_options(self, detectors, initialisers, filters=()):
        """Populate the page's dropdowns/checkboxes. Called once at startup
        with whatever tools/flight_pipeline.py actually loaded -- never
        guessed here. Every filter starts ENABLED, matching the pipeline's own
        behaviour before any of this existed: with no preview running (or
        before this call), every loaded filter runs."""
        with self._lock:
            self._options = {"detectors": list(detectors),
                             "initialisers": list(initialisers),
                             "filters": list(filters)}
            self._filters_enabled = set(filters)

    def options(self):
        with self._lock:
            return dict(self._options)

    def filters_enabled(self):
        """Copy of the set of filter names that should run this frame, or None
        if set_options() has not been called yet (main loop then applies its
        own default -- see tools/flight_pipeline.py). A COPY, not the live
        set, so a toggle arriving on the HTTP thread mid-frame cannot mutate
        the very set the main loop is in the middle of checking membership
        against."""
        with self._lock:
            return (set(self._filters_enabled)
                   if self._filters_enabled is not None else None)

    def set_filter_enabled(self, name, on):
        with self._lock:
            if self._filters_enabled is None:
                self._filters_enabled = set()
            if on:
                self._filters_enabled.add(name)
            else:
                self._filters_enabled.discard(name)

    def web_armed(self):
        """The operator's last-requested algo state. The caller (flight_
        pipeline's arm logic) ORs this with the RC switch -- see the note
        there on why this can only add an ON, never force an OFF."""
        with self._lock:
            return self._web_armed

    def set_web_armed(self, on):
        with self._lock:
            self._web_armed = bool(on)

    def request_swap(self, field, value):
        """Queue a detector/initialiser change. Applied by the main loop at
        the top of its next frame (see tools/flight_pipeline.py) -- never
        here, so detect()/init_fn() are never reassigned out from under a
        call to them in progress on that other thread."""
        with self._lock:
            self._swap[field] = value

    def pop_swaps(self):
        """{field: name, ...} pending since the last call, and clears it."""
        with self._lock:
            swap, self._swap = self._swap, {}
            return swap

    def set_control_msg(self, msg):
        """One-line human-readable outcome of the last swap, for the page."""
        with self._lock:
            self._control_msg = msg

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
            if self.path == "/api/options":
                body = json.dumps(preview.options()).encode()
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

        def do_POST(self):
            if self.path == "/api/click":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    x, y = float(body["x"]), float(body["y"])
                except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                    self.send_error(400, "expected JSON {\"x\":..,\"y\":..}")
                    return
                preview.set_click(x, y)
                body = b'{"ok":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/api/control":
                # Accepts any subset of {"algo":bool, "detector":name,
                # "initialiser":name} in one body. Validity of a name is NOT
                # checked here -- this thread has no access to what actually
                # loaded -- it is queued and the main loop applies or rejects
                # it, with the outcome surfacing via state()["control_msg"].
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(body, dict):
                        raise ValueError("not an object")
                except (ValueError, json.JSONDecodeError):
                    self.send_error(400, "expected a JSON object")
                    return
                if "algo" in body:
                    preview.set_web_armed(bool(body["algo"]))
                if "detector" in body:
                    preview.request_swap("detector", str(body["detector"]))
                if "initialiser" in body:
                    preview.request_swap("initialiser", str(body["initialiser"]))
                if "filter" in body:
                    preview.set_filter_enabled(str(body["filter"]),
                                              bool(body.get("enabled", True)))
                resp = b'{"ok":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)
                return
            self.send_error(404)

    return Handler
