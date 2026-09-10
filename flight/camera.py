"""The camera, as a flight service needs it: stamped, latest-only, and tappable.

    src = FlightCamera("/dev/thermal0", 1280, 1024).start()
    src.sink = recorder          # the GRAB thread now feeds it every frame
    t, frame, dropped = src.read()

THREE PROPERTIES, each for a reason.

STAMPED IN THE GRAB THREAD, the instant read() returns. An unstamped frame could
be anything up to a frame period old by the time the main loop looks at it, and
40 ms of unknown age is the same order as the ~200 ms camera latency the
attitude lookup is compensating for. Stamping downstream would fold that
uncertainty straight into the LOS solution.

LATEST-ONLY for the processing path. The detector is slower than the camera, so
queueing every frame would fall further and further behind; serving whatever is
newest means lag stays bounded at one frame no matter how slow detection gets.
The frames skipped that way are counted, not hidden -- `dropped` is how you know
the processing path is losing them.

BUT THE RECORDER TAPS THE GRAB THREAD, NOT THE MAIN LOOP. If recording were fed
from the processing path it would inherit the latest-only skipping, and a raw
capture would silently be missing exactly the frames the algorithm was too busy
to look at -- which are the interesting ones. Assigning `.sink` makes the grab
thread offer every frame it receives, so the recording is complete regardless of
what the tracker is doing. offer() never blocks; a recorder that cannot keep up
drops and counts, and that is its problem, not the camera's.
"""
import pathlib
import sys
import threading
import time

import cv2

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config


class FlightCamera:
    def __init__(self, device=None, width=None, height=None, fourcc=None):
        self.device = str(config.CAMERA_DEVICE if device is None else device)
        self.w = int(config.CAMERA_WIDTH if width is None else width)
        self.h = int(config.CAMERA_HEIGHT if height is None else height)
        self.fourcc = str(config.CAMERA_FOURCC if fourcc is None else fourcc)
        self.cap = None
        self.sink = None            # set to a RawRecWriter to record every frame
        self.lock = threading.Lock()
        self._latest = None         # (t_mono, frame)
        self._served = None         # the object last handed out, to detect staleness
        self.running = False
        self.thread = None
        self.grabbed = 0            # frames the grab thread received
        self.skipped = 0            # frames overwritten before the loop read them
        self.read_fail = 0

    def start(self, first_frame_timeout=None):
        first_frame_timeout = float(config.CAMERA_FIRST_FRAME_TIMEOUT_S
                                    if first_frame_timeout is None else first_frame_timeout)
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                "%s: could not open. The core permits ONE streaming reader -- stop "
                "whatever holds it first (fuser -v %s)." % (self.device, self.device))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # keep raw single-channel GREY

        self.running = True
        self.thread = threading.Thread(target=self._loop, name="grab", daemon=True)
        self.thread.start()
        deadline = time.monotonic() + first_frame_timeout
        while self._latest is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self._latest is None:
            self.stop()
            raise RuntimeError("%s: opened but delivered no frame in %.0fs -- do "
                               "--width/--height match what it actually sends?"
                               % (self.device, first_frame_timeout))
        return self

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.read_fail += 1
                time.sleep(0.01)
                continue
            t = time.monotonic()
            self.grabbed += 1
            # The recorder gets EVERY frame, straight off this thread.
            sink = self.sink
            if sink is not None:
                sink.offer(frame, t_mono=t)
            with self.lock:
                if self._latest is not None and self._latest is not self._served:
                    self.skipped += 1     # the loop never got to the previous one
                self._latest = (t, frame)

    def read(self):
        """(t_mono, frame, skipped_since_last_read). frame is None if nothing new."""
        with self.lock:
            cur = self._latest
            if cur is None or cur is self._served:
                return None, None, 0
            self._served = cur
            skipped, self.skipped = self.skipped, 0
        return cur[0], cur[1], skipped

    def stop(self):
        # Drain the grab thread before releasing: it is parked in a blocking
        # cap.read(), and tearing the interpreter down around it aborts the
        # process with "FATAL: exception not rethrown".
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=config.CAMERA_THREAD_JOIN_S)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
