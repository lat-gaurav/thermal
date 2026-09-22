"""Write the per-frame telemetry CSV without ever blocking the tracking loop.

    tel = TelemetryWriter(path, CSV_HEADER).open()
    tel.offer(row)          # called from the MAIN loop thread, never blocks
    stats = tel.close()

THE SAME DISCIPLINE AS flight/rawrec.py's RawRecWriter, applied to the one
place it was missing. offer() copies the row into a bounded queue and
returns; a writer thread drains it, doing the actual csv.writer.writerow()
and the periodic flush(). If the queue is full the row is DROPPED AND
COUNTED -- never blocks the caller. Read flight/rawrec.py's docstring first;
every argument here is the same argument, for the same reason.

WHY THIS EXISTS. tools/flight_pipeline.py's main loop used to call
csv.writer.writerow() directly, followed by a periodic csv_f.flush() every
config.FLIGHT_CSV_FLUSH_FRAMES frames -- on the SAME thread that reads the
camera, looks up attitude, runs the detector, updates the tracker and sends
the Cube uplink. Measured on a real sortie recorded to the SD-card fallback
(2026-09-18, flight-20260918-151759-66b140cb-01): that flush() blocked for up
to 6.3 s at a time when the disk's write queue backed up, and because
send_detection() for the CURRENT frame happens before the flush but every
SUBSEQUENT frame's read/detect/track/send happens after it, the effect was
not "a slow CSV write" -- it was 49 separate multi-second windows, adding up
to 29% of the sortie, in which detection, tracking and the Cube uplink all
went completely silent. Not even a valid=0 "I lost it" reached the Cube during
those windows; guidance saw nothing until RPI_COMMS.md's LAT_DET_TIMEOUT_S
(0.5 s) declared it stale on its own.

Losing a telemetry row is cheap. Losing live control because writing that row
blocked is not. tools/flight_pipeline.py's own close_episode() already
documents fixing the identical failure once, for RawRecWriter.close()
(measured: closing a 0.92 GB episode on the SD card stalled detection for
9.6 s). This is that same fix, applied to the per-frame write path instead of
the once-per-episode close path.

WHAT IS DELIBERATELY NOT HERE. No index, no finalisation step -- close()
flushes and reports, same as RawRecWriter; the file is readable throughout,
including mid-sortie (tail it, or open it in tools/telemetry_viewer.py while
the pipeline is still running).
"""
import csv
import os
import pathlib
import sys
import threading
from collections import deque

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config


class TelemetryWriter:
    """One telemetry CSV. Thread-safe for one producer and this object's writer thread."""

    def __init__(self, path, header, qdepth=None, flush_every=None, close_timeout=None):
        self.path = str(path)
        self.header = list(header)
        self.qdepth = max(2, int(config.TELEM_QDEPTH_ROWS if qdepth is None else qdepth))
        self.flush_every = max(1, int(config.FLIGHT_CSV_FLUSH_FRAMES
                                      if flush_every is None else flush_every))
        self.close_timeout = (config.RAW_CLOSE_TIMEOUT_S if close_timeout is None
                              else close_timeout)  # same budget RawRecWriter's close() gets

        self._fh = None
        self._w = None
        self._q = deque()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._thread = None
        self._closed = False
        self.offered = 0
        self.dropped = 0
        self.written = 0

    # ---- lifecycle ------------------------------------------------------------
    def open(self):
        d = os.path.dirname(os.path.abspath(self.path)) or "."
        os.makedirs(d, exist_ok=True)
        self._fh = open(self.path, "w", newline="")
        self._w = csv.writer(self._fh)
        self._w.writerow(self.header)      # once, at startup -- never in the hot path
        self._thread = threading.Thread(target=self._writer, name="telemetry", daemon=True)
        self._thread.start()
        return self

    def close(self):
        """Drain what is queued and report. The file was readable throughout."""
        if self._closed:
            return self.stats()
        self._closed = True
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=self.close_timeout)
        if self._fh is not None:
            try:
                self._fh.flush()
            except OSError:
                pass
            self._fh.close()
            self._fh = None
        return self.stats()

    # ---- producer side (the main loop) -----------------------------------------
    def offer(self, row):
        """Queue one row. Returns False if dropped. NEVER blocks on disk."""
        if self._closed or self._fh is None:
            return False
        self.offered += 1
        with self._lock:
            if len(self._q) >= self.qdepth:
                self.dropped += 1
                return False
            self._q.append(row)
        self._wake.set()
        return True

    # ---- writer thread ----------------------------------------------------------
    def _writer(self):
        n_since_flush = 0
        while True:
            with self._lock:
                row = self._q.popleft() if self._q else None
            if row is None:
                if self._closed:
                    return
                self._wake.wait(0.05)
                self._wake.clear()
                continue
            try:
                self._w.writerow(row)
            except (OSError, ValueError):
                # A row the csv module itself rejected, or the file went away
                # under us (disk pulled, fs remounted read-only). Either way
                # this thread is done; the main loop keeps flying regardless.
                return
            self.written += 1
            n_since_flush += 1
            if n_since_flush >= self.flush_every:
                n_since_flush = 0
                try:
                    self._fh.flush()
                except OSError:
                    pass

    # ---- reporting ----------------------------------------------------------------
    def stats(self):
        return {"path": self.path, "offered": self.offered,
                "written": self.written, "dropped": self.dropped}

    @property
    def active(self):
        return not self._closed and self._fh is not None
