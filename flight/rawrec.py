"""Write .rawrec captures, in the format this repo's readers already parse.

    rec = RawRecWriter(path, 1280, 1024, meta={"focal": 1516}).open()
    rec.offer(frame, t_mono)      # called from the CAMERA thread, never blocks
    stats = rec.close()

FORMAT, and why it is fixed rather than convenient. tools/rawrec_viewer.py,
rawrec_view.py, tools/det_bench.py and experiment/los_static_track.py all read
.rawrec already. A recorder that invented its own layout would produce files
none of them could open, so the layout below is not a choice:

    file header, 4096 bytes
        0x000  8s   b"RAWREC\\x00\\x01"
        0x008  IIIIII  hdr_len, rec_len, width, height, bpp, frame_bytes
        0x020  8s   pixfmt, NUL-padded
        0x028  dd   t_wall, t_mono at open
        0x038  I    length of the JSON below
        0x080  JSON descriptor, NUL-terminated  <- what read_header() parses
    then, repeating, rec_len = 32 + frame_bytes each:
        0x00  I  magic 0xA5F00DEC
        0x04  Q  seq
        0x0C  d  t_mono        <- what build_index() reads
        0x14  d  t_wall
        0x1C  H  flags
        0x1E  H  crc16 of the preceding 30 bytes

NO FINALISATION STEP, DELIBERATELY. Each frame's header and pixels go out in one
write() and there is no index or trailer, so a file is fully readable the instant
it is written. close() flushes and reports; it does not "save". That is the
property that matters in a crash: an mp4 needs release() to write its index, and
skipping it is what left 5 of 27 recordings on this rig unplayable, including
the two largest flights.

THE CAMERA THREAD MUST NEVER BLOCK ON DISK. offer() copies into a bounded queue
and returns; a writer thread drains it. If the queue is full the frame is
DROPPED and counted, because falling behind on disk must cost recording quality
and nothing else -- never a dropped camera frame, and never a stalled tracker.

IT REFUSES BEFORE THE CARD FILLS. Raw is ~118 GB/hour at 25 fps, so a long
sortie will hit the end of the disk. open() refuses if less than reserve_mb is
free, and the writer stops cleanly at that threshold instead of letting a full
disk take the CSV log or the pipeline down with it.
"""
import json
import os
import pathlib
import struct
import sys
import threading
import time
import zlib
from collections import deque

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

MAGIC_FILE = b"RAWREC\x00\x01"
MAGIC_FRAME = 0xA5F00DEC
HDR_LEN = 4096
REC_HDR = 32
_REC_FMT = "<IQddHH"                 # magic, seq, t_mono, t_wall, flags, crc16
assert struct.calcsize(_REC_FMT) == REC_HDR
_FILE_FMT = "<8sIIIIII8sddI"


class RawRecWriter:
    """One .rawrec file. Thread-safe for one producer and this object's writer thread."""

    def __init__(self, path, width, height, bpp=1, pixfmt=None, meta=None,
                 qdepth=None, reserve_mb=None, max_gb=None):
        self.path = str(path)
        self.w, self.h, self.bpp = int(width), int(height), int(bpp)
        self.pixfmt = str(config.CAMERA_FOURCC if pixfmt is None else pixfmt)
        self.meta = dict(meta or {})
        self.frame_bytes = self.w * self.h * self.bpp
        self.rec_len = REC_HDR + self.frame_bytes
        self.qdepth = max(2, int(config.RAW_QDEPTH_FRAMES if qdepth is None else qdepth))
        self.reserve_bytes = int(config.RAW_RESERVE_MB if reserve_mb is None
                                 else reserve_mb) * 1_000_000
        self.max_bytes = int((config.RAW_MAX_GB if max_gb is None else max_gb) * 1e9)

        self._fd = None
        self._q = deque()
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._thread = None
        self._closed = False
        self._seq = 0
        self.frames = 0
        self.offered = 0
        self.dropped = 0
        self.bytes_written = 0
        self.stop_reason = ""

    # ---- lifecycle ----------------------------------------------------------
    def open(self):
        d = os.path.dirname(os.path.abspath(self.path)) or "."
        # Create the directory before statvfs'ing it: a missing log directory
        # would otherwise surface as a bare ENOENT from the free-space check,
        # which reads like a disk fault rather than "that path does not exist".
        os.makedirs(d, exist_ok=True)
        st = os.statvfs(d)
        free = st.f_bavail * st.f_frsize
        if free <= self.reserve_bytes:
            raise OSError("refusing to record: %.1f GB free, reserve is %.1f GB (%s)"
                          % (free / 1e9, self.reserve_bytes / 1e9, d))
        self._fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        self._write_header()
        self._thread = threading.Thread(target=self._writer, name="rawrec", daemon=True)
        self._thread.start()
        return self

    def _write_header(self):
        meta = dict(self.meta)
        meta.update({"width": self.w, "height": self.h, "bpp": self.bpp,
                     "pixfmt": self.pixfmt, "rec_len": self.rec_len,
                     "frame_bytes": self.frame_bytes})
        js = json.dumps(meta, sort_keys=True).encode()[:HDR_LEN - 128]
        h = bytearray(HDR_LEN)
        struct.pack_into(_FILE_FMT, h, 0, MAGIC_FILE, HDR_LEN, self.rec_len,
                         self.w, self.h, self.bpp, self.frame_bytes,
                         self.pixfmt.encode().ljust(8, b"\0"),
                         time.time(), time.monotonic(), len(js))
        h[128:128 + len(js)] = js
        os.write(self._fd, h)
        self.bytes_written += HDR_LEN

    def close(self):
        """Flush what is queued and report. The file was already readable throughout."""
        if self._closed:
            return self.stats()
        self._closed = True
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=config.RAW_CLOSE_TIMEOUT_S)
        if self._fd is not None:
            try:
                os.fsync(self._fd)
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
        return self.stats()

    # ---- producer side ------------------------------------------------------
    def offer(self, frame, t_mono=None, t_wall=None, flags=0):
        """Queue one frame. Returns False if dropped. NEVER blocks on disk."""
        if self._closed or self._fd is None:
            return False
        self.offered += 1
        mv = frame if isinstance(frame, memoryview) else memoryview(frame)
        if not mv.contiguous:
            return False
        mv = mv.cast("B")
        if mv.nbytes != self.frame_bytes:
            return False
        rec = bytearray(self.rec_len)
        self._seq += 1
        tm = time.monotonic() if t_mono is None else float(t_mono)
        tw = time.time() if t_wall is None else float(t_wall)
        struct.pack_into(_REC_FMT, rec, 0, MAGIC_FRAME, self._seq, tm, tw, int(flags), 0)
        crc = zlib.crc32(bytes(rec[:REC_HDR - 2])) & 0xFFFF
        struct.pack_into("<H", rec, REC_HDR - 2, crc)
        rec[REC_HDR:] = mv
        with self._lock:
            if len(self._q) >= self.qdepth:
                self.dropped += 1
                return False
            self._q.append(rec)
        self._wake.set()
        return True

    # ---- writer thread ------------------------------------------------------
    def _writer(self):
        check_every = config.RAW_DISK_CHECK_FRAMES
        while True:
            with self._lock:
                rec = self._q.popleft() if self._q else None
            if rec is None:
                if self._closed:
                    return
                self._wake.wait(0.05)
                self._wake.clear()
                continue
            try:
                os.write(self._fd, rec)
            except OSError as e:
                self.stop_reason = "write failed: %s" % e
                self._closed = True
                return
            self.frames += 1
            self.bytes_written += self.rec_len
            if self.max_bytes and self.bytes_written >= self.max_bytes:
                self.stop_reason = "max size reached"
                self._closed = True
                return
            # Stop BEFORE the card fills, so a full disk cannot take the CSV log
            # or the pipeline down with it.
            if self.frames % check_every == 0:
                st = os.fstatvfs(self._fd)
                if st.f_bavail * st.f_frsize <= self.reserve_bytes:
                    self.stop_reason = "disk reserve reached"
                    self._closed = True
                    return

    # ---- reporting ----------------------------------------------------------
    def stats(self):
        return {"path": self.path, "frames": self.frames, "offered": self.offered,
                "dropped": self.dropped, "gb": self.bytes_written / 1e9,
                "stop_reason": self.stop_reason}

    @property
    def active(self):
        return not self._closed and self._fd is not None
