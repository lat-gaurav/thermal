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

COMPRESSION (config.RAW_COMPRESS = "zstd"), and why it exists. The SD card on the
airframe sustains ~29.5 MB/s when cool and ~24 MB/s when hot (disk_soak.py,
2026-10-28), and raw 1280x1024 GREY at 25 fps needs 32.8 MB/s -- so uncompressed
recording on the card drops 10-22% of frames no matter how the page cache is
tuned. zstd level 1 shrinks real thermal frames ~2.1x (measured on flight
20261027-182012: 15.3 MB/s needed) at ~8.6 ms/frame on one Pi 5 core.

The file is then <name>.rawrec.zst, and it is EXACTLY the .rawrec above, cut
into independent zstd frames: one for the 4096-byte header, one per record.

    zstd -d flight-S-01.rawrec.zst            -> byte-identical flight-S-01.rawrec
    tools/rawrec_unzst.py flight-S-01.rawrec.zst   (same, and survives a cut tail)

So every existing reader keeps working on the decompressed file, and the crash
property is kept: each record is a complete zstd frame the moment it is written,
so a power cut loses at most the record being written, never the file.

Compression runs on its OWN thread, between the camera's queue and the disk
writer, so compress time and disk-blocked time overlap instead of adding up.
Lossless: the decompressed pixels are the bytes the sensor sent.

THE OTHER HALF OF THE FIX IS NOT IN THIS FILE. With Linux's default write-back
(dirty_background_ratio 10 / dirty_ratio 20 of 7.9 GB RAM) the kernel let ~650 MB
pile up and then flushed it in one go; os.write() blocked for 5.5-8.3 s every ~48 s
(disk_soak, 2026-10-29). deploy/90-flight-writeback.conf caps dirty data at
32/128 MB so it trickles out continuously: max write block fell to 246 ms and the
queue peaked at 6 of 250 frames. Without it, a 2 s queue overflows.
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

try:                                  # Debian: sudo apt install python3-zstandard
    import zstandard as _zstd
except Exception:                     # recorder must still run without it
    _zstd = None

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
    """One .rawrec (or .rawrec.zst) file.

    Three threads touch it: ONE producer calling offer() (the camera thread), the
    compressor thread and the writer thread. offer() never blocks.
    """

    def __init__(self, path, width, height, bpp=1, pixfmt=None, meta=None,
                 qdepth=None, reserve_mb=None, max_gb=None, compress=None, level=None):
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

        comp = str(getattr(config, "RAW_COMPRESS", "") if compress is None else compress or "")
        comp = comp.strip().lower()
        self.warning = ""
        if comp not in ("", "none", "zstd"):
            raise ValueError("RAW_COMPRESS must be '' or 'zstd', not %r" % comp)
        if comp == "zstd" and _zstd is None:
            self.warning = ("zstandard not installed -- recording UNCOMPRESSED "
                            "(sudo apt install python3-zstandard)")
            comp = ""
        self.compress = "zstd" if comp == "zstd" else ""
        self.level = int(getattr(config, "RAW_ZSTD_LEVEL", 1) if level is None else level)
        if self.compress and not self.path.endswith(".zst"):
            self.path += ".zst"
        # One compressor, used only by the compressor thread (and by open()
        # for the header, before that thread exists). ZstdCompressor is not
        # safe to share between threads concurrently; it is never shared.
        self._cctx = (_zstd.ZstdCompressor(level=self.level, write_checksum=True,
                                           write_content_size=True)
                      if self.compress else None)

        self._fd = None
        self._q = deque()             # raw records, filled by offer()
        self._zq = deque()            # encoded records, drained by the writer
        self._lock = threading.Lock()
        self._wake = threading.Event()    # something for the compressor
        self._zwake = threading.Event()   # something for the writer
        self._zroom = threading.Event()   # the writer made room in _zq
        self._threads = []
        self._closed = False          # no more offers accepted
        self._abort = False           # stop everything now (error / limit)
        self._enc_done = False        # compressor has drained and exited
        self._seq = 0
        self.frames = 0               # records on disk
        self.offered = 0
        self.dropped = 0
        self.bytes_written = 0        # logical .rawrec bytes on disk (what decompresses out)
        self.bytes_disk = 0           # bytes actually written to the card
        self.encode_ms_sum = 0.0
        # write-path diagnostics: how long the card made us wait
        self.t_open = time.monotonic()
        self.write_ms = []          # duration of every os.write
        self.stalls = []            # (t_since_open, ms) for writes > 100 ms
        self.drop_times = []        # t_since_open of each dropped frame
        self.q_max = 0
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
        for target, name in ((self._encoder, "rawrec-enc"), (self._writer, "rawrec")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def _encode(self, buf):
        return self._cctx.compress(bytes(buf)) if self._cctx is not None else buf

    def _write_header(self):
        meta = dict(self.meta)
        meta.update({"width": self.w, "height": self.h, "bpp": self.bpp,
                     "pixfmt": self.pixfmt, "rec_len": self.rec_len,
                     "frame_bytes": self.frame_bytes})
        if self.compress:
            # Lets a reader of the decompressed file know it came from a .zst.
            meta.update({"container": "zstd", "zstd_level": self.level})
        js = json.dumps(meta, sort_keys=True).encode()[:HDR_LEN - 128]
        h = bytearray(HDR_LEN)
        struct.pack_into(_FILE_FMT, h, 0, MAGIC_FILE, HDR_LEN, self.rec_len,
                         self.w, self.h, self.bpp, self.frame_bytes,
                         self.pixfmt.encode().ljust(8, b"\0"),
                         time.time(), time.monotonic(), len(js))
        h[128:128 + len(js)] = js
        out = self._encode(h)
        os.write(self._fd, out)
        self.bytes_written += HDR_LEN
        self.bytes_disk += len(out)

    def close(self):
        """Flush what is queued and report. The file was already readable throughout."""
        if not self._closed:
            self._closed = True
        self._wake.set()
        self._zwake.set()
        deadline = time.monotonic() + config.RAW_CLOSE_TIMEOUT_S
        for t in self._threads:
            t.join(timeout=max(0.0, deadline - time.monotonic()))
        self._threads = []
        if self._fd is not None:
            try:
                os.fsync(self._fd)
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
        return self.stats()

    def _stop(self, reason):
        self.stop_reason = self.stop_reason or reason
        self._abort = True
        self._closed = True
        self._wake.set()
        self._zwake.set()
        self._zroom.set()

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
                self.drop_times.append(time.monotonic() - self.t_open)
                return False
            self._q.append(rec)
            self.q_max = max(self.q_max, len(self._q))
        self._wake.set()
        return True

    # ---- compressor thread --------------------------------------------------
    def _encoder(self):
        """_q -> zstd -> _zq. Waits (never drops) when the writer is behind:
        backpressure lands on _q, where offer() drops and counts."""
        try:
            while not self._abort:
                with self._lock:
                    rec = self._q.popleft() if self._q else None
                if rec is None:
                    if self._closed:
                        return
                    self._wake.wait(0.05)
                    self._wake.clear()
                    continue
                t0 = time.perf_counter()
                out = self._encode(rec)
                self.encode_ms_sum += 1000.0 * (time.perf_counter() - t0)
                while not self._abort:
                    with self._lock:
                        if len(self._zq) < self.qdepth:
                            self._zq.append(out)
                            break
                    self._zroom.wait(0.05)
                    self._zroom.clear()
                self._zwake.set()
        except Exception as e:                      # never let it die silently
            self._stop("compressor failed: %s" % e)
        finally:
            self._enc_done = True
            self._zwake.set()

    # ---- writer thread ------------------------------------------------------
    def _writer(self):
        check_every = config.RAW_DISK_CHECK_FRAMES
        while not self._abort:
            with self._lock:
                out = self._zq.popleft() if self._zq else None
            if out is None:
                if self._enc_done:
                    return
                self._zwake.wait(0.05)
                self._zwake.clear()
                continue
            self._zroom.set()
            try:
                tw0 = time.monotonic()
                os.write(self._fd, out)
                ms = 1000.0 * (time.monotonic() - tw0)
                self.write_ms.append(ms)
                if ms > 100:
                    self.stalls.append((tw0 - self.t_open, ms))
            except OSError as e:
                self._stop("write failed: %s" % e)
                return
            self.frames += 1
            self.bytes_written += self.rec_len
            self.bytes_disk += len(out)
            if self.max_bytes and self.bytes_disk >= self.max_bytes:
                self._stop("max size reached")
                return
            # Stop BEFORE the card fills, so a full disk cannot take the CSV log
            # or the pipeline down with it.
            if self.frames % check_every == 0:
                st = os.fstatvfs(self._fd)
                if st.f_bavail * st.f_frsize <= self.reserve_bytes:
                    self._stop("disk reserve reached")
                    return

    # ---- reporting ----------------------------------------------------------
    def stats(self):
        n = max(1, self.frames)
        return {"path": self.path, "frames": self.frames, "offered": self.offered,
                "dropped": self.dropped, "gb": self.bytes_written / 1e9,
                "gb_disk": self.bytes_disk / 1e9,
                "ratio": (self.bytes_written / self.bytes_disk) if self.bytes_disk else 1.0,
                "compress": ("zstd-%d" % self.level) if self.compress else "none",
                "encode_ms": self.encode_ms_sum / n,
                "queued": len(self._q), "queued_encoded": len(self._zq),
                "stop_reason": self.stop_reason, "warning": self.warning}

    @property
    def active(self):
        return not self._closed and self._fd is not None
