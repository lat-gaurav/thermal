#!/usr/bin/env python3
"""Turn a .rawrec.zst back into the exact .rawrec every reader in this repo opens.

    tools/rawrec_unzst.py flight-S-01.rawrec.zst              -> flight-S-01.rawrec
    tools/rawrec_unzst.py flight-S-*.rawrec.zst -o /some/dir/
    tools/rawrec_unzst.py flight-S-01.rawrec.zst --check      verify only, write nothing

flight/rawrec.py writes a .rawrec.zst as ONE INDEPENDENT ZSTD FRAME PER PIECE:
the 4096-byte file header, then each 32-byte record header + pixels. Decoding
them back to back gives a byte-identical .rawrec (`zstd -d` does the same).

Why this tool and not just `zstd -d`: after a power cut the LAST zstd frame can
be cut short. `zstd -d` reports that as an error; this decodes every complete
frame, drops only the cut one, and says how many records it recovered. It also
checks each record's magic and crc16, so a recording is verified on the way out.
"""
import argparse
import os
import pathlib
import struct
import sys
import zlib

try:
    import zstandard as zstd
except ImportError:
    sys.exit("needs zstandard:  sudo apt install python3-zstandard   (or pip install zstandard)")

HDR_LEN, REC_HDR, MAGIC_FRAME = 4096, 32, 0xA5F00DEC


def frames(f, chunk=1 << 20):
    """Yield the decoded content of each zstd frame in f, in order, streaming.

    A frame that is still incomplete at end of file (cut by a power loss) is
    dropped silently; everything before it is yielded."""
    dctx = zstd.ZstdDecompressor()
    obj, out, buf = dctx.decompressobj(), [], b""
    while True:
        if not buf:
            buf = f.read(chunk)
            if not buf:
                return
        piece = obj.decompress(buf)
        if piece:
            out.append(piece)
        if obj.eof:
            yield b"".join(out)
            buf, out, obj = obj.unused_data, [], dctx.decompressobj()
        else:
            buf = b""


def convert(src, dst, check_only):
    rec_len = None
    n_ok = n_bad = 0
    seq_first = seq_last = None
    out = None if check_only else open(dst + ".part", "wb")
    with open(src, "rb") as f:
        for i, blob in enumerate(frames(f)):
            if i == 0:
                if len(blob) != HDR_LEN or blob[:8] != b"RAWREC\x00\x01":
                    sys.exit("%s: not a .rawrec.zst (bad file header)" % src)
                rec_len = struct.unpack_from("<I", blob, 12)[0]
            else:
                if len(blob) != rec_len:
                    n_bad += 1
                    continue
                magic, seq = struct.unpack_from("<IQ", blob, 0)
                crc = struct.unpack_from("<H", blob, REC_HDR - 2)[0]
                if magic != MAGIC_FRAME or (zlib.crc32(bytes(blob[:REC_HDR - 2])) & 0xFFFF) != crc:
                    n_bad += 1
                    continue
                seq_first = seq if seq_first is None else seq_first
                seq_last = seq
                n_ok += 1
            if out:
                out.write(blob)
    if out:
        out.close()
        os.replace(dst + ".part", dst)
    gaps = (seq_last - seq_first + 1 - n_ok) if n_ok else 0
    print("%s: %d records ok, %d bad, seq %s..%s (%d missing = recorder drops)%s"
          % (os.path.basename(src), n_ok, n_bad, seq_first, seq_last, gaps,
             "" if check_only else "  -> " + dst))
    return n_bad == 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("-o", "--out", help="output directory (default: beside the input)")
    ap.add_argument("--check", action="store_true", help="verify only, write nothing")
    a = ap.parse_args()
    ok = True
    for src in a.files:
        if not src.endswith(".zst"):
            print("%s: skipped (not .zst)" % src)
            continue
        name = os.path.basename(src)[:-4]
        dst = os.path.join(a.out or os.path.dirname(os.path.abspath(src)), name)
        if not a.check and os.path.exists(dst):
            print("%s: %s exists, skipped" % (src, dst))
            continue
        ok &= convert(src, dst, a.check)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
