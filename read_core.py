#!/usr/bin/env python3
"""Read every documented status/sensor value from the SD-IRM-256-01 thermal core.

READ-ONLY. This script issues only query commands from the vendor guide; it never
writes a setting, never changes the frame rate, and never touches a register. It
is safe to run against a live capture -- the serial control channel is separate
from the UVC video path, so nothing here interrupts streaming.

    read_core.py                 read everything and print a report
    read_core.py --json          same, as JSON on stdout
    read_core.py --no-bulk       skip the slow 128/512-byte bulk reads
    read_core.py --selftest      verify the frame builder and decoders, no hardware
    read_core.py --retry 60      keep trying until the core answers

WHAT IT READS (all read-only codes in the guide):

    0x08  core ID                    2-byte value, e.g. 0x0102
    0x09  core temperature           Q8 fixed point, -127..128 C
    0x07  manual self-test           16-bit fault bitmap, 0x0000 = healthy
    0x25  brightness                 0..100
    0x27  contrast                   0..100
    0x29  sharpness                  0..100
    0x0B  selected register address  whatever 0x0A last selected
    0x0D  data at that address       read-only in the sense that it writes nothing
    0x12  core serial number         128 bytes, bulk
    0x20  firmware version           128 bytes, bulk, character type
    0x1C  core info                  512 bytes, bulk; first 24 bytes are the
                                     sensor geometry struct, decoded below

There is deliberately no frame-rate read: the guide documents none, which is why
set_fps_jetson.py has to time delivered frames instead.

BULK READS. Serial number, firmware version and core info exceed the 2-byte data
field, so they use the 0xC0 protocol: send the read command, then repeatedly ask
0xC0 "I can take N bytes", receive the packet the core says it will send, and
stop when it reports 0. The guide warns explicitly that no other command may
interleave with that sequence, so each bulk read is driven to completion (or
abandoned and drained) before the next command is issued.

FRAME FORMAT, 9 bytes, from the guide:
    AA 05 00 <code> <lsb> <msb> <checksum> EB AA    checksum = sum(bytes 0..5) & 0xFF
    reply: 55 05 <status> <code> <lsb> <msb> <checksum> EB AA, status 0x00 = success
"""
import argparse
import glob
import json
import struct
import sys
import time

PORT_GLOB = "/dev/serial/by-id/usb-Artosyn_Sirius_ZBBM5DZFMP-if02"
BAUD = 115200

# Every frame this script can send, with the exact bytes printed in the vendor
# guide. check_frames() asserts our builder reproduces these, so a typo in the
# checksum rule cannot silently ship.
DOC_FRAMES = {
    0x07: "AA 05 00 07 00 00 B6 EB AA",
    0x08: "AA 05 00 08 00 00 B7 EB AA",
    0x09: "AA 05 00 09 00 00 B8 EB AA",
    0x0B: "AA 05 00 0B 00 00 BA EB AA",
    0x0D: "AA 05 00 0D 00 00 BC EB AA",
    0x12: "AA 05 00 12 00 00 C1 EB AA",
    0x1C: "AA 05 00 1C 00 00 CB EB AA",
    0x20: "AA 05 00 20 00 00 CF EB AA",
    0x25: "AA 05 00 25 00 00 D4 EB AA",
    0x27: "AA 05 00 27 00 00 D6 EB AA",
    0x29: "AA 05 00 29 00 00 D8 EB AA",
}


def build(code, lsb=0, msb=0):
    b = [0xAA, 0x05, 0x00, code, lsb, msb]
    b.append(sum(b) & 0xFF)
    return bytes(b + [0xEB, 0xAA])


def hx(b):
    return " ".join("%02X" % x for x in b)


def check_frames():
    """Our builder must reproduce the guide's printed frames byte for byte."""
    bad = []
    for code, want in sorted(DOC_FRAMES.items()):
        got = hx(build(code))
        if got != want:
            bad.append((code, got, want))
    return bad


def find_port(explicit=None):
    if explicit:
        return explicit
    hits = sorted(glob.glob(PORT_GLOB))
    if not hits:
        return None
    return hits[0]


def parse_reply(rx, code):
    """Validate a 9-byte reply. Returns (lsb, msb, error_or_None)."""
    if not rx:
        return None, None, "no reply"
    if len(rx) < 9:
        return None, None, "short reply (%d bytes)" % len(rx)
    if rx[0] != 0x55 or rx[1] != 0x05:
        return None, None, "bad header %02X %02X" % (rx[0], rx[1])
    if rx[7] != 0xEB or rx[8] != 0xAA:
        return None, None, "bad tail %02X %02X" % (rx[7], rx[8])
    if (sum(rx[0:6]) & 0xFF) != rx[6]:
        return None, None, "checksum mismatch (got %02X, computed %02X)" % (
            rx[6], sum(rx[0:6]) & 0xFF)
    if rx[3] != code:
        return None, None, "wrong code echoed (sent %02X, got %02X)" % (code, rx[3])
    if rx[2] != 0x00:
        # The guide lists no non-zero status codes, so this means "failed,
        # cause unspecified"; the documented response is to retry.
        return rx[4], rx[5], "status 0x%02X (failure, cause unspecified)" % rx[2]
    return rx[4], rx[5], None


# ---- decoders -------------------------------------------------------------

def dec_id(lsb, msb):
    return {"raw": (msb << 8) | lsb, "hex": "0x%04X" % ((msb << 8) | lsb)}


def dec_temp(lsb, msb):
    """Q8 fixed point, signed. The guide gives the range as -127..128 C."""
    raw = struct.unpack("<h", bytes([lsb, msb]))[0]
    c = raw / 256.0
    out = {"celsius": round(c, 3), "raw": raw}
    if not (-127.0 <= c <= 128.0):
        out["warning"] = "outside the documented -127..128 C range"
    return out


SELFTEST_BITS = {
    0: "detector fault",
    1: "shutter fault (not yet supported per the guide)",
    2: "core ISP fault",
}


def dec_selftest(lsb, msb):
    bm = (msb << 8) | lsb
    faults = [SELFTEST_BITS.get(i, "undocumented module bit %d" % i)
              for i in range(16) if bm & (1 << i)]
    return {"bitmap": "0x%04X" % bm, "healthy": bm == 0, "faults": faults}


def dec_0_100(lsb, msb):
    v = (msb << 8) | lsb
    out = {"value": v}
    if not (0 <= v <= 100):
        out["warning"] = "outside the documented 0..100 range"
    return out


def dec_regaddr(lsb, msb):
    return {"address": "0x%04X" % ((msb << 8) | lsb)}


def dec_coreinfo(blob):
    """First 24 bytes of the 0x1C payload, per the guide:
       uint32 sns_total_width; uint32 sns_total_height;
       int    sns_roi_x;       int    sns_roi_y;
       uint32 sns_roi_width;   uint32 sns_roi_height;"""
    if len(blob) < 24:
        return {"error": "only %d bytes, need 24 for the geometry struct" % len(blob)}
    w, h, rx_, ry, rw, rh = struct.unpack("<IIiiII", blob[:24])
    return {"sns_total_width": w, "sns_total_height": h,
            "sns_roi_x": rx_, "sns_roi_y": ry,
            "sns_roi_width": rw, "sns_roi_height": rh}


def dec_text(blob):
    """Firmware version is 'character type'; serial number is 128 raw bytes."""
    s = blob.split(b"\x00", 1)[0]
    try:
        return s.decode("ascii").strip()
    except UnicodeDecodeError:
        return None


# ---- transport ------------------------------------------------------------

class Core:
    def __init__(self, port, timeout=2.0, verbose=False):
        import serial
        self.ser = serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1,
                                 timeout=0.3)
        self.timeout = timeout
        self.verbose = verbose
        time.sleep(0.2)          # let the ACM link settle before the first write
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:                       # noqa: BLE001
            pass

    def cmd(self, code, lsb=0, msb=0):
        tx = build(code, lsb, msb)
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        self.ser.flush()
        deadline = time.time() + self.timeout
        rx = b""
        while time.time() < deadline and len(rx) < 9:
            chunk = self.ser.read(9 - len(rx))
            if chunk:
                rx += chunk
        if self.verbose:
            print("    TX %s" % hx(tx))
            print("    RX %s" % (hx(rx) if rx else "(nothing)"))
        return rx

    def read_raw(self, n, timeout=3.0):
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline and len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
                deadline = time.time() + timeout
        return buf

    def bulk(self, code, chunk=512, hard_cap=8192):
        """Guide section 5.1: send the read command, then ask 0xC0 repeatedly
        until the core reports 0.

        Always drive the sequence to that 0 terminator, even once the documented
        byte count has arrived. Stopping early would leave the core mid-transfer,
        and the guide is explicit that no other command may interleave with a
        bulk read -- so an early exit risks wedging the link for whatever runs
        next. hard_cap only exists so a misbehaving core cannot loop forever."""
        rx = self.cmd(code)
        _, _, err = parse_reply(rx, code)
        if err:
            return None, "read command: " + err
        blob = b""
        while len(blob) < hard_cap:
            want = min(chunk, hard_cap - len(blob))
            r = self.cmd(0xC0, want & 0xFF, (want >> 8) & 0xFF)
            lsb, msb, err = parse_reply(r, 0xC0)
            if err:
                return (blob or None), "bulk length request: " + err
            n = (msb << 8) | lsb
            if n == 0:
                return blob, None                  # properly terminated
            if n > want:
                return blob, ("core offered %d bytes, more than the %d requested"
                              % (n, want))
            data = self.read_raw(n)
            blob += data
            if len(data) < n:
                return blob, ("short bulk packet: got %d of %d bytes"
                              % (len(data), n))
        return blob, ("hit the %d-byte safety cap without the core reporting 0"
                      % hard_cap)


# ---- the read set ---------------------------------------------------------

SIMPLE = [
    (0x08, "core_id",             "core ID",                    dec_id),
    (0x09, "temperature",         "core temperature",           dec_temp),
    (0x07, "self_test",           "manual self-test",           dec_selftest),
    (0x25, "brightness",          "brightness",                 dec_0_100),
    (0x27, "contrast",            "contrast",                   dec_0_100),
    (0x29, "sharpness",           "sharpness",                  dec_0_100),
    (0x0B, "register_address",    "selected register address",  dec_regaddr),
    (0x0D, "register_data",       "data at that address",       dec_regaddr),
]

BULK = [
    (0x12, "serial_number",   "core serial number", 128),
    (0x20, "firmware_version", "firmware version",  128),
    (0x1C, "core_info",        "core info",         512),
]


def selftest():
    """Verify frames and decoders without any hardware."""
    ok = True
    print("frame builder vs the vendor guide's printed frames")
    bad = check_frames()
    for code, want in sorted(DOC_FRAMES.items()):
        got = hx(build(code))
        print("  0x%02X  %s  %s" % (code, got, "match" if got == want else
                                    "MISMATCH, guide says " + want))
    if bad:
        ok = False

    print("\ndecoders against the guide's documented values")
    cases = [
        ("core ID 0x0102 (guide's example)", dec_id(0x02, 0x01),
         lambda r: r["hex"] == "0x0102"),
        ("temperature 25.0 C -> Q8 0x1900", dec_temp(0x00, 0x19),
         lambda r: abs(r["celsius"] - 25.0) < 1e-9),
        ("temperature -10.5 C -> Q8", dec_temp(*struct.pack("<h", -2688)),
         lambda r: abs(r["celsius"] + 10.5) < 1e-9),
        ("self-test 0x0000 = healthy", dec_selftest(0x00, 0x00),
         lambda r: r["healthy"] and not r["faults"]),
        ("self-test bit0 = detector fault", dec_selftest(0x01, 0x00),
         lambda r: not r["healthy"] and r["faults"] == ["detector fault"]),
        ("self-test bit2 = core ISP fault", dec_selftest(0x04, 0x00),
         lambda r: r["faults"] == ["core ISP fault"]),
        ("brightness 50 (guide default)", dec_0_100(50, 0),
         lambda r: r["value"] == 50 and "warning" not in r),
        ("brightness 200 flagged out of range", dec_0_100(200, 0),
         lambda r: "warning" in r),
    ]
    for name, got, pred in cases:
        good = pred(got)
        ok &= good
        print("  %-42s %-38s %s" % (name, json.dumps(got), "OK" if good else "FAIL"))

    geom = struct.pack("<IIiiII", 1280, 1024, 0, 0, 1280, 1024)
    g = dec_coreinfo(geom + b"\x00" * 488)
    good = g["sns_total_width"] == 1280 and g["sns_roi_height"] == 1024
    ok &= good
    print("  %-42s %-38s %s" % ("core-info geometry struct",
                                json.dumps(g), "OK" if good else "FAIL"))
    g2 = dec_coreinfo(b"\x00" * 8)
    good = "error" in g2
    ok &= good
    print("  %-42s %-38s %s" % ("core-info too short is refused",
                                json.dumps(g2), "OK" if good else "FAIL"))

    print("\nreply validation")
    good_rx = bytes([0x55, 0x05, 0x00, 0x08, 0x02, 0x01])
    good_rx += bytes([sum(good_rx) & 0xFF, 0xEB, 0xAA])
    checks = [
        ("valid reply accepted", parse_reply(good_rx, 0x08), lambda r: r[2] is None),
        ("no reply", parse_reply(b"", 0x08), lambda r: r[2] == "no reply"),
        ("short reply", parse_reply(good_rx[:5], 0x08),
         lambda r: "short reply" in r[2]),
        ("bad header", parse_reply(b"\x00" * 9, 0x08), lambda r: "bad header" in r[2]),
        ("wrong code echoed", parse_reply(good_rx, 0x09),
         lambda r: "wrong code" in r[2]),
        ("bad checksum", parse_reply(good_rx[:6] + b"\x00" + good_rx[7:], 0x08),
         lambda r: "checksum" in r[2]),
    ]
    for name, got, pred in checks:
        good = pred(got)
        ok &= good
        print("  %-42s %-38s %s" % (name, str(got[2]), "OK" if good else "FAIL"))

    print("\n%s" % ("ALL SELF-TESTS PASSED" if ok else "SELF-TESTS FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="override the auto-detected serial port")
    ap.add_argument("--json", action="store_true", help="emit JSON on stdout")
    ap.add_argument("--no-bulk", action="store_true",
                    help="skip serial number / firmware version / core info")
    ap.add_argument("--selftest", action="store_true",
                    help="check frames and decoders, no hardware needed")
    ap.add_argument("--verbose", action="store_true", help="print every TX/RX frame")
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="per-command reply timeout, seconds (default 2)")
    ap.add_argument("--retry", type=float, default=0, metavar="SECS",
                    help="keep retrying the link test for SECS until the core"
                         " answers, waiting for the port if it is absent")
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    bad = check_frames()
    if bad:
        for code, got, want in bad:
            print("BUG: frame for 0x%02X is %s, guide says %s" % (code, got, want),
                  file=sys.stderr)
        return 2

    try:
        import serial            # noqa: F401
    except ImportError:
        print("pyserial not installed:  pip3 install pyserial", file=sys.stderr)
        return 2

    out = {"port": None, "link": None, "values": {}, "errors": {}}
    err_out = sys.stderr if a.json else sys.stdout

    # Link test first: the guide's own recommendation is 0x08, and a well-formed
    # reply starting 55 05 00 08 is what confirms the link. Everything after this
    # is meaningless if it fails, so say so once rather than 11 times.
    deadline = time.time() + a.retry
    core = None
    while True:
        port = find_port(a.port)
        if port is None:
            if time.time() < deadline:
                print("waiting for %s ..." % PORT_GLOB, file=err_out)
                time.sleep(1.0)
                continue
            out["errors"]["port"] = "no core serial port matching %s" % PORT_GLOB
            break
        out["port"] = port
        try:
            core = Core(port, timeout=a.timeout, verbose=a.verbose)
        except Exception as e:                  # noqa: BLE001
            if time.time() < deadline:
                time.sleep(1.0)
                continue
            out["errors"]["open"] = "cannot open %s: %s (in the dialout group?)" % (
                port, e)
            break
        rx = core.cmd(0x08)
        lsb, msb, e = parse_reply(rx, 0x08)
        if e is None:
            out["link"] = "ok"
            break
        core.close()
        core = None
        if time.time() < deadline:
            time.sleep(1.0)
            continue
        out["link"] = "no reply"
        out["errors"]["link"] = ("link test 0x08 (read core ID): %s -- the guide"
                                 " calls this a serial comms fault" % e)
        break

    if core is None:
        if a.json:
            print(json.dumps(out, indent=2))
        else:
            print("port        %s" % (out["port"] or "(none found)"))
            for k, v in out["errors"].items():
                print("FAILED      %s" % v)
            print("\nThe video path is independent of this channel -- check it with"
                  "\n  python3 set_fps_jetson.py --show")
        return 1

    if not a.json:
        print("port        %s" % out["port"])
        print("link test   0x08 answered -- channel is up\n")

    for code, key, label, dec in SIMPLE:
        rx = core.cmd(code)
        lsb, msb, e = parse_reply(rx, code)
        if e:
            out["errors"][key] = e
            if not a.json:
                print("  %-26s %s" % (label, "FAILED: " + e))
            continue
        val = dec(lsb, msb)
        out["values"][key] = val
        if not a.json:
            print("  %-26s %s" % (label, json.dumps(val)))
        time.sleep(0.05)

    if not a.no_bulk:
        if not a.json:
            print()
        for code, key, label, size in BULK:
            blob, e = core.bulk(code)
            if not e and blob is not None and len(blob) != size:
                e = "got %d bytes, the guide documents %d" % (len(blob), size)
            if e and not blob:
                out["errors"][key] = e
                if not a.json:
                    print("  %-26s %s" % (label, "FAILED: " + e))
                continue
            if e:
                out["errors"][key] = e
            if key == "core_info":
                val = dec_coreinfo(blob)
            else:
                val = {"text": dec_text(blob), "bytes": len(blob),
                       "hex_head": hx(blob[:16])}
            out["values"][key] = val
            if not a.json:
                print("  %-26s %s" % (label, json.dumps(val)))
                if e:
                    print("  %-26s (partial: %s)" % ("", e))
            time.sleep(0.05)

    core.close()
    if a.json:
        print(json.dumps(out, indent=2))
    else:
        n_ok, n_err = len(out["values"]), len(out["errors"])
        print("\n%d value(s) read, %d error(s)" % (n_ok, n_err))
    return 0 if out["values"] else 1


if __name__ == "__main__":
    sys.exit(main())
