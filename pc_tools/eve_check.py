#!/usr/bin/env python3
"""Check an EVE.edf annotation file (checklist #59) the way OSCAR will read it.

Verifies the header against the AirSense 11 layout (EDF+D, "EDF Annotations"
31 samples + "Crc16" 1 sample, 64-byte records, record count matches the
size), the CRC-16/CCITT-FALSE of every record, and lists the events with
their end time (onset), duration and start time.

usage: python eve_check.py <..._EVE.edf> [<another> ...]
"""
import datetime as dt
import sys

KNOWN = ("obstructive apnea", "central apnea", "hypopnea", "apnea", "arousal",
         "recording starts")


def crc16_ccitt_false(buf):
    crc = 0xFFFF
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def check(path):
    b = open(path, "rb").read()
    ok = True

    def bad(msg):
        nonlocal ok
        ok = False
        print("  FAIL", msg)

    h = b[:256]
    ns = int(h[252:256])
    hdr_bytes = int(h[184:192])
    nrec = int(h[236:244])
    start = dt.datetime.strptime((h[168:176] + h[176:184]).decode(), "%d.%m.%y%H.%M.%S")
    print("%s\n  start %s  reserved %r  nrec %d  size %d"
          % (path, start, h[192:236].strip().decode(), nrec, len(b)))
    if h[192:197] != b"EDF+D":
        bad("reserved field is not EDF+D")
    if ns != 2 or hdr_bytes != 768:
        bad("expected 2 signals / 768 header bytes, got %d / %d" % (ns, hdr_bytes))
    sh = b[256:256 + 256 * ns]
    labels = [sh[i * 16:(i + 1) * 16].strip() for i in range(ns)]
    nsamp = [int(sh[(16 + 80 + 8 + 8 + 8 + 8 + 8 + 80) * ns + i * 8:
                    (16 + 80 + 8 + 8 + 8 + 8 + 8 + 80) * ns + (i + 1) * 8]) for i in range(ns)]
    if labels != [b"EDF Annotations", b"Crc16"] or nsamp != [31, 1]:
        bad("signals %r samples %r" % (labels, nsamp))
    data = b[hdr_bytes:]
    if len(data) != nrec * 64:
        bad("data %d bytes but nrec %d x 64 = %d" % (len(data), nrec, nrec * 64))
    for k in range(len(data) // 64):
        rec = data[k * 64:(k + 1) * 64]
        crc = crc16_ccitt_false(rec[:62])
        if crc.to_bytes(2, "little") != rec[62:]:
            bad("record %d CRC %04x != %s" % (k, crc, rec[62:].hex()))
        tals = [t for t in rec[:62].split(b"\x00") if t]
        if not tals or tals[0] != b"+0\x14\x14":
            bad("record %d has no time-keeping TAL" % k)
        for t in tals[1:]:
            head, _, rest = t.partition(b"\x14")
            onset, _, dur = head.partition(b"\x15")
            text = rest.rstrip(b"\x14").decode("ascii", "replace")
            on = float(onset)
            d = float(dur or 0)
            known = any(text.lower().startswith(x) for x in KNOWN)
            print("  %3d  end +%6.0f s (%s)  dur %4.0f s  start %s  %-18s%s"
                  % (k, on, (start + dt.timedelta(seconds=on)).strftime("%H:%M:%S"), d,
                     (start + dt.timedelta(seconds=on - d)).strftime("%H:%M:%S"), text,
                     "" if known else "  <- OSCAR ignores this text"))
    print("  %s" % ("OK" if ok else "PROBLEMS FOUND"))
    return ok


if __name__ == "__main__":
    res = [check(p) for p in sys.argv[1:]]
    sys.exit(0 if res and all(res) else 1)
