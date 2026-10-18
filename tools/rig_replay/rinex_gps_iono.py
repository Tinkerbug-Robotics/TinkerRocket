#!/usr/bin/env python3
"""The GPS Klobuchar of a RINEX navigation file's header, as the eight integers GPS broadcasts (subframe 4 page 18):
what a V9 mosaic rig replay build carries as its sky's ionosphere.

    rinex_gps_iono.py RINEX_NAV          -> a0,a1,a2,a3,b0,b1,b2,b3   (for -DTR_SIM_REPLAY_GPS_IONO=...)

SignalSim delays every GPS L1 signal by the Klobuchar model with the header's GPSA/GPSB (its Rinex.cpp; the
scenario's SignalSim config names the file: ephemeris.name), and puts the same values in page 18. The rig's files are
too short to broadcast page 18 (once every 12.5 min; the three flights' files hold pages 21-25 and 1-4), so the
replay build hands these to the filter as the receiver's GPSIon (TR_Sensor_Collector_Sim's getGNSSBroadcastIono):
the filter then takes off exactly the delay the sky carries. The broadcast scales are 2^-30, 2^-27, 2^-24, 2^-24
(alpha, s/semicircle^n) and 2^11, 2^14, 2^16, 2^16 (beta, s/semicircle^n); a header value that is not a whole
number of them (within 1e-3 of one, the header's 5 digits) or not a byte is refused.
"""
import re
import sys

SCALES = (2.0 ** -30, 2.0 ** -27, 2.0 ** -24, 2.0 ** -24, 2.0 ** 11, 2.0 ** 14, 2.0 ** 16, 2.0 ** 16)


def header_klobuchar(text):
    """(alpha0..3, beta0..3) from a RINEX 2 (ION ALPHA / ION BETA) or 3/4 (GPSA / GPSB IONOSPHERIC CORR) header."""
    alpha = beta = None
    for ln in text.splitlines():
        if "END OF HEADER" in ln:
            break
        label = ln[60:].strip()
        body = ln[:60].replace("D", "E").replace("d", "e")
        if label.startswith("IONOSPHERIC CORR") and ln.startswith("GPSA"):
            alpha = [float(x) for x in re.findall(r"[-+]?\d*\.\d+[eE][-+]?\d+", body[4:])][:4]
        elif label.startswith("IONOSPHERIC CORR") and ln.startswith("GPSB"):
            beta = [float(x) for x in re.findall(r"[-+]?\d*\.\d+[eE][-+]?\d+", body[4:])][:4]
        elif label.startswith("ION ALPHA"):
            alpha = [float(x) for x in re.findall(r"[-+]?\d*\.\d+[eE][-+]?\d+", body)][:4]
        elif label.startswith("ION BETA"):
            beta = [float(x) for x in re.findall(r"[-+]?\d*\.\d+[eE][-+]?\d+", body)][:4]
    if not alpha or not beta or len(alpha) != 4 or len(beta) != 4:
        raise ValueError("no GPS Klobuchar (GPSA/GPSB or ION ALPHA/BETA) in the header")
    return alpha + beta


def broadcast_bytes(values):
    out = []
    for v, s in zip(values, SCALES):
        n = v / s
        r = round(n)
        if abs(n - r) > 1e-3 * max(1.0, abs(r)) or not -128 <= r <= 127:
            raise ValueError(f"{v:g} is not a broadcast byte at scale {s:g} ({n:.4f})")
        out.append(int(r))
    return out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__.split("\n\n")[1], file=sys.stderr)
        return 2
    try:
        print(",".join(str(b) for b in broadcast_bytes(header_klobuchar(open(argv[0], errors="replace").read()))))
    except (OSError, ValueError) as e:
        print(f"rinex_gps_iono: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
