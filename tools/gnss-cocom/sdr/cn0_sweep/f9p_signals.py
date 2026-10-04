#!/usr/bin/env python3
"""ZED-F9P signal set for the wide SignalSim files: GPS (L1 C/A) + Galileo + BeiDou on, GLONASS, QZSS and SBAS off.
The files carry no GLONASS, and searching for constellations that are not there is the load that cut an F9P's RAWX
epochs before (ubx_config.py's --gps-only note). RAM layer only, so a power cycle restores the receiver's own setup;
one VALSET, so the receiver restarts its tracking once. Uses ubx_config.py's keys and framing.
    f9p_signals.py PORT"""
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDR))
import serial                                                            # noqa: E402
from ubx_config import KEYS, LAYER_RAM, collect, read_for, valset        # noqa: E402

port = sys.argv[1]
items = [(KEYS["CFG-SIGNAL-GPS_ENA"], 1), (KEYS["CFG-SIGNAL-GPS_L1CA_ENA"], 1), (KEYS["CFG-SIGNAL-GAL_ENA"], 1),
         (KEYS["CFG-SIGNAL-BDS_ENA"], 1), (KEYS["CFG-SIGNAL-GLO_ENA"], 0), (KEYS["CFG-SIGNAL-QZSS_ENA"], 0),
         (KEYS["CFG-SIGNAL-SBAS_ENA"], 0)]
with serial.Serial(port, 460800, timeout=0.2) as ser:
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(valset(items, LAYER_RAM))
    acks = [m for c, m, _ in collect(read_for(ser, 2.0)) if c == 0x05]
state = "ACK" if 0x01 in acks else ("NAK" if 0x00 in acks else "no answer")
print(f"  signals  : GPS + Galileo + BeiDou on; GLONASS, QZSS, SBAS off -- {state}")
sys.exit(0 if 0x01 in acks else 1)
