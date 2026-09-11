#!/usr/bin/env python3
"""Reset the MCU on a USB-Serial/JTAG port and capture its boot log.

The bench harness deliberately attaches WITHOUT resetting the board, so boot
lines are never on its timeline.  This does the opposite on purpose: open with
DTR/RTS low (no accidental pulse), then assert RTS -> EN low -> release, which
is a plain hard reset (esptool's hard_reset), and record for N seconds.
"""
import serial, sys, time
port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
s = serial.Serial()
s.port, s.baudrate, s.timeout = port, 115200, 0.3
s.dtr = False; s.rts = False
s.open()
s.reset_input_buffer()
s.rts = True; time.sleep(0.2); s.rts = False     # hard reset
t0 = time.time(); n = 0
with open(out, "wb") as f:
    while time.time() - t0 < secs:
        d = s.read(4096)
        if d:
            f.write(d); f.flush(); n += len(d)
s.close()
print(f"captured {n} bytes in {secs:.0f}s -> {out}")
