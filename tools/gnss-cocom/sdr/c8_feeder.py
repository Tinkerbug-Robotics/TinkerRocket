#!/usr/bin/env python3
"""Stream an IQ file to stdout from a large in-memory read-ahead buffer, for `hackrf_transfer -t -`.

hackrf_transfer reads its -t file itself, inside the loop that keeps the HackRF's few-millisecond buffer fed; one
slow disk read there is an underrun (7-13 ms each on this Mac, a 10-90 s outage for the PX1105R). Here a reader
thread pulls the file off the disk in 4 MiB blocks (F_NOCACHE: straight from the SSD, the page cache left alone)
up to AHEAD_GIB ahead, and the main thread only copies from memory into the pipe.

    c8_feeder.py FILE [AHEAD_GIB] | hackrf_transfer -t - ...
"""
import fcntl
import os
import queue
import sys
import threading
import time

path = sys.argv[1]
ahead = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
CHUNK = 4 * 2 ** 20
q = queue.Queue(maxsize=max(8, int(ahead * 2 ** 30) // CHUNK))


def reader():
    fd = os.open(path, os.O_RDONLY)
    try:
        fcntl.fcntl(fd, getattr(fcntl, "F_NOCACHE", 48), 1)
    except OSError:
        pass
    while True:
        b = os.read(fd, CHUNK)
        if not b:
            break
        q.put(b)
    q.put(None)
    os.close(fd)


threading.Thread(target=reader, daemon=True).start()
t0 = time.time()
while q.qsize() < 16 and time.time() - t0 < 5.0:     # 64 MiB primed before the first byte goes out
    time.sleep(0.01)
out = sys.stdout.buffer
try:
    while True:
        b = q.get()
        if b is None:
            break
        out.write(b)
    out.flush()
except BrokenPipeError:
    pass
