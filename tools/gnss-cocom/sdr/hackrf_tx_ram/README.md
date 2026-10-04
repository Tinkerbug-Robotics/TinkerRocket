# hackrf_tx_ram: the buffered transmitter

A drop-in for `hackrf_transfer -t` that never reads the disk inside the USB loop, built for the wide SignalSim files
(18.48 Msps, 37 MB/s).

`hackrf_transfer` fills each USB transfer with an `fread()` of the file from inside libhackrf's transfer callback. At
18.48 Msps the HackRF's own buffer holds about 0.6 ms, so any read that waits is an underrun, and every underrun
shifts the replayed signal in time; on the rig one cost the PX1105R 10-90 s of tracking. Here a reader thread keeps
up to 2 GiB of the file ahead of playback in a locked ring, reading straight from the SSD with the page cache left
alone, and the callback only copies from memory. The process takes the user-interactive QoS class before libhackrf
starts its transfer thread, so that thread inherits it.

Measured on 2026-09-30 with the same file and the receiver on a charger: `hackrf_transfer` had 14 underruns,
`hackrf_tx_ram` 3 plus the end-of-file drain, with no host shortfalls and every callback under 1 ms. The rest are
stalls on the Mac's USB path. The buffered flight that evening (the wide traveler, receiver on the Mac) had none.

## Build

    ./build.sh          # clang + libhackrf (brew install hackrf); the binary is ignored by git

## Options

The TX subset of `hackrf_transfer` with the same one-line-per-second `-B` statistics, so a runner that starts
`hackrf_transfer` can start this instead: `-t FILE -f FREQ_HZ -s RATE_HZ -a AMP -x TXVGA_DB [-b BBF_HZ] [-B]`.

| Extra | What it does |
|---|---|
| `-D` | dry run: no radio, a consumer paced at the sample rate (checks the ring keeps up) |
| `-X` | dump: the bytes the callback would send, to stdout, unpaced (checks the content) |
| `-N DB` | lowers every signal's C/N0 by DB: white Gaussian noise added as the file is read, the sum scaled back to the file's own RMS, so level and noise density stay put |
| `-C HZ` | shifts every carrier by HZ with the code untouched (the rig's -23 Hz correction on the uncorrected wide files), phase continuous, the same operation as `../signalsim/carrier_shift.py`; applied before `-N` |
| `-K` | keeps the reader thread waking every 2 ms after it stops reading (an underrun test) |
| `-L SECONDS` | loop test: fills the ring once, then transmits it over and over with no disk reads |

Environment: `RING_MIB` (2048), `PREFILL_MIB` (512), `NOISE_SEED` (1). The header of `hackrf_tx_ram.c` has the
detail, including the noise formula.

## On the rig

- `../px_run_ram.py`: `px1105r_run.py` with this transmitter in place of `hackrf_transfer`. `TX_NOISE_DB` and
  `TX_CARRIER_HZ` in the environment pass `-N` and `-C`.
- `../px_fly_ram.sh`: SignalSim flights through it, one argument per flight.
- `../tx_only.py` with `TX_BIN` set to this binary: a transmission with no receiver on the Mac, for underrun tests.

## Checks

    ./check_offline.sh WIDE.C8 NARROW.C8      # no radio: dump = file, -N, -C against carrier_shift.py, paced dry runs

`noise_check.py ORIG.C8 DUMP.bin EXPECT_DB` is the `-N` measurement it uses.
