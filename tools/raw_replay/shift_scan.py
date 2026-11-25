"""The epoch time tags moved -100..+100 ms (a receiver output delay the rig cannot see): the heading's change across
the boost epoch, and the coast and descent attitude, for a log's recorded configuration."""
import subprocess
import sys
import numpy as np
sys.path.insert(0, ".")
import scorelib as S
tr = S.Truth()
run, flags = sys.argv[1], sys.argv[2].split()
base = float(S.read_meta(f"in/{run}.txt")["t_base_s"])
print(f"{run}: {' '.join(flags)}")
print(" shift ms | heading err T+0.9 -> T+1.3 | coast nose med/max | coast roll med | descent roll med | "
      "boost total med/max | pitch vote")
for sh in range(-100, 101, 20):
    out = f"diag/{run}_shift{sh:+d}.csv"
    subprocess.run(["./build/raw_replay", "--in", f"in/{run}.txt", "--out", out, "--t-base-s", repr(base),
                    "--every", "1", "--epoch-shift-ms", str(sh)] + flags, check=True, capture_output=True)
    ser = S.harness_series(out, base)
    e = S.sample_errors(ser, tr)
    def at(t):
        k = int(np.argmin(np.abs(ser["t"] - t)))
        _, _, qt = tr.at(ser["t"][k:k + 1])
        return S.att_errors(ser["q"][k:k + 1], qt)[3][0]
    f = lambda m, p: np.abs(e[m][e["phase"] == p])
    sc = S.score(ser, tr)
    print(f" {sh:+5d}    | {at(0.9):+7.2f} -> {at(1.3):+7.2f}         | {np.median(f('nose','coast')):5.2f} / "
          f"{f('nose','coast').max():5.2f}      | {np.median(f('roll','coast')):6.2f}         | "
          f"{np.median(f('roll','descent')):6.2f}           | {np.median(f('att_total','boost')):5.2f} / "
          f"{f('att_total','boost').max():5.2f}       | {sc['pitch_vote_t']:.3f}")
