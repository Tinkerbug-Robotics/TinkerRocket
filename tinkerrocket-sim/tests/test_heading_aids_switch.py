"""#1281/#1282: the two GNSS-derived heading aids ship OFF, and a replay has to
be able to turn them back on to score them against flight data.

This is the in-tree record of what that switch is worth, so the cost of the
decision is a measured number in CI rather than a claim in an issue.
"""
import dataclasses

import pytest

import tinkerrocket_sim._ekf as _ekf
from tinkerrocket_sim.simulation import scenarios as S
from tinkerrocket_sim.simulation import metrics as M


def test_aids_are_off_by_default():
    ekf = _ekf.GpsInsEKF()
    assert ekf.gnss_heading_aids_enabled() is False


def test_the_switch_round_trips():
    ekf = _ekf.GpsInsEKF()
    ekf.set_gnss_heading_aids(True)
    assert ekf.gnss_heading_aids_enabled() is True
    ekf.set_gnss_heading_aids(False)
    assert ekf.gnss_heading_aids_enabled() is False


def _roll_metrics(fuse, seed=None):
    """Run the boost-roll-disturbance scenario with the aids forced on/off.

    `seed` overrides the scenario's own sensor_seed. It exists because the
    single-seed answer is not the answer — see
    test_the_aids_do_not_help_the_roll_null.
    """
    import tinkerrocket_sim.simulation.closed_loop_sim as C
    original = _ekf.GpsInsEKF

    class Patched(original):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.set_gnss_heading_aids(fuse)

    _ekf.GpsInsEKF = Patched
    try:
        cfg = S.roll_boost_disturbance_config()
        if seed is not None:
            cfg = dataclasses.replace(cfg, sensor_seed=seed)
        df = C.run_closed_loop(S.build_rollypolly_iii(), cfg).df
    finally:
        _ekf.GpsInsEKF = original
    apogee = int(df['altitude'].idxmax())
    df = df.iloc[:apogee + 1].reset_index(drop=True)
    coast = df[(df['time'] > 4.0) & (df['time'] < 7.0)]
    settle = M.settling_time(df['time'].to_numpy(), df['roll_rate_dps'].to_numpy(),
                             target=0.0, tol=5.0,
                             start_time=cfg.roll_kick_time_s + 0.05)
    return float(coast['roll_rate_dps'].abs().median()), settle


def test_the_aids_do_not_help_the_roll_null():
    """#1281: the sim could not sign these aids — until the roll loop stopped
    making its own coast error (2026-09-23, below).

    This test used to assert `on_med < off_med` and `on_med < 3.0`, from a
    single measurement on `sensor_seed=42`: 6.41 dps with the aids off, 1.34
    with them fused. That looked like a 5 dps effect. It is one draw.

    Swept across 17 seeds, the aids' effect on the coast roll null is:

        sim as it was (25 Hz, 0.5/1.0 m/s)   mean delta +0.02 +/- 1.94 dps
                                            (t = +0.03, aid helps on 9/17)
        sim as measured (18.18 Hz, 0.4/0.6) mean delta +0.27 +/- 1.84 dps
                                            (t = +0.60, aid helps on 8/17)

    Both are indistinguishable from zero against a seed-to-seed spread of
    ~1.9 dps, and seed 42 is a 1.1-sigma draw of the old distribution. So the
    sim never supported "switching the aids on tightens the roll null" — a
    coin flip did, and three thresholds were calibrated against it (the two
    PR #1308 relaxed, and the one this test used to carry).

    That is this issue's actual finding. It is NOT that the sim lacked GNSS
    velocity noise: the sim already carried 0.5 m/s of NE velocity noise, and
    differentiated at 25 Hz that is MORE acceleration noise than the real
    receiver produces at 18.18 Hz, not less (17.7 vs 9.7 m/s^2).

    That null held while the roll loop carried an error of its own. Until
    2026-09-23 the firmware's I term was Ki_eff * integral(e), and the V² gain
    schedule's rising Ki made it unwind a learned trim all through coast: ~6 dps
    of coast median in this scenario whether the aids were on or off, which is
    what buried them. With the I term held in fin degrees (TR_PID) the coast
    median drops to ~2.5 dps, and the aids' effect surfaces — adverse:

        held I term, seeds 1-32    mean delta +0.88 +/- 1.60 dps
                                  (t = +3.1, aid helps on 11/32)
        seeds 1-8 (this test)      +1.61 +/- 1.93   (a high draw)
        scaled I term, seeds 1-32  -0.04 +/- 2.23   (t = -0.1)

    Fusing the aids makes the sim's roll null slightly WORSE. That agrees with
    shipping them off and changes nothing in #1309, which scores them against a
    flight with a roll reference, because the sim still has no GNSS velocity
    noise model to trust here. What this test pins is the direction: the sim
    must not start claiming the aids help, and the effect stays small next to
    the seed spread.
    """
    deltas = []
    for seed in range(1, 9):
        off_med, _ = _roll_metrics(False, seed)
        on_med, _ = _roll_metrics(True, seed)
        deltas.append(on_med - off_med)
    mean = sum(deltas) / len(deltas)
    spread = (sum((d - mean) ** 2 for d in deltas) / (len(deltas) - 1)) ** 0.5

    # Measured +1.61 on these seeds (+0.88 over 32); the spread is ~1.9.
    assert mean > -0.5, (
        f"the aids now HELP the sim's roll null (mean {mean:+.2f} dps over "
        f"{len(deltas)} seeds, spread {spread:.2f}) — that reverses the "
        f"2026-09-23 measurement and bears on #1309; measure it on 32 seeds")
    assert mean < 3.0, (
        f"the aids' cost to the roll null grew to {mean:+.2f} dps over "
        f"{len(deltas)} seeds (spread {spread:.2f}) — measured +1.61 here and "
        f"+0.88 over 32 seeds; find out what changed before relaxing this")
    # And it is genuinely a spread, not a constant: if every seed agreed, the
    # effect would be systematic and the assertion above would be the wrong test.
    assert spread > 0.3, (
        f"the per-seed spread collapsed to {spread:.2f} dps — the aids' effect "
        f"became deterministic, which is not what any measurement here found")


def test_the_default_path_matches_the_explicitly_disabled_path():
    # Nothing else may be quietly enabling them.
    from tinkerrocket_sim.simulation import scenarios as _S
    import tinkerrocket_sim.simulation.closed_loop_sim as C
    cfg = _S.roll_boost_disturbance_config()
    df = C.run_closed_loop(_S.build_rollypolly_iii(), cfg).df
    apogee = int(df['altitude'].idxmax())
    coast = df.iloc[:apogee + 1]
    coast = coast[(coast['time'] > 4.0) & (coast['time'] < 7.0)]
    default_med = float(coast['roll_rate_dps'].abs().median())
    off_med, _ = _roll_metrics(False)
    assert default_med == pytest.approx(off_med, rel=1e-6)
