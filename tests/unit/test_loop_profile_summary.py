"""tools/loop_profile_summary.py against the report the firmware prints.

The report's layout lives in printf calls in the flight computer's
main/loop_profile.h, and the summary parses it with regular expressions; nothing
else ties the two together. So these tests do not carry a copy of the format:
they pull the printf format strings out of loop_profile.h, render a report with
them (C's %u and %%, which Python's % operator also takes), and parse that.
A change to either side that the other does not follow fails here.
"""
import importlib.util
import math
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "tinkerrocket-idf" / "projects" / "flight_computer" / "main" / "loop_profile.h"
_spec = importlib.util.spec_from_file_location(
    "loop_profile_summary", REPO / "tools" / "loop_profile_summary.py")
lps = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lps)


def printf_formats():
    """Every printf format string in loop_profile.h, adjacent literals joined."""
    src = HEADER.read_text()
    out = []
    for m in re.finditer(r'printf\(\s*((?:"(?:[^"\\]|\\.)*"\s*)+)', src):
        lits = re.findall(r'"((?:[^"\\]|\\.)*)"', m.group(1))
        out.append(''.join(lits).replace('\\n', '\n'))
    return out


def fmt(marker):
    hits = [f for f in printf_formats() if marker in f]
    assert len(hits) == 1, f"expected one printf containing {marker!r}, found {len(hits)}"
    return hits[0]


WINDOW = fmt('[LP] window')
TITLES = fmt('[LP]   %-15s %8s')
ROW = fmt('[LP]   %-15s %6u')
TOTAL = fmt('[LP] total cfg')
CORE0 = fmt('[LP] core0 idle')


def tenths(x):
    """A value the way the firmware prints it: integer part, then one decimal."""
    v = int(round(x * 10))
    return v // 10, v % 10


def render_window(cfg, passes_s, rows, imu=7310, prefix=''):
    """One report window. rows: (name, hits/s, us/pass, us/hit, max, core%, p0%, p1%)."""
    lines = [WINDOW % ((5, 0, cfg, 'label', int(passes_s * 5)) + tenths(passes_s)
                       + tenths(1e6 / passes_s) + (2000, int(passes_s), int(passes_s / 2), imu)
                       + tenths(imu / passes_s) + tenths(100.2))]
    lines.append(TITLES % ('section', 'hits/s', 'us/pass', 'us/hit', 'max_us', 'core%',
                           'pm0%', 'pm1%'))
    for name, hz, upp, uph, mx, core, p0, p1 in rows:
        lines.append(ROW % ((name,) + tenths(hz) + tenths(upp) + tenths(uph) + (mx,)
                            + tenths(core) + tenths(p0) + tenths(p1)))
    lines.append(TOTAL % ((cfg,) + tenths(50.0 + cfg) + tenths(10.0 + cfg) + (1200,)))
    lines.append(CORE0 % (tenths(70.0) + (840, 1200)))
    text = ''.join(lines)
    return [prefix + l for l in text.splitlines()]


ROWS = {
    # name: (hits/s, us/pass, us/hit, max_us, core%)
    'yield+wdt': (1000.0, 5.0, 5.0, 20, 0.5),
    'gate': (1000.0, 1.0, 1.0, 5, 0.1),
    'imu_mean': (1000.0, 20.0, 20.0, 60, 2.0),
    'ekf.nav': (500.0, 18.0, 36.0, 70, 1.8),
    'ekf.cov': (500.0, 57.5, 115.0, 130, 5.75),
    'telemetry': (1000.0, 13.5, 13.5, 30, 1.35),
}


def window_rows(shares):
    return [(n,) + v + shares[n] for n, v in ROWS.items()]


def capture(prefix=''):
    lines = ['I (100) FC: boot noise that is not a report']
    # A first window the summary must skip by default: absurd values.
    lines += render_window(0, 10.0, [('yield+wdt', 10.0, 999.0, 999.0, 999, 99.0, 1.0, 1.0)],
                           prefix=prefix)
    for cfg, s0, s1 in ((0, 80.0, 10.0), (1, 30.0, 4.0), (2, 70.0, 1.0)):
        shares = {n: (s0, s1) for n in ROWS}
        shares['ekf.cov'] = (s0 / 2, s1 / 2)   # one section with its own shares
        lines += render_window(cfg, 1000.0, window_rows(shares), prefix=prefix)
    return lines


def test_formats_are_all_found():
    for f in (WINDOW, TITLES, ROW, TOTAL, CORE0):
        assert f.startswith('[LP]') and f.endswith('\n')


@pytest.mark.parametrize('prefix', ['', '12.345 ', 'I (4321) '])
def test_round_trip_through_the_firmware_formats(prefix):
    windows = lps.parse(capture(prefix))
    assert [w['cfg'] for w in windows] == [0, 1, 2]       # the boot window was skipped
    assert all(w['core0_idle'] == 70.0 for w in windows)
    s = lps.merge(windows)
    assert s['windows'] == 3
    assert s['order'] == list(ROWS)
    assert s['passes_per_s'] == pytest.approx(1000.0)
    assert s['coverage'] == pytest.approx(100.2)
    cov = s['sections']['ekf.cov']
    assert cov['uph'] == pytest.approx(115.0) and cov['hz'] == pytest.approx(500.0)
    assert cov['Istall%'] == pytest.approx(40.0) and cov['Dstall%'] == pytest.approx(5.0)
    assert cov['IPCx100'] == pytest.approx(15.0) and cov['waiti%'] == pytest.approx(0.5)
    assert s['sections']['gate']['Ibusy%'] == pytest.approx(70.0)
    assert s['totals'][1] == pytest.approx((51.0, 11.0))


def test_skip_keeps_the_first_window_when_asked():
    assert [w['cfg'] for w in lps.parse(capture(), skip=0)] == [0, 0, 1, 2]


def test_per_gated_pass_groups_and_ekf_tick():
    groups, gated, ekf_hz, tick = lps.per_gated_pass(lps.merge(lps.parse(capture())))
    g = dict(groups)
    assert gated == pytest.approx(1000.0) and ekf_hz == pytest.approx(500.0)
    assert tick == pytest.approx(36.0 + 115.0)            # the EKF sections, per tick
    assert g['EKF'] == pytest.approx((36.0 + 115.0) / 2)  # every other gated pass
    assert g['imu, baro, orientation'] == pytest.approx(1.0 + 20.0)
    assert g['telemetry'] == pytest.approx(13.5)
    assert math.isclose(sum(g.values()), 5.0 + 21.0 + 75.5 + 13.5, rel_tol=1e-6)


def test_compare_and_csv(tmp_path, capsys):
    log = tmp_path / 'a.log'
    log.write_text('\n'.join(capture('1.000 ')) + '\n')
    out_csv = tmp_path / 'a.csv'
    lps.main([str(log), str(log), '--csv', str(out_csv)])
    text = capsys.readouterr().out
    assert 'EKF us per tick' in text and '151.0' in text
    rows = out_csv.read_text().splitlines()
    assert rows[0].startswith('section,hits_per_s,us_per_pass')
    assert rows[1].startswith('yield+wdt,1000.0,5.0,5.0,20,0.5')


def test_a_capture_without_reports_is_an_error(tmp_path):
    log = tmp_path / 'none.log'
    log.write_text('I (1) FC: nothing to see\n')
    with pytest.raises(SystemExit):
        lps.main([str(log)])
