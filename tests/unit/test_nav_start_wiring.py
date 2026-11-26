"""#1756, #1762: main.cpp's wiring of the navigation filter's start (flight_computer/main/nav_start_policy.h).

The policy is host-tested in tests_cpp/test_nav_start_policy.cpp (with a model of the loop's pass order and the
review's three routes) and, for the gnss-sdr-dev board, tests_cpp/test_sdr_nav_start_policy.cpp. main.cpp is not
host-built, so these tests read the source:
- #1762: every receiver takes the rule (it used to be the board's alone; an answering receiver that never fixed left
  the filter unstarted for the flight), with GNSS-absent mode starting at once as before;
- a degraded start gives way to a usable fix anywhere on the pad (READY or PRELAUNCH, which the FC enters on its
  own: the follow-up), holds the receiver's fixes and raw epochs out of the filter while it stands, and its pad
  datum only once the rocket flies (the datum is what the restart needs);
- the restart waits until the rocket has been at rest for a second (RestWatch, fed every pass), says so while it
  waits, and the gyro window its bias seed comes from fills while the degraded start stands on the pad;
- every start says whether it is degraded, so a start on a fix after a degraded one's filter was cleared is not
  handed the time-0 fix (which a filter started on a fix fuses); the degraded seed sits at the pad datum when there
  is one, its declination only from the datum; a reboot in flight keeps a degraded start (the snapshot's
  ekf_initialized byte);
- none of that sits behind a receiver's preprocessor branch: TR_RAW_GNSS_SHADOW alone guards the raw epochs' hold;
- #1756: "no usable fix" is none for a while (receiverWait), stamped where the policy assumes.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read():
    with open(MAIN_CPP) as fh:
        return fh.read()


def code(text):
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def call_args(src, name):
    i = src.index(name + "(")
    j = i + len(name) + 1
    depth = 1
    while depth:
        c = src[j]
        depth += (c == "(") - (c == ")")
        j += 1
    return src[i + len(name) + 1:j - 1]


def pp_stack(src, pos):
    """The preprocessor conditions open at pos: each #if/#ifdef/#ifndef line, '!'-prefixed in its #else/#elif."""
    stack = []
    for line in src[:pos].splitlines():
        t = line.strip()
        if re.match(r"#\s*if", t):
            stack.append(t)
        elif re.match(r"#\s*(else|elif)", t):
            stack[-1] = "!" + stack[-1].lstrip("!")
        elif re.match(r"#\s*endif", t):
            stack.pop()
    return stack


def receiver_branches(src, pos):
    """The open conditions at pos that name one receiver's image (or the one image's choice)."""
    return [c for c in pp_stack(src, pos)
            if re.search(r"TR_RAW_NAV_(SDR|MOSAIC|PX1105R|AUTO)|TR_GNSS_(SDR|DRIVER)|RAW_IS_", c)]


def body(src, signature):
    i = src.index(signature)
    while True:
        eol = src.index("\n", i)
        if src.startswith("\n{\n", eol):
            j = eol + 3
            return src[j:src.index("\n}\n", j)]
        i = src.index(signature, eol)


def test_every_receiver_takes_the_start_rule():
    src = code(read())
    assert '#include "nav_start_policy.h"' in src
    # one degraded start, for every image and receiver: no #if around it, no per-receiver branch
    assert src.count("nav_start::degradedStartDue(") == 1
    args = call_args(src, "nav_start::degradedStartDue")
    flat = " ".join(args.split())
    assert flat.startswith("normal_init || nav_start::receiverWait(nav_last_usable_fix_ms, nav_reinit_ms, millis()),")
    assert flat.endswith("rocket_state, gnss_absent_mode"), "a receiver that failed bring-up must still start at once"
    assert "!gnss_started" not in src[src.index("const bool normal_init = have_ref_pos && gnss_gate3_init;"):
                                      src.index("nav_start::degradedStartDue(")]
    i = src.index("const bool degraded_init = nav_start::degradedStartDue(")
    block = src[src.rindex("else if (!ekf_initialized)", 0, i):i]
    assert "#if" not in block, "the start rule sits behind a preprocessor branch"


def test_a_late_usable_fix_restarts_a_degraded_start_on_the_pad():
    src = code(read())
    m = re.search(r"if \(ekf_initialized && gnss_absent_flight &&\s*"
                  r"nav_start::restartOnFix\(true, have_ref_pos && gnss_gate3_init, rocket_state, nav_rest\.atRest\(\)\)\)"
                  r"\s*\{"
                  r"\s*ekf_initialized = false;\s*gnss_absent_flight = false;", src)
    assert m, "the restart on a late fix is not every receiver's"
    assert m.start() < src.index("if (rocket_state == MAG_CALIBRATION)", m.start())
    assert not receiver_branches(src, m.start())
    # the rest it waits for: fed on every pass (nothing conditional between the watch and its feed), before the
    # restart, the filter's own input (the EKF frame's specific force and rotation rate)
    st = re.search(r"static nav_start::RestWatch nav_rest;\s*nav_rest\.feed\(millis\(\), ekf_imu\.acc_x, ekf_imu\.acc_y, "
                   r"ekf_imu\.acc_z,\s*ekf_imu\.gyro_x, ekf_imu\.gyro_y, ekf_imu\.gyro_z\);", src)
    assert st and st.start() < m.start()
    assert not receiver_branches(src, st.start())
    assert src.count("nav_rest.feed(") == 1
    # and the restart does not wait in silence
    wait = src[st.end():m.start()]
    assert "!nav_rest.atRest()" in wait and "the degraded start's restart waits for the" in wait
    # the gyro window the restart's bias seed comes from fills while a degraded start stands on the pad
    ring = src.index("initGyroRing[0][initGyroHead] = ekf_imu.gyro_x;")
    cond = src[src.rindex("if (", 0, ring):ring]
    assert "if (!ekf_initialized || (gnss_absent_flight && nav_start::onThePad(rocket_state)))" in cond
    assert ring < m.start()
    # the raw filter's new start seed goes out on the next tick
    restart = src[m.end():src.index("if (rocket_state == MAG_CALIBRATION)", m.start())]
    assert re.search(r"#if TR_RAW_GNSS_SHADOW\s*raw_shadow_seed_ms = 0;\s*#endif", restart)


def test_a_degraded_start_holds_gnss_out_of_the_filter():
    src = code(read())
    # the classic filter: a fix with time 0, skipped like the degraded seed's
    assert re.search(r"ekf\.update\(use_ahrs_acc, ekf_imu, gnss_absent_flight \? kNoGnssFix : ekf_gnss, ekf_mag\);", src)
    assert "static const EkfGNSSDataLLA kNoGnssFix{};" in src
    # the raw filter: every raw receiver's epochs held (not the board's alone)
    task = body(src, "static void rawGnssMeasTask(void*)")
    h = task.index("if (raw_gnss_held)")
    assert "RAW_IS_SDR" not in task[task.rindex("\n", 0, h):h]
    assert "raw_gnss_held_n++;" in task[h:h + 200] and "continue;" in task[h:h + 200]
    assert task.index("while (sensor_collector.getGNSSRawEpoch(epoch))") < h
    # held on every pass the degraded start stands, on every raw receiver's image
    for held in re.finditer(r"raw_gnss_held = (gnss_absent_flight|degraded_init);", src):
        assert pp_stack(src, held.start())[-1:] == ["#if TR_RAW_GNSS_SHADOW"], held.group(0)
        assert not receiver_branches(src, held.start()), held.group(0)
    assert src.count("raw_gnss_held = gnss_absent_flight;") == 2, "the loop's pass and the restore"
    assert src.count("raw_gnss_held = degraded_init;") == 1, "each start"
    fix = src.index("ekf.update(use_ahrs_acc, ekf_imu, gnss_absent_flight ? kNoGnssFix : ekf_gnss, ekf_mag);")
    assert not receiver_branches(src, fix), "the classic filter's hold sits behind a receiver's branch"
    # the pad datum and the pad average take no fix once a degraded start flies; on the pad they do
    assert "const bool degraded_flying = nav_start::fixesHeldOut(gnss_absent_flight, rocket_state);" in src
    assert "degraded_armed" not in src
    assert "const bool ref_take = (!ref_from_nav || ref_nav_now) && !degraded_flying;" in src
    assert "const bool ref_take = !degraded_flying;" in src
    assert "if (gnss_gate3_init && !degraded_flying &&" in src


def test_every_start_says_whether_it_is_degraded():
    src = code(read())
    i = src.index("const bool degraded_init = nav_start::degradedStartDue(")
    on = src.index("ekf_initialized = true;", i)
    end = src.index("TR_LP_LAP(EKF_INIT);", on)
    start = src[on:end]
    flag = re.search(r"nav_filter_on_fixes = !degraded_init;\s*gnss_absent_flight = degraded_init;", start)
    assert flag, "a start on a fix must clear gnss_absent_flight: its filter fuses the time-0 fix"
    assert flag.start() < start.index("if (degraded_init) {")
    assert not receiver_branches(src, on + flag.start())
    assert "gnss_absent_flight = true;" not in src, "set only by a start, from degraded_init"
    assert "raw_gnss_held = true;" not in src


def test_the_degraded_seed_sits_at_the_pad_datum():
    src = code(read())
    i = src.index("const bool degraded_init = nav_start::degradedStartDue(")
    seed = src[i:src.index("ekf.init(ekf_imu, init_gnss, ekf_mag);", i)]
    assert "init_gnss.time_us   = 0;" in seed
    assert "init_gnss.lat_rad   = have_ref_pos ? ref_lat_rad : 0.0;" in seed
    assert "init_gnss.lon_rad   = have_ref_pos ? ref_lon_rad : 0.0;" in seed
    # the height the barometer is fused at
    assert "init_gnss.alt_m     = (double)pressure_altitude_m + ref_alt_m;" in seed
    assert "ekf_baro.altitude_m = (double)pressure_altitude_m + ref_alt_m;" in src
    # the declination from the datum alone, never from 0 N 0 E (the mag reference as it always was: the raw
    # filter has no other window for the magnetometer)
    after = src[src.index("ekf.init(ekf_imu, init_gnss, ekf_mag);", i):src.index("ekf_initialized = true;", i)]
    assert "if (have_ref_pos && yr >= 2020 && yr <= 2035) {" in after
    assert after.index("if (have_ref_pos && yr >= 2020") < after.index("TR_GeoMag::declinationRad(")


def test_a_reboot_in_flight_keeps_a_degraded_start():
    src = code(read())
    save = code(body(src, "static void buildFlightSnapshot("))
    assert "snap.ekf_initialized = nav_start::snapshotFilterByte(ekf_initialized, gnss_absent_flight);" in save
    r = src.index("ekf_initialized  = snap.ekf_initialized != 0;")
    restore = src[r:src.index("if (snap.ekf_initialized) {", r)]
    assert re.search(r"gnss_absent_flight = nav_start::restoredWithoutGnss\(snap\.ekf_initialized\);\s*"
                     r"#if TR_RAW_GNSS_SHADOW\s*raw_gnss_held = gnss_absent_flight;\s*#endif", restore)
    assert not receiver_branches(src, r)


def test_the_last_usable_fix_is_the_one_the_normal_start_takes():
    src = code(read())
    m = re.search(r"if \(have_ref_pos && gnss_gate3_init\)\s*\{[^}]*nav_last_usable_fix_ms\s*=", src)
    assert m, "nav_last_usable_fix_ms is not stamped on the normal start's own condition"
    # after the pad average (which may set have_ref_pos on this very fix), before the restart and the start read it
    assert src.index("have_ref_pos = true;", src.index("const bool gnss_gate3_init =")) < m.start()
    assert m.start() < src.index("nav_start::restartOnFix(") < src.index("nav_start::degradedStartDue(")


def test_a_reinit_of_a_filter_on_fixes_starts_its_grace():
    src = code(read())
    stamp = re.search(r"if \(nav_filter_on_fixes\)\s*\{[^}]*nav_filter_on_fixes = false;[^}]*nav_reinit_ms\s*=", src)
    assert stamp, "the re-init is not stamped"
    unstarted = src.index("else if (!ekf_initialized)", src.index("if (rocket_state == MAG_CALIBRATION)"))
    assert unstarted < stamp.start() < src.index("nav_start::degradedStartDue(")
    on = re.search(r"nav_filter_on_fixes = !degraded_init;", src)
    assert on, "a start does not record whether it flies on fixes"
    assert src.rfind("ekf_initialized = true;", 0, on.start()) > src.index("nav_start::degradedStartDue(")


def test_a_sim_reset_clears_the_receivers_timestamps():
    fn = code(body(read(), "static void resetFlightStateForSim("))
    for name in ("nav_last_usable_fix_ms = 0;", "nav_filter_on_fixes    = false;", "nav_reinit_ms          = 0;"):
        assert name in fn, name


def test_a_mag_cal_exit_starts_the_grace_again():
    fn = code(body(read(), "static void magCalBackToReady()"))
    assert re.search(r"if \(nav_filter_on_fixes \|\| nav_reinit_ms != 0\)\s*\{[^}]*nav_reinit_ms\s*=", fn)
