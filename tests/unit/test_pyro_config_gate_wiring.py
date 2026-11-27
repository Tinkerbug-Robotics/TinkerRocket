"""#1775: the pyro channel config is checked before either computer applies or saves it.

The check (TR_RocketComputerTypes/PyroConfigGate.h) is host-tested in tests_cpp/test_pyro_config_gate.cpp, with
the FC's own deploy gate showing what the refused altitudes would do. The FC and OC main.cpp are not host-built,
and the apps' copies of the rule live in Swift and Kotlin, so these tests read the source:
- the FC's PYRO_CONFIG_PENDING handler checks the frame before it adopts or saves it, and a refusal keeps the
  config held and re-sends the report; that handler and the boot load are the only writes of pyro_config;
- the FC's boot load checks the stored record and holds it as stored, and the pad health bit is the shared
  pyroChannelHealth (BAD for an enabled channel that fails, whatever its continuity);
- the OC's command 34 checks the frame before its cache, its NVS and the FC see it, and a refusal re-sends the
  readback; its boot load checks the stored record;
- the apps' bounds equal the firmware's, the iOS dashboard sheet no longer sends an unparseable field as 0,
  neither app sends a config the rocket would refuse, and the whole-profile push says so when it held one back.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
IDF = os.path.join(ROOT, "tinkerrocket-idf")
GATE = os.path.join(IDF, "components", "TR_RocketComputerTypes", "PyroConfigGate.h")
FC_MAIN = os.path.join(IDF, "projects", "flight_computer", "main", "main.cpp")
OC_MAIN = os.path.join(IDF, "projects", "out_computer", "main", "main.cpp")
IOS = os.path.join(ROOT, "TinkerRocketApp", "TinkerRocketApp")
IOS_RULE = os.path.join(IOS, "Models", "PyroTriggerRule.swift")
IOS_BLE = os.path.join(IOS, "Models", "BLEDevice.swift")
IOS_DASH = os.path.join(IOS, "Views", "DashboardView.swift")
IOS_SETTINGS = os.path.join(IOS, "Views", "SettingsView.swift")
IOS_SYNCER = os.path.join(IOS, "Models", "ActiveRocketSyncer.swift")
KT = os.path.join(ROOT, "TinkerRocketAndroid")
KT_RULE = os.path.join(KT, "core", "protocol", "src", "main", "kotlin", "com", "tinkerbug", "tinkerrocket",
                       "protocol", "PyroTriggerRule.kt")
KT_SYNCER = os.path.join(KT, "core", "session", "src", "main", "kotlin", "com", "tinkerbug", "tinkerrocket",
                         "session", "ActiveRocketSyncer.kt")
KT_SETTINGS = os.path.join(KT, "app", "src", "main", "kotlin", "com", "tinkerbug", "tinkerrocket", "app",
                           "SettingsScreen.kt")


def read(path):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without // and /* */ comments, whitespace collapsed to one space, none just inside a parenthesis."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = "\n".join(line.split("//", 1)[0] for line in text.splitlines())
    text = re.sub(r"\s+", " ", text)
    return re.sub(r"\s+\)", ")", re.sub(r"\(\s+", "(", text))


def braced(src, at):
    """The braced block that opens at the first '{' at or after index `at`, braces included."""
    i = src.index("{", at)
    depth = 0
    for j in range(i, len(src)):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                return src[i:j + 1]
    raise AssertionError("unbalanced braces")


def between(src, start, end):
    i = src.index(start)
    return src[i:src.index(end, i)]


def branches(block, condition):
    """The if-body for `condition` in `block`, and the else-body that follows it."""
    i = block.index(condition)
    then = braced(block, i)
    rest = block[block.index(then, i) + len(then):]
    assert rest.lstrip().startswith("else"), condition
    return then, braced(rest, 0)


def c_float(src, name):
    m = re.search(r"inline constexpr float " + name + r"\s*=\s*([0-9.]+)f;", src)
    assert m, name
    return float(m.group(1))


FC = code(read(FC_MAIN))
OC = code(read(OC_MAIN))
GATE_SRC = read(GATE)
FC_HANDLER = between(FC, "else if (out_pending_command == PYRO_CONFIG_PENDING)",
                     "else if (out_pending_command == PYRO_CONT_TEST)")
OC_HANDLER = between(OC, "else if (ble_cmd == 34)", "else if (ble_cmd == 35)")


def test_the_fc_checks_a_frame_before_it_adopts_or_saves_it():
    assert "#include <PyroConfigGate.h>" in read(FC_MAIN)
    check = FC_HANDLER.index("const PyroConfigVerdict pyro_verdict = pyroConfigCheck(incoming);")
    assert FC_HANDLER.index("memcpy(&incoming, cfg_payload, sizeof(PyroConfigData));") < check
    refuse, accept = branches(FC_HANDLER, "if (pyro_verdict.rc != PYRO_CFG_OK)")
    # Refused: nothing adopted or saved, and the report re-sent so the tiles show what the FC holds.
    for forbidden in ("pyro_config", "putBytes", "pyro_config_from_nvs"):
        assert forbidden not in refuse, forbidden
    assert "config_report_dirty = true;" in refuse
    assert "pyroConfigRcName(pyro_verdict.rc)" in refuse
    # Accepted: adopted from the checked copy, then saved.
    adopt = accept.index("memcpy(&pyro_config, &incoming, sizeof(PyroConfigData));")
    assert adopt < accept.index('prefs.putBytes("cfg", &pyro_config, sizeof(pyro_config));')
    assert "config_report_dirty = true;" in accept


def test_the_fc_writes_pyro_config_only_from_the_checked_frame_and_the_boot_load():
    writes = re.findall(r"memcpy\(&pyro_config\b[^;]*;|getBytes\(\"cfg\", &pyro_config[^;]*;|"
                        r"\bpyro_config\s*=[^=][^;]*;|\bpyro_config\.\w+\s*=[^=][^;]*;", FC)
    assert writes == [
        "pyro_config = {};",   # the declaration: zeroed, all four disabled
        'getBytes("cfg", &pyro_config, sizeof(pyro_config));',
        "memcpy(&pyro_config, &incoming, sizeof(PyroConfigData));",
    ], writes


def test_the_fc_boot_load_checks_the_record_and_holds_it():
    load = between(FC, 'size_t pyro_cfg_sz = prefs.getBytesLength("cfg");', "prefs.end();")
    got = load.index('prefs.getBytes("cfg", &pyro_config, sizeof(pyro_config));')
    check = load.index("const PyroConfigVerdict pyro_nvs_verdict = pyroConfigCheck(pyro_config);")
    assert got < check
    # Held as stored: dropping it would disable all four, whose health bits then read NA.
    assert "pyro_config = {}" not in load and "memset(&pyro_config" not in load


def test_the_pad_health_bit_is_the_shared_rule():
    # pyroChannelHealth (host-tested) reads BAD for an enabled channel whose trigger fails, whatever its continuity.
    loop = between(FC, "for (int i = 0; i < 4; ++i) { const SensorHealthState pst =",
                   "sh = shSet(sh, SH_PYRO_SHIFT[i], pst);")
    assert ("pyroChannelHealth(pyroChEnabled(i), pyroChMode(i), pyroChValue(i), cont_known[i], cont_state[i]);"
            in loop)
    assert FC.count("SH_PYRO_SHIFT[i]") == 1


def test_the_oc_checks_command_34_before_its_cache_its_nvs_and_the_fc_see_it():
    assert "#include <PyroConfigGate.h>" in read(OC_MAIN)
    copy = OC_HANDLER.index("memcpy(&pcfg, payload, sizeof(pcfg));")
    assert copy < OC_HANDLER.index("const PyroConfigVerdict verdict = pyroConfigCheck(pcfg);")
    refuse, accept = branches(OC_HANDLER, "if (verdict.rc != PYRO_CFG_OK)")
    for forbidden in ("cfg_pyro_", "setPendingCommandWithConfig", "putBytes"):
        assert forbidden not in refuse, forbidden
    assert "sendPyroConfigReadback();" in refuse
    for needed in ("cfg_pyro_enabled[i] = en[i];",
                   "setPendingCommandWithConfig(PYRO_CONFIG_PENDING, PYRO_CONFIG_MSG, &pcfg, sizeof(pcfg));",
                   'prefs.putBytes("cfg", &pcfg, sizeof(pcfg));'):
        assert needed in accept, needed
    # The only relay of a pyro config to the FC, and the only writes of the cache outside the boot load.
    assert OC.count("setPendingCommandWithConfig(PYRO_CONFIG_PENDING") == 1
    assert len(re.findall(r"cfg_pyro_trigger_value\[i\]\s*=", OC)) == 2


def test_the_oc_boot_load_checks_the_record():
    load = between(OC, 'size_t pyro_sz = prefs.getBytesLength("cfg");', "prefs.end();")
    assert load.index('prefs.getBytes("cfg", &pcfg, sizeof(pcfg));') < \
        load.index("const PyroConfigVerdict nvs_verdict = pyroConfigCheck(pcfg);")


def test_the_apps_bounds_are_the_firmwares():
    max_delay = c_float(GATE_SRC, "PYRO_TRIGGER_MAX_DELAY_S")
    min_alt = c_float(GATE_SRC, "PYRO_TRIGGER_MIN_ALTITUDE_M")
    assert (max_delay, min_alt) == (120.0, 30.0)
    swift = read(IOS_RULE)
    assert float(re.search(r"static let maxDelaySeconds: Float = ([0-9.]+)", swift).group(1)) == max_delay
    assert float(re.search(r"static let minAltitudeMeters: Float = ([0-9.]+)", swift).group(1)) == min_alt
    kotlin = read(KT_RULE)
    assert float(re.search(r"const val MAX_DELAY_S: Float = ([0-9.]+)f", kotlin).group(1)) == max_delay
    assert float(re.search(r"const val MIN_ALTITUDE_M: Float = ([0-9.]+)f", kotlin).group(1)) == min_alt
    # The log's rule names say the same numbers.
    assert '"delay outside 0-%d s"' % max_delay in GATE_SRC
    assert '"altitude below %d m"' % min_alt in GATE_SRC


def test_ios_never_sends_an_unparseable_field_as_zero_nor_a_refused_config():
    dash = code(read(IOS_DASH))
    sheet = braced(dash, dash.index("struct PyroConfigSheet: View"))
    assert "?? 0" not in sheet and "parseTriggerValue" not in sheet
    assert 'Button("Save") { saveAndSend(); dismiss() } .disabled(!canSave)' in sheet
    save = braced(sheet, sheet.index("func saveAndSend()"))
    assert save.index("guard canSave else { return }") < save.index("let val = pendingValue ?? loadedValue")
    settings = code(read(IOS_SETTINGS))
    parse = braced(settings, settings.index("private func parsePyroValue("))
    # Not a finite number: the stored value stays. A finite one the rocket refuses is stored, and held back below.
    assert "PyroTriggerRule.valueToSend(" in parse and "?? fallback" in parse and "accepts" not in parse
    apply = braced(settings, settings.index("private func applyPyroConfig()"))
    stored = apply.index("updateProfile {")
    assert stored < apply.index("reloadPyroValueStrings()") < \
        apply.index("guard !PyroTriggerRule.anyRefused(channels) else { return }") < apply.index("showApplied($pyroApplied)")
    rule = code(read(IOS_RULE))
    assert '.replacingOccurrences(of: ",", with: ".")' in braced(rule, rule.index("static func valueToSend("))
    ble = code(read(IOS_BLE))
    send = braced(ble, ble.index("func sendPyroConfig(channels:"))
    assert send.index("guard !PyroTriggerRule.anyRefused(channels) else { return false }") < \
        send.index("sendRawCommand(34, payload: payload)")
    # The whole-profile push says the pyro config did not go, rather than "Profile matches this rocket".
    sync = code(read(IOS_SYNCER))
    push = braced(sync, sync.index("func pushProfileToRocket()"))
    held = push.index("if !pyroSent, let ch = PyroTriggerRule.firstRefused(pyro) { "
                      "syncState = .failed(PyroTriggerRule.notSentText(channel: ch)) return }")
    assert push.index("let pyroSent = device.sendPyroConfig(channels: pyro)") < held < push.index("self.syncState = .synced")


def test_android_never_stores_a_nan_entry_nor_sends_a_refused_config():
    syncer = code(read(KT_SYNCER))
    push = syncer[syncer.index("ConfigGroup.PYRO -> {"):]
    hold = push.index("if (!PyroTriggerRule.anyRefused(channels)) {")
    assert hold < push.index("s.sendCommandFrame(Commands.pyroConfig(channels))") < \
        push.index("s.mirrorPyroConfig(channels)")
    assert syncer.count("Commands.pyroConfig(") == 1
    whole = syncer[syncer.index("public fun pushProfileToRocket()"):]
    held = whole.index("PyroTriggerRule.firstRefused(pyroChannels(profile))?.let { "
                       "_syncState.value = SyncState.Failed(PyroTriggerRule.notSentText(it)) return }")
    assert held < whole.index("_syncState.value = SyncState.Synced")
    settings = code(read(KT_SETTINGS))
    field = settings[settings.index('NumField(if (isTime) "Delay" else "Altitude"'):]
    field = field[:field.index("Text(")]
    # Not a finite number: nothing stored. A finite one the rocket refuses is stored, and held back by the push.
    assert "toFloatOrNull" not in field and "PyroTriggerRule.entry(s)?.let" in field and "accepts" not in field
    assert ".replace(',', '.')" in read(KT_RULE)
