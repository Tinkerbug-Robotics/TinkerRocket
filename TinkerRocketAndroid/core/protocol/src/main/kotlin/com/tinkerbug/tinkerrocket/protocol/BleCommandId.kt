package com.tinkerbug.tinkerrocket.protocol

/**
 * BLE command numbers — the app→device byte that leads every command frame
 * `[cmd u8][payload]` on the command characteristic.
 *
 * PARSED BY `tools/check_ble_command_ids.py` (wire-codes CI): every command
 * number in this object is diffed against the Swift app's send sites and the
 * firmware dispatch chains.  Rules the checker enforces / relies on:
 *  - `const val NAME = N` declarations only, all in THIS file.
 *  - The OC and BS command spaces are INDEPENDENT — the same number can mean
 *    different things per device type.  Suffix conventions make that visible:
 *    `_OC` = out-computer/rocket link only, `_BS` = base-station link only,
 *    unsuffixed = same meaning on both.
 *  - cmds 70..72 (OTA) dispatch inside TR_BLE_To_APP.cpp, not the main.cpp
 *    chains the checker greps — they are legitimate despite not appearing in
 *    either dispatch.
 *
 * Payload layouts are pinned byte-exactly by tests_cpp/fixtures/wire/commands/.
 */
public object BleCommandId {
    // File operations (sent as raw writes in iOS — Data([N, ...]), not
    // sendCommand; the checker's KNOWN_RAW_WRITE_CMDS covers them).
    public const val FILE_LIST: Int = 2            // [page u8]
    public const val FILE_DELETE: Int = 3          // [name utf8]
    public const val FILE_DOWNLOAD: Int = 4        // [name utf8]

    // Simulated launch (sent from SimulationView on iOS).
    public const val SIM_CONFIG: Int = 5           // SimConfigData 16 B
    public const val SIM_START: Int = 6

    public const val POWER_TOGGLE: Int = 8         // BLIND toggle — UI gate #377
    public const val TIME_SYNC: Int = 9            // [year u16][mo][d][h][m][s] UTC
    public const val LORA_CONFIG: Int = 10         // [freq f32][bw f32][sf][cr][txpwr i8]
    public const val SOUNDS_ENABLE: Int = 11       // [bool]
    public const val SERVO_CONFIG: Int = 12        // ServoConfigData 22 B (#267)
    public const val PID_CONFIG: Int = 13          // PIDConfigData 20 B
    public const val SERVO_ENABLE: Int = 14        // [bool]
    public const val HOP_DISABLE_BS: Int = 17      // [bool] — BS only
    public const val REQUEST_CONFIG: Int = 20
    public const val GAIN_SCHED_ENABLE: Int = 22   // [bool]
    public const val TOGGLE_LOGGING: Int = 23
    public const val SERVO_TEST: Int = 24          // ServoTestAnglesData 8 B
    public const val SERVO_TEST_STOP: Int = 25
    public const val ROLL_PROFILE: Int = 26        // RollProfileData 76 B
    public const val ROLL_PROFILE_CLEAR: Int = 27
    public const val GUIDANCE_POINT: Int = 28      // GuidancePointData 20 B (#435)
    public const val ROLL_CTRL_CONFIG: Int = 31    // RollControlConfigData 16 B
    public const val GUIDANCE_ENABLE: Int = 32     // [bool]
    public const val CAMERA_CONFIG: Int = 33       // CameraConfigData 1 B
    public const val PYRO_CONFIG: Int = 34         // PyroConfigData 24 B
    public const val PYRO_CONT_TEST: Int = 35      // [ch]
    public const val PYRO_FIRE_TEST: Int = 36      // [ch]
    public const val SET_NAME: Int = 40            // utf8, firmware rejects > 20 B
    public const val SET_NETWORK_ID: Int = 41      // [nid u8]
    public const val SET_ROCKET_ID: Int = 42       // [rid u8] 1..254
    public const val SET_FOCUS_ROCKET_BS: Int = 45 // [rid u8], 0 = auto (#390)
    public const val BS_LOGGING: Int = 46          // [bool] (#390)

    // cmd 50 is the canonical dual-namespace number (the checker's docstring
    // example): mag-cal start on a rocket link, relay envelope on a BS link.
    public const val MAG_CAL_START_OC: Int = 50
    public const val RELAY_TO_ROCKET_BS: Int = 50  // [target_rid][inner cmd+payload ≤26 B]

    public const val MAG_CAL_ABORT_OC: Int = 51
    public const val MAG_CAL_ACCEPT_OC: Int = 52
    public const val MAG_CAL_RETRY_OC: Int = 53
    public const val MAG_CAL_COMPUTE_FIT_OC: Int = 54
    public const val MAG_CAL_APPLY_OC: Int = 55    // MagCalApplyData 14 B
    // 56/57/58 were double-assigned once (#132/#148) — that history is why
    // profile-driven cal I/O lives at 61/62/63, not here.
    public const val MAG_CAL_VERIFY_DONE_OC: Int = 56
    public const val MAG_CAL_VERIFY_RESET_OC: Int = 57
    public const val MAG_CAL_FORCE_APPLY_OC: Int = 58
    public const val FREQ_SCAN: Int = 60           // [start f32][stop f32][step u16][dwell u16]
    public const val MAG_CAL_READ_OC: Int = 61
    public const val SENSOR_CAL_APPLY_OC: Int = 62 // SensorCalApplyData 18 B
    public const val SENSOR_CAL_READ_OC: Int = 63
    public const val IMU_ORIENT: Int = 64          // [0xFF auto | code 0..23]
    public const val GUIDANCE_CONFIG: Int = 65     // GuidanceConfigData 45 B — first 36 FROZEN (#534)
    public const val FIN_CONFIG: Int = 66          // FinConfigData 18 B
    public const val IMU_RATE: Int = 67            // [rate_hz u16] 960/1920/3840

    // OTA — dispatched in TR_BLE_To_APP.cpp, invisible to the main.cpp greps.
    public const val OTA_BEGIN: Int = 70           // [target u8][size u32][sha256 32 B]
    public const val OTA_FINISH: Int = 71
    public const val OTA_ABORT: Int = 72
}
