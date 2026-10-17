package com.tinkerbug.tinkerrocket.session

/**
 * What the Settings screen may offer for a directly connected rocket, decided
 * in one place so the screen and its tests agree.  iOS twin:
 * `SettingsAccess.evaluate` in SettingsAccess.swift; both suites run the cases
 * in tests_cpp/fixtures/app_behavior/settings_access.json.
 *
 * The rule is the one the rocket imposes.  The settings the flight computer
 * stores and answers need it running: with its rail off or its boot
 * unfinished, the OC's #366 queue HOLDS the writes and drains them much later,
 * out of order, or not at all.  The out computer's own settings (LoRa
 * telemetry on/off, network ID) and the firmware update need only the
 * Bluetooth link to the out computer, so they stay available whatever the
 * flight computer is doing.  That matters most for the update: it is how a
 * flight computer that cannot finish booting gets a working image without
 * opening the airframe (reported 2026-10-16, an FC parked on "Starting
 * sensors").
 */
public enum class SettingsAccess {
    /** The flight computer is running its flight software: every setting. */
    FULL,

    /** The flight computer's rail is off. */
    ROCKET_OFF,

    /**
     * The rail is on, but the flight computer is not running its flight
     * software: still starting, stuck partway, silent, or in rescue mode.
     */
    FLIGHT_COMPUTER_NOT_RUNNING;

    /**
     * Settings the flight computer stores and answers: calibrations, IMU
     * mounting and rate, camera, servo control, PID, roll and guidance, fin
     * layout, pyro channels and pyro tests.
     */
    public val allowsFlightComputerSettings: Boolean get() = this == FULL

    /**
     * The out computer's own settings and the firmware update.  Always on for
     * a connected rocket: none of them goes through the flight computer.
     */
    public val allowsOutComputerSettings: Boolean get() = true
}

/**
 * @param hasReceivedTelemetry #377: before the first frame, telemetry is all
 *   defaults, and a defaulted pwrPinOn reads exactly like a rocket that is off.
 * @param bootStepReported "bs" is present, which the OC forwards only while the
 *   FC is in setup_fc() (or rescue mode).
 */
public fun settingsAccess(
    isConnected: Boolean,
    isBaseStation: Boolean,
    hasReceivedTelemetry: Boolean,
    pwrPinOn: Boolean,
    state: String,
    bootStepReported: Boolean,
): SettingsAccess {
    // Offline, profile edits are saved and pushed on a later connect.  A base
    // station has its own read-only layout, and its relayed telemetry never
    // carries pwrPinOn (the LoRa packet has no room), so a gate scoped to it
    // would lock its Settings for good.
    if (!isConnected || isBaseStation) return SettingsAccess.FULL
    if (hasReceivedTelemetry && !pwrPinOn) return SettingsAccess.ROCKET_OFF
    // INITIALIZATION is what the OC's zeroed rocket_state reads until the FC
    // sends its first NonSensorData, i.e. until loop_fc() runs.
    if (state == "INITIALIZATION" || bootStepReported) {
        return SettingsAccess.FLIGHT_COMPUTER_NOT_RUNNING
    }
    return SettingsAccess.FULL
}

/**
 * The words that stand in for the flight computer's settings while it is not
 * running.  iOS twin: FlightComputerNotRunningSection.title/explanation in
 * SettingsView.swift, worded the same.  They matter more than usual, because
 * the operator reading them may be looking at a rocket that will not boot: say
 * what it is doing, what still works, and the way out that does not involve
 * opening the airframe.
 */
public object FlightComputerNotice {
    /** [bootStepReported]: "bs" is present.  [rescue]: it is FCB_RESCUE. */
    public fun title(bootStepReported: Boolean, rescue: Boolean): String = when {
        !bootStepReported -> "Waiting for flight computer"
        rescue -> "Flight computer in rescue mode"
        else -> "Flight computer starting"
    }

    public fun explanation(bootStepReported: Boolean, rescue: Boolean): String = when {
        !bootStepReported ->
            "The flight computer hasn't reported in yet. Its settings unlock once it is " +
                "running. The radio, network ID and firmware below are on the out computer " +
                "and work now."
        rescue ->
            "Its last boots did not finish, so it started only what a firmware update " +
                "needs. Update the flight computer from Firmware below, or power-cycle the " +
                "rocket to try a normal boot. Its settings come back once it boots normally."
        else ->
            "The settings stored on the flight computer unlock once it finishes starting. " +
                "The radio, network ID and firmware below are on the out computer and work " +
                "now. If it is stuck, you can still try a flight computer update: the update " +
                "tells you whether the flight computer answered."
    }
}
