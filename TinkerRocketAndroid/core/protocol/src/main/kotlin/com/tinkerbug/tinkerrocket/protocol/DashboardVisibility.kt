package com.tinkerbug.tinkerrocket.protocol

/**
 * Whether the dashboard may draw the rocket state banner. iOS twin:
 * `DashboardVisibility.showStateBanner` in `DashboardView.swift`.
 *
 * SYNCING carries TWO different meanings on the wire, and telling them apart is
 * the entire job of this function:
 *
 *  * **base station, no rocket caught** (#95) — the BS still pushes a frame so
 *    its own battery/logging/RSSI stay live, but the rocket half is a zero-init
 *    `LoRaDataSI`. State 0 IS `INITIALIZATION` (`RocketComputerTypes.h`), so
 *    drawing the banner invents a rocket sitting on the pad underneath a
 *    "Syncing…" notice. Hide it.
 *  * **direct link** (#831) — the OC reports SYNCING until the FC has sent its
 *    first `NonSensorData` frame, i.e. while `setup_fc()` is still running, so
 *    that stale zeroed continuity cannot render green. Here the state and its
 *    boot-progress line are the ONLY real information, and they are exactly
 *    what the operator is waiting on. Draw it.
 *
 * Treating those as one condition is what hid the state for the whole ~25 s FC
 * boot on iOS; the boot only reappeared at READY, which displays as PRELAUNCH.
 *
 * A pure function rather than an inline check so it is unit testable without
 * Compose — the app module has no test source set (see [pyroContinuityOf]).
 */
public fun showStateBanner(
    dataStatus: TelemetryData.DataStatus,
    isBaseStation: Boolean,
): Boolean = !(isBaseStation && dataStatus == TelemetryData.DataStatus.SYNCING)
