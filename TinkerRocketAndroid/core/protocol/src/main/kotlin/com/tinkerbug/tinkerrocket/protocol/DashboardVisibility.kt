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

/**
 * Whether the dashboard may draw the rocket VALUE cards — flight summary, IMU,
 * GPS. iOS twin: `DashboardView.showValueViews(_:)`.
 *
 * #1047: the same two meanings of SYNCING as [showStateBanner], read the other
 * way round. A base station that has caught no rocket still pushes a frame so
 * its own battery, logging and RSSI stay live, and the rocket half of it is a
 * zero-init `LoRaDataSI` — but `TR_BLE_To_APP` emits `nsat`, `palt`, `soc` and
 * `vol` unconditionally, so those zeros arrive as readings rather than as
 * absent keys. Rendering them puts `0.0%`, `0.00 V`, `0.000000, 0.000000` and
 * `0 sats` on screen, in the same six-decimal monospace a real fix uses,
 * underneath an explicit "Syncing…" notice.
 *
 * iOS has gated this since the card layout was written; Android ported only
 * [showStateBanner] and left the value cards ungated.
 *
 * The window is pad setup only — the tracked-rocket `active` flag is set on the
 * first packet and never cleared, so SYNCING does not come back in flight or
 * during recovery. That is why this is a fabricated-reading bug rather than a
 * flight-data one.
 *
 * Note the asymmetry with [showStateBanner], which is deliberate: on a DIRECT
 * link SYNCING means the FC is still booting, and there the state banner is the
 * only real information there is, so it draws. The value cards are zeros in
 * both readings of SYNCING, so they are hidden in both.
 */
public fun showValueViews(
    dataStatus: TelemetryData.DataStatus,
): Boolean = dataStatus != TelemetryData.DataStatus.SYNCING

/**
 * Whether the battery card may draw the ROCKET row.
 *
 * #1047: mirrors iOS `showBSOnlyBattery`. On a base-station link with no rocket
 * caught, the base station's own battery is a live, true reading and must keep
 * rendering — it is what tells the operator the BS is up. Only the rocket row
 * is fabricated, so drop that row rather than the whole card.
 */
public fun showRocketBatteryRow(
    dataStatus: TelemetryData.DataStatus,
    isBaseStation: Boolean,
): Boolean = !(isBaseStation && dataStatus == TelemetryData.DataStatus.SYNCING)
