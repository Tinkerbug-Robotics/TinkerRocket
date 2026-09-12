package com.tinkerbug.tinkerrocket.protocol

/**
 * Four-state pyro continuity. iOS twin: `BLEDevice.PyroContinuity`.
 *
 * The states exist because "no reading" and "measured open" are different
 * facts and must not look alike. Collapsing them to a Bool is what made a
 * never-tested channel display a confident red NO CONT (bench 2026-08-17, and
 * again on the iOS Settings row in #828) — which an operator reads as a dead
 * igniter or an already-fired charge, on a channel that is in fact live.
 */
/**
 * #1060: how old the last telemetry frame may be before continuity stops being
 * a measurement. Matches iOS `BLEDevice.telemetryStaleThresholdMs`; the two
 * must move together or the platforms disagree about a safety readout.
 */
public const val TELEMETRY_STALE_THRESHOLD_MS: Long = 3000L

public enum class PyroContinuity {
    /** Measured: continuity present. */
    PRESENT,

    /** Measured: open circuit — fired, or nothing connected. Red is for this alone. */
    OPEN,

    /** Link is good, but no reading has been taken this session. */
    UNTESTED,

    /** Stream stale or disconnected — nothing here is trustworthy. */
    NO_DATA,
}

/**
 * Path-aware continuity for the pyro UIs, #297 fail-safe included (a non-live
 * stream is [PyroContinuity.NO_DATA], never a held-over green).
 *
 * A pure function rather than a [DeviceSession]-style member so it is unit
 * testable without Compose or coroutines — the app module has no test source
 * set. iOS twin: `BLEDevice.pyroContinuity(channel:)`.
 *
 * @param dataStatus the session's EFFECTIVE data status, not
 *   [TelemetryData.dataStatus] — on a relay link the focused rocket can be
 *   stale while the base station's own frames are live.
 * @param telemetryAgeMs milliseconds since the last telemetry frame, or null
 *   if none has ever arrived. #1060: a connected link that stops notifying
 *   leaves [dataStatus] LIVE indefinitely, so without this a green CONT is
 *   held for as long as the connection stands.
 */
public fun pyroContinuityOf(
    telemetry: TelemetryData,
    channel: Int,
    isConnected: Boolean,
    dataStatus: TelemetryData.DataStatus,
    isBaseStation: Boolean,
    telemetryAgeMs: Long?,
): PyroContinuity {
    if (!isConnected || dataStatus != TelemetryData.DataStatus.LIVE) {
        return PyroContinuity.NO_DATA
    }

    // #1060: connected is not the same as talking. iOS has carried this guard
    // since f25af04 (`BLEDevice.swift`, threshold 3000 ms); the Android half
    // was never ported, and this function took no clock at all.
    //
    // The window is narrow but it is the one that matters: a direct link where
    // the OC holds the BLE connection up and stops sending frames. The BLE
    // flight-log download is such a path — its `while (!eof)` loop runs inside
    // loop_oc and blocks the telemetry send. An operator pulling a log off an
    // ARMED rocket would read a green CONT that is not a current measurement.
    // Stale-green on a safety readout is the failure direction that matters.
    if (telemetryAgeMs == null || telemetryAgeMs > TELEMETRY_STALE_THRESHOLD_MS) {
        return PyroContinuity.NO_DATA
    }

    // Preferred on BOTH paths: the measured bits are reported for every
    // channel and are the only source that distinguishes untested from open.
    telemetry.pyroMeasuredContinuity(channel)?.let { measured ->
        return when (measured) {
            TelemetryData.SensorHealth.OK -> PyroContinuity.PRESENT
            TelemetryData.SensorHealth.BAD -> PyroContinuity.OPEN
            // NA here means "not tested yet", DEGRADED "configured, untested".
            else -> PyroContinuity.UNTESTED
        }
    }

    return if (isBaseStation) {
        // Relay: the 65-byte LoRa downlink carries no pyro_status at all, so
        // continuity rides the sensor-health scorecard. On firmware predating
        // the measured bits only the CONFIG-GATED ones exist, where an
        // unconfigured channel (NA) is simply unknowable and DEGRADED means
        // configured-but-untested — both are untested, NOT an open circuit.
        when (telemetry.pyroHealth(channel)) {
            TelemetryData.SensorHealth.OK -> PyroContinuity.PRESENT
            TelemetryData.SensorHealth.BAD -> PyroContinuity.OPEN
            else -> PyroContinuity.UNTESTED
        }
    } else {
        // Direct link, firmware predating the measured bits: the raw "ps" cont
        // bit is all there is.
        //
        // #1048: a SET bit still proves presence — the FC sets it as
        // `cont_known && cont_state`, so it can only be high on a channel that
        // was measured and found continuous. A CLEAR bit proves nothing: it
        // conflates "never measured" with "measured open", and those are the
        // two states this enum exists to keep apart (#828).
        //
        // This used to return OPEN, which renders a confident red "NO CONT" —
        // an operator reads that as a dead igniter or an already-fired charge,
        // on a channel that may be perfectly live. iOS has resolved the
        // identical frame to .untested since 4874a79; the port never happened,
        // and the old behaviour was pinned by a test rather than merely
        // missed.
        //
        // The cases that are not transient are what decide it: against
        // firmware predating the measured bits the fallback is permanent, and
        // the #382 READY->INFLIGHT promotion skips pyroPrelaunchContTest
        // entirely, so a degraded-mode flight arms with all four channels
        // unmeasured and the in-flight reveal shows four red badges.
        if (telemetry.pyroCont(channel)) PyroContinuity.PRESENT else PyroContinuity.UNTESTED
    }
}
