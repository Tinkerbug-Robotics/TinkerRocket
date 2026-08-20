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
 */
public fun pyroContinuityOf(
    telemetry: TelemetryData,
    channel: Int,
    isConnected: Boolean,
    dataStatus: TelemetryData.DataStatus,
    isBaseStation: Boolean,
): PyroContinuity {
    if (!isConnected || dataStatus != TelemetryData.DataStatus.LIVE) {
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
        // bit is all there is, and it genuinely cannot tell open from
        // untested. Deliberately preserved rather than "improved" into
        // reporting UNTESTED forever against an older rocket.
        if (telemetry.pyroCont(channel)) PyroContinuity.PRESENT else PyroContinuity.OPEN
    }
}
