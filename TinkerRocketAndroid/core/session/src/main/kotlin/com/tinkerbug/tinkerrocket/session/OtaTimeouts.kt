package com.tinkerbug.tinkerrocket.session

/**
 * Every wait in the OTA flow, in one table — the Kotlin twin of iOS
 * `OTATimeouts`.
 *
 * Split out of [OtaSession] because the timing contract is pure data and the
 * part most likely to drift from iOS: nothing about it needs a transport, so
 * it can be checked directly.
 *
 * The contract these numbers satisfy is [OtaStage.crossesRelay]. A stage that
 * waits on the FC answering over the OC relay MUST get a longer window on the
 * FC path; a stage that only waits on the phone's own link to the OC MUST get
 * the same one. Bench 2026-07-28 found `FINISH` violating that — flat 15 s on
 * both paths, which expired while a 591.7 kB FC flash was still running, so
 * the app reported failure on a transfer that went on to succeed. See #627.
 *
 * Both platforms check themselves against
 * `tests_cpp/fixtures/app_behavior/ota_timeouts.json`, so changing one without
 * the other fails that platform's build.
 */
public enum class OtaStage {
    BEGIN,
    FINISH,
    FW_PUBLISH,
    DISCONNECT,
    RECONNECT,
    ;

    /** Does finishing this stage depend on the FC answering over the relay? */
    public val crossesRelay: Boolean
        get() = when (this) {
            BEGIN, FINISH, FW_PUBLISH -> true
            DISCONNECT, RECONNECT -> false
        }

    /** Name as it appears in the shared fixture. */
    public val wireName: String
        get() = when (this) {
            BEGIN -> "begin"
            FINISH -> "finish"
            FW_PUBLISH -> "fwPublish"
            DISCONNECT -> "disconnect"
            RECONNECT -> "reconnect"
        }
}

public object OtaTimeouts {

    /** Poll interval for every await in the flow. */
    public const val POLL_MS: Long = 50

    public fun millis(stage: OtaStage, targetIsFc: Boolean): Long = when (stage) {
        // FC erases its OTA slot before it can answer ready.
        OtaStage.BEGIN -> if (targetIsFc) 20_000 else 5_000
        // FC drains the I2S ring, writes the tail, SHA-256s the image and sets
        // the boot partition — every status hop crossing the relay.
        OtaStage.FINISH -> if (targetIsFc) 60_000 else 15_000
        // FC reboots, then the OC re-queries its identity before the new
        // version can reach the app.
        OtaStage.FW_PUBLISH -> if (targetIsFc) 40_000 else 10_000
        // Local link events — the relay plays no part, so no FC stretch.
        OtaStage.DISCONNECT -> 5_000
        OtaStage.RECONNECT -> 60_000
    }
}
