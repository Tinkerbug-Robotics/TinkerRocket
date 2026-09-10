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

    /**
     * Ceiling on the chunk pump for FC-relay OTAs only — a local OC OTA stays
     * uncapped (straight to flash, no relay, happy at ~68 kB/s).
     *
     * #627: on the relay path the OC re-sends every chunk to the FC over I2S.
     * Outrun that drain and the OC's 16-deep feed queue fills, receive buffers
     * stop being retired, NimBLE's ACL mbuf pool exhausts and the link wedges
     * — the OC stops receiving entirely (bench 2026-07-28: `enq` frozen at
     * 3418 for 135 s amid thousands of `ACL buf alloc failed`). The FC then
     * sees a hole and, being forward-only, discards everything after it.
     *
     * Measured on one board, one image: iOS pumps 11.4 kB/s and the relay
     * works; Android pumps ~68 kB/s and it fails. iOS was never *correct*, only
     * slow enough to stay under a limit nobody had written down. This writes it
     * down — nearly a no-op on iOS, the actual fix on Android.
     *
     * BENCH-MEASURED 2026-09-10 (#811), V9 pair, 647,472 B image over the
     * relay, OC console captured. The cap is what sets the transfer time —
     * `image / cap` — and the idle-fill everyone kept trying to optimise is
     * just `1 - cap/50KB` falling out of it:
     *
     *   cap      pump    idle   qdepth peak        mbuf failures
     *   12 KB/s  53.0 s  77%    2                  0
     *   20 KB/s  31.3 s  62%    3                  0    <- here
     *   30 KB/s  20.7 s  41%    13 (startup only)  0
     *   40 KB/s  15.2 s  19%    16 (full)          4    <- over the edge
     *
     * At 40 the OC's 16-frame feed queue saturates and NimBLE starts failing
     * ACL allocations — the #627 signature above, reproduced deliberately.
     * 30 also worked, with the queue empty in steady state; 20 is chosen over
     * it because these are ONE RUN EACH, on one board, one image, Android
     * only, and #627's lesson is exactly that such a number does not
     * generalise. 20 nearly halves the transfer with the queue barely touched.
     *
     * If more is wanted, the 16-frame queue is the lever rather than this
     * constant: it is ~54 ms deep at drain rate and it is what saturates.
     */
    public const val FC_RELAY_MAX_BYTES_PER_SEC: Long = 20_000

    /**
     * How long to wait before sending the next chunk to keep the FC-relay pump
     * under [FC_RELAY_MAX_BYTES_PER_SEC].
     *
     * Paced against total bytes sent rather than per-chunk, so the real time
     * the write itself took is credited instead of added — a fixed per-chunk
     * sleep would drift the effective rate well below the cap.
     */
    public fun fcRelayPaceDelayMs(bytesSent: Long, elapsedMs: Long): Long {
        val targetMs = bytesSent * 1000L / FC_RELAY_MAX_BYTES_PER_SEC
        return if (targetMs > elapsedMs) targetMs - elapsedMs else 0L
    }

    public fun millis(stage: OtaStage, targetIsFc: Boolean): Long = when (stage) {
        // FC erases its OTA slot before it can answer ready.
        OtaStage.BEGIN -> if (targetIsFc) 20_000 else 5_000
        // FC drains the I2S ring, writes the tail, SHA-256s the image and sets
        // the boot partition — every status hop crossing the relay.
        //
        // The LOCAL window is NOT device-side verification: the bench measured that at
        // 380 ms. It is the tail of the transfer still draining out of the phone's
        // BLE stack after the pump loop has returned — the app stopped writing ~26 s
        // before the device logged OTA_FINISH on an 815,696 B image (console,
        // 2026-09-10). The device reports `writing` at 2 Hz throughout, so the
        // no-progress budget in OtaSession.awaitFinish is what actually carries this,
        // and the number below only has to cover the largest gap between updates.
        //
        // 15 s was not enough: on 2026-09-10 an 815 kB out-computer image and a
        // 770 kB base-station image both outlasted it and the app reported
        // "Device did not finalize OTA within 15s" on flashes that committed and
        // booted. Same failure #627 fixed on the FC path in 2026-07; the local
        // path kept the old number and the images grew into it. 45 s stays under
        // the FC's 60 s, which the crossesRelay contract requires.
        OtaStage.FINISH -> if (targetIsFc) 60_000 else 45_000
        // FC reboots, then the OC re-queries its identity before the new
        // version can reach the app.
        OtaStage.FW_PUBLISH -> if (targetIsFc) 120_000 else 10_000
        // Local link events — the relay plays no part, so no FC stretch.
        OtaStage.DISCONNECT -> 5_000
        OtaStage.RECONNECT -> 60_000
    }
}
