package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlin.math.abs
import kotlin.math.floor

/**
 * Drift-Cast guidance-point upload confirmation (#435) — a semantics-exact
 * port of the iOS `GuidanceTargetEcho` / `GuidanceSendFlow` /
 * `GuidancePointRc` block in BLETypes.swift (the behavioral reference).
 *
 * Pure logic only: the 6 s confirmation timeout is owned by the caller (iOS:
 * DriftCastSendButton; Android: the session/UI layer) — the machine exposes
 * the pure transitions ([GuidanceSendFlow.onEcho] / [GuidanceSendFlow.onTimeout])
 * and never touches a clock.
 */

/**
 * FC-authoritative guidance aim-point state, relayed by the OC as a small
 * `{"type":"guid_target",...}` JSON frame (OutStatusQueryData v5 tail).
 * Live state, NOT folded into the rocket config — it changes with every
 * processed cmd 28 / cmd-65 apply, and the Drift-Cast send button gates its
 * success on it. `status` describes the CURRENT target; `lastRc` the result
 * of the LAST processed cmd-28 upload. They are separate fields so a
 * rejected upload does not erase the display of a still-active target.
 * Absent entirely (null on the device session) on pre-#435 firmware.
 */
public data class GuidanceTargetEcho(
    /**
     * Bumps on EVERY processed cmd 28 (accept or reject) and on any cmd-65
     * apply that changes the echo content. The send flow captures a baseline
     * before sending and treats any advance as "the FC processed something".
     */
    val seq: Int,
    val status: Int,   // GUID_TGT_*
    val lastRc: Int,   // GUID_RC_* (see GuidancePointRc)
    /**
     * Receipt-time geodetic point as the FC stored it, f32-rounded on the
     * wire (ulp ~0.4 m at mid-latitudes — verification precision, the FC
     * keeps the f64 originals for navigation).
     */
    val lat: Double,
    val lon: Double,
    val altM: Int,     // stored/echoed only — station-keep ignores altitude
) {
    /**
     * Pure confirmation predicate for a cmd-28 upload (unit-tested).
     * `baselineSeq` is the echo seq captured immediately before sending.
     *
     * Attribution caveats (the reason [GuidanceSendFlow] exists):
     * - `baselineSeq < 0` means NO baseline — nothing can be attributed to
     *   the upload, so every frame is [GuidanceSendOutcome.Pending] (the flow
     *   adopts the first frame as its baseline instead of misreading it as a
     *   verdict).
     * - `tgt_seq` also bumps on echo-changing cmd-65 applies, which never
     *   write `tgt_last_rc` — so `rc == NONE` can never be a cmd-28 verdict
     *   (the FC writes a nonzero GUID_RC_* on every processed cmd 28) and
     *   maps to Pending, and a Rejected/Mismatch result may still be an
     *   interleaved cmd-65 frame rather than OUR verdict.  Only Confirmed is
     *   self-authenticating (fresh ACCEPTED + GEO_ACTIVE + coordinate match);
     *   the flow treats the failures as provisional.
     */
    public fun outcome(baselineSeq: Int, sentLat: Double, sentLon: Double): GuidanceSendOutcome {
        if (baselineSeq < 0) return GuidanceSendOutcome.Pending
        if (seq == baselineSeq) return GuidanceSendOutcome.Pending
        if (lastRc == GuidancePointRc.NONE.raw) return GuidanceSendOutcome.Pending
        if (lastRc != GuidancePointRc.ACCEPTED.raw) return GuidanceSendOutcome.Rejected(lastRc)
        if (status != STATUS_GEO_ACTIVE ||
            abs(lat - sentLat) >= LAT_MATCH_TOL_DEG ||
            abs(lon - sentLon) >= LON_MATCH_TOL_DEG
        ) {
            return GuidanceSendOutcome.Mismatch
        }
        return GuidanceSendOutcome.Confirmed
    }

    public companion object {
        // tgt_status values — mirror GUID_TGT_* in RocketComputerTypes.h.
        public const val STATUS_NONE: Int = 0        // no aim point (overhead / cleared)
        public const val STATUS_GEO_ACTIVE: Int = 1  // cmd-28 geodetic point active
        public const val STATUS_EN_ACTIVE: Int = 2   // cmd-65 profile E/N point active

        // Confirmation-match tolerances: the echo is f32-rounded (ulp ~4e-6
        // deg), leaving >4x margin under these. Lon is looser than lat
        // because a degree of longitude shrinks with cos(lat) while the f32
        // ulp doesn't.
        public const val LAT_MATCH_TOL_DEG: Double = 2e-5   // ~2.2 m
        public const val LON_MATCH_TOL_DEG: Double = 3e-5   // ~2.6 m at mid-latitudes

        /**
         * Parse the OC's compact readback frame. Defensive per the MTU-budget
         * rule (#282): every field individually optional.  Returns null when
         * the frame is not a `"type":"guid_target"` object, mirroring the iOS
         * failable `init?(json:)`.
         */
        public fun fromJson(json: JsonObject): GuidanceTargetEcho? {
            if (jsonString(json, "type") != "guid_target") return null
            return GuidanceTargetEcho(
                seq = jsonInt(json, "seq") ?: 0,
                status = jsonInt(json, "st") ?: 0,
                lastRc = jsonInt(json, "rc") ?: 0,
                lat = jsonDouble(json, "lat") ?: 0.0,
                lon = jsonDouble(json, "lon") ?: 0.0,
                altM = jsonInt(json, "alt") ?: 0,
            )
        }

        // JSON reads mirror iOS `dict["k"] as? Int` / `as? Double` NSNumber
        // bridging: numbers only (never strings/bools/objects); `as? Int`
        // accepts a JSON float only when it is exactly integral (3.0 → 3,
        // 3.7 → nil → default); `as? Double` accepts any JSON number.

        private fun numericPrimitive(json: JsonObject, key: String): JsonPrimitive? {
            val el = json[key] ?: return null
            if (el is JsonNull) return null
            val p = el as? JsonPrimitive ?: return null
            return if (p.isString) null else p
        }

        private fun jsonInt(json: JsonObject, key: String): Int? {
            val p = numericPrimitive(json, key) ?: return null
            p.content.toLongOrNull()?.let {
                return it.coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong()).toInt()
            }
            val d = p.content.toDoubleOrNull() ?: return null
            if (!d.isFinite() || floor(d) != d) return null
            return d.toLong().coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong()).toInt()
        }

        private fun jsonDouble(json: JsonObject, key: String): Double? {
            val p = numericPrimitive(json, key) ?: return null
            return p.content.toDoubleOrNull()
        }

        private fun jsonString(json: JsonObject, key: String): String? {
            val el = json[key] ?: return null
            val p = el as? JsonPrimitive ?: return null
            return if (p.isString) p.content else null
        }
    }
}

/** Result of matching a guidance-target echo against a just-sent point. */
public sealed interface GuidanceSendOutcome {
    /** seq hasn't advanced — FC hasn't processed it yet. */
    public data object Pending : GuidanceSendOutcome

    /** Accepted, geodetic-active, coordinates match. */
    public data object Confirmed : GuidanceSendOutcome

    /** FC processed and refused (GUID_RC_* reason). */
    public data class Rejected(val rc: Int) : GuidanceSendOutcome

    /** FC accepted SOMETHING, but not our point. */
    public data object Mismatch : GuidanceSendOutcome
}

/**
 * Pure verdict state machine for one cmd-28 send (#435, unit-tested).
 *
 * Frame-attribution rules — each one closes a demonstrated false-verdict
 * path in the naive "first seq advance is the answer" reading:
 * - No baseline (`baselineSeq < 0`, e.g. send raced the connect-time
 *   readback): the FIRST frame — typically the OC's stale connect push of a
 *   PREVIOUS session's state — becomes the baseline, never a verdict.
 *   Previously it could paint a false green for an upload the FC rejected,
 *   or never even received.
 * - Confirmed is self-authenticating (fresh ACCEPTED + GEO_ACTIVE +
 *   coordinate match after a real seq advance) and settles immediately.
 * - Rejected / Mismatch frames are only PROVISIONAL: an interleaved cmd-65
 *   apply (connect-time profile sync, settings self-apply) also bumps seq
 *   while carrying a STALE rc, so the true accept may still be one poll
 *   behind. The provisional reason settles as Failed only at timeout, and a
 *   later genuine Confirmed upgrades over it.
 * Net effect: false green and false red are both closed; the worst case for
 * a genuine rejection is that its reason displays at the timeout instead of
 * instantly.
 *
 * The confirmation timeout itself (6 s on iOS) lives in the caller — this
 * class only exposes [onTimeout] as the pure "timer fired" transition.
 */
public class GuidanceSendFlow(
    baselineSeq: Int,
    public val sentLat: Double,
    public val sentLon: Double,
) {
    public sealed interface Verdict {
        public data object Waiting : Verdict
        public data object Confirmed : Verdict
        public data class Failed(val message: String) : Verdict
    }

    public var baselineSeq: Int = baselineSeq
        private set

    public var provisionalFailure: String? = null
        private set

    public var verdict: Verdict = Verdict.Waiting
        private set

    public fun onEcho(echo: GuidanceTargetEcho) {
        if (verdict != Verdict.Waiting) return
        if (baselineSeq < 0) {
            // Unattributable — adopt as baseline (see rules above).
            baselineSeq = echo.seq
            return
        }
        when (val outcome = echo.outcome(baselineSeq, sentLat, sentLon)) {
            GuidanceSendOutcome.Pending -> Unit
            GuidanceSendOutcome.Confirmed -> verdict = Verdict.Confirmed
            is GuidanceSendOutcome.Rejected ->
                provisionalFailure = GuidancePointRc.messageForRaw(outcome.rc)
            GuidanceSendOutcome.Mismatch ->
                provisionalFailure =
                    "The rocket confirmed a different aim point than the one sent — " +
                        "resend, and check nothing else is pushing settings."
        }
    }

    public fun onTimeout() {
        if (verdict != Verdict.Waiting) return
        verdict = Verdict.Failed(provisionalFailure ?: TIMEOUT_MESSAGE)
    }

    public companion object {
        public const val TIMEOUT_MESSAGE: String =
            "No confirmation from the rocket within 6 s. The firmware may predate #435, " +
                "or the link dropped — the aim point is NOT confirmed."
    }
}

/** FC cmd-28 result codes — mirror GUID_RC_* in RocketComputerTypes.h (#435). */
public enum class GuidancePointRc(public val raw: Int) {
    NONE(0),               // GUID_RC_NONE — no cmd 28 processed since boot
    ACCEPTED(1),           // GUID_RC_ACCEPTED
    REJECTED_RADIUS(2),    // GUID_RC_REJ_RADIUS — point > GUIDANCE_MAX_AIM_RADIUS_M
    REJECTED_STATE(3),     // GUID_RC_REJ_STATE — not READY/PRELAUNCH, or post-flight lockout
    REJECTED_LAW(4),       // GUID_RC_REJ_LAW — law is PN, a point would be silently inert (#376)
    REJECTED_NO_REF(5),    // GUID_RC_REJ_NOREF — no GNSS launch-site reference yet
    REJECTED_BAD_VALUE(6); // GUID_RC_REJ_BADVAL — non-finite / out-of-range lat/lon/alt

    public val message: String
        get() = when (this) {
            NONE -> "No upload has been processed."
            ACCEPTED -> "Aim point accepted."
            REJECTED_RADIUS ->
                "Aim point is more than 100 m from the launch reference — recompute closer to the pad."
            REJECTED_STATE -> "Rocket must be in READY or PRELAUNCH (and not post-flight)."
            REJECTED_LAW ->
                "Rocket guidance law is PN — set the profile's law to Station-Keep and push settings first."
            REJECTED_NO_REF -> "Rocket has no GNSS reference yet — wait for pad fix averaging."
            REJECTED_BAD_VALUE -> "Rejected: invalid values."
        }

    public companion object {
        public fun fromRaw(raw: Int): GuidancePointRc? = entries.firstOrNull { it.raw == raw }

        public fun messageForRaw(rc: Int): String =
            fromRaw(rc)?.message ?: "Rejected (unknown code $rc)."
    }
}
