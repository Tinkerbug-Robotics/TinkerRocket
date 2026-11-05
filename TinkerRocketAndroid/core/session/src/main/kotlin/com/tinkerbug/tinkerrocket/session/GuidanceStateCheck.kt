package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.RocketConfig

/**
 * Does the flight computer hold the guidance setting the profile shows?
 * Port of iOS `GuidanceStateCheck`.
 *
 * GTV Blue, 2026-11-01: the app showed station-keeping ON and the flight
 * computer flew with it OFF.  Every screen showed the profile, and the one
 * value the out computer reported (`"ge"`) was its own copy.  The flight
 * computer's own on/off now rides config_guid (`"gen"`), and this compares the
 * profile against that and nothing else.  Pure, so the cases are unit-tested.
 */
public sealed interface GuidanceStateCheck {
    /** The flight computer holds what the profile shows. */
    public data object Agrees : GuidanceStateCheck

    /** Profile says guidance; the flight computer has it off. */
    public data object RocketOff : GuidanceStateCheck

    /** The flight computer has guidance on; the profile says roll control. */
    public data object RocketOn : GuidanceStateCheck

    /** Both on, flying different laws (0 = PN, 1 = station-keep). */
    public data class LawDiffers(val rocket: Int, val profile: Int) : GuidanceStateCheck

    /** Profile says guidance and nothing from the flight computer confirms it. */
    public data class Unverified(val why: String) : GuidanceStateCheck

    /**
     * Profile says roll control and the flight computer is not reporting:
     * nothing to warn about.
     */
    public data object NotApplicable : GuidanceStateCheck

    /** The rocket will not fly what the profile shows. */
    public val isMismatch: Boolean
        get() = this is RocketOff || this is RocketOn || this is LawDiffers

    /** One line for the header, Settings and the preflight step; null when there is nothing to say. */
    public val message: String?
        get() = when (this) {
            Agrees, NotApplicable -> null
            RocketOff -> "Rocket guidance OFF — profile says ON"
            RocketOn -> "Rocket guidance ON — profile says OFF"
            is LawDiffers -> "Rocket flies ${lawName(rocket)} — profile says ${lawName(profile)}"
            is Unverified -> "Rocket guidance unverified — $why"
        }

    public companion object {
        public fun evaluate(
            profile: RocketProfile?,
            config: RocketConfig?,
            isConnected: Boolean,
            isBaseStation: Boolean,
        ): GuidanceStateCheck {
            if (profile == null) return NotApplicable
            val wantOn = profile.guidanceEnabled
            fun unknown(why: String): GuidanceStateCheck = if (wantOn) Unverified(why) else NotApplicable
            if (!isConnected) return unknown("not connected")
            // Settings never ride LoRa: a base-station link cannot see them.
            if (isBaseStation) return unknown("connect to the rocket directly to check")
            if (config == null) return unknown("rocket settings not read yet")
            val extras = config.guidanceExtras
                ?: return unknown("flight computer not reporting (power it on)")
            val rocketOn = extras.enabled
                ?: return unknown("this rocket's firmware doesn't report it")
            return when {
                wantOn && !rocketOn -> RocketOff
                !wantOn && rocketOn -> RocketOn
                !wantOn -> Agrees
                extras.guidanceLaw == profile.pnGuidanceLaw -> Agrees
                else -> LawDiffers(rocket = extras.guidanceLaw, profile = profile.pnGuidanceLaw)
            }
        }

        public fun lawName(law: Int): String = when (law) {
            0 -> "PN"
            1 -> "station-keep"
            else -> "law $law"
        }
    }
}
