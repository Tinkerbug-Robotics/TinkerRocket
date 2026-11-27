package com.tinkerbug.tinkerrocket.protocol

/*
 * Which app this build is, and what it shows for the rocket it talks to.
 * iOS twin: AppEdition.swift — keep the two in step.
 *
 * One codebase, two editions (docs/app-editions.md):
 *
 *   STORE  The Play Store app, com.tinkerbug.rocket, "Tinker Rocket".
 *          Supports the Tinker-Beetle only, the first flight computer
 *          released. The `store` product flavor (BuildConfig.TR_STORE).
 *   DEV    Every board. The `dev` flavor, com.tinkerbug.tinkerrocket,
 *          "Tinker Rocket Dev": the app already sideloaded on the bench
 *          phones, so its flights and settings stay where they are.
 *
 * A feature here follows what the BOARD has, not just the edition. The board
 * files say it: board_m2.h has "no servos -- no fin control" and no camera
 * channel; board_v10.h has three servo outputs and a GoPro/RunCam port. So the
 * dev edition hides the fin and camera screens too while a Beetle is the
 * rocket in front of it, and shows them when it cannot tell.
 *
 * The edition is passed in rather than read here: this module cannot see the
 * app's BuildConfig, and passing it is what lets the store rules be tested
 * from any build.
 */

public enum class AppEdition { STORE, DEV }

/** The family a board revision names, from its letter: "M2 (Tinker-Beetle V2)", "M3", "V10". */
public enum class BoardFamily {
    /** The M family: the Tinker-Beetle (M2 = Beetle V2, M3 = Beetle V3). */
    BEETLE,

    /** The V family: the Mantis (V10, V11) and the V boards before it (V9). */
    MANTIS,
    ;

    public companion object {
        /**
         * Null when the text names no family this app knows: empty,
         * unprovisioned, or a letter without a revision number after it.
         */
        public fun fromRevision(revision: String?): BoardFamily? {
            val r = revision?.trim()?.uppercase() ?: return null
            if (r.length < 2 || !r[1].isDigit()) return null
            return when (r[0]) {
                'M' -> BEETLE
                'V' -> MANTIS
                else -> null
            }
        }

        /**
         * The rocket's family from everything the link says about it, best
         * source first: what each board was PROVISIONED as (#773 step 2), then
         * the board its running firmware was built for — every board
         * provisioned before #773 reports no revision, so that fallback is the
         * ordinary case.
         */
        public fun of(
            fcBoard: String?,
            ocBoard: String?,
            fcFirmware: String?,
            ocFirmware: String?,
        ): BoardFamily? =
            fromRevision(fcBoard)
                ?: fromRevision(ocBoard)
                ?: fromRevision(EspImage.boardSuffix(fcFirmware))
                ?: fromRevision(EspImage.boardSuffix(ocFirmware))
    }
}

/**
 * What this build shows for the rocket in front of it. [board] is null when
 * nothing is connected, on a base station relay, or for a board that has not
 * said what it is.
 */
public data class AppFeatures(val edition: AppEdition, val board: BoardFamily?) {

    /**
     * The fin servos and everything that only reaches the fins through them:
     * servo control, PID gains, servo calibration, control mode, roll
     * control, roll profile, guidance, fin layout, Servo Test and Ground
     * Test (and Drift Cast's send, which iOS has and Android has not ported).
     * The Beetle has no servo outputs.
     */
    public val finControl: Boolean
        get() = when (edition) {
            AppEdition.STORE -> false
            AppEdition.DEV -> board != BoardFamily.BEETLE
        }

    /**
     * The camera (GoPro / RunCam): its settings, the Start camera control,
     * the Status card's Camera badge, the camera in the rocket's summary, and
     * the pre-flight "camera recording" check. The Beetle has no camera port.
     */
    public val camera: Boolean
        get() = when (edition) {
            AppEdition.STORE -> false
            AppEdition.DEV -> board != BoardFamily.BEETLE
        }

    /**
     * Whether this edition supports the rocket at all. The store edition
     * supports the Beetle only; an unknown board gets the benefit of the
     * doubt, because a Beetle reads as unknown until its identity arrives.
     */
    public val boardSupported: Boolean
        get() = edition != AppEdition.STORE || board == null || board == BoardFamily.BEETLE

    public companion object {
        /**
         * Whether to offer a release image in the firmware catalog. The store
         * edition offers rocket images for the Beetle only (plus images built
         * for every board, which carry no board suffix); base-station images
         * are all offered, in both editions.
         */
        public fun offersFirmware(image: FirmwareImage, edition: AppEdition): Boolean {
            if (edition != AppEdition.STORE || image.project == EspImage.PROJECT_BS) return true
            val board = image.board ?: return true
            return BoardFamily.fromRevision(board) == BoardFamily.BEETLE
        }
    }
}
