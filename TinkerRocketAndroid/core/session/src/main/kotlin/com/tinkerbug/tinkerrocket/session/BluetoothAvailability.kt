package com.tinkerbug.tinkerrocket.session

/**
 * #1413: why Bluetooth is unavailable, in a form a screen can act on.
 *
 * Android had less than iOS did here. iOS at least named the state; Android
 * showed the user the developer's own guess — the scanner threw
 * `BleTransportException("BLE scanner unavailable (adapter off?)")` and
 * [FleetManager] put `e.message` straight into the status pill, question mark
 * and all. Nothing checked the adapter, nothing offered a way to turn it on,
 * and the permission screen could not tell "never asked" from "denied".
 *
 * The decisions live here, free of Android imports, so they can be tested
 * without a device or a robot — the states themselves need a phone with its
 * radio switched off or its grant revoked, which no unit test can arrange.
 *
 * The twin is `BluetoothAvailability.swift` in the iOS app. Keep the two in
 * step: same states, same shape of advice, platform-appropriate wording.
 */
public enum class BluetoothAvailability {
    /** Before anything has been read. Says nothing rather than guessing. */
    UNKNOWN,
    READY,

    /** Adapter present and permitted, but switched off. */
    ADAPTER_OFF,

    /** BLUETOOTH_SCAN / BLUETOOTH_CONNECT not granted. */
    PERMISSION_DENIED,

    /** No Bluetooth hardware at all. */
    NO_ADAPTER,
    ;

    /** The line beside the status dot. */
    public val headline: String
        get() = when (this) {
            READY -> "Bluetooth ready"
            ADAPTER_OFF -> "Bluetooth is off"
            PERMISSION_DENIED -> "Bluetooth access needed"
            NO_ADAPTER -> "Bluetooth not supported"
            UNKNOWN -> "Bluetooth not ready"
        }

    /**
     * One quiet line under the headline, or null. Never a banner and never a
     * recoloured dot — an advisory that shouts is worse than the bare string
     * it replaces.
     */
    public val advice: String?
        get() = when (this) {
            READY, UNKNOWN -> null
            ADAPTER_OFF -> "Turn Bluetooth on to find your flight computers."
            PERMISSION_DENIED -> "TinkerRocket was denied the Nearby devices permission."
            // Nothing the reader can do, and nothing has gone wrong.
            NO_ADAPTER -> null
        }

    /**
     * What to offer alongside the advice. Unlike iOS, Android can hand the
     * user a working switch for both cases: a system dialog for the adapter,
     * and the app's own settings page for the grant.
     */
    public val fix: BluetoothFix
        get() = when (this) {
            ADAPTER_OFF -> BluetoothFix.ENABLE_BLUETOOTH
            PERMISSION_DENIED -> BluetoothFix.APP_SETTINGS
            READY, NO_ADAPTER, UNKNOWN -> BluetoothFix.NONE
        }

    /** Whether a scan could actually run; the Scan control gates on it. */
    public val canScan: Boolean get() = this == READY

    public companion object {
        /**
         * Precedence matters. A missing adapter beats everything — there is
         * nothing to permit or switch on. A denied grant comes next, because
         * on Android 12+ the adapter state cannot be read reliably without
         * it, so "off" would be a guess at that point.
         */
        public fun of(
            hasAdapter: Boolean,
            adapterOn: Boolean,
            permissionsGranted: Boolean,
        ): BluetoothAvailability = when {
            !hasAdapter -> NO_ADAPTER
            !permissionsGranted -> PERMISSION_DENIED
            !adapterOn -> ADAPTER_OFF
            else -> READY
        }
    }
}

/** The action offered next to a [BluetoothAvailability.advice] line. */
public enum class BluetoothFix {
    NONE,

    /** System "turn on Bluetooth?" dialog (BluetoothAdapter.ACTION_REQUEST_ENABLE). */
    ENABLE_BLUETOOTH,

    /** This app's page in Settings, where its permissions live. */
    APP_SETTINGS,
    ;

    /** Button text, or null when there is no button. */
    public val label: String?
        get() = when (this) {
            NONE -> null
            ENABLE_BLUETOOTH -> "Turn on Bluetooth"
            APP_SETTINGS -> "Open Settings"
        }
}
