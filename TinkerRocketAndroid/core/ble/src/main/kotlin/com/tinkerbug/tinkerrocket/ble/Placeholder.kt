package com.tinkerbug.tinkerrocket.ble

/**
 * Placeholder pending RealBleTransport (the serial GATT op queue over
 * BluetoothGatt) — lands with the Phase 2 bench seam.  This file exists so
 * the module's Kotlin+AGP toolchain wiring is compile-verified from day one.
 */
public object BleModule {
    /** The one scan filter the app is ever allowed to use (#547). */
    public const val SERVICE_UUID: String = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
}
