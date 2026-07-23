package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.flow.Flow

/**
 * The GATT-operation-level transport seam (android-port plan §3).
 *
 * Everything above this interface — demux ladders, connect choreography,
 * state machines, download/OTA engines — runs identically against the real
 * Android stack (:core:ble RealBleTransport), the scripted FakeFirmware, and
 * recorded-session replay.  The serial one-outstanding-op GATT queue lives
 * INSIDE the real implementation: it is an Android-stack constraint, not
 * domain semantics.
 */
public interface BleTransport {
    public val events: Flow<TransportEvent>

    public suspend fun connect()
    public suspend fun requestMtu(target: Int): Int
    public suspend fun enableNotifications(char: TrCharacteristic)
    public suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean)
    public suspend fun read(char: TrCharacteristic): ByteArray
    public suspend fun readRssi(): Int
    public fun disconnect()
}

public sealed interface TransportEvent {
    public data class Connected(val mtu: Int) : TransportEvent
    public data class MtuChanged(val mtu: Int) : TransportEvent
    public data class Disconnected(val reason: Int) : TransportEvent
    public data class Notification(val char: TrCharacteristic, val bytes: ByteArray) : TransportEvent
    public data class WriteCompleted(val char: TrCharacteristic) : TransportEvent
    public data class Rssi(val dbm: Int) : TransportEvent
}

/**
 * The four characteristics on the TinkerRocket service.  UUIDs mirror iOS
 * BLEDevice.swift; the SERVICE UUID is also the ONLY scan filter — never
 * filter or gate on the advertised name (#547).
 */
public enum class TrCharacteristic(public val uuid: String) {
    TELEMETRY("beb5483e-36e1-4688-b7f5-ea07361b26a8"),      // notify, JSON
    COMMAND("cba1d466-344c-4be3-ab3f-189f80dd7518"),        // write-with-response
    FILE_OPS("8d53dc1d-1db7-4cd3-868b-8a527460aa84"),       // notify + explicit read
    FILE_TRANSFER("1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"), // notify chunks / OTA writes
    ;

    public companion object {
        public const val SERVICE_UUID: String = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
    }
}
