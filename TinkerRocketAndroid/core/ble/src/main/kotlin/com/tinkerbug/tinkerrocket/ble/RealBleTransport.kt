@file:SuppressLint("MissingPermission")
// BLUETOOTH_CONNECT/SCAN are runtime permissions the APP layer must hold
// before any transport is created — this module never prompts.  Suppressed
// file-wide instead of sprinkling checks that could only throw later anyway.

package com.tinkerbug.tinkerrocket.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
import android.content.Context
import android.os.Build
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportEvent
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeout
import java.util.UUID

/**
 * [BleTransport] over raw [BluetoothGatt] — the ONLY class in the app that
 * talks to the Android BLE stack for a connection (android-port plan §2).
 *
 * Design contract (everything above the seam is pinned by 118 JVM tests):
 *  - **Serial op queue**: Android silently drops a GATT call issued while
 *    another is outstanding.  Every suspend op here holds [opMutex] for its
 *    full issue→callback round-trip, so exactly one op is ever in flight —
 *    including the RSSI poll and the file-list delayed read, which arrive
 *    through the same suspend API.
 *  - **Binder-thread discipline**: [BluetoothGattCallback] runs on binder
 *    threads.  Callbacks only complete a [CompletableDeferred] or emit into
 *    [events] (buffered SharedFlow) — no state is touched.  Notification
 *    payloads are COPIED before crossing (vendor stacks reuse the buffer).
 *  - **No connection-priority pinning**: firmware owns conn-param policy
 *    (#519/#524).  Downloads/OTA may boost temporarily via
 *    [requestConnectionPriorityHigh]/[releaseConnectionPriority] — the
 *    session layer decides when; this class never does it implicitly.
 *  - **Status 133**: the classic vendor-stack instant-fail.  connect()
 *    retries once after a short pause; anything past that is the caller's
 *    (reconnect ladder's) problem.
 *  - iOS parity note: CoreBluetooth negotiates MTU implicitly and serializes
 *    ops internally — both are explicit here by design (plan §3).
 */
public class RealBleTransport(
    private val context: Context,
    private val device: BluetoothDevice,
    private val autoConnect: Boolean,
    /** Optional wire tap: every op/event line for the session recorder. */
    private val tap: ((String) -> Unit)? = null,
) : BleTransport {

    private val _events = MutableSharedFlow<TransportEvent>(extraBufferCapacity = 4096)
    override val events: Flow<TransportEvent> = _events

    private val opMutex = Mutex()
    private var gatt: BluetoothGatt? = null
    private val chars = mutableMapOf<TrCharacteristic, BluetoothGattCharacteristic>()

    // One pending completion per callback kind; guarded by opMutex (set
    // before the GATT call, completed from the binder thread).
    private var pendingConnect: CompletableDeferred<Int>? = null
    private var pendingDiscovery: CompletableDeferred<Int>? = null
    private var pendingMtu: CompletableDeferred<Int>? = null
    private var pendingDescriptorWrite: CompletableDeferred<Int>? = null
    private var pendingWrite: CompletableDeferred<Int>? = null
    private var pendingRead: CompletableDeferred<Pair<Int, ByteArray>>? = null
    private var pendingRssi: CompletableDeferred<Pair<Int, Int>>? = null

    private fun emit(event: TransportEvent) {
        tap?.invoke("event:$event")
        // extraBufferCapacity 4096: tryEmit only fails if collectors stall for
        // thousands of events; treat as programming error, not silent loss.
        check(_events.tryEmit(event)) { "transport event buffer overflow" }
    }

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            tap?.invoke("cb:connState status=$status state=$newState")
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> pendingConnect?.complete(status)
                BluetoothProfile.STATE_DISCONNECTED -> {
                    // A failure BEFORE the connect resolved reports through
                    // the pending op; afterwards it is the async drop event.
                    val p = pendingConnect
                    if (p != null && !p.isCompleted) {
                        p.complete(if (status == 0) STATUS_DISCONNECTED_EARLY else status)
                    } else {
                        emit(TransportEvent.Disconnected(status))
                    }
                    g.close()
                }
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            pendingDiscovery?.complete(status)
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            if (status == 0) emit(TransportEvent.MtuChanged(mtu))
            pendingMtu?.complete(if (status == 0) mtu else -status)
        }

        override fun onDescriptorWrite(
            g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int,
        ) {
            pendingDescriptorWrite?.complete(status)
        }

        override fun onCharacteristicWrite(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int,
        ) {
            // Fires for BOTH write types; the without-response completion is
            // the OTA pacing signal (one chunk per callback, plan §2).
            trOf(c)?.let { emit(TransportEvent.WriteCompleted(it)) }
            pendingWrite?.complete(status)
        }

        // API 33+ delivers the value as a parameter (already a safe copy).
        override fun onCharacteristicChanged(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray,
        ) {
            trOf(c)?.let { emit(TransportEvent.Notification(it, value.copyOf())) }
        }

        @Deprecated("pre-33 path")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < 33) {
                // COPY IMMEDIATELY — the framework reuses this buffer.
                val value = c.value?.copyOf() ?: return
                trOf(c)?.let { emit(TransportEvent.Notification(it, value)) }
            }
        }

        override fun onCharacteristicRead(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray, status: Int,
        ) {
            pendingRead?.complete(status to value.copyOf())
        }

        @Deprecated("pre-33 path")
        @Suppress("DEPRECATION")
        override fun onCharacteristicRead(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int,
        ) {
            if (Build.VERSION.SDK_INT < 33) {
                pendingRead?.complete(status to (c.value?.copyOf() ?: ByteArray(0)))
            }
        }

        override fun onReadRemoteRssi(g: BluetoothGatt, rssi: Int, status: Int) {
            if (status == 0) emit(TransportEvent.Rssi(rssi))
            pendingRssi?.complete(status to rssi)
        }
    }

    override suspend fun connect(): Unit = opMutex.withLock {
        check(gatt == null) { "connect() called twice" }
        var attempt = 0
        while (true) {
            attempt++
            tap?.invoke("op:connect attempt=$attempt autoConnect=$autoConnect")
            val connected = CompletableDeferred<Int>()
            pendingConnect = connected
            gatt = device.connectGatt(context, autoConnect, callback, BluetoothDevice.TRANSPORT_LE)
                ?: throw BleTransportException("connectGatt returned null")
            val status = try {
                // autoConnect waits for the device to appear — no timeout.
                if (autoConnect) connected.await()
                else withTimeout(CONNECT_TIMEOUT_MS) { connected.await() }
            } catch (t: Throwable) {
                gatt?.close(); gatt = null; pendingConnect = null
                throw BleTransportException("connect timeout", t)
            }
            pendingConnect = null
            if (status == 0) break
            gatt?.close(); gatt = null
            // Vendor-stack 133 instant-fail: one retry after a settle pause.
            if (status == 133 && attempt == 1 && !autoConnect) {
                kotlinx.coroutines.delay(STATUS_133_RETRY_DELAY_MS)
                continue
            }
            throw BleTransportException("connect failed, status $status")
        }

        // Service discovery + characteristic resolution is part of connect():
        // above the seam the link is "connected" only when usable (iOS folds
        // discovery into the choreography the same way).
        val g = gatt!!
        val discovered = CompletableDeferred<Int>()
        pendingDiscovery = discovered
        check(g.discoverServices()) { "discoverServices() refused" }
        val dStatus = try {
            withTimeout(DISCOVERY_TIMEOUT_MS) { discovered.await() }
        } finally {
            pendingDiscovery = null
        }
        if (dStatus != 0) throw BleTransportException("service discovery failed, status $dStatus")

        val service = g.getService(UUID.fromString(TrCharacteristic.SERVICE_UUID))
            ?: throw BleTransportException("TinkerRocket service missing")
        for (tc in TrCharacteristic.entries) {
            chars[tc] = service.getCharacteristic(UUID.fromString(tc.uuid))
                ?: throw BleTransportException("characteristic ${tc.name} missing")
        }
        emit(TransportEvent.Connected(DEFAULT_MTU))
    }

    override suspend fun requestMtu(target: Int): Int = op("mtu:$target") {
        val d = CompletableDeferred<Int>()
        pendingMtu = d
        check(requireGatt().requestMtu(target)) { "requestMtu refused" }
        val granted = try { withTimeout(OP_TIMEOUT_MS) { d.await() } } finally { pendingMtu = null }
        if (granted < 0) throw BleTransportException("MTU negotiation failed, status ${-granted}")
        granted
    }

    override suspend fun enableNotifications(char: TrCharacteristic): Unit = op("cccd:$char") {
        val g = requireGatt()
        val c = chars.getValue(char)
        check(g.setCharacteristicNotification(c, true)) { "setCharacteristicNotification refused" }
        val cccd = c.getDescriptor(CCCD_UUID)
            ?: throw BleTransportException("$char has no CCCD")
        val d = CompletableDeferred<Int>()
        pendingDescriptorWrite = d
        if (Build.VERSION.SDK_INT >= 33) {
            val rc = g.writeDescriptor(cccd, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            check(rc == BluetoothGatt.GATT_SUCCESS) { "writeDescriptor refused rc=$rc" }
        } else {
            @Suppress("DEPRECATION")
            cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            @Suppress("DEPRECATION")
            check(g.writeDescriptor(cccd)) { "writeDescriptor refused" }
        }
        val status = try {
            withTimeout(OP_TIMEOUT_MS) { d.await() }
        } finally {
            pendingDescriptorWrite = null
        }
        if (status != 0) throw BleTransportException("CCCD write failed, status $status")
    }

    override suspend fun write(
        char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean,
    ): Unit = op("write:$char:${bytes.size}b resp=$withResponse") {
        val g = requireGatt()
        val c = chars.getValue(char)
        val writeType = if (withResponse) BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        else BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        val d = CompletableDeferred<Int>()
        pendingWrite = d
        if (Build.VERSION.SDK_INT >= 33) {
            val rc = g.writeCharacteristic(c, bytes, writeType)
            check(rc == BluetoothGatt.GATT_SUCCESS) { "writeCharacteristic refused rc=$rc" }
        } else {
            @Suppress("DEPRECATION")
            c.value = bytes
            c.writeType = writeType
            @Suppress("DEPRECATION")
            check(g.writeCharacteristic(c)) { "writeCharacteristic refused" }
        }
        // Both write types complete via onCharacteristicWrite — awaiting it
        // for without-response too is what makes the queue the OTA pacer
        // (one chunk per callback, plan §2/S3).
        val status = try { withTimeout(OP_TIMEOUT_MS) { d.await() } } finally { pendingWrite = null }
        if (status != 0) throw BleTransportException("write $char failed, status $status")
    }

    override suspend fun read(char: TrCharacteristic): ByteArray = op("read:$char") {
        val g = requireGatt()
        val d = CompletableDeferred<Pair<Int, ByteArray>>()
        pendingRead = d
        check(g.readCharacteristic(chars.getValue(char))) { "readCharacteristic refused" }
        val (status, value) = try {
            withTimeout(OP_TIMEOUT_MS) { d.await() }
        } finally {
            pendingRead = null
        }
        if (status != 0) throw BleTransportException("read $char failed, status $status")
        value
    }

    override suspend fun readRssi(): Int = op("rssi") {
        val d = CompletableDeferred<Pair<Int, Int>>()
        pendingRssi = d
        check(requireGatt().readRemoteRssi()) { "readRemoteRssi refused" }
        val (status, rssi) = try {
            withTimeout(OP_TIMEOUT_MS) { d.await() }
        } finally {
            pendingRssi = null
        }
        if (status != 0) throw BleTransportException("readRemoteRssi failed, status $status")
        rssi
    }

    override fun disconnect() {
        tap?.invoke("op:disconnect")
        // Async by contract: the Disconnected event arrives via the callback
        // (which also closes the gatt).
        gatt?.disconnect()
    }

    /**
     * Session-scoped connection-priority boost for downloads/OTA — the
     * SESSION decides when; never pinned (#519/#524: firmware owns
     * conn-param policy, and a pinned HIGH defeats OC idle power).
     */
    public fun requestConnectionPriorityHigh() {
        gatt?.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
    }

    public fun releaseConnectionPriority() {
        gatt?.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_BALANCED)
    }

    private suspend fun <T> op(label: String, body: suspend () -> T): T = opMutex.withLock {
        tap?.invoke("op:$label")
        body()
    }

    private fun requireGatt(): BluetoothGatt =
        gatt ?: throw BleTransportException("not connected")

    private fun trOf(c: BluetoothGattCharacteristic): TrCharacteristic? =
        TrCharacteristic.entries.firstOrNull { it.uuid.equals(c.uuid.toString(), ignoreCase = true) }

    public companion object {
        private val CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        private const val DEFAULT_MTU = 23
        private const val CONNECT_TIMEOUT_MS = 15_000L
        private const val DISCOVERY_TIMEOUT_MS = 10_000L
        private const val OP_TIMEOUT_MS = 10_000L
        private const val STATUS_133_RETRY_DELAY_MS = 500L
        /** Clean-close-before-connect-resolved marker (status was 0). */
        public const val STATUS_DISCONNECTED_EARLY: Int = -1
    }
}

public class BleTransportException(message: String, cause: Throwable? = null) :
    Exception(message, cause)
