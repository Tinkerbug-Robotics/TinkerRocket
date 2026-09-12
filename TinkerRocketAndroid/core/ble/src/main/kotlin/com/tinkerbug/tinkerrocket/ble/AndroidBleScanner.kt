@file:SuppressLint("MissingPermission")

package com.tinkerbug.tinkerrocket.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.ParcelUuid
import com.tinkerbug.tinkerrocket.session.BleAdvertisement
import com.tinkerbug.tinkerrocket.session.BleScanner
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportFactory
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow

/**
 * [BleScanner] over the platform LE scanner, with ONE platform scan shared by
 * every collector (#863).
 *
 * THE scan filter is the TinkerRocket service UUID and NOTHING else (#547):
 * no name filter — renamed devices advertise the raw user-set unit name, and
 * only factory defaults carry the TR-R-/TR-B- prefixes.  The advertised name
 * comes from the SCAN RECORD, never `device.name` (the stack's cached name
 * goes stale across firmware re-flashes — the iOS
 * CBAdvertisementDataLocalNameKey rule).
 *
 * ## Why this is ref-counted (#863)
 *
 * [BleScanner]'s contract has always said "implementations must support
 * concurrent collectors (ref-count the platform scan)". This one did not: every
 * collector got its own `startScan` with its own [ScanCallback], and its
 * `awaitClose` stopped only its own scan.
 *
 * Android silently throttles an app that starts more than **5 scans per 30 s**,
 * reporting it as `SCAN_FAILED_SCANNING_TOO_FREQUENTLY` — so the violation
 * manufactured the very error it then had to survive. Two places hold
 * concurrent collectors today, exactly as the contract anticipates: the
 * `resumeLastSession` path (a `scan()` plus its own sighting-wait) and the
 * reconnect ladder's endgame (the same pair, then a fresh collection per loop
 * iteration). #830 paced the endgame at 8 s and made a throttle survivable
 * rather than fatal; it did not remove the doubling.
 *
 * ## The part that needed care: terminal errors
 *
 * Each collector used to get its own `close(cause)`. A shared scan has to fan
 * `onScanFailed` out to EVERY collector, and the three behaviours that must
 * survive are:
 *
 *  * adapter off at collection start closes that collector with
 *    [BleTransportException];
 *  * `onScanFailed` closes every live collector with the error code;
 *  * cancelling one collector does NOT stop the platform scan; cancelling the
 *    last one does.
 *
 * A plain SharedFlow cannot carry a terminal error, so the fan-out is explicit:
 * each collector registers a [Sink] and the shared callback walks a snapshot of
 * them. A closed collector runs its own `awaitClose`, which unregisters it and
 * — if it was the last — stops the platform scan. So a failure tears the whole
 * thing down through the same path a normal cancellation uses, rather than a
 * second one that could disagree with it.
 */
public class AndroidBleScanner(private val context: Context) : BleScanner {

    /** One live collector. [fail] closes it with a terminal error. */
    private class Sink(
        val emit: (BleAdvertisement) -> Unit,
        val fail: (Throwable) -> Unit,
    )

    private val lock = Any()
    private val sinks = LinkedHashSet<Sink>()
    private var platformCb: ScanCallback? = null
    private var activeScanner: BluetoothLeScanner? = null

    // ── Test observability (#863) ───────────────────────────────────────────
    // The thing being fixed is how many times we call the PLATFORM, which no
    // amount of flow assertion can see. The bench test reads these.

    /** Platform `startScan` calls since construction. */
    @Volatile
    public var platformScanStarts: Int = 0
        private set

    /** Platform `stopScan` calls since construction. */
    @Volatile
    public var platformScanStops: Int = 0
        private set

    /** Live collectors right now. */
    public val activeCollectors: Int get() = synchronized(lock) { sinks.size }

    override fun advertisements(): Flow<BleAdvertisement> = callbackFlow {
        val sink = Sink(
            emit = { trySend(it) },
            // close() is idempotent, so a fan-out that races this collector's
            // own cancellation is harmless.
            fail = { close(it) },
        )
        val startFailure = register(sink)
        if (startFailure != null) {
            close(startFailure)
            return@callbackFlow
        }
        awaitClose { unregister(sink) }
    }

    /**
     * Add [sink]; start the platform scan if it is the first.
     *
     * Returns null on success, or the failure to close this collector with.
     * On failure the sink is NOT registered, so no unregister is owed.
     */
    private fun register(sink: Sink): Throwable? {
        synchronized(lock) {
            if (sinks.isEmpty()) {
                val manager =
                    context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
                // #1413: this message is not a log line — FleetManager puts it
                // straight into the status pill, so the user was being shown
                // "BLE scanner unavailable (adapter off?)", question mark and
                // all. Ask the adapter instead of guessing at it, and say the
                // answer in words a reader can act on.
                val adapter = manager.adapter
                    ?: return BleTransportException("Bluetooth is not available on this device")
                if (!adapter.isEnabled) {
                    return BleTransportException("Bluetooth is off")
                }
                val scanner = adapter.bluetoothLeScanner
                    ?: return BleTransportException("Bluetooth scanning is unavailable")

                val cb = object : ScanCallback() {
                    override fun onScanResult(callbackType: Int, result: ScanResult) {
                        val ad = BleAdvertisement(
                            deviceId = result.device.address,
                            advertisedName = result.scanRecord?.deviceName,
                            rssi = result.rssi,
                        )
                        // Snapshot under the lock, emit outside it: trySend can
                        // run arbitrary downstream work and this is a binder
                        // thread.
                        for (s in snapshot()) s.emit(ad)
                    }

                    override fun onScanFailed(errorCode: Int) {
                        val e = BleTransportException("scan failed, code $errorCode")
                        // Every live collector learns, not just whichever one
                        // happened to own the callback.
                        for (s in snapshot()) s.fail(e)
                    }
                }

                runCatching {
                    scanner.startScan(
                        listOf(
                            ScanFilter.Builder()
                                .setServiceUuid(
                                    ParcelUuid.fromString(TrCharacteristic.SERVICE_UUID),
                                )
                                .build(),
                        ),
                        ScanSettings.Builder()
                            // Foreground add-device / reconnect search: latency wins.
                            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                            .build(),
                        cb,
                    )
                }.onFailure { return it }

                platformCb = cb
                activeScanner = scanner
                platformScanStarts++
            }
            sinks.add(sink)
        }
        return null
    }

    /** Remove [sink]; stop the platform scan if it was the last. */
    private fun unregister(sink: Sink) {
        synchronized(lock) {
            if (!sinks.remove(sink)) return
            if (sinks.isNotEmpty()) return
            val cb = platformCb
            val scanner = activeScanner
            platformCb = null
            activeScanner = null
            if (cb != null && scanner != null) {
                runCatching { scanner.stopScan(cb) }
                platformScanStops++
            }
        }
    }

    private fun snapshot(): List<Sink> = synchronized(lock) { sinks.toList() }
}

/** [TransportFactory] minting one [RealBleTransport] per connection attempt. */
public class AndroidTransportFactory(
    private val context: Context,
    private val tap: ((String) -> Unit)? = null,
) : TransportFactory {
    override fun create(deviceId: String, autoConnect: Boolean): BleTransport {
        val manager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        return RealBleTransport(
            context = context,
            device = manager.adapter.getRemoteDevice(deviceId),
            autoConnect = autoConnect,
            tap = tap,
        )
    }
}
