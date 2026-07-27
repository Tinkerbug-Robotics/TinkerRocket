@file:SuppressLint("MissingPermission")

package com.tinkerbug.tinkerrocket.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothManager
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
 * [BleScanner] over the platform LE scanner.
 *
 * THE scan filter is the TinkerRocket service UUID and NOTHING else (#547):
 * no name filter — renamed devices advertise the raw user-set unit name, and
 * only factory defaults carry the TR-R-/TR-B- prefixes.  The advertised name
 * comes from the SCAN RECORD, never `device.name` (the stack's cached name
 * goes stale across firmware re-flashes — the iOS
 * CBAdvertisementDataLocalNameKey rule).
 *
 * Android scan-throttle note: starting/stopping scans >5x/30s gets the app
 * silently throttled by the platform — the fleet's single 15 s scan window
 * with explicit stop stays well under it.
 */
public class AndroidBleScanner(private val context: Context) : BleScanner {

    override fun advertisements(): Flow<BleAdvertisement> = callbackFlow {
        val manager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val scanner = manager.adapter?.bluetoothLeScanner
            ?: run { close(BleTransportException("BLE scanner unavailable (adapter off?)")); return@callbackFlow }

        val cb = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) {
                trySend(
                    BleAdvertisement(
                        deviceId = result.device.address,
                        advertisedName = result.scanRecord?.deviceName,
                        rssi = result.rssi,
                    ),
                )
            }

            override fun onScanFailed(errorCode: Int) {
                close(BleTransportException("scan failed, code $errorCode"))
            }
        }

        scanner.startScan(
            listOf(
                ScanFilter.Builder()
                    .setServiceUuid(ParcelUuid.fromString(TrCharacteristic.SERVICE_UUID))
                    .build(),
            ),
            ScanSettings.Builder()
                // Foreground add-device / reconnect search: latency wins.
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build(),
            cb,
        )
        awaitClose { runCatching { scanner.stopScan(cb) } }
    }
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
