package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.platform.LocalContext
import androidx.core.content.ContextCompat
import com.tinkerbug.tinkerrocket.session.BluetoothAvailability

/**
 * #1413: the live Bluetooth state, for screens that need to say something
 * about it.
 *
 * Nothing in the app asked the adapter anything before this — there was no
 * `BluetoothAdapter.isEnabled` call anywhere in the sources. The user found out
 * the radio was off by tapping Scan and reading the scanner's exception text in
 * the status pill.
 *
 * Updates live: the adapter broadcasts ACTION_STATE_CHANGED when it is switched
 * on or off, including from the notification-shade tile while the app is in
 * front, which is the common case at a launch site.
 */
@Composable
fun rememberBluetoothAvailability(): BluetoothAvailability {
    val context = LocalContext.current

    fun read(): BluetoothAvailability {
        val adapter = (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)
            ?.adapter
        // BLUETOOTH_CONNECT is what ACTION_REQUEST_ENABLE and most adapter
        // reads need on API 31+; BLUETOOTH_SCAN is what scanning needs. The
        // screens that show this are gated on both, but read them rather than
        // assume, so a revoked grant shows as denied instead of as "off".
        val granted = listOf(
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
        ).all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }
        return BluetoothAvailability.of(
            hasAdapter = adapter != null,
            // isEnabled needs no permission, but guard anyway: on a denied
            // grant `granted` already decides the answer above.
            adapterOn = granted && adapter?.isEnabled == true,
            permissionsGranted = granted,
        )
    }

    var state by remember { mutableStateOf(read()) }

    DisposableEffect(context) {
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(ctx: Context?, intent: Intent?) {
                state = read()
            }
        }
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED),
            ContextCompat.RECEIVER_NOT_EXPORTED,
        )
        // A re-read on (re)composition covers the case the broadcast cannot:
        // the state changed while this screen was away.
        state = read()
        onDispose { context.unregisterReceiver(receiver) }
    }

    return state
}

/**
 * #1413: the system "turn on Bluetooth?" dialog. Nothing in the app offered a
 * route to it before — no ACTION_REQUEST_ENABLE, no Settings intent of any
 * kind.
 *
 * The result is deliberately ignored: the adapter broadcast above is what
 * updates the UI, and it fires whether the user accepted here, switched the
 * radio on from the shade instead, or turned it on and straight back off.
 */
@Composable
fun rememberBluetoothEnableLauncher(): () -> Unit {
    val launcher = rememberLauncherForActivityResult(
        ActivityResultContracts.StartActivityForResult(),
    ) { /* see above */ }
    return { launcher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)) }
}
