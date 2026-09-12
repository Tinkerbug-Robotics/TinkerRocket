package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.WifiOff
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.LocalTrColors
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.distinctUntilChanged

/**
 * #1092 item 3: is there usable internet right now?
 *
 * iOS twin: `NetworkMonitor` (NWPathMonitor) + `OfflinePill`. Android had no
 * ConnectivityManager, no NetworkCallback and not even ACCESS_NETWORK_STATE,
 * so the map had no way to say why it had gone blank.
 *
 * The tile stack already behaves correctly offline — MapScreen forces
 * `MapLibre.setConnected(true)` so the renderer keeps asking, and
 * TileProxyServer serves a hatch placeholder on an upstream miss. What the
 * operator got was hatching over an unsaved area and no explanation. This
 * supplies the explanation; it changes no tile behaviour.
 *
 * Lives in :app rather than :core:maps because that module is deliberately
 * pure JVM with zero Android dependencies.
 */
public fun networkOnline(context: Context): Flow<Boolean> = callbackFlow {
    val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager
    if (cm == null) {
        // No connectivity service to ask. Claim online rather than paint a
        // permanent OFFLINE pill on a device whose stack we cannot see — a
        // wrong "offline" is worse than a missing one, because the pill exists
        // to explain blank tiles and would then be explaining nothing.
        trySend(true)
        awaitClose { }
        return@callbackFlow
    }

    // VALIDATED is the capability that matters, not INTERNET alone: a captive
    // portal or a connected-but-dead wifi reports INTERNET without it, and that
    // is exactly the case where tiles stop arriving while the status bar still
    // shows bars.
    fun online(n: Network?): Boolean {
        val caps = n?.let { cm.getNetworkCapabilities(it) } ?: return false
        return caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET) &&
            caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED)
    }

    // Seed from the active network so the first frame is right, rather than
    // flashing a pill until the first callback lands.
    trySend(online(cm.activeNetwork))

    val cb = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) { trySend(online(network)) }
        override fun onLost(network: Network) { trySend(online(cm.activeNetwork)) }
        override fun onCapabilitiesChanged(network: Network, caps: NetworkCapabilities) {
            trySend(
                caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET) &&
                    caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED),
            )
        }
    }
    val req = NetworkRequest.Builder()
        .addCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
        .build()
    runCatching { cm.registerNetworkCallback(req, cb) }
        .onFailure {
            // Missing permission, or a stack that refuses the request: same
            // reasoning as above — do not invent an offline state.
            trySend(true)
        }
    awaitClose { runCatching { cm.unregisterNetworkCallback(cb) } }
}.distinctUntilChanged()

/**
 * Small "OFFLINE" pill — renders nothing while online. iOS twin: `OfflinePill`.
 *
 * Advisory, in the repo's usual sense: it explains blank tiles, it never
 * blocks anything, and it does not recolour the map.
 */
@Composable
public fun OfflinePill(modifier: Modifier = Modifier) {
    val context = LocalContext.current
    val flow = remember(context) { networkOnline(context) }
    val online by flow.collectAsState(initial = true)
    if (online) return
    val tr = LocalTrColors.current
    Row(
        modifier
            .background(MaterialTheme.colorScheme.surface.copy(alpha = 0.85f), RoundedCornerShape(20.dp))
            .padding(horizontal = 10.dp, vertical = 5.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            Icons.Filled.WifiOff,
            contentDescription = null,
            tint = tr.statusWarn,
            modifier = Modifier.padding(end = 6.dp),
        )
        Text(
            "OFFLINE",
            style = MaterialTheme.typography.labelSmall,
            color = tr.statusWarn,
        )
    }
}
