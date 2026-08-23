package com.tinkerbug.tinkerrocket.app

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.IBinder
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch

/**
 * Foreground service holding the process (and its BLE links) alive while any
 * device is connected — the Android replacement for iOS CoreBluetooth state
 * restoration (android-port plan §4/§6).  `connectedDevice` type keeps it
 * exempt from most Doze restriction while a link is up; the notification is
 * the user-visible session surface.
 *
 * Lifecycle: AppContainer starts it from a process-scoped collector on
 * `fleet.linkActive` (BLUETOOTH_CONNECT is necessarily granted by then — the
 * FGS start-ordering constraint), and it stops itself when that goes false.
 * The fleet itself lives in AppContainer (process scope); this service only
 * pins the process.
 *
 * #829: it used to stop when `fleet.devices` emptied, and be started only by
 * a Compose LaunchedEffect.  Both halves were wrong.  handleDisconnect empties
 * the map BEFORE the reconnect ladder starts, so every transient drop stopped
 * the service; and the Compose starter could not restore it, because while the
 * activity is stopped the Recomposer's frame clock is paused, and by the time
 * it resumes the ladder has refilled the map — so the effect's key reads the
 * same as it did before and never re-runs.  The service stayed dead for the
 * rest of the session, leaving a cached, reclaimable process holding the GATT
 * links mid-flight, with START_NOT_STICKY meaning it would not come back.
 *
 * `linkActive` stays true across the drop-then-reconnect window, so the
 * service now simply never stops there.
 */
class FleetService : Service() {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)
    private var collecting = false

    override fun onCreate() {
        super.onCreate()
        val channel = NotificationChannel(
            CHANNEL_ID, "Active connection", NotificationManager.IMPORTANCE_LOW,
        )
        getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        startForeground(
            NOTIFICATION_ID,
            buildNotification("Connected"),
            ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE,
        )
        // A redelivered start must not stack a second pair of collectors on
        // the same fleet — startForeground above is idempotent, this is not.
        if (!collecting) {
            collecting = true
            val fleet = (application as TinkerRocketApp).container.fleet
            scope.launch {
                fleet.linkActive.collect { active ->
                    if (!active) stopSelf()
                }
            }
            scope.launch {
                fleet.devices.collect { devices ->
                    // Empty here does NOT mean "done" any more — it means a
                    // reconnect ladder is running (#829). Say so, rather than
                    // leaving the last device's name up as if nothing happened.
                    val text = if (devices.isEmpty()) {
                        "Reconnecting…"
                    } else {
                        devices.values.joinToString { it.session.displayName }
                    }
                    getSystemService(NotificationManager::class.java)
                        .notify(NOTIFICATION_ID, buildNotification(text))
                }
            }
        }
        return START_NOT_STICKY   // accept process death; recover by rescan (plan §6)
    }

    private fun buildNotification(text: String): Notification =
        Notification.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.stat_sys_data_bluetooth)
            .setContentTitle("TinkerRocket")
            .setContentText(text)
            .setOngoing(true)
            .build()

    override fun onDestroy() {
        scope.cancel()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    companion object {
        private const val CHANNEL_ID = "fleet"
        private const val NOTIFICATION_ID = 1

        /**
         * Safe to call when already running — a repeat start just re-enters
         * onStartCommand.  Wrapped because a background start is refused on
         * API 31+ (ForegroundServiceStartNotAllowedException): in practice the
         * first connect is user-initiated so the app is visible, but a resume
         * path could in principle land while backgrounded, and losing the
         * process pin is not worth crashing over.
         */
        fun start(context: Context) {
            runCatching {
                context.startForegroundService(Intent(context, FleetService::class.java))
            }.onFailure {
                Log.w("FleetService", "foreground start refused: $it")
            }
        }
    }
}
