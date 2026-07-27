package com.tinkerbug.tinkerrocket.app

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.IBinder
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
 * Lifecycle: AppContainer starts it on the first connect (BLUETOOTH_CONNECT
 * is necessarily granted by then — the FGS start-ordering constraint) and it
 * stops itself when the fleet empties.  The fleet itself lives in
 * AppContainer (process scope); this service only pins the process.
 */
class FleetService : Service() {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)

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
        val fleet = (application as TinkerRocketApp).container.fleet
        scope.launch {
            fleet.devices.collect { devices ->
                if (devices.isEmpty()) {
                    stopSelf()
                } else {
                    val names = devices.values.joinToString { it.session.displayName }
                    getSystemService(NotificationManager::class.java)
                        .notify(NOTIFICATION_ID, buildNotification(names))
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

        fun start(context: Context) {
            context.startForegroundService(Intent(context, FleetService::class.java))
        }
    }
}
