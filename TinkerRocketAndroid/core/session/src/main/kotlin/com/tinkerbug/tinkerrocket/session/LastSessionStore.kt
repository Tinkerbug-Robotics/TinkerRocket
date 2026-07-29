package com.tinkerbug.tinkerrocket.session

/**
 * Remembers which device the app was connected to when it last went away, so a
 * cold start can resume it (#633).
 *
 * Android has no equivalent of CoreBluetooth state restoration: iOS gets its
 * connections handed back by the OS after a relaunch and additionally re-scans
 * on Bluetooth power-on (`BLEFleet.swift:397`). Android wakes up knowing
 * nothing, so the app has to remember for itself.
 *
 * Deliberately just an id and a display name, not a whole session: the point is
 * only to know *what to look for*. The reconnect itself goes through the normal
 * sighting-gated path, which never autoConnects blind from a MAC.
 */
public interface LastSessionStore {

    /** Called on every successful connect. */
    public fun saveLastConnected(deviceId: String, name: String)

    /** Called ONLY on an explicit user disconnect — see [FleetManager]. */
    public fun clearLastConnected()

    /** The last connected device, or null if there is nothing to resume. */
    public fun loadLastConnected(): LastConnected?

    public data class LastConnected(val deviceId: String, val name: String)

    /**
     * Non-persistent default, so existing constructions (tests, composition
     * fakes) keep working unchanged and simply never resume across a restart.
     */
    public class InMemory : LastSessionStore {
        private var value: LastConnected? = null
        override fun saveLastConnected(deviceId: String, name: String) {
            value = LastConnected(deviceId, name)
        }
        override fun clearLastConnected() {
            value = null
        }
        override fun loadLastConnected(): LastConnected? = value
    }
}
