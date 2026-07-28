package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Registry semantics for the "My Devices" screen — port of the iOS
 * KnownDeviceStoreTests (every case): readback upsert, the offline
 * pending-edit queue and its push-on-next-connect behavior, the provisioning
 * gate, and the legacy knownDeviceIDs migration.  The UserDefaults seam is
 * replaced by an in-memory [KnownDeviceStorage] fake shared across
 * [makeStore] calls (= same defaults suite).
 */
class KnownDeviceStoreTest {

    /** In-memory stand-in for the per-test UserDefaults suite. */
    private class FakeStorage : KnownDeviceStorage {
        var devicesJson: String? = null
        var legacyIds: List<String>? = null
        override fun loadDevicesJson(): String? = devicesJson
        override fun saveDevicesJson(json: String) { devicesJson = json }
        override fun loadLegacyKnownIds(): List<String>? = legacyIds
        override fun removeLegacyKnownIds() { legacyIds = null }
    }

    /** Records pushes instead of writing to a live BLE connection. */
    private class PusherSpy : DeviceIdentityPusher {
        val names = mutableListOf<String>()
        val nids = mutableListOf<Int>()
        val rids = mutableListOf<Int>()
        override fun sendSetUnitName(name: String) { names.add(name) }
        override fun sendSetNetworkId(nid: Int) { nids.add(nid) }
        override fun sendSetRocketId(rid: Int) { rids.add(rid) }
    }

    private val storage = FakeStorage()

    private fun makeStore(): KnownDeviceStore = KnownDeviceStore(storage)

    private fun deviceList(store: KnownDeviceStore): List<KnownDevice> = store.devices.value

    /** Simulate a config_identity readback for a rocket. */
    private fun report(
        store: KnownDeviceStore,
        unitID: String = "a1b2c3d4",
        name: String = "Atlas",
        type: BleDeviceType = BleDeviceType.ROCKET,
        nid: Int = 42,
        rid: Int = 3,
        pusher: DeviceIdentityPusher? = null,
    ) {
        store.deviceDidReportIdentity(
            unitID = unitID, name = name, deviceType = type,
            networkID = nid, rocketID = rid, pusher = pusher,
        )
    }

    // MARK: - Readback intake

    @Test
    fun `readback creates unprovisioned record`() {
        val store = makeStore()
        report(store)

        val rec = assertNotNull(store.device("a1b2c3d4"))
        assertEquals("Atlas", rec.name)
        assertEquals(BleDeviceType.ROCKET, rec.deviceType)
        assertEquals(42, rec.networkID)
        assertEquals(3, rec.rocketID)
        assertNotNull(rec.lastSeenRefSeconds)
        // A readback alone must NOT mark the device provisioned — that would
        // stop the first-connect sheet from ever appearing.
        assertFalse(rec.provisioned)
        assertFalse(store.isProvisioned("a1b2c3d4"))
    }

    @Test
    fun `readback refreshes existing record`() {
        val store = makeStore()
        report(store, name = "Atlas", nid = 42)
        report(store, name = "Renamed On Other Phone", nid = 77)

        val rec = store.device("a1b2c3d4")
        assertEquals(1, deviceList(store).size)
        assertEquals("Renamed On Other Phone", rec?.name)
        assertEquals(77, rec?.networkID)
    }

    @Test
    fun `empty unitID is ignored`() {
        val store = makeStore()
        report(store, unitID = "")
        assertTrue(deviceList(store).isEmpty())
        // Empty id reads as provisioned so the sheet can't pop pre-readback.
        assertTrue(store.isProvisioned(""))
    }

    @Test
    fun `unknown type readback keeps learned type`() {
        // A device never changes species — a readback without "dt" (pre-dt
        // firmware) must not downgrade a learned type to unknown.
        val store = makeStore()
        report(store, type = BleDeviceType.ROCKET)
        report(store, type = BleDeviceType.UNKNOWN)
        assertEquals(BleDeviceType.ROCKET, store.device("a1b2c3d4")?.deviceType)
    }

    // MARK: - Provisioning gate

    @Test
    fun `mark provisioned`() {
        val store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        assertTrue(store.isProvisioned("a1b2c3d4"))
        // Survives a reload.
        assertTrue(makeStore().isProvisioned("a1b2c3d4"))
    }

    @Test
    fun `forget removes and reprompts`() {
        val store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        store.forget("a1b2c3d4")
        assertNull(store.device("a1b2c3d4"))
        // Next connect treats it as brand new.
        assertFalse(store.isProvisioned("a1b2c3d4"))
        assertFalse(makeStore().isProvisioned("a1b2c3d4"))
    }

    // MARK: - Connected edits (push immediately)

    @Test
    fun `connected rename pushes and updates`() {
        val store = makeStore()
        report(store)
        val spy = PusherSpy()

        store.setName("  Ares 2  ", unitID = "a1b2c3d4", pusher = spy)

        assertEquals(listOf("Ares 2"), spy.names)   // trimmed
        val rec = store.device("a1b2c3d4")
        assertEquals("Ares 2", rec?.name)
        assertNull(rec?.pendingName)
    }

    @Test
    fun `connected network and rocket id push`() {
        val store = makeStore()
        report(store)
        val spy = PusherSpy()

        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = spy)
        store.setRocketId(7, unitID = "a1b2c3d4", pusher = spy)

        assertEquals(listOf(99), spy.nids)
        assertEquals(listOf(7), spy.rids)
        val rec = store.device("a1b2c3d4")
        assertEquals(99, rec?.networkID)
        assertEquals(7, rec?.rocketID)
        assertFalse(rec?.hasPendingChanges ?: true)
    }

    @Test
    fun `invalid values rejected`() {
        val store = makeStore()
        report(store)
        val spy = PusherSpy()

        store.setName("   ", unitID = "a1b2c3d4", pusher = spy)
        store.setNetworkId(0, unitID = "a1b2c3d4", pusher = spy)   // 0 = unset sentinel
        store.setRocketId(0, unitID = "a1b2c3d4", pusher = spy)    // firmware rejects
        store.setRocketId(255, unitID = "a1b2c3d4", pusher = spy)  // firmware rejects

        assertTrue(spy.names.isEmpty())
        assertTrue(spy.nids.isEmpty())
        assertTrue(spy.rids.isEmpty())
        assertEquals("Atlas", store.device("a1b2c3d4")?.name)
    }

    @Test
    fun `name clamped to 20 bytes`() {
        val store = makeStore()
        report(store)
        val spy = PusherSpy()
        store.setName("x".repeat(30), unitID = "a1b2c3d4", pusher = spy)
        assertEquals(listOf("x".repeat(20)), spy.names)

        // The firmware limit is 20 UTF-8 BYTES, not characters — it rejects
        // longer payloads silently (no write, no echo), so the clamp must
        // count bytes and never split a character.  🚀 is 4 bytes.
        store.setName("🚀".repeat(10), unitID = "a1b2c3d4", pusher = spy)
        assertEquals("🚀".repeat(5), spy.names.last())
        assertTrue(spy.names.last().toByteArray(Charsets.UTF_8).size <= 20)
    }

    // MARK: - Offline edits (queue, then apply on next readback)

    @Test
    fun `offline edits queue as pending`() {
        val store = makeStore()
        report(store)

        store.setName("Ares 2", unitID = "a1b2c3d4", pusher = null)
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)
        store.setRocketId(7, unitID = "a1b2c3d4", pusher = null)

        val rec = assertNotNull(store.device("a1b2c3d4"))
        // Current values untouched; targets queued.
        assertEquals("Atlas", rec.name)
        assertEquals("Ares 2", rec.pendingName)
        assertEquals(99, rec.pendingNetworkID)
        assertEquals(7, rec.pendingRocketID)
        assertTrue(rec.hasPendingChanges)
        // Display prefers the target state.
        assertEquals("Ares 2", rec.effectiveName)
        assertEquals(99, rec.effectiveNetworkID)
        assertEquals(7, rec.effectiveRocketID)
    }

    @Test
    fun `clear pending drops queue only`() {
        val store = makeStore()
        report(store)
        store.setName("Ares 2", unitID = "a1b2c3d4", pusher = null)
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)

        store.clearPending("a1b2c3d4")

        val rec = assertNotNull(store.device("a1b2c3d4"))
        assertFalse(rec.hasPendingChanges)
        assertEquals("Atlas", rec.name)       // current values untouched
        assertEquals(42, rec.networkID)
    }

    @Test
    fun `offline edit matching current clears pending`() {
        val store = makeStore()
        report(store, name = "Atlas", nid = 42)
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)
        // User changes their mind back to the device's actual value.
        store.setNetworkId(42, unitID = "a1b2c3d4", pusher = null)
        assertFalse(store.device("a1b2c3d4")?.hasPendingChanges ?: true)
    }

    @Test
    fun `pending applied on next readback`() {
        val store = makeStore()
        report(store)
        store.setName("Ares 2", unitID = "a1b2c3d4", pusher = null)
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)

        // Device reconnects and reports its (old) identity.
        val spy = PusherSpy()
        report(store, name = "Atlas", nid = 42, pusher = spy)

        assertEquals(listOf("Ares 2"), spy.names)
        assertEquals(listOf(99), spy.nids)
        assertTrue(spy.rids.isEmpty())   // nothing queued for rid
        val rec = assertNotNull(store.device("a1b2c3d4"))
        assertFalse(rec.hasPendingChanges)
        assertEquals("Ares 2", rec.name)       // optimistic until the echo
        assertEquals(99, rec.networkID)
    }

    @Test
    fun `pending matching readback clears without push`() {
        // The device already has the queued value (e.g. set from another
        // phone) — don't push a redundant write, just clear the queue.
        val store = makeStore()
        report(store, nid = 42)
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)

        val spy = PusherSpy()
        report(store, nid = 99, pusher = spy)

        assertTrue(spy.nids.isEmpty())
        val rec = assertNotNull(store.device("a1b2c3d4"))
        assertFalse(rec.hasPendingChanges)
        assertEquals(99, rec.networkID)
    }

    @Test
    fun `echo readback after push is plain upsert`() {
        // After a pending push, the firmware echoes config_identity with the
        // new values.  Pending was cleared at push time, so the echo must not
        // re-push (no loop) and must land the confirmed values.
        val store = makeStore()
        report(store)
        store.setName("Ares 2", unitID = "a1b2c3d4", pusher = null)

        val spy = PusherSpy()
        report(store, name = "Atlas", pusher = spy)          // reconnect → push
        report(store, name = "Ares 2", pusher = spy)         // firmware echo

        assertEquals(listOf("Ares 2"), spy.names)            // exactly one push
        assertEquals("Ares 2", store.device("a1b2c3d4")?.name)
    }

    // MARK: - Persistence + migration

    @Test
    fun `persistence round trip`() {
        val store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        store.setNetworkId(99, unitID = "a1b2c3d4", pusher = null)

        val reloaded = makeStore()
        val rec = assertNotNull(reloaded.device("a1b2c3d4"))
        assertEquals("Atlas", rec.name)
        assertEquals(BleDeviceType.ROCKET, rec.deviceType)
        assertTrue(rec.provisioned)
        assertEquals(99, rec.pendingNetworkID)   // queue survives relaunch
    }

    @Test
    fun `legacy knownDeviceIDs migration`() {
        storage.legacyIds = listOf("aaaa1111", "bbbb2222")

        val store = makeStore()

        // Legacy-known devices must not re-trigger the provisioning sheet.
        assertTrue(store.isProvisioned("aaaa1111"))
        assertTrue(store.isProvisioned("bbbb2222"))
        // Names/types are unknown until the next readback.
        assertEquals("", store.device("aaaa1111")?.name)
        // Source key is consumed so forget() can't be undone by re-migration.
        assertNull(storage.legacyIds)

        store.forget("aaaa1111")
        assertFalse(makeStore().isProvisioned("aaaa1111"))
    }

    @Test
    fun `migration merges with existing records`() {
        // Migration must not clobber a record that already exists (e.g. the
        // readback landed before the legacy array was folded in).
        val store = makeStore()
        report(store)
        storage.legacyIds = listOf("a1b2c3d4")

        val reloaded = makeStore()
        assertEquals(1, deviceList(reloaded).size)
        assertEquals("Atlas", reloaded.device("a1b2c3d4")?.name)
        // The legacy array only ever held provisioned devices — merging must
        // carry that over to the surviving record.
        assertTrue(reloaded.isProvisioned("a1b2c3d4"))
    }

    // MARK: - Scan-time type recovery (renamed devices lose the TR- prefix)

    @Test
    fun `device type for advertised name`() {
        // The firmware advertises the raw unit name, so a device renamed to
        // "Atlas" no longer carries the TR-R- prefix the name heuristic
        // needs — the registry is the only thing that still knows its type.
        val store = makeStore()
        report(store, unitID = "r1", name = "Atlas", type = BleDeviceType.ROCKET)
        report(store, unitID = "bs1", name = "Pad Station", type = BleDeviceType.BASE_STATION)

        assertEquals(BleDeviceType.ROCKET, store.deviceTypeForAdvertisedName("Atlas"))
        assertEquals(BleDeviceType.BASE_STATION, store.deviceTypeForAdvertisedName("Pad Station"))
        assertNull(store.deviceTypeForAdvertisedName("TR-R-1a2b"))  // not in registry
        assertNull(store.deviceTypeForAdvertisedName(""))
    }

    @Test
    fun `device type for advertised name - ambiguous or unknown gives null`() {
        val store = makeStore()
        // Two devices sharing a name: no unambiguous answer.
        report(store, unitID = "r1", name = "Atlas", type = BleDeviceType.ROCKET)
        report(store, unitID = "bs1", name = "Atlas", type = BleDeviceType.BASE_STATION)
        assertNull(store.deviceTypeForAdvertisedName("Atlas"))

        // A record whose own type is unknown can't type a scan result.
        report(store, unitID = "x1", name = "Mystery", type = BleDeviceType.UNKNOWN)
        assertNull(store.deviceTypeForAdvertisedName("Mystery"))
    }

    // MARK: - Ordering

    @Test
    fun `rockets sort before base stations`() {
        val store = makeStore()
        report(store, unitID = "bs1", name = "Pad Station", type = BleDeviceType.BASE_STATION)
        report(store, unitID = "r2", name = "Zephyr", type = BleDeviceType.ROCKET)
        report(store, unitID = "r1", name = "Atlas", type = BleDeviceType.ROCKET)

        assertEquals(
            listOf("Atlas", "Zephyr", "Pad Station"),
            deviceList(store).map { it.name },
        )
    }
}

/**
 * Pins the BLE-name → device-type heuristic — port of the iOS
 * DeviceTypeFromNameTests.  Only factory-default names carry the
 * TR-R-/TR-B- prefix — a device renamed through My Devices advertises its
 * raw name and MUST come back UNKNOWN here (the scan path then recovers the
 * type from the known-device registry).  The front-page scanner must never
 * filter on this: the scan is service-UUID-gated, and the old TR-* / Tinker
 * name guard made renamed devices invisible.
 */
class DeviceTypeFromNameTest {

    @Test
    fun `factory default prefixes`() {
        assertEquals(BleDeviceType.ROCKET, BleDeviceType.fromName("TR-R-1a2b"))
        assertEquals(BleDeviceType.BASE_STATION, BleDeviceType.fromName("TR-B-3c4d"))
    }

    @Test
    fun `legacy names`() {
        assertEquals(BleDeviceType.ROCKET, BleDeviceType.fromName("TinkerRocket"))
        assertEquals(BleDeviceType.BASE_STATION, BleDeviceType.fromName("TinkerBaseStation"))
    }

    @Test
    fun `renamed devices are unknown not mistyped`() {
        // Raw user names carry no type information — the heuristic must
        // say so rather than guess, so the registry hint can take over.
        assertEquals(BleDeviceType.UNKNOWN, BleDeviceType.fromName("Atlas"))
        assertEquals(BleDeviceType.UNKNOWN, BleDeviceType.fromName("RollyPolly III"))
    }

    @Test
    fun `legacy substring misparse pinned - SUBSONIC reads as base station`() {
        // NOT in the iOS test file, but pins the ported-verbatim oddity: the
        // legacy "BS" substring rule misreads a rocket named "SUBSONIC" as a
        // base station.  The registry hint outranks this at the scan site;
        // if iOS ever fixes the heuristic, change both together.
        assertEquals(BleDeviceType.BASE_STATION, BleDeviceType.fromName("SUBSONIC"))
    }
}
