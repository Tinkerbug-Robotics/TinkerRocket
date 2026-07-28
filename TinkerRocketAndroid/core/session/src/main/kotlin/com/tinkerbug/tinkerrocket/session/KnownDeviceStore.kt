package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Commands
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import java.text.BreakIterator

/**
 * App-side registry of every physical device (rocket / base station) the app
 * has ever completed an identity readback with, keyed by the immutable
 * hardware unitID — a semantics-exact port of iOS `KnownDeviceStore.swift`
 * (#547).  Backs the "My Devices" management screen: names and network IDs
 * live ON the device (identity NVS, #150), so this registry is a cache of
 * the last readback plus a queue of edits made while the device was offline.
 * Pending edits are pushed on the next config_identity readback; the
 * firmware echoes a fresh readback after every identity-set command (cmds
 * 40/41/42), which folds device truth back into the record.
 *
 * Subsumes the legacy "knownDeviceIDs" UserDefaults array that only gated
 * the first-connect provisioning sheet — migrated once, then removed.
 */

/** Rocket / base-station discriminator (iOS BLEDeviceType, raw "R"/"B"/"?"). */
public enum class BleDeviceType(public val raw: String) {
    ROCKET("R"),
    BASE_STATION("B"),
    UNKNOWN("?");

    public companion object {
        public fun fromRaw(raw: String): BleDeviceType? = entries.firstOrNull { it.raw == raw }

        /**
         * Parse from BLE advertised name (e.g. "TR-R-Atlas" → ROCKET).
         *
         * Precedence at the scan site is: known-device registry match FIRST
         * (see [KnownDeviceStore.deviceTypeForAdvertisedName]), then the
         * TR-R-/TR-B- factory prefix, then the legacy substring heuristic.
         * Only factory-default names carry the prefix — a device renamed
         * through My Devices advertises its raw name and MUST come back
         * UNKNOWN here (the registry then recovers the type).  The scanner
         * must NEVER filter on this: the scan is service-UUID-gated (#547).
         *
         * Regression-pinned oddity, ported verbatim: the legacy substring
         * rules ("Base"/"BS" → base station, "Tinker" → rocket) misparse a
         * rocket named e.g. "SUBSONIC" as a base station ("BS" substring).
         * The registry hint outranks this at the scan site, so it only bites
         * fresh installs — do not "fix" it here without changing iOS too.
         */
        public fun fromName(name: String): BleDeviceType {
            if (name.startsWith("TR-R-")) return ROCKET
            if (name.startsWith("TR-B-")) return BASE_STATION
            // Legacy names
            if (name.contains("Base") || name.contains("BS")) return BASE_STATION
            if (name.contains("Tinker")) return ROCKET
            return UNKNOWN
        }
    }
}

/**
 * The three identity pushes + nothing else, so the store can be unit-tested
 * with a spy instead of a live BLE connection (same seam as the iOS
 * DeviceIdentityPusher protocol; the device session implements it).
 */
public interface DeviceIdentityPusher {
    public fun sendSetUnitName(name: String)
    public fun sendSetNetworkId(nid: Int)
    public fun sendSetRocketId(rid: Int)
}

/**
 * Persistence seam (iOS: UserDefaults).  [loadDevicesJson]/[saveDevicesJson]
 * carry the registry under the "knownDevices.v1" key; the legacy pair mirrors
 * the one-time "knownDeviceIDs" migration source.  The stored JSON schema is
 * IDENTICAL to the iOS Codable output — see [KnownDevice] — so a value synced
 * or restored across platforms round-trips.
 */
public interface KnownDeviceStorage {
    /** The JSON array previously passed to [saveDevicesJson], or null. */
    public fun loadDevicesJson(): String?

    public fun saveDevicesJson(json: String)

    /** The legacy "knownDeviceIDs" string array, or null when absent. */
    public fun loadLegacyKnownIds(): List<String>?

    public fun removeLegacyKnownIds()
}

/**
 * One physical device as last seen over BLE, plus queued offline edits.
 *
 * Stored-JSON schema (must stay byte-compatible with iOS JSONEncoder output
 * of the Swift struct): keys are the Swift property names — `unitID`, `name`,
 * `deviceType` (raw string "R"/"B"/"?"), `networkID`, `rocketID`,
 * `provisioned`, `lastSeen`, `pendingName`, `pendingNetworkID`,
 * `pendingRocketID`; null optionals are OMITTED.  `lastSeen` uses the Swift
 * default Date encoding: a Double of seconds since the reference date
 * 2001-01-01T00:00:00Z (epochSeconds − 978307200).
 */
public data class KnownDevice(
    val unitID: String,             // immutable hardware id — the key
    val name: String = "",          // device's own unit_name at last readback
    val deviceType: BleDeviceType = BleDeviceType.UNKNOWN,
    val networkID: Int = 0,         // u8; last readback nid (0 = factory/unset)
    val rocketID: Int = 0,          // u8; rockets only; 0 = unset
    val provisioned: Boolean = false, // user finished (or skipped) first-connect setup
    /** Swift-Date seconds since 2001-01-01T00:00:00Z; see [REFERENCE_DATE_EPOCH_SECONDS]. */
    val lastSeenRefSeconds: Double? = null,

    // Edits queued while the device was offline; cleared when pushed.
    val pendingName: String? = null,
    val pendingNetworkID: Int? = null,
    val pendingRocketID: Int? = null,
) {
    public val id: String get() = unitID

    public val hasPendingChanges: Boolean
        get() = pendingName != null || pendingNetworkID != null || pendingRocketID != null

    // Target state for display: what the device will be once pending applies.
    public val effectiveName: String get() = pendingName ?: name
    public val effectiveNetworkID: Int get() = pendingNetworkID ?: networkID
    public val effectiveRocketID: Int get() = pendingRocketID ?: rocketID

    /** [lastSeenRefSeconds] converted to Unix epoch milliseconds, for display. */
    public val lastSeenEpochMillis: Long?
        get() = lastSeenRefSeconds?.let {
            ((it + REFERENCE_DATE_EPOCH_SECONDS) * 1000.0).toLong()
        }

    public companion object {
        /** Unix epoch seconds of the Swift Date reference date (2001-01-01T00:00:00Z). */
        public const val REFERENCE_DATE_EPOCH_SECONDS: Long = 978_307_200L
    }
}

public class KnownDeviceStore(
    private val storage: KnownDeviceStorage,
    private val nowEpochMillis: () -> Long = System::currentTimeMillis,
) {
    /** Rockets first, then base stations, each sorted by display name. */
    private val _devices = MutableStateFlow<List<KnownDevice>>(emptyList())
    public val devices: StateFlow<List<KnownDevice>> = _devices.asStateFlow()

    init {
        load()
        migrateLegacyIfNeeded()
    }

    // MARK: - Lookup

    public fun device(unitID: String): KnownDevice? =
        _devices.value.firstOrNull { it.unitID == unitID }

    /**
     * Gates the first-connect provisioning sheet.  Empty unitID reads as
     * provisioned so the sheet never pops before the identity readback.
     */
    public fun isProvisioned(unitID: String): Boolean {
        if (unitID.isEmpty()) return true
        return device(unitID)?.provisioned ?: false
    }

    /**
     * Best-effort device type for a scan result, matched by advertised
     * name.  The firmware advertises the raw unit name, and renames happen
     * through this app, so a renamed device's adv name equals its record's
     * name.  null when there's no single unambiguous match (unknown name,
     * or two records sharing one).
     */
    public fun deviceTypeForAdvertisedName(name: String): BleDeviceType? {
        if (name.isEmpty()) return null
        val matches = _devices.value.filter {
            it.name == name && it.deviceType != BleDeviceType.UNKNOWN
        }
        if (matches.size != 1) return null
        return matches[0].deviceType
    }

    // MARK: - Readback intake

    /**
     * Fold a config_identity readback into the registry, then push any
     * pending offline edits.  Pending fields are cleared at push time
     * (fire-and-forget): the firmware echoes a fresh readback after each
     * identity-set command, so the record converges on device truth, and
     * clearing first makes the echo pass a plain upsert — no re-push loop
     * even if the firmware rejects or clamps a value.
     */
    public fun deviceDidReportIdentity(
        unitID: String,
        name: String,
        deviceType: BleDeviceType,
        networkID: Int,
        rocketID: Int,
        pusher: DeviceIdentityPusher?,
    ) {
        if (unitID.isEmpty()) return
        withRecord(unitID) { rec0 ->
            var rec = rec0.copy(
                name = name,
                // Keep a previously learned type if this readback doesn't know
                // it (pre-"dt" firmware) — a device never changes species.
                deviceType = if (deviceType != BleDeviceType.UNKNOWN) deviceType else rec0.deviceType,
                networkID = networkID,
                rocketID = rocketID,
                lastSeenRefSeconds =
                    nowEpochMillis() / 1000.0 - KnownDevice.REFERENCE_DATE_EPOCH_SECONDS,
            )
            rec = applyPending(
                rec, readback = name,
                getPending = { it.pendingName },
                clearPending = { it.copy(pendingName = null) },
                setCurrent = { r, v -> r.copy(name = v) },
                push = pusher?.let { p -> { v: String -> p.sendSetUnitName(v) } },
            )
            rec = applyPending(
                rec, readback = networkID,
                getPending = { it.pendingNetworkID },
                clearPending = { it.copy(pendingNetworkID = null) },
                setCurrent = { r, v -> r.copy(networkID = v) },
                push = pusher?.let { p -> { v: Int -> p.sendSetNetworkId(v) } },
            )
            rec = applyPending(
                rec, readback = rocketID,
                getPending = { it.pendingRocketID },
                clearPending = { it.copy(pendingRocketID = null) },
                setCurrent = { r, v -> r.copy(rocketID = v) },
                push = pusher?.let { p -> { v: Int -> p.sendSetRocketId(v) } },
            )
            rec
        }
    }

    /**
     * Apply one queued offline edit against a fresh readback.  The queue is
     * cleared BEFORE pushing so the firmware's echo readback runs this as a
     * plain upsert — no re-push loop even if the device rejects or clamps
     * the value.  A queued value the device already has is just dropped.
     */
    private fun <V> applyPending(
        rec: KnownDevice,
        readback: V,
        getPending: (KnownDevice) -> V?,
        clearPending: (KnownDevice) -> KnownDevice,
        setCurrent: (KnownDevice, V) -> KnownDevice,
        push: ((V) -> Unit)?,
    ): KnownDevice {
        val queued = getPending(rec) ?: return rec
        var out = clearPending(rec)
        if (queued == readback) return out
        out = setCurrent(out, queued)
        push?.invoke(queued)
        return out
    }

    /**
     * Called by the provisioning sheet on Save/Skip so the device stops
     * re-prompting.  Creates the record if the readback hasn't yet.
     */
    public fun markProvisioned(unitID: String) {
        if (unitID.isEmpty()) return
        withRecord(unitID) { it.copy(provisioned = true) }
    }

    // MARK: - Edits (push now when connected, queue when offline)

    /**
     * Rename.  With a live pusher the new name is sent immediately and the
     * record updated optimistically (the firmware's readback echo confirms);
     * offline it's queued for the next connect.  Clamped to the firmware's
     * 20-byte payload limit so the device can never silently reject it.
     */
    public fun setName(newName: String, unitID: String, pusher: DeviceIdentityPusher?) {
        val clamped = Commands.utf8Clamped(newName.trim(), maxBytes = 20)
        if (clamped.isEmpty() || unitID.isEmpty()) return
        applyEdit(
            clamped, unitID,
            getCurrent = { it.name },
            setCurrent = { r, v -> r.copy(name = v) },
            setPending = { r, v -> r.copy(pendingName = v) },
            push = pusher?.let { p -> { v: String -> p.sendSetUnitName(v) } },
        )
    }

    public fun setNetworkId(nid: Int, unitID: String, pusher: DeviceIdentityPusher?) {
        // 0 is the unset sentinel (#150); u8 range mirrors the iOS UInt8 type.
        if (nid !in 1..255 || unitID.isEmpty()) return
        applyEdit(
            nid, unitID,
            getCurrent = { it.networkID },
            setCurrent = { r, v -> r.copy(networkID = v) },
            setPending = { r, v -> r.copy(pendingNetworkID = v) },
            push = pusher?.let { p -> { v: Int -> p.sendSetNetworkId(v) } },
        )
    }

    public fun setRocketId(rid: Int, unitID: String, pusher: DeviceIdentityPusher?) {
        if (rid !in 1..254 || unitID.isEmpty()) return   // firmware rejects 0/255
        applyEdit(
            rid, unitID,
            getCurrent = { it.rocketID },
            setCurrent = { r, v -> r.copy(rocketID = v) },
            setPending = { r, v -> r.copy(pendingRocketID = v) },
            push = pusher?.let { p -> { v: Int -> p.sendSetRocketId(v) } },
        )
    }

    /**
     * Shared push-now-vs-queue tail for the setters above.  Connected:
     * push + optimistic record update + clear any stale queue.  Offline:
     * queue the target, where queueing the device's current value just
     * clears the queue (a changed-my-mind no-op).
     */
    private fun <V> applyEdit(
        value: V,
        unitID: String,
        getCurrent: (KnownDevice) -> V,
        setCurrent: (KnownDevice, V) -> KnownDevice,
        setPending: (KnownDevice, V?) -> KnownDevice,
        push: ((V) -> Unit)?,
    ) {
        withRecord(unitID) { rec ->
            if (push != null) {
                push(value)
                setPending(setCurrent(rec, value), null)
            } else {
                setPending(rec, if (value == getCurrent(rec)) null else value)
            }
        }
    }

    /** Drop all queued offline edits without touching the device. */
    public fun clearPending(unitID: String) {
        if (device(unitID)?.hasPendingChanges != true) return
        withRecord(unitID) {
            it.copy(pendingName = null, pendingNetworkID = null, pendingRocketID = null)
        }
    }

    /**
     * Drop the record entirely.  The device is treated as brand new on its
     * next connect (provisioning sheet pops again).
     */
    public fun forget(unitID: String) {
        _devices.value = _devices.value.filterNot { it.unitID == unitID }
        persist()
    }

    // MARK: - Record plumbing

    /** Get-or-create the record, transform it, then sort + persist + publish. */
    private fun withRecord(unitID: String, mutate: (KnownDevice) -> KnownDevice) {
        val rec = mutate(device(unitID) ?: KnownDevice(unitID = unitID))
        val rest = _devices.value.filterNot { it.unitID == unitID }
        _devices.value = sorted(rest + rec)
        persist()
    }

    private fun rank(t: BleDeviceType): Int = when (t) {
        BleDeviceType.ROCKET -> 0
        BleDeviceType.BASE_STATION -> 1
        BleDeviceType.UNKNOWN -> 2
    }

    // iOS uses localizedCaseInsensitiveCompare (locale collation); plain
    // case-insensitive ordering here — see docs/android-parity-ledger.md.
    private fun sorted(list: List<KnownDevice>): List<KnownDevice> =
        list.sortedWith(
            compareBy<KnownDevice> { rank(it.deviceType) }
                .thenBy(String.CASE_INSENSITIVE_ORDER) { it.effectiveName }
                .thenBy { it.unitID },
        )

    // MARK: - Persistence

    // iOS: JSONDecoder over the "knownDevices.v1" blob, wrapped in try? — ANY
    // malformed record (bad type on a strict field, missing unitID) discards
    // the WHOLE load, leaving an empty registry.  deviceType alone is
    // decoded tolerantly ((try? …) ?? .unknown).  Mirrored exactly.
    private fun load() {
        val text = storage.loadDevicesJson() ?: return
        val decoded = try {
            val arr = Json.parseToJsonElement(text) as? JsonArray ?: return
            arr.map { el -> decodeRecord(el as? JsonObject ?: throw MalformedRecord()) }
        } catch (_: Exception) {
            return
        }
        _devices.value = sorted(decoded)
    }

    private class MalformedRecord : RuntimeException(null, null, false, false)

    private fun decodeRecord(obj: JsonObject): KnownDevice = KnownDevice(
        unitID = reqString(obj, "unitID"),
        name = optString(obj, "name") ?: "",
        // Tolerate additive schema changes: unknown/garbage deviceType raw
        // degrades to UNKNOWN instead of failing the load.
        deviceType = tolerantDeviceType(obj),
        networkID = optU8(obj, "networkID") ?: 0,
        rocketID = optU8(obj, "rocketID") ?: 0,
        provisioned = optBool(obj, "provisioned") ?: false,
        lastSeenRefSeconds = optDouble(obj, "lastSeen"),
        pendingName = optString(obj, "pendingName"),
        pendingNetworkID = optU8(obj, "pendingNetworkID"),
        pendingRocketID = optU8(obj, "pendingRocketID"),
    )

    private fun prim(obj: JsonObject, key: String): JsonPrimitive? {
        val el = obj[key] ?: return null
        if (el is JsonNull) return null
        return el as? JsonPrimitive ?: throw MalformedRecord()
    }

    private fun reqString(obj: JsonObject, key: String): String {
        val p = prim(obj, key) ?: throw MalformedRecord()
        if (!p.isString) throw MalformedRecord()
        return p.content
    }

    private fun optString(obj: JsonObject, key: String): String? {
        val p = prim(obj, key) ?: return null
        if (!p.isString) throw MalformedRecord()
        return p.content
    }

    private fun optU8(obj: JsonObject, key: String): Int? {
        val p = prim(obj, key) ?: return null
        if (p.isString) throw MalformedRecord()
        val v = p.content.toLongOrNull() ?: throw MalformedRecord()
        if (v !in 0..255) throw MalformedRecord()   // UInt8 decode range
        return v.toInt()
    }

    private fun optBool(obj: JsonObject, key: String): Boolean? {
        val p = prim(obj, key) ?: return null
        return when {
            p.isString -> throw MalformedRecord()
            p.content == "true" -> true
            p.content == "false" -> false
            else -> throw MalformedRecord()
        }
    }

    private fun optDouble(obj: JsonObject, key: String): Double? {
        val p = prim(obj, key) ?: return null
        if (p.isString) throw MalformedRecord()
        return p.content.toDoubleOrNull() ?: throw MalformedRecord()
    }

    // Encoded shape mirrors Swift's synthesized Codable encode: property-name
    // keys, nil optionals omitted (encodeIfPresent), Date as reference-date
    // seconds Double.
    private fun persist() {
        val arr = buildJsonArray {
            for (d in _devices.value) {
                add(
                    buildJsonObject {
                        put("unitID", d.unitID)
                        put("name", d.name)
                        put("deviceType", d.deviceType.raw)
                        put("networkID", d.networkID)
                        put("rocketID", d.rocketID)
                        put("provisioned", d.provisioned)
                        d.lastSeenRefSeconds?.let { put("lastSeen", it) }
                        d.pendingName?.let { put("pendingName", it) }
                        d.pendingNetworkID?.let { put("pendingNetworkID", it) }
                        d.pendingRocketID?.let { put("pendingRocketID", it) }
                    },
                )
            }
        }
        storage.saveDevicesJson(arr.toString())
    }

    private fun tolerantDeviceType(obj: JsonObject): BleDeviceType {
        return try {
            val p = prim(obj, "deviceType") ?: return BleDeviceType.UNKNOWN
            if (!p.isString) return BleDeviceType.UNKNOWN
            BleDeviceType.fromRaw(p.content) ?: BleDeviceType.UNKNOWN
        } catch (_: MalformedRecord) {
            BleDeviceType.UNKNOWN
        }
    }

    /**
     * One-time fold of the legacy provisioning-gate array ("knownDeviceIDs",
     * bare unitIDs) into real records.  Names/types arrive on each device's
     * next readback.  The legacy key is deleted afterwards so a forgotten
     * device can't be resurrected by re-migration.
     */
    private fun migrateLegacyIfNeeded() {
        val legacy = storage.loadLegacyKnownIds() ?: return
        var list = _devices.value
        for (unitID in legacy) {
            if (unitID.isEmpty()) continue
            val idx = list.indexOfFirst { it.unitID == unitID }
            list = if (idx >= 0) {
                list.toMutableList().also { it[idx] = it[idx].copy(provisioned = true) }
            } else {
                list + KnownDevice(unitID = unitID, provisioned = true)
            }
        }
        _devices.value = sorted(list)
        persist()
        storage.removeLegacyKnownIds()
    }

}
