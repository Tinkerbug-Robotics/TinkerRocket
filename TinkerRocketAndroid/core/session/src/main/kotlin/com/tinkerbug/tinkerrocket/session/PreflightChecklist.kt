package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import java.util.UUID

/**
 * Pre-flight checklist data model — port of iOS PreflightChecklist.swift.
 *
 * One MASTER checklist (the live template every rocket starts from) plus a
 * per-rocket config that stores only the diff: excluded master ids, extra
 * rocket-specific items, and the checked state of the current run.  Editing
 * a master step updates it on every rocket that includes it.
 *
 * MANUAL steps are hand-checked; AUTO steps are verified from telemetry /
 * sync state and can never be hand-checked — the list only completes when
 * the app has observed each condition.
 *
 * Stored JSON is SCHEMA-IDENTICAL to iOS Codable output (key names,
 * Apple-epoch dates as Double seconds, uppercase UUID strings) so
 * checklists could round-trip across platforms — the RocketProfile contract.
 */
public enum class PreflightItemKind(public val wire: String) {
    MANUAL("manual"),

    // Auto steps — verified from telemetry / app state.
    CONNECTED("connected"),
    SETTINGS_SYNCED("settingsSynced"),
    GNSS_FIX("gnssFix"),
    CAMERA_RECORDING("cameraRecording"),
    LOGGING_ACTIVE("loggingActive"),
    PYRO_ARMED("pyroArmed"),
    PYRO_CONTINUITY("pyroContinuity"),
    ;

    public val isAuto: Boolean get() = this != MANUAL

    /** Default title when added from the auto library (label stays editable). */
    public val defaultTitle: String
        get() = when (this) {
            MANUAL -> ""
            CONNECTED -> "Rocket powered on & connected"
            SETTINGS_SYNCED -> "Settings applied to rocket"
            GNSS_FIX -> "GNSS fix acquired"
            CAMERA_RECORDING -> "Camera recording"
            LOGGING_ACTIVE -> "Flight logging active"
            PYRO_ARMED -> "Pyro channels armed"
            PYRO_CONTINUITY -> "Pyro continuity good"
        }

    public val defaultDetail: String
        get() = when (this) {
            MANUAL -> ""
            CONNECTED -> "Verified from the live telemetry link."
            SETTINGS_SYNCED -> "The active rocket profile has been pushed and confirmed."
            GNSS_FIX -> "Position fix with enough satellites for a 3D solution."
            CAMERA_RECORDING -> "The flight computer reports the camera is recording."
            LOGGING_ACTIVE -> "The flight computer is writing the flight log."
            PYRO_ARMED -> "The shared ARM FET is on."
            PYRO_CONTINUITY -> "Every channel enabled in this rocket's profile shows " +
                "continuity. Requires the flight battery — USB power alone fakes " +
                "continuity on all channels."
        }

    public companion object {
        /**
         * An auto kind added by a NEWER app decodes as MANUAL here rather
         * than failing — the step stays visible and checkable instead of
         * vanishing (iOS does the same).
         */
        public fun fromWire(s: String?): PreflightItemKind =
            entries.firstOrNull { it.wire == s } ?: MANUAL
    }
}

/** One checklist step; `id` keys the checked state, so it must be stable. */
public data class PreflightItem(
    val id: UUID = UUID.randomUUID(),
    val title: String,
    val detail: String = "",
    val kind: PreflightItemKind = PreflightItemKind.MANUAL,
) {
    public companion object {
        /** A step seeded from the auto library. */
        public fun auto(kind: PreflightItemKind): PreflightItem =
            PreflightItem(title = kind.defaultTitle, detail = kind.defaultDetail, kind = kind)
    }
}

/** The master checklist: the ordered template every rocket starts from. */
public data class PreflightMaster(
    val items: List<PreflightItem> = emptyList(),
    val updatedAtMs: Long,
)

/**
 * A rocket's diff against the master plus the current run's checked state.
 * `checked` maps item id (uppercase UUID string) to when it was checked
 * (epoch ms); only manual steps ever appear in it.
 *
 * [orderedIds] is this rocket's step order, set the first time the operator
 * reorders its list.  Empty = the default order (master order, extras
 * appended).  Later-added steps append after the custom-ordered block and
 * stale ids are skipped — see [PreflightChecklist.applyOrder].
 */
public data class PreflightRocketConfig(
    val profileId: UUID,
    val disabledMasterIds: List<UUID> = emptyList(),
    val extraItems: List<PreflightItem> = emptyList(),
    val orderedIds: List<UUID> = emptyList(),
    val checked: Map<String, Long> = emptyMap(),
    val updatedAtMs: Long,
) {
    public fun isChecked(itemId: UUID): Boolean =
        checked.containsKey(itemId.toString().uppercase())
}

/** Status of one AUTO step, evaluated from live app state. */
public sealed interface PreflightAutoStatus {
    public data object Satisfied : PreflightAutoStatus

    /** Condition not yet observed; [reason] says what's missing. */
    public data class Pending(val reason: String) : PreflightAutoStatus

    /**
     * The step doesn't apply to this rocket (e.g. no camera configured).
     * Counts as done — an inapplicable step must not block completion.
     */
    public data class NotApplicable(val reason: String) : PreflightAutoStatus

    public val countsAsDone: Boolean get() = this !is Pending
}

/** Everything the auto evaluation can see — pure-testable, no BLE stack. */
public data class PreflightAutoContext(
    val isConnected: Boolean = false,
    val hasTelemetry: Boolean = false,
    /**
     * Base-station relay link: some signals never ride LoRa (settings sync
     * is a direct-BLE flow), so the affected steps go N/A, not pending-forever.
     */
    val isRelay: Boolean = false,
    val telemetry: TelemetryData = TelemetryData(),
    val syncState: ActiveRocketSyncer.SyncState = ActiveRocketSyncer.SyncState.Idle,
    val profile: RocketProfile? = null,
)

public data class PreflightProgress(val done: Int = 0, val total: Int = 0) {
    public val isComplete: Boolean get() = total > 0 && done == total
}

public object PreflightChecklist {

    /** iOS LastValidRocketFix.minSatsForValidFix — the 3D-solution floor. */
    public const val MIN_SATS_FOR_FIX: Int = 4

    /**
     * A rocket's effective checklist: master items minus its exclusions,
     * plus its extras — in the rocket's custom order when it has one, else
     * master order with extras appended.
     */
    public fun effectiveItems(
        master: PreflightMaster,
        config: PreflightRocketConfig?,
    ): List<PreflightItem> {
        if (config == null) return master.items
        val disabled = config.disabledMasterIds.toSet()
        val base = master.items.filterNot { it.id in disabled } + config.extraItems
        return applyOrder(base, config.orderedIds)
    }

    /**
     * Sort [items] by their position in [orderedIds].  Ids not listed
     * (steps added after the order was saved) append after the ordered
     * block in their base order; listed ids with no matching item (deleted
     * or excluded steps) are skipped.  An empty order is the identity.
     */
    public fun applyOrder(
        items: List<PreflightItem>,
        orderedIds: List<UUID>,
    ): List<PreflightItem> {
        if (orderedIds.isEmpty()) return items
        val position = HashMap<UUID, Int>()
        orderedIds.forEachIndexed { idx, id -> position.putIfAbsent(id, idx) }
        // Partition rather than sort: base order must be PRESERVED for the
        // unlisted tail (iOS twin does the same).
        val (listed, unlisted) = items.partition { it.id in position }
        return listed.sortedBy { position[it.id]!! } + unlisted
    }

    /** Live status of one auto step; null for manual steps. */
    public fun autoStatus(
        kind: PreflightItemKind,
        ctx: PreflightAutoContext,
    ): PreflightAutoStatus? {
        if (!kind.isAuto) return null

        // Everything below reads telemetry — no link, nothing to verify against.
        if (!ctx.isConnected) return PreflightAutoStatus.Pending("Not connected")

        return when (kind) {
            PreflightItemKind.MANUAL -> null

            PreflightItemKind.CONNECTED ->
                if (ctx.hasTelemetry) PreflightAutoStatus.Satisfied
                else PreflightAutoStatus.Pending("Waiting for telemetry")

            PreflightItemKind.SETTINGS_SYNCED -> when {
                ctx.isRelay -> PreflightAutoStatus.NotApplicable("Needs a direct connection")
                ctx.syncState is ActiveRocketSyncer.SyncState.Synced ->
                    PreflightAutoStatus.Satisfied
                ctx.syncState is ActiveRocketSyncer.SyncState.NoProfile ->
                    PreflightAutoStatus.Pending("No active rocket selected")
                ctx.syncState is ActiveRocketSyncer.SyncState.Failed ->
                    PreflightAutoStatus.Pending(
                        (ctx.syncState as ActiveRocketSyncer.SyncState.Failed).message,
                    )
                else -> PreflightAutoStatus.Pending("Not yet applied")
            }

            PreflightItemKind.GNSS_FIX -> {
                if (!ctx.hasTelemetry) return PreflightAutoStatus.Pending("Waiting for telemetry")
                val t = ctx.telemetry
                val lat = t.latitude
                val lon = t.longitude
                // Same usability bar as the map's latched fix (LastValidRocketFix):
                // a real position and enough satellites for a 3D solution.
                if (lat != null && lon != null && !(lat == 0.0 && lon == 0.0) &&
                    t.numSats >= MIN_SATS_FOR_FIX
                ) {
                    PreflightAutoStatus.Satisfied
                } else {
                    PreflightAutoStatus.Pending("No fix (${t.numSats} sats)")
                }
            }

            PreflightItemKind.CAMERA_RECORDING -> {
                val p = ctx.profile
                if (p != null && p.cameraType == 0) {
                    return PreflightAutoStatus.NotApplicable("No camera on this rocket")
                }
                if (!ctx.hasTelemetry) return PreflightAutoStatus.Pending("Waiting for telemetry")
                if (ctx.telemetry.cameraRecording) PreflightAutoStatus.Satisfied
                else PreflightAutoStatus.Pending("Not recording")
            }

            PreflightItemKind.LOGGING_ACTIVE -> {
                if (!ctx.hasTelemetry) return PreflightAutoStatus.Pending("Waiting for telemetry")
                if (ctx.telemetry.loggingActive) PreflightAutoStatus.Satisfied
                else PreflightAutoStatus.Pending("Not logging")
            }

            PreflightItemKind.PYRO_ARMED -> {
                val channels = ctx.profile?.let(::enabledPyroChannels).orEmpty()
                if (channels.isEmpty()) {
                    return PreflightAutoStatus.NotApplicable("No pyro channels enabled")
                }
                if (!ctx.hasTelemetry) return PreflightAutoStatus.Pending("Waiting for telemetry")
                if (ctx.telemetry.pyroArmed) PreflightAutoStatus.Satisfied
                else PreflightAutoStatus.Pending("Not armed")
            }

            PreflightItemKind.PYRO_CONTINUITY -> {
                val channels = ctx.profile?.let(::enabledPyroChannels).orEmpty()
                if (channels.isEmpty()) {
                    return PreflightAutoStatus.NotApplicable("No pyro channels enabled")
                }
                if (!ctx.hasTelemetry) return PreflightAutoStatus.Pending("Waiting for telemetry")
                val open = channels.filterNot { ctx.telemetry.pyroCont(it) }
                if (open.isEmpty()) PreflightAutoStatus.Satisfied
                else PreflightAutoStatus.Pending("Ch ${open.joinToString(", ")} open")
            }
        }
    }

    /**
     * Rollup: manual steps count when checked, auto steps when satisfied or
     * N/A.  An empty list is never "complete" — no checklist, no badge.
     */
    public fun progress(
        items: List<PreflightItem>,
        config: PreflightRocketConfig?,
        ctx: PreflightAutoContext,
    ): PreflightProgress {
        var done = 0
        for (item in items) {
            val status = autoStatus(item.kind, ctx)
            if (status != null) {
                if (status.countsAsDone) done++
            } else if (config?.isChecked(item.id) == true) {
                done++
            }
        }
        return PreflightProgress(done = done, total = items.size)
    }

    /** Pyro channels (1–4) enabled in the profile. */
    public fun enabledPyroChannels(p: RocketProfile): List<Int> = buildList {
        if (p.pyro1Enabled) add(1)
        if (p.pyro2Enabled) add(2)
        if (p.pyro3Enabled) add(3)
        if (p.pyro4Enabled) add(4)
    }
}

/**
 * iOS-schema-identical JSON codec.  Decode-with-defaults for every field
 * (a missing key never fails the file) with the RocketProfileCodec date
 * convention: Apple-epoch seconds as Double, uppercase UUID strings.
 */
public object PreflightCodec {

    private const val APPLE_EPOCH_OFFSET_S = 978307200.0  // 2001-01-01T00:00:00Z

    private fun appleDate(ms: Long): JsonPrimitive =
        JsonPrimitive(ms / 1000.0 - APPLE_EPOCH_OFFSET_S)

    private fun JsonObject.dateMs(key: String, default: Long): Long =
        (this[key] as? JsonPrimitive)?.doubleOrNull
            ?.let { ((it + APPLE_EPOCH_OFFSET_S) * 1000.0).toLong() } ?: default

    private fun JsonObject.str(k: String) =
        (this[k] as? JsonPrimitive)?.takeIf { it.isString }?.content

    private fun JsonObject.uuid(k: String): UUID? =
        str(k)?.let { runCatching { UUID.fromString(it) }.getOrNull() }

    private fun uuidJson(id: UUID) = JsonPrimitive(id.toString().uppercase())

    // ── Items ────────────────────────────────────────────────────────────

    private fun encodeItem(item: PreflightItem): JsonObject = buildJsonObject {
        put("id", uuidJson(item.id))
        put("title", JsonPrimitive(item.title))
        put("detail", JsonPrimitive(item.detail))
        put("kind", JsonPrimitive(item.kind.wire))
    }

    private fun decodeItem(el: JsonObject): PreflightItem = PreflightItem(
        id = el.uuid("id") ?: UUID.randomUUID(),
        title = el.str("title") ?: "",
        detail = el.str("detail") ?: "",
        kind = PreflightItemKind.fromWire(el.str("kind")),
    )

    private fun decodeItems(el: JsonArray?): List<PreflightItem> =
        el?.mapNotNull { (it as? JsonObject)?.let(::decodeItem) }.orEmpty()

    private fun decodeUuidList(el: JsonArray?): List<UUID> =
        el?.mapNotNull { item ->
            (item as? JsonPrimitive)?.takeIf { it.isString }?.content
                ?.let { runCatching { UUID.fromString(it) }.getOrNull() }
        }.orEmpty()

    // ── Master ───────────────────────────────────────────────────────────

    public fun encodeMaster(master: PreflightMaster): String = buildJsonObject {
        put("items", buildJsonArray { master.items.forEach { add(encodeItem(it)) } })
        put("updatedAt", appleDate(master.updatedAtMs))
    }.toString()

    /** Returns null only on unparseable JSON. */
    public fun decodeMaster(text: String, nowMs: Long): PreflightMaster? {
        val o = runCatching { Json.parseToJsonElement(text).jsonObject }.getOrNull() ?: return null
        return PreflightMaster(
            items = decodeItems(o["items"] as? JsonArray),
            updatedAtMs = o.dateMs("updatedAt", nowMs),
        )
    }

    // ── Per-rocket config ────────────────────────────────────────────────

    public fun encodeConfig(config: PreflightRocketConfig): String = buildJsonObject {
        put("profileId", uuidJson(config.profileId))
        put(
            "disabledMasterIds",
            buildJsonArray { config.disabledMasterIds.forEach { add(uuidJson(it)) } },
        )
        put("extraItems", buildJsonArray { config.extraItems.forEach { add(encodeItem(it)) } })
        put(
            "orderedIds",
            buildJsonArray { config.orderedIds.forEach { add(uuidJson(it)) } },
        )
        put(
            "checked",
            buildJsonObject { config.checked.forEach { (k, ms) -> put(k, appleDate(ms)) } },
        )
        put("updatedAt", appleDate(config.updatedAtMs))
    }.toString()

    /** Returns null on unparseable JSON or a missing `profileId` (iOS parity). */
    public fun decodeConfig(text: String, nowMs: Long): PreflightRocketConfig? {
        val o = runCatching { Json.parseToJsonElement(text).jsonObject }.getOrNull() ?: return null
        val profileId = o.uuid("profileId") ?: return null
        return PreflightRocketConfig(
            profileId = profileId,
            disabledMasterIds = decodeUuidList(o["disabledMasterIds"] as? JsonArray),
            extraItems = decodeItems(o["extraItems"] as? JsonArray),
            orderedIds = decodeUuidList(o["orderedIds"] as? JsonArray),
            checked = (o["checked"] as? JsonObject)?.let { obj ->
                obj.keys.associateWith { k -> obj.dateMs(k, nowMs) }
            }.orEmpty(),
            updatedAtMs = o.dateMs("updatedAt", nowMs),
        )
    }
}
