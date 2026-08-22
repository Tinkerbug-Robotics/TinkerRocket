package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File
import java.util.UUID

/**
 * Pre-flight checklist store — port of iOS PreflightStore.swift.
 *
 * Persistence mirrors [RocketProfileStore]: JSON files under [directory]
 * (master.json + one <profile-uuid>.json per configured rocket), atomic
 * writes, skip-corrupt on load — a bad file loses one rocket's diff, never
 * the master or the set.  A config whose profile was since deleted is
 * inert; deleteConfig() is called from the profile-delete UI.
 *
 * Single-writer: call mutators from the fleet dispatcher.
 */
public class PreflightStore(
    private val directory: File,
    private val nowMs: () -> Long = System::currentTimeMillis,
) {
    private val _master = MutableStateFlow(PreflightMaster(updatedAtMs = 0L))
    public val master: StateFlow<PreflightMaster> = _master.asStateFlow()

    private val _configs = MutableStateFlow<Map<UUID, PreflightRocketConfig>>(emptyMap())
    public val configs: StateFlow<Map<UUID, PreflightRocketConfig>> = _configs.asStateFlow()

    init {
        load()
    }

    // ── Master CRUD ──────────────────────────────────────────────────────

    public fun addMasterItem(item: PreflightItem): PreflightItem {
        mutateMaster { it.copy(items = it.items + item) }
        return item
    }

    public fun updateMasterItem(id: UUID, mutate: (PreflightItem) -> PreflightItem) {
        mutateMaster { m ->
            m.copy(items = m.items.map { if (it.id == id) mutate(it).copy(id = id) else it })
        }
    }

    /**
     * Remove a master step everywhere: from the master, from every rocket's
     * exclusion list, and from every rocket's checked state.
     */
    public fun deleteMasterItem(id: UUID) {
        mutateMaster { m -> m.copy(items = m.items.filterNot { it.id == id }) }
        val key = id.toString().uppercase()
        for (profileId in _configs.value.keys) {
            mutateConfig(profileId) { c ->
                c.copy(
                    disabledMasterIds = c.disabledMasterIds.filterNot { it == id },
                    checked = c.checked - key,
                )
            }
        }
    }

    /** Move the step at [fromIndex] to [toIndex]; out-of-range is a no-op. */
    public fun moveMasterItem(fromIndex: Int, toIndex: Int) {
        mutateMaster { m -> m.copy(items = moved(m.items, fromIndex, toIndex)) }
    }

    /** Auto kinds already in the master — the add menu greys these out. */
    public fun masterAutoKinds(): Set<PreflightItemKind> =
        _master.value.items.map { it.kind }.filter { it.isAuto }.toSet()

    // ── Per-rocket config ────────────────────────────────────────────────

    /** Stored config for a rocket, or null if never customized. */
    public fun config(profileId: UUID): PreflightRocketConfig? = _configs.value[profileId]

    /** This rocket's effective checklist (master minus exclusions + extras). */
    public fun effectiveItems(profileId: UUID): List<PreflightItem> =
        PreflightChecklist.effectiveItems(_master.value, _configs.value[profileId])

    public fun setMasterItem(itemId: UUID, enabled: Boolean, profileId: UUID) {
        mutateConfig(profileId) { c ->
            val without = c.disabledMasterIds.filterNot { it == itemId }
            if (enabled) {
                c.copy(disabledMasterIds = without)
            } else {
                // A re-included step must come back UNCHECKED — the old
                // check is stale evidence.
                c.copy(
                    disabledMasterIds = without + itemId,
                    checked = c.checked - itemId.toString().uppercase(),
                )
            }
        }
    }

    public fun addExtraItem(item: PreflightItem, profileId: UUID): PreflightItem {
        mutateConfig(profileId) { it.copy(extraItems = it.extraItems + item) }
        return item
    }

    public fun updateExtraItem(
        itemId: UUID,
        profileId: UUID,
        mutate: (PreflightItem) -> PreflightItem,
    ) {
        mutateConfig(profileId) { c ->
            c.copy(
                extraItems = c.extraItems.map {
                    if (it.id == itemId) mutate(it).copy(id = itemId) else it
                },
            )
        }
    }

    public fun deleteExtraItem(itemId: UUID, profileId: UUID) {
        mutateConfig(profileId) { c ->
            c.copy(
                extraItems = c.extraItems.filterNot { it.id == itemId },
                checked = c.checked - itemId.toString().uppercase(),
            )
        }
    }

    /** Move a rocket's extra step at [fromIndex] to [toIndex]; out-of-range no-ops. */
    public fun moveExtraItem(profileId: UUID, fromIndex: Int, toIndex: Int) {
        mutateConfig(profileId) { c ->
            c.copy(extraItems = moved(c.extraItems, fromIndex, toIndex))
        }
    }

    /** Drop a rocket's whole config (called when its profile is deleted). */
    public fun deleteConfig(profileId: UUID) {
        if (profileId !in _configs.value) return
        _configs.value = _configs.value - profileId
        fileFor(profileId).delete()
    }

    // ── Run state (manual checks) ────────────────────────────────────────

    public fun setChecked(itemId: UUID, checked: Boolean, profileId: UUID) {
        val key = itemId.toString().uppercase()
        mutateConfig(profileId) { c ->
            if (checked) {
                c.copy(checked = c.checked + (key to nowMs()))
            } else {
                c.copy(checked = c.checked - key)
            }
        }
    }

    public fun isChecked(itemId: UUID, profileId: UUID): Boolean =
        _configs.value[profileId]?.isChecked(itemId) ?: false

    /** Clear every manual check for the next flight. */
    public fun resetRun(profileId: UUID) {
        mutateConfig(profileId) { it.copy(checked = emptyMap()) }
    }

    private fun moved(
        items: List<PreflightItem>,
        fromIndex: Int,
        toIndex: Int,
    ): List<PreflightItem> {
        if (fromIndex !in items.indices || toIndex !in items.indices) return items
        val mutable = items.toMutableList()
        mutable.add(toIndex, mutable.removeAt(fromIndex))
        return mutable
    }

    // ── Disk ─────────────────────────────────────────────────────────────

    private fun masterFile() = File(directory, "master.json")

    private fun fileFor(profileId: UUID) =
        File(directory, "${profileId.toString().uppercase()}.json")

    private fun mutateMaster(mutate: (PreflightMaster) -> PreflightMaster) {
        val updated = mutate(_master.value).copy(updatedAtMs = nowMs())
        _master.value = updated
        write(masterFile(), PreflightCodec.encodeMaster(updated))
    }

    /** All config edits funnel through here — get-or-create, stamp, persist. */
    private fun mutateConfig(
        profileId: UUID,
        mutate: (PreflightRocketConfig) -> PreflightRocketConfig,
    ) {
        val current = _configs.value[profileId]
            ?: PreflightRocketConfig(profileId = profileId, updatedAtMs = nowMs())
        val updated = mutate(current).copy(profileId = profileId, updatedAtMs = nowMs())
        _configs.value = _configs.value + (profileId to updated)
        write(fileFor(profileId), PreflightCodec.encodeConfig(updated))
    }

    private fun load() {
        directory.mkdirs()
        val masterText = masterFile().takeIf { it.exists() }
            ?.let { runCatching { it.readText() }.getOrNull() }
        _master.value = masterText?.let { PreflightCodec.decodeMaster(it, nowMs()) }
            ?: PreflightMaster(updatedAtMs = nowMs())

        val loaded = directory.listFiles { f -> f.extension == "json" && f.name != "master.json" }
            .orEmpty()
            .mapNotNull { f ->
                // Skip-corrupt: one bad file loses one rocket's diff, never the set.
                runCatching { PreflightCodec.decodeConfig(f.readText(), nowMs()) }.getOrNull()
            }
            .associateBy { it.profileId }
        _configs.value = loaded
    }

    private fun write(file: File, text: String) {
        directory.mkdirs()
        val tmp = File(directory, "${file.name}.tmp")
        tmp.writeText(text)
        check(tmp.renameTo(file) || file.let { tmp.copyTo(it, overwrite = true); tmp.delete(); true })
    }
}
