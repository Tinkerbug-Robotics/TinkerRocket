package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File
import java.util.UUID

/** Where the active profile id persists (iOS: UserDefaults). */
public interface ActiveProfileStorage {
    public fun loadActiveId(): String?
    public fun saveActiveId(id: String?)
}

/**
 * File-per-profile store (iOS RocketProfileStore.swift): one JSON per UUID
 * under [directory]; a corrupt file loses ONE profile, never the set.
 * Atomic writes (temp + rename).  The iOS legacy-@AppStorage migration is
 * not ported — no Android install predates profiles.
 *
 * Single-writer: call mutators from the fleet dispatcher.
 */
public class RocketProfileStore(
    private val directory: File,
    private val activeStorage: ActiveProfileStorage,
    private val nowMs: () -> Long = System::currentTimeMillis,
) {
    private val _profiles = MutableStateFlow<List<RocketProfile>>(emptyList())
    public val profiles: StateFlow<List<RocketProfile>> = _profiles.asStateFlow()

    private val _activeId = MutableStateFlow<UUID?>(null)
    public val activeId: StateFlow<UUID?> = _activeId.asStateFlow()

    public val activeProfile: RocketProfile?
        get() = _activeId.value?.let { id -> _profiles.value.firstOrNull { it.id == id } }

    init {
        load()
        _activeId.value = activeStorage.loadActiveId()
            ?.let { runCatching { UUID.fromString(it) }.getOrNull() }
            ?.takeIf { id -> _profiles.value.any { it.id == id } }
    }

    public fun add(name: String): RocketProfile {
        val profile = RocketProfile.makeDefault(dedupedName(name), nowMs())
        _profiles.value = sorted(_profiles.value + profile)
        save(profile)
        return profile
    }

    public fun duplicate(id: UUID): RocketProfile? {
        val source = _profiles.value.firstOrNull { it.id == id } ?: return null
        val copy = source.copy(
            id = UUID.randomUUID(),
            name = dedupedName(source.name),
            createdAtMs = nowMs(),
            updatedAtMs = nowMs(),
            lastUsedUnitID = null,
        )
        _profiles.value = sorted(_profiles.value + copy)
        save(copy)
        return copy
    }

    public fun rename(id: UUID, newName: String) {
        update(id) { it.copy(name = dedupedName(newName)) }
        _profiles.value = sorted(_profiles.value)
    }

    public fun delete(id: UUID) {
        _profiles.value = _profiles.value.filterNot { it.id == id }
        fileFor(id).delete()
        if (_activeId.value == id) setActive(null)
    }

    /**
     * Make [id] the ONE profile bound to [unitId], releasing any other that
     * still claims that board.
     *
     * Binding has to be exclusive or it isn't a binding. Assigning a second
     * profile to a rocket used to leave the first one still claiming it, and
     * the connect-time lookup takes the FIRST match in a list sorted by NAME
     * — so which profile a board came back on was decided alphabetically
     * rather than by anything the user did. Bench-caught on #915.
     */
    public fun bind(id: UUID, unitId: String) {
        if (unitId.isEmpty()) return
        profiles.value
            .filter { it.lastUsedUnitID == unitId && it.id != id }
            .forEach { other -> update(other.id) { it.copy(lastUsedUnitID = null) } }
        update(id) { it.copy(lastUsedUnitID = unitId) }
    }

    public fun setActive(id: UUID?) {
        _activeId.value = id?.takeIf { candidate -> _profiles.value.any { it.id == candidate } }
        activeStorage.saveActiveId(_activeId.value?.toString()?.uppercase())
    }

    public fun update(id: UUID, mutate: (RocketProfile) -> RocketProfile) {
        val updated = _profiles.value.map { p ->
            if (p.id == id) mutate(p).copy(id = p.id, updatedAtMs = nowMs()) else p
        }
        _profiles.value = updated
        updated.firstOrNull { it.id == id }?.let { save(it) }
    }

    // ── Disk ─────────────────────────────────────────────────────────────

    private fun fileFor(id: UUID) = File(directory, "${id.toString().uppercase()}.json")

    private fun load() {
        directory.mkdirs()
        val loaded = directory.listFiles { f -> f.extension == "json" }
            .orEmpty()
            .mapNotNull { f ->
                // Skip-corrupt: one bad file loses one profile, never the set.
                runCatching { RocketProfileCodec.decode(f.readText(), nowMs()) }.getOrNull()
            }
        _profiles.value = sorted(loaded)
    }

    private fun save(profile: RocketProfile) {
        directory.mkdirs()
        val tmp = File(directory, "${profile.id.toString().uppercase()}.json.tmp")
        tmp.writeText(RocketProfileCodec.encode(profile))
        check(tmp.renameTo(fileFor(profile.id)) || fileFor(profile.id).let { tmp.copyTo(it, overwrite = true); tmp.delete(); true })
    }

    private fun sorted(list: List<RocketProfile>) =
        list.sortedBy { it.name.lowercase() }

    private fun dedupedName(name: String): String {
        val names = _profiles.value.map { it.name }.toSet()
        if (name !in names) return name
        var i = 2
        while ("$name $i" in names) i++
        return "$name $i"
    }
}
