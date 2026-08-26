package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import java.io.File
import java.util.UUID
import kotlin.test.AfterTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Pre-flight checklist: effective-list composition (live master template +
 * per-rocket diff), auto-step evaluation, progress rollup, the iOS-schema
 * JSON codec, and the file-per-entity store.  Twin of the iOS
 * PreflightChecklistTests / PreflightStoreTests.
 */
class PreflightTest {

    private fun manual(title: String) = PreflightItem(title = title)

    private fun connectedCtx(
        fs: Int = 0,
        ps: Int = 0,
        sats: Int = 0,
        lat: Double? = null,
        lon: Double? = null,
        isRelay: Boolean = false,
        syncState: ActiveRocketSyncer.SyncState = ActiveRocketSyncer.SyncState.Idle,
        profile: RocketProfile? = null,
    ) = PreflightAutoContext(
        isConnected = true,
        hasTelemetry = true,
        isRelay = isRelay,
        telemetry = TelemetryData(
            flightStatusBits = fs, pyroStatusBits = ps,
            numSats = sats, latitude = lat, longitude = lon,
        ),
        syncState = syncState,
        profile = profile,
    )

    // ── Effective list ───────────────────────────────────────────────────

    @Test
    fun effectiveList_isMasterWhenNoConfig() {
        val master = PreflightMaster(items = listOf(manual("A"), manual("B")), updatedAtMs = 0)
        assertEquals(
            listOf("A", "B"),
            PreflightChecklist.effectiveItems(master, null).map { it.title },
        )
    }

    @Test
    fun effectiveList_excludesDisabledAndAppendsExtras() {
        val a = manual("A")
        val b = manual("B")
        val master = PreflightMaster(items = listOf(a, b), updatedAtMs = 0)
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            disabledMasterIds = listOf(a.id),
            extraItems = listOf(manual("Extra")),
            updatedAtMs = 0,
        )
        assertEquals(
            listOf("B", "Extra"),
            PreflightChecklist.effectiveItems(master, config).map { it.title },
        )
    }

    /** The live-template contract: a master edit reaches configured rockets. */
    @Test
    fun masterEdit_flowsThroughToConfiguredRocket() {
        val a = manual("A")
        val config = PreflightRocketConfig(profileId = UUID.randomUUID(), updatedAtMs = 0)
        val edited = PreflightMaster(
            items = listOf(a.copy(title = "A, but sharper")), updatedAtMs = 0,
        )
        assertEquals(
            listOf("A, but sharper"),
            PreflightChecklist.effectiveItems(edited, config).map { it.title },
        )
    }

    // ── Per-rocket ordering ──────────────────────────────────────────────

    @Test
    fun orderedIds_reorderAndInterleaveTheEffectiveList() {
        val a = manual("A")
        val b = manual("B")
        val extra = manual("Extra")
        val master = PreflightMaster(items = listOf(a, b), updatedAtMs = 0)
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            extraItems = listOf(extra),
            orderedIds = listOf(b.id, extra.id, a.id),
            updatedAtMs = 0,
        )
        assertEquals(
            listOf("B", "Extra", "A"),
            PreflightChecklist.effectiveItems(master, config).map { it.title },
        )
    }

    @Test
    fun orderedIds_unlistedItemsAppendAndStaleIdsAreSkipped() {
        val a = manual("A")
        val b = manual("B")
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            // Stale ghost id + only B listed: B first, A appends, ghost skipped.
            orderedIds = listOf(UUID.randomUUID(), b.id),
            updatedAtMs = 0,
        )
        // "C" added after the order was saved appends at the end.
        val master = PreflightMaster(items = listOf(a, b, manual("C")), updatedAtMs = 0)
        assertEquals(
            listOf("B", "A", "C"),
            PreflightChecklist.effectiveItems(master, config).map { it.title },
        )
    }

    // ── Auto evaluation ──────────────────────────────────────────────────

    @Test
    fun manualHasNoAutoStatus() {
        assertNull(PreflightChecklist.autoStatus(PreflightItemKind.MANUAL, PreflightAutoContext()))
    }

    @Test
    fun everyAutoKind_pendsWhenDisconnected() {
        val ctx = PreflightAutoContext()   // not connected
        for (kind in PreflightItemKind.entries.filter { it.isAuto }) {
            assertEquals(
                PreflightAutoStatus.Pending("Not connected"),
                PreflightChecklist.autoStatus(kind, ctx),
                kind.name,
            )
        }
    }

    @Test
    fun connected_needsTelemetry() {
        assertEquals(
            PreflightAutoStatus.Pending("Waiting for telemetry"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.CONNECTED, PreflightAutoContext(isConnected = true),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(PreflightItemKind.CONNECTED, connectedCtx()),
        )
    }

    @Test
    fun settingsSynced_followsSyncerState() {
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED,
                connectedCtx(syncState = ActiveRocketSyncer.SyncState.Synced),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("Rocket settings not read yet"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED,
                connectedCtx(syncState = ActiveRocketSyncer.SyncState.AwaitingSync),
            ),
        )
        // #915: the app took the rocket's own settings — the two agree, which
        // is what this step checks.  What changed rides its own status line.
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED,
                connectedCtx(
                    syncState = ActiveRocketSyncer.SyncState.Adopted(listOf("PID gains")),
                ),
            ),
        )
        // A relay link can't sync — N/A, not pending-forever.
        assertEquals(
            PreflightAutoStatus.NotApplicable("Needs a direct connection"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED, connectedCtx(isRelay = true),
            ),
        )
    }

    @Test
    fun gnssFix_needsPositionAndSats() {
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.GNSS_FIX, connectedCtx(sats = 7, lat = 40.0, lon = -105.0),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("No fix (3 sats)"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.GNSS_FIX, connectedCtx(sats = 3, lat = 40.0, lon = -105.0),
            ),
        )
        // (0, 0) origin and missing lat/lon both fail regardless of sats.
        assertEquals(
            PreflightAutoStatus.Pending("No fix (9 sats)"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.GNSS_FIX, connectedCtx(sats = 9, lat = 0.0, lon = 0.0),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("No fix (9 sats)"),
            PreflightChecklist.autoStatus(PreflightItemKind.GNSS_FIX, connectedCtx(sats = 9)),
        )
    }

    @Test
    fun cameraRecording_readsFsBitAndProfile() {
        val withCam = RocketProfile.makeDefault("cam", nowMs = 0)   // cameraType 2 = RunCam
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.CAMERA_RECORDING, connectedCtx(fs = 0x20, profile = withCam),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("Not recording"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.CAMERA_RECORDING, connectedCtx(fs = 0, profile = withCam),
            ),
        )
        assertEquals(
            PreflightAutoStatus.NotApplicable("No camera on this rocket"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.CAMERA_RECORDING,
                connectedCtx(fs = 0x20, profile = withCam.copy(cameraType = 0)),
            ),
        )
    }

    @Test
    fun loggingActive_readsFsBit() {
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.LOGGING_ACTIVE, connectedCtx(fs = 0x40),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("Not logging"),
            PreflightChecklist.autoStatus(PreflightItemKind.LOGGING_ACTIVE, connectedCtx()),
        )
    }

    @Test
    fun pyroArmed_gatedOnEnabledChannels() {
        val p = RocketProfile.makeDefault("pyro", nowMs = 0)
        // No channels enabled (factory default) — N/A even when armed.
        assertEquals(
            PreflightAutoStatus.NotApplicable("No pyro channels enabled"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.PYRO_ARMED, connectedCtx(ps = 0x001, profile = p),
            ),
        )
        val armed = p.copy(pyro1Enabled = true)
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.PYRO_ARMED, connectedCtx(ps = 0x001, profile = armed),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("Not armed"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.PYRO_ARMED, connectedCtx(ps = 0, profile = armed),
            ),
        )
    }

    @Test
    fun pyroContinuity_checksOnlyEnabledChannels() {
        val p = RocketProfile.makeDefault("pyro", nowMs = 0)
            .copy(pyro1Enabled = true, pyro3Enabled = true)
        // ch1 cont (0x002) + ch3 cont (0x020); ch2/4 open but disabled.
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.PYRO_CONTINUITY,
                connectedCtx(ps = 0x002 or 0x020, profile = p),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Pending("Ch 3 open"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.PYRO_CONTINUITY, connectedCtx(ps = 0x002, profile = p),
            ),
        )
    }

    // ── Progress rollup ──────────────────────────────────────────────────

    @Test
    fun progress_countsManualChecksAndAutoStates() {
        val m1 = manual("wadding")
        val m2 = manual("igniter")
        val cam = PreflightItem.auto(PreflightItemKind.CAMERA_RECORDING)
        val master = PreflightMaster(items = listOf(m1, m2, cam), updatedAtMs = 0)
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            checked = mapOf(m1.id.toString().uppercase() to 1L),
            updatedAtMs = 0,
        )
        val profile = RocketProfile.makeDefault("r", nowMs = 0).copy(cameraType = 0)

        val progress = PreflightChecklist.progress(
            PreflightChecklist.effectiveItems(master, config), config,
            connectedCtx(profile = profile),
        )
        assertEquals(2, progress.done)   // m1 checked + camera N/A
        assertEquals(3, progress.total)
        assertFalse(progress.isComplete)
    }

    @Test
    fun autoStep_cannotBeSatisfiedByManualCheck() {
        val cam = PreflightItem.auto(PreflightItemKind.CAMERA_RECORDING)
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            // A stray checked entry for an auto item must not count.
            checked = mapOf(cam.id.toString().uppercase() to 1L),
            updatedAtMs = 0,
        )
        val progress = PreflightChecklist.progress(
            listOf(cam), config,
            connectedCtx(profile = RocketProfile.makeDefault("r", nowMs = 0)),
        )
        assertEquals(0, progress.done)
    }

    @Test
    fun emptyChecklist_isNeverComplete() {
        assertFalse(
            PreflightChecklist.progress(emptyList(), null, PreflightAutoContext()).isComplete,
        )
    }

    // ── Codec ────────────────────────────────────────────────────────────

    @Test
    fun masterRoundTrip() {
        val master = PreflightMaster(
            items = listOf(manual("A"), PreflightItem.auto(PreflightItemKind.LOGGING_ACTIVE)),
            updatedAtMs = 1_700_000_000_000,
        )
        val decoded = PreflightCodec.decodeMaster(PreflightCodec.encodeMaster(master), 0)
        assertEquals(master, decoded)
    }

    @Test
    fun configRoundTrip() {
        val extra = manual("rail buttons")
        val config = PreflightRocketConfig(
            profileId = UUID.randomUUID(),
            disabledMasterIds = listOf(UUID.randomUUID()),
            extraItems = listOf(extra),
            orderedIds = listOf(extra.id, UUID.randomUUID()),
            checked = mapOf(extra.id.toString().uppercase() to 1_700_000_000_000),
            updatedAtMs = 1_700_000_123_000,
        )
        val decoded = PreflightCodec.decodeConfig(PreflightCodec.encodeConfig(config), 0)
        assertEquals(config, decoded)
    }

    /** An iOS-written file (Apple-epoch dates, uppercase UUIDs) decodes. */
    @Test
    fun decode_iosShapedConfig() {
        val json = """
            {"profileId":"AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE",
             "disabledMasterIds":["11111111-2222-3333-4444-555555555555"],
             "extraItems":[{"id":"99999999-8888-7777-6666-555555555555",
                            "title":"Rail buttons","detail":"","kind":"manual"}],
             "checked":{"99999999-8888-7777-6666-555555555555":776587200.0},
             "updatedAt":776587200.5}
        """.trimIndent()
        val config = PreflightCodec.decodeConfig(json, 0)!!
        assertEquals("AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE", config.profileId.toString().uppercase())
        assertEquals(listOf("Rail buttons"), config.extraItems.map { it.title })
        assertTrue(config.isChecked(UUID.fromString("99999999-8888-7777-6666-555555555555")))
        // 776587200 s after 2001-01-01 = 2025-08-11T06:40:00Z.
        assertEquals(1754894400000L, config.checked.values.single())
    }

    @Test
    fun decode_unknownKindFallsBackToManual() {
        val json = """{"items":[{"id":"11111111-2222-3333-4444-555555555555",
            "title":"future step","detail":"","kind":"quantumFluxCheck"}],"updatedAt":0}"""
        val master = PreflightCodec.decodeMaster(json, 0)!!
        assertEquals(PreflightItemKind.MANUAL, master.items.single().kind)
        assertEquals("future step", master.items.single().title)
    }

    @Test
    fun decode_garbageReturnsNull() {
        assertNull(PreflightCodec.decodeMaster("not json {", 0))
        assertNull(PreflightCodec.decodeConfig("not json {", 0))
        // Config without a profileId is unusable.
        assertNull(PreflightCodec.decodeConfig("""{"extraItems":[]}""", 0))
    }
}

class PreflightStoreTest {

    private val dir = File(
        System.getProperty("java.io.tmpdir"),
        "preflight-test-${UUID.randomUUID()}",
    )
    private var clock = 1_000L

    @AfterTest
    fun cleanup() {
        dir.deleteRecursively()
    }

    private fun makeStore() = PreflightStore(dir, nowMs = { clock++ })

    @Test
    fun masterAddEditDeletePersists() {
        val store = makeStore()
        val item = store.addMasterItem(manualItem("Wadding"))
        store.updateMasterItem(item.id) { it.copy(title = "Recovery wadding") }

        val reloaded = makeStore()
        assertEquals(listOf("Recovery wadding"), reloaded.master.value.items.map { it.title })

        reloaded.deleteMasterItem(item.id)
        assertTrue(makeStore().master.value.items.isEmpty())
    }

    @Test
    fun masterMovePersistsOrder() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val c = store.addMasterItem(manualItem("C"))
        store.moveMasterItem(a.id, delta = +1)
        store.moveMasterItem(a.id, delta = +1)
        assertEquals(listOf("B", "C", "A"), makeStore().master.value.items.map { it.title })
        // Out-of-range and unknown ids are no-ops, not crashes.
        store.moveMasterItem(a.id, delta = +1)
        store.moveMasterItem(c.id, delta = -5)
        store.moveMasterItem(UUID.randomUUID(), delta = -1)
        assertEquals(listOf("B", "C", "A"), makeStore().master.value.items.map { it.title })
    }

    @Test
    fun deleteMasterItemScrubsRocketDiffs() {
        val store = makeStore()
        val item = store.addMasterItem(manualItem("A"))
        val rocket = UUID.randomUUID()
        store.setMasterItem(item.id, enabled = false, profileId = rocket)
        store.setChecked(item.id, checked = true, profileId = rocket)

        store.deleteMasterItem(item.id)
        val config = makeStore().config(rocket)!!
        assertTrue(config.disabledMasterIds.isEmpty())
        assertTrue(config.checked.isEmpty())
    }

    @Test
    fun rocketConfigDiffAndEffectiveList() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val rocket = UUID.randomUUID()

        store.setMasterItem(a.id, enabled = false, profileId = rocket)
        store.addExtraItem(manualItem("Rail buttons"), rocket)

        val reloaded = makeStore()
        assertEquals(
            listOf("B", "Rail buttons"),
            reloaded.effectiveItems(rocket).map { it.title },
        )
        reloaded.setMasterItem(a.id, enabled = true, profileId = rocket)
        assertEquals(
            listOf("A", "B", "Rail buttons"),
            reloaded.effectiveItems(rocket).map { it.title },
        )
        // An untouched rocket just sees the master.
        assertEquals(
            listOf("A", "B"),
            reloaded.effectiveItems(UUID.randomUUID()).map { it.title },
        )
    }

    @Test
    fun disablingMasterItemDropsItsCheck() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        val rocket = UUID.randomUUID()
        store.setChecked(a.id, checked = true, profileId = rocket)
        assertTrue(store.isChecked(a.id, rocket))

        store.setMasterItem(a.id, enabled = false, profileId = rocket)
        // Re-including later must come back UNCHECKED — stale evidence.
        store.setMasterItem(a.id, enabled = true, profileId = rocket)
        assertFalse(store.isChecked(a.id, rocket))
    }

    @Test
    fun moveEffectiveItemInterleavesExtrasAndPersists() {
        val store = makeStore()
        store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val rocket = UUID.randomUUID()
        val extra = store.addExtraItem(manualItem("Extra"), rocket)

        // [A, B, Extra] → move Extra between the master steps.
        store.moveEffectiveItem(rocket, extra.id, delta = -1)
        assertEquals(
            listOf("A", "Extra", "B"),
            makeStore().effectiveItems(rocket).map { it.title },
        )
        // Other rockets keep the default order, and a new master step
        // appends after the custom-ordered block.
        assertEquals(listOf("A", "B"), store.effectiveItems(UUID.randomUUID()).map { it.title })
        store.addMasterItem(manualItem("C"))
        assertEquals(
            listOf("A", "Extra", "B", "C"),
            store.effectiveItems(rocket).map { it.title },
        )
        // Unknown ids and out-of-range deltas are no-ops.
        store.moveEffectiveItem(rocket, UUID.randomUUID(), delta = +1)
        store.moveEffectiveItem(rocket, extra.id, delta = -5)
        assertEquals(
            listOf("A", "Extra", "B", "C"),
            store.effectiveItems(rocket).map { it.title },
        )
    }

    @Test
    fun excludedStepKeepsItsSlotWhenReIncluded() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val rocket = UUID.randomUUID()
        store.moveEffectiveItem(rocket, a.id, delta = +1)   // [B, A]

        store.setMasterItem(a.id, enabled = false, profileId = rocket)
        assertEquals(listOf("B"), store.effectiveItems(rocket).map { it.title })
        store.setMasterItem(a.id, enabled = true, profileId = rocket)
        assertEquals(listOf("B", "A"), store.effectiveItems(rocket).map { it.title })
    }

    /** The review-found hazard: a move must NOT erase an excluded step's slot. */
    @Test
    fun reorderWhileExcludedKeepsTheRememberedSlot() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val c = store.addMasterItem(manualItem("C"))
        val rocket = UUID.randomUUID()
        store.moveEffectiveItem(rocket, c.id, delta = -1)   // [A, C, B]

        store.setMasterItem(a.id, enabled = false, profileId = rocket)
        assertEquals(listOf("C", "B"), store.effectiveItems(rocket).map { it.title })
        // Reorder WHILE A is excluded — A's first-place slot must survive.
        store.moveEffectiveItem(rocket, c.id, delta = +1)   // visible [B, C]
        store.setMasterItem(a.id, enabled = true, profileId = rocket)
        assertEquals(
            listOf("A", "B", "C"),
            makeStore().effectiveItems(rocket).map { it.title },
        )
    }

    /** First-ever move must remember master positions of already-excluded steps. */
    @Test
    fun firstMoveRemembersExcludedMasterPositions() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val c = store.addMasterItem(manualItem("C"))
        val rocket = UUID.randomUUID()

        store.setMasterItem(a.id, enabled = false, profileId = rocket)
        store.moveEffectiveItem(rocket, c.id, delta = -1)   // visible [C, B]
        store.setMasterItem(a.id, enabled = true, profileId = rocket)
        // A comes back at its master position (first), not appended last.
        assertEquals(listOf("A", "C", "B"), store.effectiveItems(rocket).map { it.title })
    }

    @Test
    fun deleteScrubsOrderedIds() {
        val store = makeStore()
        val a = store.addMasterItem(manualItem("A"))
        store.addMasterItem(manualItem("B"))
        val rocket = UUID.randomUUID()
        val extra = store.addExtraItem(manualItem("Extra"), rocket)
        store.moveEffectiveItem(rocket, extra.id, delta = -2)   // [Extra, A, B]

        store.deleteMasterItem(a.id)
        store.deleteExtraItem(extra.id, rocket)
        assertEquals(1, makeStore().config(rocket)!!.orderedIds.size)
        assertEquals(listOf("B"), store.effectiveItems(rocket).map { it.title })
    }

    @Test
    fun checkedRoundTripAndReset() {
        val store = makeStore()
        val item = store.addMasterItem(manualItem("A"))
        val rocket = UUID.randomUUID()

        store.setChecked(item.id, checked = true, profileId = rocket)
        assertTrue(makeStore().isChecked(item.id, rocket))

        store.setChecked(item.id, checked = false, profileId = rocket)
        assertFalse(makeStore().isChecked(item.id, rocket))

        store.setChecked(item.id, checked = true, profileId = rocket)
        store.resetRun(rocket)
        assertFalse(makeStore().isChecked(item.id, rocket))
    }

    @Test
    fun deleteConfigRemovesFile() {
        val store = makeStore()
        val rocket = UUID.randomUUID()
        store.addExtraItem(manualItem("X"), rocket)
        assertEquals(1, makeStore().configs.value.size)

        store.deleteConfig(rocket)
        assertNull(store.config(rocket))
        assertTrue(makeStore().configs.value.isEmpty())
    }

    @Test
    fun corruptConfigFileLosesOneRocketNotTheSet() {
        val store = makeStore()
        val rocketA = UUID.randomUUID()
        val rocketB = UUID.randomUUID()
        store.addExtraItem(manualItem("A"), rocketA)
        store.addExtraItem(manualItem("B"), rocketB)

        File(dir, "${rocketA.toString().uppercase()}.json").writeText("not json {")

        val reloaded = makeStore()
        assertNull(reloaded.config(rocketA))
        assertEquals(listOf("B"), reloaded.config(rocketB)!!.extraItems.map { it.title })
    }

    @Test
    fun corruptMasterYieldsEmptyMasterButKeepsConfigs() {
        val store = makeStore()
        val rocket = UUID.randomUUID()
        store.addMasterItem(manualItem("A"))
        store.addExtraItem(manualItem("X"), rocket)

        File(dir, "master.json").writeText("garbage")

        val reloaded = makeStore()
        assertTrue(reloaded.master.value.items.isEmpty())
        assertEquals(listOf("X"), reloaded.config(rocket)!!.extraItems.map { it.title })
    }

    private fun manualItem(title: String) = PreflightItem(title = title)
}
