package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * #838 item 5 — the chart column picker's group list drifted from the CSV
 * writer and nothing noticed for a year.
 *
 * #142/#143 renamed the per-detector apogee columns in `buildCsvHeader` and
 * added a voted master, but `columnGroups` kept `"Altitude Apogee Flag"` and
 * `"Velocity Apogee Flag"`. Both pickers intersect the groups with the file's
 * actual headers (`group.columns.filter { it in plottable }`), so the dead
 * names were silently dropped rather than shown as ghosts — and the 16
 * columns that DO exist were unreachable. An operator investigating a
 * deployment-timing anomaly could not plot when the master apogee vote
 * latched or when a pyro fired.
 *
 * Silent in both directions, which is why it lasted: a stale name shows
 * nothing, and a new column nobody grouped shows nothing either. These two
 * tests close both directions.
 */
class ColumnGroupParityTest {

    private val header: List<String> =
        CsvGenerator().buildCsvHeader().trim().split(",")

    private val grouped: List<String> =
        FlightCsvData.columnGroups.flatMap { it.columns }

    /** A rename in the writer must not leave a dead name in a group. */
    @Test
    fun everyGroupedColumnIsWrittenOrAKnownLegacyName() {
        val known = header.toSet() + FlightCsvData.legacyColumnAliases
        val dead = grouped.filterNot { it in known }
        assertTrue(
            dead.isEmpty(),
            "column groups name columns the writer does not emit: $dead — " +
                "either the writer renamed them (update the group) or they are " +
                "old-file names (add them to legacyColumnAliases)",
        )
    }

    /** A NEW column must not be invisible to the picker. */
    @Test
    fun everyWrittenColumnIsPlottableOrExplicitlyNot() {
        val covered = grouped.toSet() + FlightCsvData.nonPlottableColumns
        val orphans = header.filterNot { it in covered }
        assertTrue(
            orphans.isEmpty(),
            "columns are written but in no group, so they can never be charted: " +
                "$orphans — add them to a ColumnGroup, or to nonPlottableColumns " +
                "if they are not a plottable series",
        )
    }

    /** The specific columns the drift hid, named so a revert is obvious. */
    @Test
    fun theColumnsTheDriftHidAreReachable() {
        for (name in listOf(
            "Apogee Detector: Baro", "Apogee Detector: Velocity",
            "Apogee Detector: GPS", "Apogee Detector: Pitch",
            "Apogee Flag (Master)",
            "Pyro 1 Continuity", "Pyro 4 Continuity",
            "Pyro 1 Fired", "Pyro 4 Fired",
            "Reboot Recovery", "FC Guidance Enabled", "EKF Ticks",
        )) {
            assertTrue(name in grouped, "$name is written but not plottable")
            assertTrue(name in header, "$name is grouped but not written")
        }
    }

    /** The stale names are gone, not merely joined by the new ones. */
    @Test
    fun theRenamedAwayNamesAreNotStillListed() {
        assertTrue("Altitude Apogee Flag" !in grouped)
        assertTrue("Velocity Apogee Flag" !in grouped)
    }

    /** Legacy aliases earn their place: they must NOT be current columns. */
    @Test
    fun everyLegacyAliasIsGenuinelyRetired() {
        val stillWritten = FlightCsvData.legacyColumnAliases.filter { it in header }
        assertEquals(
            emptyList(), stillWritten,
            "these are current columns, not legacy names — remove them from " +
                "legacyColumnAliases so they are actually checked",
        )
    }

    /**
     * The two platforms must offer the SAME columns — this was one defect
     * duplicated, and a one-sided fix would turn it into a parity gap. The
     * iOS twin (ColumnGroupParityTests) asserts this same literal list.
     */
    @Test
    fun groupsMatchTheIosTwinsShape() {
        assertEquals(
            listOf("Altitude & Position", "Velocity", "Acceleration", "Rotation",
                   "Environment", "Power", "GPS", "Flags", "Apogee", "Pyro", "Diagnostics"),
            FlightCsvData.columnGroups.map { it.name },
            "group names/order drifted from iOS FlightCSVData.columnGroups",
        )
    }

    /** No duplicate names across groups — the picker would show two rows. */
    @Test
    fun noColumnAppearsInTwoGroups() {
        val dupes = grouped.groupingBy { it }.eachCount().filter { it.value > 1 }.keys
        assertEquals(emptySet(), dupes)
    }
}
