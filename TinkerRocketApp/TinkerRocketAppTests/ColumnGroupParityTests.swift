import XCTest
@testable import TinkerRocketApp

/// #838 item 5 — the chart column picker's group list drifted from the CSV
/// writer and nothing noticed for a year.
///
/// #142/#143 renamed the per-detector apogee columns in `buildCSVHeader` and
/// added a voted master, but `columnGroups` kept `"Altitude Apogee Flag"` and
/// `"Velocity Apogee Flag"`. The picker intersects the groups with the file's
/// actual headers (`group.columns.filter { availableColumns.contains($0) }`),
/// so the dead names were silently dropped rather than shown as ghosts — and
/// the 16 columns that DO exist were unreachable. An operator investigating a
/// deployment-timing anomaly could not plot when the master apogee vote
/// latched or when a pyro fired.
///
/// Silent in both directions, which is why it lasted: a stale name shows
/// nothing, and a new column nobody grouped shows nothing either. Both
/// directions are checked here, and the Kotlin twin
/// (`ColumnGroupParityTest`) does the same — the finding notes this was one
/// defect duplicated across platforms rather than a parity gap.
final class ColumnGroupParityTests: XCTestCase {

    private var header: [String] {
        CSVGenerator().buildCSVHeader()
            .trimmingCharacters(in: .whitespacesAndNewlines)
            .components(separatedBy: ",")
    }

    private var grouped: [String] {
        FlightCSVData.columnGroups.flatMap { $0.columns }
    }

    /// A rename in the writer must not leave a dead name in a group.
    func testEveryGroupedColumnIsWrittenOrAKnownLegacyName() {
        let known = Set(header).union(FlightCSVData.legacyColumnAliases)
        let dead = grouped.filter { !known.contains($0) }
        XCTAssertTrue(
            dead.isEmpty,
            "column groups name columns the writer does not emit: \(dead) — either the "
                + "writer renamed them (update the group) or they are old-file names "
                + "(add them to legacyColumnAliases)"
        )
    }

    /// A NEW column must not be invisible to the picker.
    func testEveryWrittenColumnIsPlottableOrExplicitlyNot() {
        let covered = Set(grouped).union(FlightCSVData.nonPlottableColumns)
        let orphans = header.filter { !covered.contains($0) }
        XCTAssertTrue(
            orphans.isEmpty,
            "columns are written but in no group, so they can never be charted: \(orphans) "
                + "— add them to a group, or to nonPlottableColumns if they are not a "
                + "plottable series"
        )
    }

    /// The specific columns the drift hid, named so a revert is obvious.
    func testTheColumnsTheDriftHidAreReachable() {
        let hidden = [
            "Apogee Detector: Baro", "Apogee Detector: Velocity",
            "Apogee Detector: GPS", "Apogee Detector: Pitch",
            "Apogee Flag (Master)",
            "Pyro 1 Continuity", "Pyro 4 Continuity",
            "Pyro 1 Fired", "Pyro 4 Fired",
            "Reboot Recovery", "FC Guidance Enabled", "EKF Ticks",
        ]
        for name in hidden {
            XCTAssertTrue(grouped.contains(name), "\(name) is written but not plottable")
            XCTAssertTrue(header.contains(name), "\(name) is grouped but not written")
        }
    }

    /// The stale names are gone, not merely joined by the new ones.
    func testTheRenamedAwayNamesAreNotStillListed() {
        XCTAssertFalse(grouped.contains("Altitude Apogee Flag"))
        XCTAssertFalse(grouped.contains("Velocity Apogee Flag"))
    }

    /// Legacy aliases earn their place: they must NOT be current columns.
    func testEveryLegacyAliasIsGenuinelyRetired() {
        let stillWritten = FlightCSVData.legacyColumnAliases.filter { header.contains($0) }
        XCTAssertTrue(
            stillWritten.isEmpty,
            "these are current columns, not legacy names: \(stillWritten) — remove them "
                + "from legacyColumnAliases so they are actually checked"
        )
    }

    func testNoColumnAppearsInTwoGroups() {
        var seen = Set<String>()
        var dupes = Set<String>()
        for name in grouped where !seen.insert(name).inserted { dupes.insert(name) }
        XCTAssertTrue(dupes.isEmpty, "duplicate column names across groups: \(dupes)")
    }

    /// The two platforms must offer the SAME columns — this was one defect
    /// duplicated, and a one-sided fix would turn it into a parity gap.
    func testGroupsMatchTheKotlinTwinsShape() {
        let names = FlightCSVData.columnGroups.map { $0.name }
        XCTAssertEqual(
            names,
            ["Altitude & Position", "Velocity", "Acceleration", "Rotation",
             "Environment", "Power", "GPS", "Flags", "Apogee", "Pyro", "Diagnostics"],
            "group names/order drifted from core/protocol's FlightCsvData.columnGroups"
        )
    }
}
