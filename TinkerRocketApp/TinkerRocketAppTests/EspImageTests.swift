import XCTest
@testable import TinkerRocketApp

/// #773. The header values here come from real images built out of this repo on
/// 2026-09-09, so the offsets are pinned against the actual ESP-IDF layout
/// rather than against a reading of it:
///
///   flight_computer      chip 0x0012 (ESP32-P4)  "537dc3ff-dirty-v9+20260909-1835"
///   out_computer         chip 0x0009 (ESP32-S3)  "7410cdc6-dirty-v9+20260909-2056"
///   rocket_computer_mini chip 0x0009 (ESP32-S3)  "7410cdc6-dirty+20260909-2043"
///
/// Mirrors core/protocol EspImageTest.kt case for case.
final class EspImageTests: XCTestCase {

    private func image(project: String,
                       version: String,
                       chipId: Int,
                       magic: UInt8 = 0xE9,
                       appDescMagic: UInt32 = 0xABCD_5432,
                       size: Int = 4096) -> Data {
        var b = [UInt8](repeating: 0, count: size)
        b[0] = magic
        b[12] = UInt8(chipId & 0xFF)
        b[13] = UInt8((chipId >> 8) & 0xFF)
        for i in 0..<4 { b[32 + i] = UInt8((appDescMagic >> (8 * UInt32(i))) & 0xFF) }
        func put(_ off: Int, _ s: String, _ len: Int) {
            let bytes = Array(s.utf8)
            for i in 0..<min(bytes.count, len - 1) { b[off + i] = bytes[i] }
        }
        put(48, version, 32)
        put(80, project, 32)
        put(112, "14:35:47", 16)
        put(128, "Sep  9 2026", 16)
        put(144, "v6.0.1-dirty", 32)
        return Data(b)
    }

    func testParsesARealFlightComputerHeader() {
        let img = EspImage.parse(image(project: "flight_computer",
                                       version: "537dc3ff-dirty-v9+20260909-1835",
                                       chipId: 0x0012))!
        XCTAssertEqual(img.projectName, "flight_computer")
        XCTAssertEqual(img.version, "537dc3ff-dirty-v9+20260909-1835")
        XCTAssertEqual(img.chipId, 0x0012)
        XCTAssertEqual(img.chipName, "ESP32-P4")
        XCTAssertEqual(img.idfVersion, "v6.0.1-dirty")
        XCTAssertEqual(img.buildDate, "Sep  9 2026")
        XCTAssertEqual(img.boardSuffix, "v9")
    }

    func testTheMinisFlightComputerImageIsLabelledNotUniversal() {
        // The suffix letter is not always `v`. This parsed as "no board" until
        // 2026-09-10, so the catalog read the one image that fits exactly one
        // board as the image that fits every board — and offered it as the
        // default to an out-of-the-box V7/V8/V9 whose revision is unknown.
        // The firmware never had the bug: TR_OTA_Receiver strstr's "-m1" and
        // refused, so the app proposed a flash the board then rejected.
        let img = EspImage.parse(image(project: "flight_computer",
                                       version: "7410cdc6-dirty-m1+20260909-2043",
                                       chipId: 0x0009))!
        XCTAssertEqual(img.boardSuffix, "m1")
    }

    func testABaseStationImageCarriesItsRevision() {
        // Two base-station images ship in every release and they are not
        // interchangeable — V2 is an 8 MB board, V3 a 16 MB one with a
        // different partition table. Neither carried a suffix until
        // base_station/CMakeLists.txt started stamping one.
        let img = EspImage.parse(image(project: "base_station",
                                       version: "abc1234-v3+20260910-0950",
                                       chipId: 0x0009))!
        XCTAssertEqual(img.boardSuffix, "v3")
    }

    func testTheMinisOwnProjectLabelsItselfWithB() {
        let img = EspImage.parse(image(project: "rocket_computer_mini",
                                       version: "7410cdc6-b1+20260909-2043",
                                       chipId: 0x0009))!
        XCTAssertEqual(img.boardSuffix, "b1")
    }

    func testTheDateTailIsNotMistakenForABoardSuffix() {
        // The build date is "+20260909-2043"; a laxer pattern reads "-2043".
        let img = EspImage.parse(image(project: "rocket_computer_mini",
                                       version: "7410cdc6-dirty+20260909-2043",
                                       chipId: 0x0009))!
        XCTAssertNil(img.boardSuffix)
    }

    func testABuildWithNoBoardSuffixReportsNone() {
        let img = EspImage.parse(image(project: "rocket_computer_mini",
                                       version: "7410cdc6-dirty+20260909-2043",
                                       chipId: 0x0009))!
        XCTAssertNil(img.boardSuffix)
        XCTAssertEqual(img.chipName, "ESP32-S3")
    }

    func testRefusesAnythingThatIsNotAnEspIdfImage() {
        XCTAssertNil(EspImage.parse(Data(count: 16)))
        XCTAssertNil(EspImage.parse(image(project: "x", version: "y", chipId: 9, magic: 0x50)))
        XCTAssertNil(EspImage.parse(image(project: "x", version: "y", chipId: 9, appDescMagic: 0)))
        let v = EspImage.check(Data(count: 16), expectedProject: EspImage.projectFC)
        XCTAssertTrue(v.isRefusal)
        if case .refuse(_, let why) = v {
            XCTAssertTrue(why.contains("not an ESP-IDF firmware image"))
        }
    }

    func testTheCaseThisExistsFor_baseStationImageAimedAtTheOutComputer() {
        // Both are ESP32-S3 with byte-identical app slots, so nothing downstream
        // separates them: TR_OTA checks size and SHA-256 only.
        let bs = image(project: "base_station", version: "abc1234-v5+20260909-1200", chipId: 0x0009)
        let v = EspImage.check(bs, expectedProject: EspImage.projectOC, expectedChipId: 0x0009)
        XCTAssertTrue(v.isRefusal)
        if case .refuse(_, let why) = v {
            XCTAssertTrue(why.contains("base_station"))
            XCTAssertTrue(why.contains("out_computer"))
        }
    }

    func testTheRightImageForTheRightUnitPassesClean() {
        let oc = image(project: "out_computer", version: "7410cdc6-dirty-v9+20260909-2056",
                       chipId: 0x0009)
        let v = EspImage.check(oc, expectedProject: EspImage.projectOC,
                               expectedChipId: 0x0009,
                               runningVersion: "0000000-v9+20260901-0900")
        guard case .ok(let img) = v else { return XCTFail("expected ok, got \(v)") }
        XCTAssertEqual(img.projectName, "out_computer")
    }

    func testAChipMismatchWarnsRatherThanRefuses() {
        // The flight computer is an ESP32-P4 on V9 and an ESP32-S3 on the mini,
        // so this cannot be a hard gate without blocking a legitimate update.
        let fc = image(project: "flight_computer", version: "abc1234+20260909-1200", chipId: 0x0009)
        let v = EspImage.check(fc, expectedProject: EspImage.projectFC, expectedChipId: 0x0012)
        guard case .warn(_, let why) = v else { return XCTFail("expected warn, got \(v)") }
        XCTAssertTrue(why.contains("ESP32-S3"))
        XCTAssertTrue(why.contains("ESP32-P4"))
    }

    func testABoardSuffixDisagreeingWithTheRunningFirmwareWarns() {
        let fc = image(project: "flight_computer", version: "abc1234-v8+20260909-1200",
                       chipId: 0x0012)
        let v = EspImage.check(fc, expectedProject: EspImage.projectFC,
                               expectedChipId: 0x0012,
                               runningVersion: "0000000-v9+20260901-0900")
        guard case .warn(_, let why) = v else { return XCTFail("expected warn, got \(v)") }
        XCTAssertTrue(why.contains("v8"))
        XCTAssertTrue(why.contains("v9"))
    }

    func testAnUnnamedProgramIsRefusedByNameFreeWording() {
        let v = EspImage.check(image(project: "", version: "abc+1", chipId: 0x0009),
                               expectedProject: EspImage.projectBS)
        XCTAssertTrue(v.isRefusal)
        if case .refuse(_, let why) = v { XCTAssertTrue(why.contains("unnamed program")) }
    }

    func testFieldsStopAtTheFirstNulAndNeverRunIntoTheNextOne() {
        let img = EspImage.parse(image(project: "out_computer", version: "v1", chipId: 0x0009))!
        XCTAssertEqual(img.projectName, "out_computer")
        XCTAssertEqual(img.version, "v1")
        XCTAssertEqual(img.idfVersion, "v6.0.1-dirty")
    }

    // MARK: - #773 step 2: the provisioned revision beats the image's own claim

    func testTheProvisionedBoardWinsOverTheRunningVersion() {
        // The scenario the feature exists for: a V9 board wrongly flashed with
        // a V8 image reports v8 forever, so comparing against the RUNNING
        // version happily agrees with the mistake. The board's own answer
        // catches it.
        let v8 = image(project: "flight_computer", version: "abc-v8+1", chipId: 0x0012)
        let v = EspImage.check(v8, expectedProject: EspImage.projectFC,
                               runningVersion: "0000000-v8+20260901-0900",
                               provisionedBoard: "v9")
        guard case .warn(_, let why) = v else { return XCTFail("expected warn, got \(v)") }
        XCTAssertTrue(why.contains("provisioned as v9"))
    }

    func testAProvisionedBoardThatAgreesIsSilent() {
        let v = EspImage.check(
            image(project: "flight_computer", version: "abc-v9+1", chipId: 0x0012),
            expectedProject: EspImage.projectFC,
            runningVersion: "0000000-v8+1", provisionedBoard: "V9")
        guard case .ok = v else { return XCTFail("expected ok, got \(v)") }
    }

    func testWithoutAProvisionedBoardItFallsBackAndSaysSo() {
        let v = EspImage.check(
            image(project: "flight_computer", version: "abc-v8+1", chipId: 0x0012),
            expectedProject: EspImage.projectFC,
            runningVersion: "0000000-v9+1", provisionedBoard: nil)
        guard case .warn(_, let why) = v else { return XCTFail("expected warn, got \(v)") }
        XCTAssertTrue(why.contains("not provisioned"))
        XCTAssertTrue(why.contains("image's own claim"))
    }

    func testAnEmptyProvisionedStringIsTreatedAsUnprovisioned() {
        let v = EspImage.check(
            image(project: "flight_computer", version: "abc-v8+1", chipId: 0x0012),
            expectedProject: EspImage.projectFC,
            runningVersion: "0000000-v9+1", provisionedBoard: "  ")
        guard case .warn(_, let why) = v else { return XCTFail("expected warn, got \(v)") }
        XCTAssertTrue(why.contains("not provisioned"))
    }

    func testAnImageWithNoBoardSuffixIsNeverBoardWarned() {
        // rocket_computer_mini carries no suffix; there is nothing to compare
        // and inventing a mismatch would be worse than saying nothing.
        let v = EspImage.check(
            image(project: "rocket_computer_mini", version: "abc+20260909-2043", chipId: 0x0009),
            expectedProject: EspImage.projectMini,
            expectedChipId: 0x0009, provisionedBoard: "v9")
        guard case .ok = v else { return XCTFail("expected ok, got \(v)") }
    }
}
