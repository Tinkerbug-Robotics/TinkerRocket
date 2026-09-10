import XCTest
@testable import TinkerRocketApp

/// #773 step 4c: the sequencing between "check for updates" and bytes an
/// operator can flash. Android twin: `FirmwareCatalogSessionTest.kt`.
@MainActor
final class FirmwareCatalogSessionTests: XCTestCase {

    private let sha = String(repeating: "a", count: 64)

    private func imageJson(_ project: String, _ board: String?,
                           size: Int, version: String) -> String {
        let boardField = board.map { ",\"board\":\"\($0)\"" } ?? ""
        let file = project + (board.map { "-\($0)" } ?? "")
        return "{\"file\":\"\(file).bin\",\"project\":\"\(project)\",\"version\":\"\(version)\","
            + "\"chip_id\":9,\"chip\":\"ESP32-S3\",\"size\":\(size),\"sha256\":\"\(sha)\"\(boardField)}"
    }

    private func manifest(_ images: [String]) -> String {
        "{\"manifest_version\":1,\"tag\":\"fw-v1.0.0\",\"images\":[\(images.joined(separator: ","))]}"
    }

    private var listing: String {
        "[{\"tag_name\":\"fw-v1.0.0\",\"prerelease\":false,\"assets\":["
        + "{\"name\":\"manifest.json\",\"browser_download_url\":\"https://x.test/fw-v1.0.0/manifest.json\"},"
        + "{\"name\":\"out_computer-v9.bin\",\"browser_download_url\":\"https://x.test/fw-v1.0.0/oc.bin\"},"
        + "{\"name\":\"base_station-v2.bin\",\"browser_download_url\":\"https://x.test/fw-v1.0.0/bs.bin\"}]}]"
    }

    private func session(_ manifestBody: String,
                         assets: [String: Data?] = [:],
                         sha256: String? = nil) -> FirmwareCatalogSession {
        var responses: [String: Data?] = [
            FirmwareReleaseLocator.releasesURL: Data(listing.utf8),
            "https://x.test/fw-v1.0.0/manifest.json": Data(manifestBody.utf8),
        ]
        for (k, v) in assets { responses[k] = v }
        let fixed = sha256 ?? sha
        let table = responses
        return FirmwareCatalogSession(repository: FirmwareRepository(
            fetch: { table[$0] ?? nil }, sha256: { _ in fixed }))
    }

    /// Poll until the session leaves a transient state.
    private func settle(_ s: FirmwareCatalogSession,
                        _ what: String = "settle") async throws {
        for _ in 0..<200 {
            switch s.state {
            case .checking, .downloading: try await Task.sleep(nanoseconds: 5_000_000)
            default: return
            }
        }
        XCTFail("never settled: \(what)")
    }

    func testAReleaseWithAnImageForThisUnitIsOfferedBestFirst() async throws {
        let s = session(manifest([imageJson("out_computer", "v8", size: 64, version: "abc-v8+1"),
                                  imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)

        guard case .ready(let rel, let images, let best, let running, _) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertEqual(rel.tag, "fw-v1.0.0")
        XCTAssertEqual(images.count, 2)
        XCTAssertEqual(best?.board, "v9", "the unit's own revision leads")
        XCTAssertFalse(running)
    }

    func testAnUnprovisionedUnitIsIdentifiedByTheFirmwareItIsRunning() async throws {
        // THE BENCH CASE, 2026-09-10. A real V9 out computer against
        // fw-v0.1.0: no provisioned board (it predates #773 step 2, as every
        // board in the field does), three board-specific images, and the
        // catalog recommended `m1` — the ROCKET-COMPUTER-MINI image — with a
        // tick beside it, because with nothing to rank on the sort fell back
        // to the board string and `m1` beats `v8` and `v9` on spelling.
        //
        // The unit had been saying which board it was the whole time, in the
        // version string the screen displays two rows above.
        let s = session(manifest([imageJson("out_computer", "m1", size: 64, version: "abc-m1+1"),
                                  imageJson("out_computer", "v8", size: 64, version: "abc-v8+1"),
                                  imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: nil,
                runningVersion: "e1a4bee4-v9+20260910-1112")
        try await settle(s)

        guard case .ready(_, _, let best, _, let boardKnown) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertEqual(best?.board, "v9", "the running version says v9, so v9 it is")
        XCTAssertTrue(boardKnown)
    }

    func testAUnitThatSaysNothingAboutItsBoardGetsNoRecommendation() async throws {
        // Nothing provisioned and nothing readable in the version — a pre-#8
        // image, or a board flashed with a suffixless build. Ranking is
        // impossible, so recommend nothing rather than the alphabet.
        let s = session(manifest([imageJson("out_computer", "m1", size: 64, version: "abc-m1+1"),
                                  imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: nil,
                runningVersion: "abc123+20260910")
        try await settle(s)

        guard case .ready(_, let images, let best, _, let boardKnown) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertNil(best, "no basis to choose, so no tick")
        XCTAssertEqual(images.count, 2, "still listed for a deliberate choice")
        XCTAssertFalse(boardKnown, "and the screen can say WHY there is no default")
    }

    func testAProvisionedBoardStillWinsOverTheRunningVersion() async throws {
        // Provisioning is the board's own answer; the version is the image's
        // claim about itself, and a wrongly flashed board claims the wrong
        // thing until it is flashed again. EspImage.check has always ordered
        // them this way and the catalog now matches.
        let s = session(manifest([imageJson("out_computer", "v8", size: 64, version: "abc-v8+1"),
                                  imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v8",
                runningVersion: "abc-v9+1")
        try await settle(s)

        guard case .ready(_, _, let best, _, _) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertEqual(best?.board, "v8", "the board's answer, not the image's claim")
    }

    func testNoNetworkSaysSoAndDoesNotBlameTheFirmware() async throws {
        // At a launch site the overwhelmingly likely cause is no signal.
        // Telling an operator their firmware is missing sends them looking in
        // the wrong place.
        let s = FirmwareCatalogSession(repository: FirmwareRepository(
            fetch: { _ in nil }, sha256: { _ in self.sha }))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)

        guard case .failed(let reason) = s.state else { return XCTFail("expected failed") }
        XCTAssertTrue(reason.contains("connection"), reason)
    }

    func testAReleaseCarryingNothingForThisUnitSaysWhichRelease() async throws {
        let s = session(manifest([imageJson("base_station", "v2", size: 64, version: "abc-v2+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)

        guard case .failed(let reason) = s.state else { return XCTFail("expected failed") }
        XCTAssertTrue(reason.contains("fw-v1.0.0"), reason)
        XCTAssertTrue(reason.contains("out_computer"), reason)
    }

    func testABoardWithNoMatchingImageStillGetsTheListJustNoDefault() async throws {
        // The catalog refuses to GUESS a default when the board is known and
        // nothing matches — but refusing to guess is not refusing to show.
        let s = session(manifest([imageJson("out_computer", "v8", size: 64, version: "abc-v8+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v12")
        try await settle(s)

        guard case .ready(_, let images, let best, _, _) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertEqual(images.count, 1, "still offered for a deliberate choice")
        XCTAssertNil(best, "but never defaulted to a revision this is not")
    }

    func testAnImageTheUnitAlreadyRunsIsFlaggedRatherThanHidden() async throws {
        let s = session(manifest([imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9",
                runningVersion: "abc-v9+1")
        try await settle(s)

        guard case .ready(_, _, _, let running, _) = s.state else {
            return XCTFail("expected ready, got \(s.state)")
        }
        XCTAssertTrue(running, "re-flashing is allowed, but say it is a re-flash")
    }

    func testAVerifiedDownloadHandsTheBytesOver() async throws {
        let bytes = Data((0..<64).map { UInt8($0) })
        let s = session(manifest([imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]),
                        assets: ["https://x.test/fw-v1.0.0/oc.bin": bytes])
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)
        guard case .ready(_, _, let best, _, _) = s.state, let img = best else {
            return XCTFail("expected ready")
        }
        s.download(img)
        try await settle(s)

        guard case .downloaded(_, _, let got) = s.state else {
            return XCTFail("expected downloaded, got \(s.state)")
        }
        XCTAssertEqual(got, bytes)
    }

    func testACorruptDownloadIsRefusedAndNeverSurfacesBytes() async throws {
        let bytes = Data((0..<64).map { UInt8($0) })
        let s = session(manifest([imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]),
                        assets: ["https://x.test/fw-v1.0.0/oc.bin": bytes],
                        sha256: String(repeating: "b", count: 64))
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)
        guard case .ready(_, _, let best, _, _) = s.state, let img = best else {
            return XCTFail("expected ready")
        }
        s.download(img)
        try await settle(s)

        guard case .failed(let reason) = s.state else { return XCTFail("expected failed") }
        XCTAssertTrue(reason.contains("does not match"), reason)
    }

    func testAFailedTransferReadsAsTryAgainNotAsABadImage() async throws {
        let s = session(manifest([imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]),
                        assets: ["https://x.test/fw-v1.0.0/oc.bin": Data?.none])
        s.check(expectedProject: EspImage.projectOC, provisionedBoard: "v9")
        try await settle(s)
        guard case .ready(_, _, let best, _, _) = s.state, let img = best else {
            return XCTFail("expected ready")
        }
        s.download(img)
        try await settle(s)

        guard case .failed(let reason) = s.state else { return XCTFail("expected failed") }
        XCTAssertTrue(reason.contains("did not finish"), reason)
    }

    func testDownloadingBeforeCheckingIsRefusedRatherThanCrashing() async throws {
        let s = session(manifest([imageJson("out_computer", "v9", size: 64, version: "abc-v9+1")]))
        s.download(FirmwareImage(file: "x.bin", project: "out_computer", version: "v",
                                 chipId: 9, chip: "ESP32-S3", sizeBytes: 64,
                                 sha256: sha, board: "v9", idfVersion: "", buildDate: ""))

        guard case .failed(let reason) = s.state else { return XCTFail("expected failed") }
        XCTAssertTrue(reason.contains("check for updates"), reason)
    }
}
