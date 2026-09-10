import XCTest
@testable import TinkerRocketApp

/// #773 step 4: finding the firmware release on GitHub and fetching from it.
/// Android twin: `FirmwareReleaseTest.kt`, case for case.
final class FirmwareReleaseTests: XCTestCase {

    private func rel(_ tag: String, prerelease: Bool = false,
                     assets: [String] = ["manifest.json", "flight_computer-V9.bin"]) -> String {
        let a = assets.map {
            "{\"name\":\"\($0)\",\"browser_download_url\":\"https://example.test/\(tag)/\($0)\"}"
        }.joined(separator: ",")
        return "{\"tag_name\":\"\(tag)\",\"prerelease\":\(prerelease),\"assets\":[\(a)]}"
    }

    private func listing(_ releases: String...) -> String {
        "[\(releases.joined(separator: ","))]"
    }

    func testBoardAndAppReleasesAreNotFirmwareReleases() throws {
        // The trap this exists for. GitHub's /releases/latest returns the
        // newest release of ANY kind, and this repo also tags board and
        // Android releases — at the time of writing it answers a gerber zip.
        // Firmware is found by tag prefix, never by "latest". The fixture is
        // a real capture of this repo's /releases, shared with Android.
        let here = URL(fileURLWithPath: #filePath)
        let repo = here.deletingLastPathComponent()   // TinkerRocketAppTests
            .deletingLastPathComponent()              // TinkerRocketApp
            .deletingLastPathComponent()              // repo root
        let golden = repo
            .appendingPathComponent("TinkerRocketAndroid/core/protocol/src/test/resources")
            .appendingPathComponent("github_releases_golden.json")
        let data = try Data(contentsOf: golden)
        XCTAssertTrue(String(decoding: data, as: UTF8.self)
            .contains("rocket-computer-mini-v1.0.1"))
        XCTAssertNil(FirmwareReleaseLocator.newest(data))
        XCTAssertTrue(FirmwareReleaseLocator.firmwareReleases(data).isEmpty)
    }

    func testPicksTheNewestByVersionNotByApiOrder() {
        // The API sorts by creation date, which is usually the same thing and
        // occasionally is not — a re-cut or edited tag moves in that ordering.
        let json = listing(rel("fw-v1.0.0"), rel("fw-v1.10.0"), rel("fw-v1.9.0"))
        XCTAssertEqual(FirmwareReleaseLocator.newest(json)?.tag, "fw-v1.10.0")
    }

    func testPrereleasesAreSkippedUnlessAskedFor() {
        let json = listing(rel("fw-v2.0.0", prerelease: true), rel("fw-v1.0.0"))
        XCTAssertEqual(FirmwareReleaseLocator.newest(json)?.tag, "fw-v1.0.0")
        XCTAssertEqual(
            FirmwareReleaseLocator.newest(json, includePrereleases: true)?.tag, "fw-v2.0.0")
    }

    func testAReleaseWithNoManifestIsSkipped() {
        // Either it predates the manifest or the publish failed. Either way
        // there is nothing the app can act on.
        let json = listing(rel("fw-v2.0.0", assets: ["flight_computer-V9.bin"]),
                           rel("fw-v1.0.0"))
        XCTAssertEqual(FirmwareReleaseLocator.newest(json)?.tag, "fw-v1.0.0")
    }

    func testABareVersionOutranksTheSameVersionWithASuffix() {
        let json = listing(rel("fw-v1.0.0-rc1"), rel("fw-v1.0.0"))
        XCTAssertEqual(FirmwareReleaseLocator.newest(json)?.tag, "fw-v1.0.0")
    }

    func testJunkIsAnEmptyListNotACrash() {
        XCTAssertTrue(FirmwareReleaseLocator.firmwareReleases("").isEmpty)
        XCTAssertTrue(FirmwareReleaseLocator.firmwareReleases("not json").isEmpty)
        XCTAssertTrue(FirmwareReleaseLocator.firmwareReleases("{}").isEmpty)
        XCTAssertNil(FirmwareReleaseLocator.newest("[]"))
    }

    // MARK: - fetching

    private let bytes = Data((0..<64).map { UInt8($0) })
    private let goodSha = String(repeating: "a", count: 64)

    private func image(sha: String? = nil, size: Int64 = 64) -> FirmwareImage {
        FirmwareImage(file: "flight_computer-V9.bin", project: "flight_computer",
                      version: "abc-v9+1", chipId: 18, chip: "ESP32-P4",
                      sizeBytes: size, sha256: sha ?? goodSha, board: "v9",
                      idfVersion: "v6.0.1", buildDate: "Sep  9 2026")
    }

    private func repo(_ responses: [String: Data?],
                      sha: String? = nil) -> FirmwareRepository {
        let fixed = sha ?? goodSha
        return FirmwareRepository(fetch: { responses[$0] ?? nil }, sha256: { _ in fixed })
    }

    func testLatestManifestWalksTheListingThenTheManifest() async {
        let manifestJson = "{\"manifest_version\":1,\"tag\":\"fw-v1.0.0\",\"images\":["
            + "{\"file\":\"flight_computer-V9.bin\",\"project\":\"flight_computer\","
            + "\"size\":64,\"sha256\":\"\(goodSha)\",\"board\":\"v9\","
            + "\"chip\":\"ESP32-P4\",\"chip_id\":18}]}"
        let r = await repo([
            FirmwareReleaseLocator.releasesURL: Data(listing(rel("fw-v1.0.0")).utf8),
            "https://example.test/fw-v1.0.0/manifest.json": Data(manifestJson.utf8),
        ]).latestManifest()
        XCTAssertEqual(r?.release.tag, "fw-v1.0.0")
        XCTAssertEqual(r?.manifest.images.count, 1)
        // The raw text comes back too, so a cache stores what it verified
        // rather than a re-serialization that could differ from it.
        XCTAssertEqual(r?.manifestJSON, manifestJson)
    }

    func testAReleaseRoundTripsThroughItsOwnListingJSON() {
        // The cache writes this and reads it back through the SAME parser the
        // network path uses, so there is one codec for release JSON rather
        // than two that can disagree. Quoting matters: an asset name or URL
        // with a quote or backslash in it must survive.
        let original = FirmwareRelease(
            tag: "fw-v1.0.0", isPrerelease: true,
            assets: ["manifest.json": "https://example.test/a b/manifest.json",
                     "odd\"name.bin": "https://example.test/x\\y.bin"])
        let back = FirmwareReleaseLocator
            .firmwareReleases(original.toListingJSON(), includePrereleases: true)
        XCTAssertEqual(back.count, 1)
        XCTAssertEqual(back.first, original)
    }

    func testNoNetworkIsNilNotAThrow() async {
        let r = await repo([:]).latestManifest()
        XCTAssertNil(r?.release)
    }

    func testAVerifiedDownloadReturnsTheBytes() async throws {
        let release = try XCTUnwrap(FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0"))))
        let got = await repo([
            "https://example.test/fw-v1.0.0/flight_computer-V9.bin": bytes,
        ]).download(release: release, image: image())
        XCTAssertEqual(got, .ok(bytes))
    }

    func testATruncatedDownloadIsRefusedAndSaysSo() async throws {
        // Size is checked before the hash purely so this reports as truncated
        // rather than as a checksum mismatch. Same refusal, clearer reason.
        let release = try XCTUnwrap(FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0"))))
        let got = await repo([
            "https://example.test/fw-v1.0.0/flight_computer-V9.bin": Data(count: 10),
        ]).download(release: release, image: image())
        guard case .corrupt(let why) = got else { return XCTFail("expected corrupt, got \(got)") }
        XCTAssertTrue(why.contains("10 bytes"), why)
    }

    func testAChecksumMismatchIsRefusedAndNeverReturned() async throws {
        // A tampered file and a corrupted one look identical here. Neither
        // belongs on a flight computer, so neither is handed back.
        let release = try XCTUnwrap(FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0"))))
        let got = await repo(
            ["https://example.test/fw-v1.0.0/flight_computer-V9.bin": bytes],
            sha: String(repeating: "b", count: 64)
        ).download(release: release, image: image())
        guard case .corrupt(let why) = got else { return XCTFail("expected corrupt, got \(got)") }
        XCTAssertTrue(why.contains("checksum"), why)
    }

    func testAnImageNotInTheReleaseIsUnreachableNotCorrupt() async throws {
        let release = try XCTUnwrap(
            FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0", assets: ["manifest.json"]))))
        let got = await repo([:]).download(release: release, image: image())
        guard case .unreachable = got else { return XCTFail("expected unreachable, got \(got)") }
    }

    func testAFailedTransferIsUnreachableNotCorrupt() async throws {
        // The distinction matters to the operator: one is "try again on better
        // signal", the other is "do not flash this".
        let release = try XCTUnwrap(FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0"))))
        let got = await repo(
            ["https://example.test/fw-v1.0.0/flight_computer-V9.bin": Data?.none]
        ).download(release: release, image: image())
        guard case .unreachable = got else { return XCTFail("expected unreachable, got \(got)") }
    }
}
