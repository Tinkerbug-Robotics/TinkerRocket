import CryptoKit
import XCTest
@testable import TinkerRocketApp

/// #773: firmware kept on the phone so a field with no signal is survivable.
/// Android twin: `FirmwareCacheTest.kt`, case for case.
final class FirmwareCacheTests: XCTestCase {

    private func realSha(_ d: Data) -> String {
        SHA256.hash(data: d).map { String(format: "%02x", $0) }.joined()
    }

    private func tmp() -> URL {
        let u = FileManager.default.temporaryDirectory
            .appendingPathComponent("fwcache-\(UUID().uuidString)")
        try? FileManager.default.createDirectory(at: u, withIntermediateDirectories: true)
        return u
    }

    private var bytes: Data { Data((0..<512).map { UInt8($0 % 251) }) }
    private var sha: String { realSha(bytes) }

    private func image(_ file: String = "out_computer-V9.bin",
                       sha s: String? = nil, size: Int64 = 512) -> FirmwareImage {
        FirmwareImage(file: file, project: "out_computer", version: "abc-v9+1",
                      chipId: 9, chip: "ESP32-S3", sizeBytes: size,
                      sha256: s ?? sha, board: "v9", idfVersion: "", buildDate: "")
    }

    private func cache(_ dir: URL) -> FirmwareCache {
        FirmwareCache(directory: dir, sha256: { self.realSha($0) })
    }

    func testAnImageSurvivesARoundTripAndIsHandedBack() {
        let c = cache(tmp())
        XCTAssertTrue(c.putImage(image(), bytes))
        XCTAssertTrue(c.hasImage(image()))
        XCTAssertEqual(c.image(image()), bytes)
    }

    func testACorruptedFileIsRefusedAndDeletedRatherThanServed() {
        // The file name is a CLAIM about the contents, not proof: a
        // half-written file from a killed app carries the right name and the
        // wrong bytes. Flashing a rocket from something nobody re-checked is
        // exactly what the manifest exists to prevent, so every read re-hashes.
        let dir = tmp()
        let c = cache(dir)
        c.putImage(image(), bytes)
        let f = dir.appendingPathComponent("images/\(sha)")
        try? Data(repeating: 7, count: 512).write(to: f)   // same size, wrong bytes

        XCTAssertNil(c.image(image()), "content decides, not the name")
        XCTAssertFalse(FileManager.default.fileExists(atPath: f.path),
                       "and it is dropped, not left to be retried")
    }

    func testATruncatedFileIsRefusedToo() {
        let dir = tmp()
        let c = cache(dir)
        c.putImage(image(), bytes)
        try? Data(count: 100).write(to: dir.appendingPathComponent("images/\(sha)"))

        XCTAssertNil(c.image(image()))
    }

    func testTheCatalogRoundTripsThroughTheSameParsersTheNetworkUses() {
        // One codec for release JSON, not two that can disagree.
        let c = cache(tmp())
        let release = FirmwareRelease(
            tag: "fw-v0.1.0", isPrerelease: false,
            assets: ["manifest.json": "https://x.test/fw-v0.1.0/manifest.json",
                     "out_computer-V9.bin": "https://x.test/fw-v0.1.0/oc.bin"])
        let json = "{\"manifest_version\":1,\"tag\":\"fw-v0.1.0\",\"images\":["
            + "{\"file\":\"out_computer-V9.bin\",\"project\":\"out_computer\","
            + "\"version\":\"abc-v9+1\",\"chip_id\":9,\"chip\":\"ESP32-S3\","
            + "\"size\":512,\"sha256\":\"\(sha)\",\"board\":\"v9\"}]}"
        let manifest = FirmwareManifest.parse(json)!

        XCTAssertTrue(c.putCatalog(FetchedCatalog(release: release, manifest: manifest,
                                                  manifestJSON: json)))
        guard let back = c.catalog() else { return XCTFail("nothing came back") }
        XCTAssertEqual(back.0.tag, "fw-v0.1.0")
        XCTAssertEqual(back.0.assets, release.assets, "the download URLs must survive")
        XCTAssertEqual(back.1.images.count, 1)
        XCTAssertEqual(back.1.images[0].sha256, sha)
    }

    func testNoCatalogIsNilRatherThanACrash() {
        XCTAssertNil(cache(tmp()).catalog())
    }

    func testAPrereleaseCatalogIsReadableBack() {
        // The cache reads with includePrereleases = true deliberately: it is
        // replaying what was already chosen, not choosing again, and dropping
        // it here would silently empty the cache of an rc a tester fetched.
        let c = cache(tmp())
        let release = FirmwareRelease(tag: "fw-v0.2.0-rc1", isPrerelease: true,
                                      assets: ["manifest.json": "https://x.test/m.json"])
        let json = "{\"manifest_version\":1,\"tag\":\"fw-v0.2.0-rc1\",\"images\":["
            + "{\"file\":\"o.bin\",\"project\":\"out_computer\",\"version\":\"v\","
            + "\"chip_id\":9,\"chip\":\"ESP32-S3\",\"size\":512,\"sha256\":\"\(sha)\"}]}"
        c.putCatalog(FetchedCatalog(release: release,
                                    manifest: FirmwareManifest.parse(json)!, manifestJSON: json))

        XCTAssertEqual(c.catalog()?.0.tag, "fw-v0.2.0-rc1")
    }

    func testPruneKeepsTheCurrentReleaseAndDropsTheRest() {
        let dir = tmp()
        let c = cache(dir)
        let other = Data(repeating: 3, count: 64)
        c.putImage(image(), bytes)
        c.putImage(image("old.bin", sha: realSha(other), size: 64), other)
        XCTAssertEqual(c.heldShas().count, 2)

        c.prune(keep: [sha])

        XCTAssertEqual(c.heldShas(), [sha])
        XCTAssertEqual(c.image(image()), bytes, "the kept one is still good")
    }

    func testTwoImagesWithTheSameBytesShareOneFile() {
        // Content addressing: an unchanged image across two releases is stored
        // once, so a phone does not pay twice for the same bytes.
        let c = cache(tmp())
        c.putImage(image("a.bin"), bytes)
        c.putImage(image("b.bin"), bytes)

        XCTAssertEqual(c.heldShas().count, 1)
        XCTAssertEqual(c.bytesHeld(), 512)
    }
}
