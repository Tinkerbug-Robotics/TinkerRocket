import SwiftUI
import XCTest
@testable import TinkerRocketApp

/// Render-smoke tests for the two Android → iOS back-ports (flag chips +
/// health dot row): construct synthetic telemetry, render each view with
/// ImageRenderer, and assert a real image came out.  Catches crashes and
/// zero-size layouts in view bodies that nothing else executes headlessly.
/// PNGs land in NSTemporaryDirectory()/tr_backport_previews for bench
/// eyeballing — a path that exists everywhere, so CI just ignores them.
@MainActor
final class BackportRenderTests: XCTestCase {

    private func render<V: View>(_ view: V, name: String) throws -> CGSize {
        let renderer = ImageRenderer(content: view.frame(width: 390).padding(12))
        renderer.scale = 3
        let image = try XCTUnwrap(renderer.uiImage, "\(name) rendered no image")
        XCTAssertGreaterThan(image.size.width, 10)
        XCTAssertGreaterThan(image.size.height, 10)

        // keepAlways attachment: the sim reclaims the test container the
        // moment the run ends, so a tmp-dir write is unreadable afterwards —
        // the result bundle is the only durable place for render output.
        let attachment = XCTAttachment(image: image)
        attachment.name = name
        attachment.lifetime = .keepAlways
        add(attachment)
        return image.size
    }

    func testFlagChips_renderAllStates() throws {
        var off = TelemetryData()
        off.flight_status_bits = 0
        _ = try render(FlightEventFlagsView(telemetry: off), name: "chips_all_off")

        var mid = TelemetryData()
        mid.flight_status_bits = 0x01 | 0x200 | 0x40   // LAUNCH + BURNOUT + LOG
        _ = try render(FlightEventFlagsView(telemetry: mid), name: "chips_mid_flight")

        var landed = TelemetryData()
        landed.flight_status_bits = 0x01 | 0x200 | 0x04 | 0x08 | 0x40
        _ = try render(FlightEventFlagsView(telemetry: landed), name: "chips_landed")
    }

    func testHealthCard_dotRow_rendersMixedStates() throws {
        var t = TelemetryData()
        // baro OK, imu DEGRADED, ekf BAD, mag NA, gnss OK, batt OK,
        // pyro1 OK (shift 12), storage OK (shift 20).
        t.sensor_health = 1 | (2 << 2) | (3 << 4) | (0 << 6) | (1 << 8) |
            (1 << 10) | (1 << 12) | (1 << 20)

        // The row is rendered DIRECTLY: ImageRenderer draws ScrollView
        // children as empty (verified — the first cut rendered the card and
        // got a banner over blank space), so the card render below only
        // proves the banner, and this one proves the dots.
        _ = try render(HealthDotRow(rows: t.sensorHealthRows), name: "health_dot_row")
        _ = try render(HealthCardView(telemetry: t), name: "health_card_banner")

        XCTAssertEqual(
            t.sensorHealthRows.map(\.name),
            ["Baro", "IMU", "EKF", "GNSS", "Battery", "Mag", "Storage", "Pyro 1"],
            "row set drives the dot row; pin it so a rename shows up here"
        )
    }
}
