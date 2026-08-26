import XCTest
import MapKit
import SwiftUI
@testable import TinkerRocketApp

/// #836 item 2 — the rocket and predicted-landing callouts could never be read.
///
/// `updateUIView` opened with a `removeAnnotations` / `addAnnotation` pair,
/// commented "cheap" — which it is, in CPU. What it cost was the callout.
/// The parent wraps RocketMapView in `TimelineView(.periodic(by: 1.0))` so the
/// "last fix Ns ago" subtitle keeps ticking, so the teardown ran once a
/// second — and **removing an annotation dismisses its callout**. Tapping
/// either pin popped a callout that vanished inside a second, on the screen
/// whose whole job is telling you where to walk.
///
/// These tests assert the mechanism rather than MapKit's rendered selection:
/// what dismisses a callout is the `removeAnnotation` call, so "the callout
/// survives" is exactly "no remove happened". That also keeps the suite off a
/// live MKMapView — instantiating one here crashed the whole xctest process on
/// the CI simulator while passing locally.
private final class MapSurfaceSpy: RocketMapSurface {
    private(set) var added: [MKAnnotation] = []
    private(set) var removed: [MKAnnotation] = []
    private(set) var overlays: [MKOverlay] = []
    private(set) var overlayRemovals = 0

    var drawnOverlays: [MKOverlay] { overlays }

    /// Pins currently on the map.  `added`/`removed` are CALL LOGS — a pin
    /// that is added, removed and re-added appears twice in `added` — while
    /// this is the live set.  The distinction is the point of the suite: the
    /// bug was extra calls, not a wrong final picture.
    private(set) var annotations: [MKAnnotation] = []

    func addAnnotation(_ annotation: MKAnnotation) {
        added.append(annotation)
        if !annotations.contains(where: { $0 === annotation }) {
            annotations.append(annotation)
        }
    }
    func removeAnnotation(_ annotation: MKAnnotation) {
        removed.append(annotation)
        annotations.removeAll { $0 === annotation }
    }
    func addOverlay(_ overlay: MKOverlay) { overlays.append(overlay) }
    func removeOverlays(_ overlays: [MKOverlay]) {
        overlayRemovals += 1
        self.overlays.removeAll { o in overlays.contains { $0 === o } }
    }
}

final class MapAnnotationReconcileTests: XCTestCase {

    private let padA = CLLocationCoordinate2D(latitude: 40.1, longitude: -105.2)
    private let padB = CLLocationCoordinate2D(latitude: 40.2, longitude: -105.3)

    private func makeCoordinator() -> RocketMapView.Coordinator {
        RocketMapView.Coordinator(
            RocketMapView(tileSource: .constant(.appleStandard),
                          region: .constant(MKCoordinateRegion()))
        )
    }

    /// THE regression. Ten ticks of the 1 Hz clock with the subtitle
    /// advancing — exactly what happens while the operator reads the callout.
    func testTickingTheClockNeverRemovesAPin() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "3 sats · 1 s ago",
                              landing: padB, landingSubtitle: "±30 m")
        XCTAssertEqual(map.added.count, 2)

        for age in 2...11 {
            coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "3 sats · \(age) s ago",
                                  landing: padB, landingSubtitle: "±30 m")
        }

        XCTAssertTrue(map.removed.isEmpty,
                      "a pin was removed on a telemetry tick — that dismisses its callout")
        XCTAssertEqual(map.added.count, 2, "pins were re-added, so they had been torn down")
    }

    func testSubtitleUpdatesInPlaceOnTheSameObject() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "1 s ago",
                              landing: nil, landingSubtitle: nil)
        let pin = map.added.first as? MKPointAnnotation
        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "2 s ago",
                              landing: nil, landingSubtitle: nil)

        // Same object, new text: that is what keeps an open callout alive AND
        // keeps the age ticking inside it.
        XCTAssertEqual(map.added.count, 1, "annotation was replaced, not updated")
        XCTAssertEqual(pin?.subtitle, "2 s ago")
        XCTAssertEqual(pin?.title, "TinkerRocket")
    }

    func testMovingRocketMovesTheSamePin() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        let pin = map.added.first as? MKPointAnnotation
        coord.syncAnnotations(on: map, rocket: padB, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)

        XCTAssertEqual(map.added.count, 1, "a move must not leave the old pin behind")
        XCTAssertTrue(map.removed.isEmpty)
        XCTAssertEqual(pin?.coordinate.latitude ?? 0, padB.latitude, accuracy: 1e-9)
    }

    func testLandingPinAddedAndRemovedWithThePrediction() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.count, 1)

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: padB, landingSubtitle: "±30 m")
        XCTAssertEqual(map.annotations.count, 2)

        // The predictor going quiet must take its pin with it, not strand it —
        // and must not take the rocket's pin along with it.
        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.count, 1)
        XCTAssertEqual(map.removed.count, 1)
        // The landing pin, not the rocket's.  Identified by title because the
        // annotation subclass is file-private to MapView.swift.
        XCTAssertEqual(map.removed.first?.title ?? nil, "Predicted Landing")
    }

    func testRocketPinRemovedWhenTheFixGoesAwayAndComesBackOnce() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertTrue(map.annotations.isEmpty)

        // Repeated empty passes must not remove it again.
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.removed.count, 1)

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.count, 1)
    }

    // MARK: - Data overlays

    func testOverlaysRebuiltOnlyWhenTheirInputsChange() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        let track = [padA, padB]

        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 30, descentTrack: track)
        let circle = map.overlays.compactMap { $0 as? MKCircle }.first
        XCTAssertNotNil(circle)
        XCTAssertEqual(map.overlays.count, 2)          // circle + polyline
        XCTAssertEqual(map.overlayRemovals, 1)

        // A latched prediction re-sent on every tick must not re-render.
        for _ in 0..<5 {
            coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 30, descentTrack: track)
        }
        XCTAssertEqual(map.overlayRemovals, 1, "unchanged overlays were torn down and rebuilt")
        XCTAssertTrue(map.overlays.compactMap { $0 as? MKCircle }.first === circle)
        XCTAssertEqual(map.overlays.count, 2, "overlays accumulated")
    }

    func testShrinkingUncertaintyRedrawsTheCircle() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()

        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 90, descentTrack: [])
        let wide = map.overlays.compactMap { $0 as? MKCircle }.first
        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 30, descentTrack: [])
        let tight = map.overlays.compactMap { $0 as? MKCircle }.first

        XCTAssertFalse(wide === tight, "a re-prediction must actually redraw")
        XCTAssertEqual(tight?.radius ?? 0, 30, accuracy: 0.5)
        XCTAssertEqual(map.overlays.count, 1)
    }

    func testZeroRadiusDrawsNoCircle() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 0, descentTrack: [])
        XCTAssertTrue(map.overlays.isEmpty)
    }
}
