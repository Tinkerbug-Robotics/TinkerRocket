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
/// second, and removing an annotation dismisses its callout. Tapping either
/// pin popped a callout that vanished inside a second — on the screen whose
/// whole job is telling you where to walk.
///
/// The pins are now long-lived objects mutated in place.
final class MapAnnotationReconcileTests: XCTestCase {

    private let padA = CLLocationCoordinate2D(latitude: 40.1, longitude: -105.2)
    private let padB = CLLocationCoordinate2D(latitude: 40.2, longitude: -105.3)

    private func makeCoordinator() -> RocketMapView.Coordinator {
        RocketMapView.Coordinator(
            RocketMapView(tileSource: .constant(.appleStandard),
                          region: .constant(MKCoordinateRegion()))
        )
    }

    /// THE regression. A selected annotation is one with an open callout;
    /// MapKit clears the selection when the annotation is removed.
    func testOpenCalloutSurvivesRepeatedUpdates() {
        let map = MKMapView()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "3 sats · 1 s ago",
                              landing: nil, landingSubtitle: nil)
        let pin = try? XCTUnwrap(map.annotations.first { !($0 is MKUserLocation) })
        map.selectAnnotation(pin as! MKAnnotation, animated: false)
        XCTAssertEqual(map.selectedAnnotations.count, 1, "precondition: callout is open")

        // Ten ticks of the 1 Hz clock, subtitle advancing each time — exactly
        // what the TimelineView does while the operator reads the callout.
        for age in 2...11 {
            coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "3 sats · \(age) s ago",
                                  landing: nil, landingSubtitle: nil)
        }

        XCTAssertEqual(map.selectedAnnotations.count, 1,
                       "the callout was dismissed by a telemetry tick")
    }

    func testSubtitleUpdatesInPlaceOnTheSameObject() {
        let map = MKMapView()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "1 s ago",
                              landing: nil, landingSubtitle: nil)
        let first = map.annotations.first { !($0 is MKUserLocation) } as? MKPointAnnotation

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: "2 s ago",
                              landing: nil, landingSubtitle: nil)
        let second = map.annotations.first { !($0 is MKUserLocation) } as? MKPointAnnotation

        // Same object, new text: that is what keeps an open callout alive AND
        // keeps the age ticking inside it.
        XCTAssertTrue(first === second, "annotation was replaced, not updated")
        XCTAssertEqual(second?.subtitle, "2 s ago")
    }

    func testMovingRocketMovesTheSamePin() {
        let map = MKMapView()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        let before = map.annotations.first { !($0 is MKUserLocation) } as? MKPointAnnotation
        coord.syncAnnotations(on: map, rocket: padB, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        let after = map.annotations.first { !($0 is MKUserLocation) } as? MKPointAnnotation

        XCTAssertTrue(before === after)
        XCTAssertEqual(after?.coordinate.latitude ?? 0, padB.latitude, accuracy: 1e-9)
        XCTAssertEqual(map.annotations.filter { !($0 is MKUserLocation) }.count, 1,
                       "a move must not leave the old pin behind")
    }

    func testLandingPinAddedAndRemovedWithThePrediction() {
        let map = MKMapView()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.filter { !($0 is MKUserLocation) }.count, 1)

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: padB, landingSubtitle: "±30 m")
        XCTAssertEqual(map.annotations.filter { !($0 is MKUserLocation) }.count, 2)

        // Predictor going quiet must take its pin with it, not strand it.
        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.filter { !($0 is MKUserLocation) }.count, 1)
    }

    func testRocketPinRemovedWhenTheFixGoesAway() {
        let map = MKMapView()
        let coord = makeCoordinator()

        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertTrue(map.annotations.filter { !($0 is MKUserLocation) }.isEmpty)

        // And comes back on the same object rather than accumulating.
        coord.syncAnnotations(on: map, rocket: padA, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil)
        XCTAssertEqual(map.annotations.filter { !($0 is MKUserLocation) }.count, 1)
    }

    // MARK: - Data overlays

    func testOverlaysRebuiltOnlyWhenTheirInputsChange() {
        let map = MKMapView()
        let coord = makeCoordinator()
        let track = [padA, padB]

        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 30, descentTrack: track)
        let first = map.overlays.compactMap { $0 as? MKCircle }.first
        XCTAssertNotNil(first)
        XCTAssertEqual(map.overlays.count, 2)          // circle + polyline

        // A latched prediction re-sent on every tick must not re-render.
        for _ in 0..<5 {
            coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 30, descentTrack: track)
        }
        XCTAssertTrue(map.overlays.compactMap { $0 as? MKCircle }.first === first,
                      "unchanged overlays were torn down and rebuilt")
        XCTAssertEqual(map.overlays.count, 2, "overlays accumulated")
    }

    func testShrinkingUncertaintyRedrawsTheCircle() {
        let map = MKMapView()
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
        let map = MKMapView()
        let coord = makeCoordinator()
        coord.syncDataOverlays(on: map, landing: padB, uncertaintyRadiusM: 0, descentTrack: [])
        XCTAssertTrue(map.overlays.isEmpty)
    }
}
