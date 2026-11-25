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
    /// Restyle calls (a pin's focus changed) — must never be a remove/add.
    private(set) var restyled: [MKAnnotation] = []
    func restyleRocketPin(_ annotation: MKAnnotation) { restyled.append(annotation) }
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
                          region: .constant(MKCoordinateRegion()),
                          followRocket: .constant(true))
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

/// Three rockets, one base station: one pin per displayed rocket, under the
/// same #836 rule — a pin is added when its rocket joins the set and removed
/// when it leaves; every other change is made in place.
final class MultiRocketMapPinTests: XCTestCase {

    private let k1 = RocketKey(networkID: 7, rocketID: 1)
    private let k2 = RocketKey(networkID: 7, rocketID: 2)
    private let k3 = RocketKey(networkID: 7, rocketID: 3)

    private func makeCoordinator() -> RocketMapView.Coordinator {
        RocketMapView.Coordinator(
            RocketMapView(tileSource: .constant(.appleStandard),
                          region: .constant(MKCoordinateRegion()),
                          followRocket: .constant(true))
        )
    }

    private func pin(_ key: RocketKey, _ title: String, lat: Double,
                     subtitle: String? = nil, focused: Bool = false) -> RocketMapPin {
        RocketMapPin(key: key,
                     coordinate: CLLocationCoordinate2D(latitude: lat, longitude: -118.2),
                     title: title, subtitle: subtitle, focused: focused)
    }

    private func sync(_ coord: RocketMapView.Coordinator, _ map: MapSurfaceSpy, _ pins: [RocketMapPin]) {
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil,
                              landing: nil, landingSubtitle: nil, pins: pins)
    }

    func testThreeRockets_threePinsTitledByName() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        sync(coord, map, [pin(k1, "Booster", lat: 34.1, focused: true),
                          pin(k2, "Pad Rocket", lat: 34.2),
                          pin(k3, "Dart", lat: 34.3)])
        XCTAssertEqual(map.annotations.count, 3)
        XCTAssertEqual(Set(map.annotations.compactMap { $0.title ?? nil }),
                       ["Booster", "Pad Rocket", "Dart"])
    }

    func testTicksAndMoves_neverRemoveOrReAdd() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        for tick in 0..<10 {
            sync(coord, map, [pin(k1, "Booster", lat: 34.1 + Double(tick) * 0.001,
                                  subtitle: "9 sats • \(tick)s ago", focused: true),
                              pin(k2, "Pad Rocket", lat: 34.2, subtitle: "8 sats • \(tick)s ago"),
                              pin(k3, "Dart", lat: 34.3, subtitle: "10 sats • \(tick)s ago")])
        }
        XCTAssertEqual(map.added.count, 3, "pins were re-added, so they had been torn down")
        XCTAssertTrue(map.removed.isEmpty, "a tick removed a pin — that dismisses its callout")
        let booster = map.annotations.first { $0.title ?? nil == "Booster" }
        XCTAssertEqual(booster?.coordinate.latitude ?? 0, 34.109, accuracy: 1e-9)
        XCTAssertEqual(booster?.subtitle ?? nil, "9 sats • 9s ago")
    }

    func testARocketLeaving_removesOnlyItsPin_andComingBackAddsItOnce() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        let all = [pin(k1, "Booster", lat: 34.1, focused: true),
                   pin(k2, "Pad Rocket", lat: 34.2),
                   pin(k3, "Dart", lat: 34.3)]
        sync(coord, map, all)
        sync(coord, map, [all[0], all[2]])          // Pad Rocket hidden
        sync(coord, map, [all[0], all[2]])
        XCTAssertEqual(map.removed.count, 1)
        XCTAssertEqual(map.removed.first?.title ?? nil, "Pad Rocket")
        XCTAssertEqual(map.annotations.count, 2)

        sync(coord, map, all)
        XCTAssertEqual(map.added.count, 4)
        XCTAssertEqual(map.annotations.count, 3)
    }

    func testFocusChange_restylesInPlace() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        sync(coord, map, [pin(k1, "Booster", lat: 34.1, focused: true),
                          pin(k2, "Pad Rocket", lat: 34.2)])
        sync(coord, map, [pin(k1, "Booster", lat: 34.1),
                          pin(k2, "Pad Rocket", lat: 34.2, focused: true)])
        XCTAssertEqual(map.added.count, 2)
        XCTAssertTrue(map.removed.isEmpty, "a focus switch must not cost an open callout")
        XCTAssertEqual(Set(map.restyled.compactMap { $0.title ?? nil }), ["Booster", "Pad Rocket"])

        // Unchanged focus: nothing to restyle.
        sync(coord, map, [pin(k1, "Booster", lat: 34.1),
                          pin(k2, "Pad Rocket", lat: 34.2, focused: true)])
        XCTAssertEqual(map.restyled.count, 2)
    }

    func testLandingPin_isIndependentOfTheRocketSet() {
        let map = MapSurfaceSpy()
        let coord = makeCoordinator()
        let landing = CLLocationCoordinate2D(latitude: 34.0, longitude: -118.0)
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil, landing: landing,
                              landingSubtitle: "±30 m",
                              pins: [pin(k1, "Booster", lat: 34.1), pin(k2, "Dart", lat: 34.2)])
        coord.syncAnnotations(on: map, rocket: nil, rocketSubtitle: nil, landing: landing,
                              landingSubtitle: "±30 m", pins: [pin(k1, "Booster", lat: 34.1)])
        XCTAssertEqual(map.annotations.count, 2, "Booster + predicted landing")
        XCTAssertEqual(map.removed.first?.title ?? nil, "Dart")
    }

    /// The fix cache keys on the LIVE network id, so a fix recorded before the
    /// identity readback sits under network 0 next to the real one. Only the
    /// dashboard's own keys may draw.
    func testPinsComeFromTheDashboardsKeysOnly_notStaleNetworkZeroEntries() {
        func fix(_ rid: UInt8, _ lat: Double) -> LastValidRocketFix {
            LastValidRocketFix(rocketID: rid, latitude: lat, longitude: -118.2,
                               numSats: 9, fixDate: Date())
        }
        let subjects = [k1, k2, k3].map {
            RocketSubject(id: .known($0), name: "Rocket \($0.rocketID)", direct: nil, relays: [])
        }
        let fixes: [RocketKey: LastValidRocketFix] = [
            RocketKey(networkID: 0, rocketID: 1): fix(1, 34.0),   // pre-identity leftover
            k1: fix(1, 34.1), k2: fix(2, 34.2), k3: fix(3, 34.3),
            RocketKey(networkID: 7, rocketID: 9): fix(9, 35.0),   // not on the dashboard
        ]
        let pins = MapView.rocketPins(for: subjects, fixes: fixes, focused: k2) { _ in nil }
        XCTAssertEqual(pins.map(\.key), [k1, k2, k3])
        XCTAssertEqual(pins.map(\.focused), [false, true, false])
        XCTAssertEqual(pins.first?.coordinate.latitude ?? 0, 34.1, accuracy: 1e-9)
    }
}
