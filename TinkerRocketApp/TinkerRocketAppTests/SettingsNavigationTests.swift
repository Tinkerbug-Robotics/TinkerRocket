import SwiftUI
import XCTest
@testable import TinkerRocketApp

/// Settings follows the fleet's live BLEDevice through a reconnect
/// (DashboardSheetLiveDeviceTests), and its Form picks a different layout from
/// device state: base station, no rocket profile, rocket off, flight computer
/// not running, flight computer running. So a reconnect, or the flight
/// computer rebooting, changes the layout under a screen pushed from it, and
/// the firmware screen always meets one of them, because every flash ends in a
/// reboot. Its row sits in a different branch of the Form in each layout, and a
/// destination NavigationLink whose row moves is popped: the update would
/// vanish from under the operator at the reboot.
///
/// These host the real SettingsView, push the firmware screen, and rebuild the
/// device the way the reboot does. They are async so that the teardown which
/// releases the hosted screens runs inside a task: on iOS 26.2 (CI's runtime)
/// a main-actor object with an isolated deinit, the fleet's OTASession among
/// them, aborts the process when it is released outside one
/// (swiftlang/swift#87316).
@MainActor
final class SettingsNavigationTests: XCTestCase {

    private typealias LinkedDevice = FirmwareUpdateLiveDeviceTests.LinkedDevice

    private var tempDir: URL!
    private var suiteName: String!
    private var defaults: UserDefaults!
    private var window: UIWindow?

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("SettingsNavTests-\(UUID().uuidString)", isDirectory: true)
        suiteName = "SettingsNavTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
    }

    override func tearDown() async throws {
        window?.isHidden = true
        window = nil
        try? FileManager.default.removeItem(at: tempDir)
        defaults.removePersistentDomain(forName: suiteName)
    }

    /// What `BLEFleet.didConnect` does, minus CoreBluetooth.
    private func connect(_ device: BLEDevice, to fleet: BLEFleet) {
        fleet.devices.append(device)
        device.isConnected = true
    }

    /// What `BLEFleet.didDisconnectPeripheral` does.
    private func drop(_ device: BLEDevice, from fleet: BLEFleet) {
        device.isConnected = false
        fleet.devices.removeAll { $0 === device }
    }

    /// A telemetry frame; fs bit 0x10 is the FC power rail (pwr_pin_on).
    private func feed(_ device: BLEDevice, railOn: Bool, state: String = "READY") {
        device.parseTelemetryData("""
            {"st":"\(state)","fs":\(railOn ? 16 : 0)}
            """.data(using: .utf8)!)
    }

    // MARK: - Tests

    /// A base station's flash. It reboots, and the rebuilt BLEDevice is named
    /// from the peripheral's cached name, which iOS can lose across a flash:
    /// "Unknown", so `.unknown` until the identity readback lands. Settings
    /// reads it as not a base station and swaps the base-station layout for
    /// the no-profile one, then back when the readback says "B".
    func testABaseStationReboot_keepsTheFirmwareScreenUp() async throws {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let link = UUID()
        let opened = LinkedDevice(link, name: "TR-B-Test")
        connect(opened, to: fleet)
        XCTAssertTrue(opened.isBaseStation)

        let root = await host(SettingsView(device: opened)
            .environmentObject(fleet)
            .environmentObject(store))
        // Active Rocket, Link Mode, LoRa Frequency, LoRa TX Power, Network,
        // then Firmware: Version, "Update firmware…", App build.
        try await open(row: IndexPath(item: 1, section: 5), expecting: "Firmware update", in: root)

        drop(opened, from: fleet)
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update", "Up through the drop")

        let rebuilt = LinkedDevice(link, name: "Unknown")
        connect(rebuilt, to: fleet)
        XCTAssertEqual(rebuilt.deviceType, .unknown)
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update",
                       "The rebuilt device changed Settings' layout; the screen must stay")

        rebuilt.deviceType = .baseStation   // the identity readback
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update")
    }

    /// An out-computer flash on a rocket with a profile. The OC reboots and the
    /// rebuilt BLEDevice's first frame reports the flight computer's rail off,
    /// which swaps the rocket's tabbed layout for the rocket-off one (the
    /// firmware row moves from the General tab to the top level), then back
    /// when the rail comes on.
    func testAnOutComputerReboot_keepsTheFirmwareScreenUp() async throws {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        store.setActive(store.add(name: "Atlas").id)
        let link = UUID()
        let opened = LinkedDevice(link)
        connect(opened, to: fleet)
        feed(opened, railOn: true)

        let root = await host(SettingsView(device: opened)
            .environmentObject(fleet)
            .environmentObject(store))
        // The General tab: the tab picker, Rocket, Calibration, IMU Mounting,
        // IMU Logging Rate, Radio, then Firmware: Version, "Update firmware…".
        try await open(row: IndexPath(item: 1, section: 6), expecting: "Firmware update", in: root)

        drop(opened, from: fleet)
        let rebuilt = LinkedDevice(link)
        connect(rebuilt, to: fleet)
        feed(rebuilt, railOn: false)
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update",
                       "The rail-off frame changed Settings' layout; the screen must stay")

        feed(rebuilt, railOn: true)
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update")
    }

    /// A flight computer flash. The out computer and the link stay up, so
    /// there is no new BLEDevice, but the FC reboots into its setup, and while
    /// the OC reports INITIALIZATION Settings swaps the FC's settings out
    /// (SettingsAccess), which moves the firmware row off the General tab;
    /// then back once the FC runs.
    func testAFlightComputerReboot_keepsTheFirmwareScreenUp() async throws {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        store.setActive(store.add(name: "Atlas").id)
        let oc = LinkedDevice(UUID())
        connect(oc, to: fleet)
        feed(oc, railOn: true)

        let root = await host(SettingsView(device: oc)
            .environmentObject(fleet)
            .environmentObject(store))
        try await open(row: IndexPath(item: 1, section: 6), expecting: "Firmware update", in: root)

        feed(oc, railOn: true, state: "INITIALIZATION")
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update",
                       "The FC's boot changed Settings' layout; the screen must stay")

        feed(oc, railOn: true)
        await settle()
        XCTAssertEqual(pushedTitle(root), "Firmware update")
    }

    /// Back out of the firmware screen. That releases its FirmwareCatalogSession,
    /// whose isolated deinit aborted the app on iOS 26.2 (swiftlang/swift#87316):
    /// every time the operator left the screen. The pop is dispatched, so the
    /// release happens in SwiftUI's own update outside any task, as after a
    /// Back tap.
    func testBackingOutOfTheFirmwareScreen() async throws {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let bs = LinkedDevice(UUID(), name: "TR-B-Test")
        connect(bs, to: fleet)

        let root = await host(SettingsView(device: bs)
            .environmentObject(fleet)
            .environmentObject(store))
        try await open(row: IndexPath(item: 1, section: 5), expecting: "Firmware update", in: root)

        let nav = try XCTUnwrap(navigation(root))
        DispatchQueue.main.async { _ = nav.popViewController(animated: true) }
        await settle()
        XCTAssertNil(pushedTitle(root), "Back must land on Settings, with the app still running")
    }

    /// Settings' other pushes are still destination links, now inside a
    /// NavigationStack. They must still open.
    func testTheDestinationLinksStillOpen() async throws {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        store.setActive(store.add(name: "Atlas").id)
        let rocket = LinkedDevice(UUID())
        connect(rocket, to: fleet)
        feed(rocket, railOn: true)

        let root = await host(SettingsView(device: rocket)
            .environmentObject(fleet)
            .environmentObject(store))
        // Calibration: Magnetometer Calibration, its note, Pad Calibration.
        try await open(row: IndexPath(item: 2, section: 2), expecting: "Pad Calibration", in: root)
    }

    // MARK: - Hosting

    private func host<V: View>(_ view: V) async -> UIViewController {
        let scene = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }.first
        let w = scene.map { UIWindow(windowScene: $0) } ?? UIWindow()
        // Tall enough that every row of Settings has a cell.
        w.frame = CGRect(x: 0, y: 0, width: 390, height: 2400)
        let controller = UIHostingController(rootView: view)
        w.rootViewController = controller
        w.makeKeyAndVisible()
        window = w
        await settle()
        return controller
    }

    /// Long enough for a push or pop animation to finish. The main thread runs
    /// SwiftUI's updates while this waits.
    private func settle(_ seconds: TimeInterval = 1.5) async {
        try? await Task.sleep(nanoseconds: UInt64(seconds * 1_000_000_000))
    }

    private func controllers(_ root: UIViewController) -> [UIViewController] {
        [root] + root.children.flatMap(controllers)
            + (root.presentedViewController.map(controllers) ?? [])
    }

    private func navigation(_ root: UIViewController) -> UINavigationController? {
        controllers(root).compactMap { $0 as? UINavigationController }.first
    }

    /// The title of the screen pushed over Settings, nil while none is.
    private func pushedTitle(_ root: UIViewController) -> String? {
        guard let nav = navigation(root), nav.viewControllers.count > 1 else { return nil }
        return nav.topViewController?.navigationItem.title ?? "(untitled)"
    }

    /// Tap the Form row at `row` (section, item), then wait for the screen
    /// titled `title` to be pushed.
    ///
    /// By position, because a unit test can't read SwiftUI's row text: it
    /// draws its own, and exposes it only to an accessibility client. The
    /// pushed title confirms the row, so if Settings' sections move, this
    /// fails saying which row to update rather than passing on the wrong one.
    private func open(row: IndexPath, expecting title: String,
                      in root: UIViewController) async throws {
        let form = try XCTUnwrap(collectionViews(root.view).first, "Settings drew no Form")
        guard row.section < form.numberOfSections,
              row.item < form.numberOfItems(inSection: row.section) else {
            return XCTFail("Settings has no row \(row) for \(title); its sections moved")
        }
        form.selectItem(at: row, animated: false, scrollPosition: [])
        form.delegate?.collectionView?(form, didSelectItemAt: row)
        let deadline = Date().addingTimeInterval(5)
        while pushedTitle(root) == nil, Date() < deadline { await settle(0.1) }
        await settle()
        XCTAssertEqual(pushedTitle(root), title,
                       "Row \(row) must open \(title); if Settings' sections moved, update it")
    }

    private func collectionViews(_ view: UIView) -> [UICollectionView] {
        (view as? UICollectionView).map { [$0] } ?? view.subviews.flatMap(collectionViews)
    }
}
