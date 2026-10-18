import SwiftUI
import XCTest
@testable import TinkerRocketApp

/// The GNSS antenna offset section on the real Settings screen (cmd 81).
/// Each commit is written to the rocket, and a Measured offset of zeros tells
/// the navigation filter the antenna sits on the IMU, with tight doubt. So
/// the switch to Measured starts the three fields empty, and a commit (a
/// field losing focus, or Settings closing) with any of them empty or
/// invalid stores and sends nothing.
///
/// These host SettingsView, drive its Unknown / Measured picker and the
/// three fields through UIKit, and read what reaches the rocket's command
/// tap. They are async so that the teardown which releases the hosted screen
/// runs inside a task (iOS 26.2, see SettingsNavigationTests).
///
/// The footer tests read the section's footer as VoiceOver would, and check
/// the endings Android's AntennaOffsetCaptionTest checks, so the two apps'
/// wording cannot drift apart.
@MainActor
final class AntennaOffsetDraftTests: XCTestCase {

    private typealias LinkedDevice = FirmwareUpdateLiveDeviceTests.LinkedDevice

    private var tempDir: URL!
    private var suiteName: String!
    private var defaults: UserDefaults!
    private var window: UIWindow?
    private var sent: [(UInt8, Data)] = []
    /// The simulator's accessibility automation setting before a footer test
    /// turned it on; put back in tearDown.
    private var automationWas: Int32?

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("AntennaDraftTests-\(UUID().uuidString)", isDirectory: true)
        suiteName = "AntennaDraftTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
        sent = []
    }

    override func tearDown() async throws {
        window?.isHidden = true
        window = nil
        if let was = automationWas { AccessibilityAutomation.set(was) }
        automationWas = nil
        try? FileManager.default.removeItem(at: tempDir)
        defaults.removePersistentDomain(forName: suiteName)
    }

    private var antennaFrames: [Data] { sent.filter { $0.0 == 81 }.map { $0.1 } }

    // MARK: - Tests

    func testSwitchingToMeasuredStartsTheFieldsEmpty() async throws {
        let (store, root) = await openSettings(profile: .unknown)
        XCTAssertTrue(try antennaFields(root).isEmpty, "Unknown shows no fields")

        try await chooseMeasured(root)

        let fields = try antennaFields(root)
        XCTAssertEqual(fields.count, 3)
        XCTAssertEqual(fields.map { $0.text ?? "" }, ["", "", ""],
                       "an Unknown profile's zeros are no measurement")
        XCTAssertEqual(fields.map { $0.placeholder ?? "" }, ["", "", ""],
                       "a grey 0 would look like a zero entered")
        // The picker and the three fields; the error row waits for typing.
        XCTAssertEqual(try antennaRowCount(root), 4, "no error before the user types")
        XCTAssertTrue(antennaFrames.isEmpty, "the switch itself sends nothing")
        XCTAssertEqual(store.activeProfile?.antennaOffset, .unknown)
    }

    func testOneFieldAloneIsNotSentOnBlur() async throws {
        let (store, root) = await openSettings(profile: .unknown)
        try await chooseMeasured(root)
        let fields = try antennaFields(root)

        await enter("312", in: fields[0])
        fields[0].resignFirstResponder()
        await settle()

        XCTAssertTrue(antennaFrames.isEmpty, "Y and Z are empty: nothing goes out")
        XCTAssertEqual(store.activeProfile?.antennaOffset, .unknown)
        XCTAssertEqual(try antennaRowCount(root), 5, "typed: the error row names Y and Z")
    }

    func testClosingSettingsWithTwoFieldsSendsNothing() async throws {
        let (store, root) = await openSettings(profile: .unknown)
        try await chooseMeasured(root)
        let fields = try antennaFields(root)

        await enter("312", in: fields[0])
        await enter("-25", in: fields[1])
        // Still typing in Y when Settings goes away: the flush finds Z empty.
        close()
        await settle()

        XCTAssertTrue(antennaFrames.isEmpty, "Z is empty: nothing goes out")
        XCTAssertEqual(store.activeProfile?.antennaOffset, .unknown)
        withExtendedLifetime(root) {}
    }

    /// This one and the next are the controls for the three above: with all
    /// three typed the same steps do send, once, so the silence above is the
    /// rule and not a harness that never reached the fields.
    func testClosingSettingsWithAllThreeSendsThem() async throws {
        let (store, root) = await openSettings(profile: .unknown)
        try await chooseMeasured(root)
        let fields = try antennaFields(root)

        await enter("312", in: fields[0])
        await enter("-25", in: fields[1])
        await enter("4", in: fields[2])
        close()
        await settle()

        XCTAssertEqual(antennaFrames, [Data([0x01, 0x38, 0x01, 0xE7, 0xFF, 0x04, 0x00])])
        XCTAssertEqual(store.activeProfile?.antennaOffset,
                       AntennaOffset(known: true, xMm: 312, yMm: -25, zMm: 4))
        withExtendedLifetime(root) {}
    }

    func testAllThreeGoOutTogether() async throws {
        let (store, root) = await openSettings(profile: .unknown)
        try await chooseMeasured(root)
        let fields = try antennaFields(root)

        await enter("312", in: fields[0])
        await enter("-25", in: fields[1])
        await enter("4", in: fields[2])
        XCTAssertTrue(antennaFrames.isEmpty, "moving between the fields commits nothing yet")
        fields[2].resignFirstResponder()
        await settle()

        // 312 = 0x0138, -25 = 0xFFE7, 4 = 0x0004.
        XCTAssertEqual(antennaFrames, [Data([0x01, 0x38, 0x01, 0xE7, 0xFF, 0x04, 0x00])])
        XCTAssertEqual(store.activeProfile?.antennaOffset,
                       AntennaOffset(known: true, xMm: 312, yMm: -25, zMm: 4))
    }

    func testAMeasuredProfileShowsItsMillimeters() async throws {
        let offset = AntennaOffset(known: true, xMm: 312, yMm: -25, zMm: 4)
        let (_, root) = await openSettings(profile: offset)
        XCTAssertEqual(try antennaFields(root).map { $0.text ?? "" }, ["312", "-25", "4"])
    }

    // MARK: - The footer

    func testTheFooterSaysAnAgreeingFlightComputerReportIsInUse() async throws {
        try readFooters()
        let offset = AntennaOffset(known: true, xMm: 312, yMm: -25, zMm: 4)
        let (_, root) = await openSettings(profile: offset)

        let footer = try antennaFooter(root)
        XCTAssertTrue(footer.hasPrefix("From the flight computer\u{2019}s IMU chip to the center of the GNSS antenna"),
                      footer)
        XCTAssertTrue(footer.hasSuffix(" In use by the flight computer."), footer)
    }

    /// A commit sends the offset, and the readback's mirror of it is what was
    /// sent, not the flight computer's report.
    func testTheFooterSaysACommitIsSentButNotYetReported() async throws {
        try readFooters()
        let (_, root) = await openSettings(profile: .unknown)
        XCTAssertTrue(try antennaFooter(root).hasSuffix(" In use by the flight computer."))
        try await chooseMeasured(root)
        XCTAssertTrue(try antennaFooter(root).hasSuffix(" Not sent yet: enter all three, then tap Done."))

        let fields = try antennaFields(root)
        await enter("312", in: fields[0])
        await enter("-25", in: fields[1])
        await enter("4", in: fields[2])
        fields[2].resignFirstResponder()
        await settle()

        XCTAssertEqual(antennaFrames.count, 1)
        let footer = try antennaFooter(root)
        XCTAssertTrue(footer.hasSuffix(" Sent to the rocket; the flight computer has not reported it yet."),
                      footer)
        XCTAssertFalse(footer.contains("In use"), footer)
    }

    func testTheFooterSaysADifferentOutComputerCopyIsTheOutComputers() async throws {
        try readFooters()
        let copy = AntennaOffset(known: true, xMm: 312, yMm: -25, zMm: 4)
        let (_, root) = await openSettings(profile: .unknown, report: copy, source: "oc")

        let footer = try antennaFooter(root)
        XCTAssertTrue(footer.hasPrefix("Unknown (the default): the flight computer allows for a GNSS antenna up to 30 cm"),
                      footer)
        XCTAssertTrue(footer.hasSuffix(
            " The out computer holds X 312, Y -25, Z 4 mm; the flight computer has not reported it yet."),
                      footer)
        XCTAssertFalse(footer.contains("flight computer reports"), footer)
    }

    // MARK: - The screen

    /// Settings on a connected, running rocket whose readback carries the
    /// setting (so the section is unlocked): `report` from `source`, the
    /// profile's own offset from the flight computer unless a test says
    /// otherwise. Every command the rocket is sent is recorded.
    private func openSettings(profile offset: AntennaOffset, report: AntennaOffset? = nil,
                              source: String = "fc") async
        -> (RocketProfileStore, UIViewController) {
        let fleet = BLEFleet()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let id = store.add(name: "Atlas").id
        store.setActive(id)
        store.update(id) { $0.antennaOffset = offset }

        let rocket = LinkedDevice(UUID())
        fleet.devices.append(rocket)
        rocket.isConnected = true
        rocket.commandTap = { [weak self] in self?.sent.append(($0, $1)) }
        rocket.parseTelemetryData(#"{"st":"READY","fs":16}"#.data(using: .utf8)!)
        let r = report ?? offset
        rocket.parseTelemetryData("""
            {"type":"config","antk":\(r.known ? 1 : 0),"antx":\(r.xMm),"anty":\(r.yMm),"antz":\(r.zMm),"antsrc":"\(source)"}
            """.data(using: .utf8)!)

        let root = await host(SettingsView(device: rocket)
            .environmentObject(fleet)
            .environmentObject(store))
        return (store, root)
    }

    private func host<V: View>(_ view: V) async -> UIViewController {
        let scene = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }.first
        let w = scene.map { UIWindow(windowScene: $0) } ?? UIWindow()
        // Tall enough that every row of the General tab has a cell.
        w.frame = CGRect(x: 0, y: 0, width: 390, height: 2400)
        let controller = UIHostingController(rootView: view)
        w.rootViewController = controller
        w.makeKeyAndVisible()
        window = w
        await settle()
        return controller
    }

    /// Take Settings off the screen, as closing the sheet does.
    private func close() {
        window?.rootViewController = UIViewController()
    }

    private func settle(_ seconds: TimeInterval = 0.5) async {
        try? await Task.sleep(nanoseconds: UInt64(seconds * 1_000_000_000))
    }

    /// The section's Unknown / Measured picker.
    private func antennaPicker(_ root: UIViewController) throws -> UISegmentedControl {
        let pickers = views(UISegmentedControl.self, in: root.view)
        return try XCTUnwrap(pickers.first {
            $0.numberOfSegments == 2 && $0.titleForSegment(at: 0) == "Unknown"
                && $0.titleForSegment(at: 1) == "Measured"
        }, "Settings drew no Unknown / Measured picker")
    }

    private func chooseMeasured(_ root: UIViewController) async throws {
        let picker = try antennaPicker(root)
        picker.selectedSegmentIndex = 1
        picker.sendActions(for: .valueChanged)
        await settle()
    }

    /// The Form, and the section the picker sits in.
    private func antennaSection(_ root: UIViewController) throws -> (UICollectionView, Int) {
        let form = try XCTUnwrap(views(UICollectionView.self, in: root.view).first, "Settings drew no Form")
        let picker = try antennaPicker(root)
        var cell: UIView? = picker
        while let v = cell, !(v is UICollectionViewCell) { cell = v.superview }
        let row = try XCTUnwrap((cell as? UICollectionViewCell).flatMap { form.indexPath(for: $0) })
        return (form, row.section)
    }

    /// The section's rows: the picker, the three fields once Measured is
    /// shown, and the error row when there is one.
    private func antennaRowCount(_ root: UIViewController) throws -> Int {
        let (form, section) = try antennaSection(root)
        return form.numberOfItems(inSection: section)
    }

    /// The text fields in the section, top to bottom: the three millimeter
    /// fields once Measured is shown, none before.
    private func antennaFields(_ root: UIViewController) throws -> [UITextField] {
        let (form, section) = try antennaSection(root)
        return (0..<form.numberOfItems(inSection: section)).compactMap { item in
            form.cellForItem(at: IndexPath(item: item, section: section))
                .flatMap { views(UITextField.self, in: $0).first }
        }
    }

    /// Focus the field and type over whatever it holds, as the keyboard does
    /// after Select All. Over, not after: a field seeded "0" would otherwise
    /// turn "-25" into "0-25" and refuse it, which hides the seed.
    private func enter(_ text: String, in field: UITextField) async {
        field.becomeFirstResponder()
        await settle(0.3)
        field.selectedTextRange = field.textRange(from: field.beginningOfDocument, to: field.endOfDocument)
        field.insertText(text)
        await settle(0.3)
    }

    private func views<T: UIView>(_ type: T.Type, in view: UIView) -> [T] {
        ((view as? T).map { [$0] } ?? []) + view.subviews.flatMap { views(type, in: $0) }
    }

    /// SwiftUI draws its text itself and builds the accessibility tree only
    /// for an accessibility client, so a footer test first turns on the
    /// simulator's accessibility automation setting (tearDown puts it back).
    private func readFooters() throws {
        automationWas = try XCTUnwrap(AccessibilityAutomation.get(), "no accessibility automation setting")
        AccessibilityAutomation.set(1)
    }

    /// The section's footer text, as VoiceOver reads it.
    private func antennaFooter(_ root: UIViewController) throws -> String {
        let (form, section) = try antennaSection(root)
        let footer = try XCTUnwrap(form.supplementaryView(
            forElementKind: UICollectionView.elementKindSectionFooter,
            at: IndexPath(item: 0, section: section)), "the section drew no footer")
        return try XCTUnwrap(accessibilityLabels(footer).first, "the footer has no text")
    }

    private func accessibilityLabels(_ element: Any) -> [String] {
        guard let o = element as? NSObject else { return [] }
        if o.isAccessibilityElement, let label = o.accessibilityLabel, !label.isEmpty { return [label] }
        let children = (o.accessibilityElements ?? []) + ((o as? UIView)?.subviews ?? [])
        return children.flatMap { accessibilityLabels($0) }
    }
}

/// The simulator's accessibility automation setting, from libAccessibility
/// (private API, the AccessibilitySnapshot library's way in); `get` is nil
/// where the library or its symbols are missing.
private enum AccessibilityAutomation {
    private typealias Getter = @convention(c) () -> Int32
    private typealias Setter = @convention(c) (Int32) -> Void
    private static let library = dlopen("/usr/lib/libAccessibility.dylib", RTLD_NOW)

    static func get() -> Int32? {
        guard let symbol = dlsym(library, "_AXSAutomationEnabled") else { return nil }
        return unsafeBitCast(symbol, to: Getter.self)()
    }

    static func set(_ on: Int32) {
        guard let symbol = dlsym(library, "_AXSSetAutomationEnabled") else { return }
        unsafeBitCast(symbol, to: Setter.self)(on)
    }
}
