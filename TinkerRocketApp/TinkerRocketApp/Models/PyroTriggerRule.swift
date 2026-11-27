import Foundation

/// #1775: the check the flight computer and the out computer apply to an
/// enabled pyro channel's trigger (firmware `PyroConfigGate.h`). The rocket
/// refuses a config that fails it and keeps the one it holds; the editors check
/// it first, so a value the rocket would refuse is neither sent nor saved to
/// the profile.
///
/// - Time after apogee: 0 to 120 s. A delay that outlasts the descent never
///   fires, since the flight computer services the triggers only in flight.
/// - Altitude on descent: at least 30 m above the pad. At 0 m or below a
///   channel fires at touchdown or never.
/// - The value must be finite, and the mode 0 or 1.
///
/// A disabled channel is never checked: the rocket never reads its trigger.
enum PyroTriggerRule {
    static let maxDelaySeconds: Float = 120
    static let minAltitudeMeters: Float = 30

    /// Whether an enabled channel with this trigger passes. `value` is in
    /// canonical units: seconds for mode 0, meters for mode 1.
    static func accepts(mode: UInt8, value: Float) -> Bool {
        guard value.isFinite else { return false }
        switch mode {
        case 0: return value >= 0 && value <= maxDelaySeconds
        case 1: return value >= minAltitudeMeters
        default: return false
        }
    }

    /// The range the rocket accepts, in the display unit. The altitude is
    /// rounded UP, so the number shown is itself accepted (30 m is 98.4 ft,
    /// and 98 ft is under it).
    static func rangeText(mode: UInt8, unitSystem: UnitSystem) -> String {
        switch mode {
        case 0:
            return "The rocket accepts 0 to \(Int(maxDelaySeconds)) s."
        case 1:
            let shown = Int(UnitFormatter.metersToDisplay(Double(minAltitudeMeters),
                                                          system: unitSystem).rounded(.up))
            return "The rocket accepts \(shown) \(UnitFormatter.altitudeUnit(unitSystem)) or more."
        default:
            return "Choose a trigger."
        }
    }

    /// The first enabled channel of these four that fails, 1-4, or nil.
    static func firstRefused(_ channels: [(enabled: Bool, mode: UInt8, value: Float)]) -> Int? {
        channels.firstIndex { $0.enabled && !accepts(mode: $0.mode, value: $0.value) }.map { $0 + 1 }
    }

    /// Whether any enabled channel fails. A config with one is not sent: the
    /// rocket would refuse the whole frame and keep what it holds.
    static func anyRefused(_ channels: [(enabled: Bool, mode: UInt8, value: Float)]) -> Bool {
        firstRefused(channels) != nil
    }

    /// The whole-profile push's status when it held the pyro config back.
    static func notSentText(channel: Int) -> String {
        "Pyro settings not sent: channel \(channel)'s trigger is outside what the rocket accepts, "
            + "so it kept its own. Everything else was sent."
    }

    /// The value a pyro editor would send for its text field, in canonical
    /// units, or nil when the text cannot be sent.
    ///
    /// Text left as it was loaded, in the mode it was loaded in, keeps the
    /// stored value: an altitude is shown rounded (whole feet or meters), so
    /// parsing the shown text back would send a different value, and one at
    /// the 30 m floor would come back under it. Text that does not parse, or
    /// parses to NaN or infinity, is nil; the dashboard sheet used to send it
    /// as 0 (`Float(s) ?? 0`). A comma is read as the decimal point: the
    /// decimal pad types "1,5" in a comma-decimal locale, which `Float` does
    /// not parse.
    static func valueToSend(text: String, mode: UInt8, unitSystem: UnitSystem,
                            loadedText: String, loadedMode: UInt8,
                            loadedValue: Float) -> Float? {
        if text == loadedText && mode == loadedMode { return loadedValue }
        let trimmed = text.trimmingCharacters(in: .whitespaces)
            .replacingOccurrences(of: ",", with: ".")
        guard let entered = Float(trimmed), entered.isFinite else { return nil }
        if mode == 0 { return entered }
        let meters = Float(UnitFormatter.displayToMeters(Double(entered), system: unitSystem))
        return meters.isFinite ? meters : nil
    }
}
