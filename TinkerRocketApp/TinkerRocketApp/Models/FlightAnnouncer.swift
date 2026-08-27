//
//  FlightAnnouncer.swift
//  TinkerRocketApp
//
//  Voice announcements during rocket flight: policy (this class) behind an
//  AnnouncerSpeech seam, engine (SystemSpeech) owning AVSpeechSynthesizer +
//  AVAudioSession.  The split is the parity-ledger Phase 9 seam — Android's
//  FlightAnnouncer has the same shape, which is what lets the 15 flight-
//  profile policy tests run on both platforms.
//
//  Callouts, in flight order: burnout speed, altitude every 5 s (post-burnout,
//  pre-apogee), apogee, recovery deployments and their chute verdict (#813,
//  see DeploymentWatcher), descent every 10 s, landing with distance.
//

import Foundation
import AVFoundation
import Combine

/// Subset of `FlightAnnouncer` that `BLEDevice` calls during dispatch.
/// Defined as a protocol so tests can substitute a lightweight spy without
/// instantiating `AVSpeechSynthesizer` / `AVAudioSession`, and so the
/// dispatch logic in `BLEDevice` can be exercised end-to-end.
protocol TelemetryAnnouncer: AnyObject {
    func processTelemetry(_ telemetry: TelemetryData)
    func reset()
}

/// The speech engine as the policy sees it — same shape as Android's
/// `AnnouncerSpeech` so the policy tests port across unchanged.
protocol AnnouncerSpeech: AnyObject {
    /// True while an utterance is playing (or queued in the engine).
    var isBusy: Bool { get }
    /// Audio-session health for the dashboard's voice indicator.
    var sessionActive: Bool { get }
    var lastError: String? { get }
    /// Engine → policy notification that session health changed (the policy
    /// republishes it for SwiftUI).
    var onStatusChange: (() -> Void)? { get set }

    /// Speak `text`. With `interrupt` the engine cancels anything currently
    /// playing first (the announceImmediate path); without it the engine just
    /// starts — the policy has already checked `isBusy` and chosen to speak.
    func speak(_ text: String, interrupt: Bool)
    /// Cancel any current utterance immediately.
    func stop()
    /// (Re)configure + activate the audio session — enable/testVoice path.
    func activate()
    /// Release the audio session (voice toggled off).
    func deactivate()
}

class FlightAnnouncer: NSObject, ObservableObject, TelemetryAnnouncer {

    @Published var isEnabled: Bool = false

    // Status surface for the dashboard "voice ready" indicator
    // (the toolbar speaker icon colour reads from these).
    @Published private(set) var audioSessionActive: Bool = false
    @Published private(set) var lastSessionError: String?

    // MARK: - Seams

    private let speech: AnnouncerSpeech
    /// Injectable clock for the 5 s / 10 s cadence gates (tests drive it).
    private let now: () -> Date
    /// Read per callout so a settings change applies immediately.
    private let unitSystem: () -> UnitSystem
    /// Test seam for the recovery expectations; production reads `profileStore`.
    private let recoveryOverride: (() -> RecoveryProfile?)?

    /// The active rocket's profile, for grading a deployment against what THIS
    /// rocket is supposed to do (#813).  Set by the dashboard alongside the
    /// announcer hand-off, same shape as `LandingPredictor.attach`.  Weak: the
    /// store outlives the announcer and must not be retained by it.
    weak var profileStore: RocketProfileStore?

    // MARK: - Private State

    private var cancellables = Set<AnyCancellable>()

    // Previous telemetry for edge detection
    private var previousTelemetry: TelemetryData?

    // Launch position for horizontal distance computation
    private var launchLocation: (lat: Double, lon: Double)?

    // Timing for periodic announcements
    private var lastAltitudeAnnounceTime: Date = .distantPast
    private var lastDescentAnnounceTime: Date = .distantPast

    // One-shot event tracking (reset on PRELAUNCH)
    private var burnoutAnnounced = false
    private var apogeeAnnounced = false
    private var landedAnnounced = false

    // Burnout detection: a burnout is a RISE followed by a PLATEAU, and both
    // halves have to be observed.  `max_speed_mps` is a running maximum that
    // never decreases, so "it stopped increasing" is true at every instant
    // AFTER burnout — including minutes later under canopy.  Treating the
    // plateau alone as the signal announces a long-past burnout to anyone who
    // starts watching late: voice toggled on mid-flight, or a BLE reconnect.
    // `nil` = no baseline sampled yet; `sawSpeedIncrease` = we watched it climb.
    private var lastMaxSpeed: Float?
    private var sawSpeedIncrease = false
    private var maxSpeedStableCount: Int = 0
    private static let burnoutStableThreshold = 3  // consecutive unchanged updates to confirm burnout

    /// Recovery-event detector (#813).  Self-guards on apogee and landing.
    private let deployment = DeploymentWatcher()

    // MARK: - Constants

    private static let altitudeInterval: TimeInterval = 5.0       // seconds between altitude callouts
    private static let descentInterval: TimeInterval = 10.0       // seconds between descent callouts
    private static let burnoutMinSpeed: Float = 10.0              // ignore burnout below this speed (m/s)

    private static let enabledKey = "voiceAnnouncementsEnabled"

    // MARK: - Init

    /// Production: real speech engine, wall clock, user's unit setting.
    convenience override init() {
        self.init(speech: SystemSpeech())
    }

    init(speech: AnnouncerSpeech,
         now: @escaping () -> Date = Date.init,
         unitSystem: @escaping () -> UnitSystem = { .current },
         persistEnabled: Bool = true,
         recovery: (() -> RecoveryProfile?)? = nil) {
        self.speech = speech
        self.now = now
        self.unitSystem = unitSystem
        self.recoveryOverride = recovery
        super.init()

        speech.onStatusChange = { [weak self] in
            guard let self else { return }
            self.audioSessionActive = self.speech.sessionActive
            self.lastSessionError = self.speech.lastError
        }

        if persistEnabled {
            isEnabled = UserDefaults.standard.bool(forKey: Self.enabledKey)
        }
        // Activating up-front is what makes the iPhone hardware volume
        // buttons control *media* volume instead of ringer volume — that's
        // how the operator dials in callout loudness, no in-app slider.
        if isEnabled {
            speech.activate()
        }

        $isEnabled
            .dropFirst()
            .sink { [weak self] enabled in
                guard let self = self else { return }
                if persistEnabled {
                    UserDefaults.standard.set(enabled, forKey: Self.enabledKey)
                }
                if enabled {
                    self.speech.activate()
                    // Audible confirmation that voice is alive and at the
                    // current phone volume. Without this the toggle is
                    // silent and the operator can't tell if it's working
                    // until the first in-flight callout fires.
                    self.speech.speak("Voice ready", interrupt: true)
                } else {
                    self.speech.stop()
                    self.speech.deactivate()
                }
            }
            .store(in: &cancellables)
    }

    // MARK: - Main Entry Point

    /// Called on every BLE telemetry update. Detects events and triggers announcements.
    func processTelemetry(_ telemetry: TelemetryData) {
        guard isEnabled else {
            previousTelemetry = telemetry
            return
        }

        // Skip stale or syncing frames.  When the rocket goes silent
        // mid-flight (powered off, out of range) the BS keeps forwarding the
        // last-known telemetry with data_status=STALE and a growing `age`.
        // Time-gated callouts (descent/altitude) would otherwise re-fire the
        // same frozen reading every interval forever.  Skipping here also
        // lets the currently-playing utterance finish naturally — we just
        // don't queue a new one.
        guard telemetry.data_status == .live else {
            previousTelemetry = telemetry
            return
        }

        let prev = previousTelemetry
        let state = telemetry.state

        // --- Reset on PRELAUNCH transition ---
        if state == "PRELAUNCH" && prev?.state != "PRELAUNCH" {
            resetFlightState()
            // Capture launch location from current GPS
            if let lat = telemetry.latitude, let lon = telemetry.longitude,
               !lat.isNaN && !lon.isNaN {
                launchLocation = (lat: lat, lon: lon)
            }
        }

        // --- Capture launch location on launch flag if not set ---
        if telemetry.launch_flag && launchLocation == nil {
            if let lat = telemetry.latitude, let lon = telemetry.longitude,
               !lat.isNaN && !lon.isNaN {
                launchLocation = (lat: lat, lon: lon)
            }
        }

        // --- INFLIGHT announcements (before apogee) ---
        if state == "INFLIGHT" && !telemetry.past_apogee {
            checkBurnout(telemetry)
            // Only announce altitude after burnout — during powered flight there
            // is too much happening and the rapidly changing values aren't useful.
            if burnoutAnnounced {
                checkPeriodicAltitude(telemetry)
            }
        }

        // --- Apogee detection ---
        if telemetry.alt_apo && !(prev?.alt_apo ?? false) && !apogeeAnnounced {
            apogeeAnnounced = true
            let alt = telemetry.max_alt_m.map {
                UnitFormatter.spokenAltitude(Double($0), system: unitSystem())
            } ?? "altitude unknown"
            announceImmediate("Apogee. \(alt)")
            // First descent callout 5 seconds after apogee (not the full 10s interval)
            lastDescentAnnounceTime = now().addingTimeInterval(-(Self.descentInterval - 5.0))
        }

        // --- Recovery events (#813) ---
        // Before the cadence callout: a deployment should pre-empt the
        // periodic descent readout, not race it.
        checkDeployment(telemetry, state: state)

        // --- Descent callouts (after apogee, before landed) ---
        if telemetry.past_apogee && !telemetry.landed_flag && state == "INFLIGHT" {
            checkDescentCallout(telemetry)
        }

        // --- Landing detection ---
        if telemetry.landed_flag && !(prev?.landed_flag ?? false) && !landedAnnounced {
            landedAnnounced = true
            let distance = horizontalDistanceString(telemetry)
            announceImmediate("Landed. \(distance)")
        } else if state == "LANDED" && prev?.state != "LANDED" && !landedAnnounced {
            // Fallback: state transition to LANDED
            landedAnnounced = true
            let distance = horizontalDistanceString(telemetry)
            announceImmediate("Landed. \(distance)")
        }

        previousTelemetry = telemetry
    }

    /// Speak a test phrase so the user can verify volume and voice before flight
    func testVoice() {
        speech.activate()
        speech.speak("Voice check. Announcements are enabled.", interrupt: true)
    }

    /// Reset when disconnecting or starting a new flight
    func reset() {
        resetFlightState()
        speech.stop()
    }

    // MARK: - Event Detectors

    private func checkBurnout(_ telemetry: TelemetryData) {
        guard !burnoutAnnounced else { return }
        guard let maxSpeed = telemetry.max_speed_mps, maxSpeed > Self.burnoutMinSpeed else { return }

        guard let last = lastMaxSpeed else {
            // First sample we've seen.  A running maximum reads identically
            // during the burn and long after it, so one frame proves nothing.
            // Adopt it as the baseline and wait for evidence either way.
            lastMaxSpeed = maxSpeed
            return
        }

        if maxSpeed > last + 0.5 {
            // Speed still increasing — reset counter (0.5 m/s tolerance for telemetry jitter)
            lastMaxSpeed = maxSpeed
            sawSpeedIncrease = true
            maxSpeedStableCount = 0
            return
        }

        // Plateau.  Only means burnout if we actually watched the climb that
        // came before it — otherwise we are just late to a flight already in
        // progress, and the "max speed" we'd read out is minutes stale.
        guard sawSpeedIncrease else { return }
        maxSpeedStableCount += 1
        if maxSpeedStableCount >= Self.burnoutStableThreshold {
            burnoutAnnounced = true
            announceImmediate("Burnout. Max speed \(UnitFormatter.spokenSpeed(Double(last), system: unitSystem()))")
        }
    }

    /// The active recovery expectations: injected in tests, from the profile
    /// store in production.  Nil is normal — no profile selected.
    private func currentRecovery() -> RecoveryProfile? {
        if let recoveryOverride { return recoveryOverride() }
        guard let p = profileStore?.activeProfile else { return nil }
        return RecoveryProfile(p)
    }

    /// Drogue and main deployment, detected from the descent-rate curve — the
    /// only recovery signal present on BOTH the direct BLE link and the
    /// base-station relay (see `DeploymentWatcher`).
    ///
    /// The deployment itself interrupts, because it is the callout the flyer is
    /// waiting for.  The verdict follows a few seconds later on the ordinary
    /// path, once the rate has settled enough to be worth quoting.
    private func checkDeployment(_ telemetry: TelemetryData, state: String) {
        guard let event = deployment.step(
            now: now(),
            altitudeRateMps: telemetry.altitude_rate,
            altAglM: telemetry.pressure_alt,
            afterApogee: telemetry.past_apogee,
            landed: telemetry.landed_flag || state == "LANDED",
            profile: currentRecovery()
        ) else { return }

        switch event {
        case let .deployed(kind, _):
            announceImmediate(Self.deployedPhrase(kind))
            // Hold the cadence off so the readout that follows is the verdict,
            // not a periodic callout carrying the same number.
            lastDescentAnnounceTime = now()
        case let .verdict(_, rateMps, nominal):
            announce(verdictPhrase(rateMps: rateMps, nominal: nominal))
            lastDescentAnnounceTime = now()
        }
    }

    static func deployedPhrase(_ kind: DeploymentKind) -> String {
        switch kind {
        case .drogue: return "Drogue out."
        case .main:   return "Main out."
        case .chute:  return "Chute out."
        }
    }

    /// A nominal rate earns the words "good chute"; anything else is reported
    /// as the bare number (#813).  Saying "no chute" at 200 m changes nothing
    /// the flyer can do and is the callout most likely to be wrong — the rate
    /// itself is the honest signal, and they can judge it.
    private func verdictPhrase(rateMps: Double, nominal: Bool) -> String {
        let rate = UnitFormatter.spokenSpeed(rateMps, system: unitSystem())
        return nominal ? "Good chute, descending \(rate)" : "Descending \(rate)"
    }

    /// Direction word for a SIGNED altitude rate, or nil within the deadband
    /// (near-level → omit direction).  Single source of truth so a callout's
    /// spoken direction can never contradict the rate's sign.  #235: "climbing"
    /// was announced during descent because the word came from the apogee-phase
    /// flag (`alt_apo`, which clears below ~15 m AGL near landing) instead of
    /// the rate.
    static func climbDescendWord(forRate rate: Float, deadband: Float = 1.0) -> String? {
        if rate >  deadband { return "climbing" }
        if rate < -deadband { return "descending" }
        return nil
    }

    private func checkPeriodicAltitude(_ telemetry: TelemetryData) {
        let t = now()
        guard t.timeIntervalSince(lastAltitudeAnnounceTime) >= Self.altitudeInterval else { return }
        guard let alt = telemetry.pressure_alt else { return }

        lastAltitudeAnnounceTime = t
        let altStr = UnitFormatter.spokenAltitude(Double(alt), system: unitSystem())
        // Only include the vertical rate after burnout — during powered flight
        // it changes too rapidly to be meaningful.  The direction word comes
        // from the rate's sign (#235), never the apogee phase, so it can't
        // contradict the motion (e.g. "climbing" while descending near landing).
        if burnoutAnnounced, let rate = telemetry.altitude_rate,
           let word = Self.climbDescendWord(forRate: rate) {
            announce("\(altStr), \(word) \(UnitFormatter.spokenSpeed(Double(abs(rate)), system: unitSystem()))")
        } else {
            announce(altStr)
        }
    }

    private func checkDescentCallout(_ telemetry: TelemetryData) {
        let t = now()
        guard t.timeIntervalSince(lastDescentAnnounceTime) >= Self.descentInterval else { return }
        guard let alt = telemetry.pressure_alt else { return }

        lastDescentAnnounceTime = t

        let altStr = UnitFormatter.spokenAltitude(Double(alt), system: unitSystem())
        if let rate = telemetry.altitude_rate,
           let word = Self.climbDescendWord(forRate: rate) {
            announce("\(altStr), \(word) \(UnitFormatter.spokenSpeed(Double(abs(rate)), system: unitSystem()))")
        } else {
            announce(altStr)
        }
    }

    // MARK: - Horizontal Distance

    private func horizontalDistanceString(_ telemetry: TelemetryData) -> String {
        guard let launch = launchLocation,
              let lat = telemetry.latitude, let lon = telemetry.longitude,
              !lat.isNaN && !lon.isNaN else {
            return ""
        }

        let dist = haversineDistance(lat1: launch.lat, lon1: launch.lon,
                                     lat2: lat, lon2: lon)
        return UnitFormatter.spokenDistance(dist, system: unitSystem()) + " away"
    }

    /// Haversine formula: returns distance in meters between two GPS coordinates
    private func haversineDistance(lat1: Double, lon1: Double,
                                   lat2: Double, lon2: Double) -> Double {
        let R = 6371000.0 // Earth radius in meters
        let dLat = (lat2 - lat1) * .pi / 180.0
        let dLon = (lon2 - lon1) * .pi / 180.0
        let a = sin(dLat / 2) * sin(dLat / 2) +
                cos(lat1 * .pi / 180.0) * cos(lat2 * .pi / 180.0) *
                sin(dLon / 2) * sin(dLon / 2)
        let c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c
    }

    // MARK: - Speak policy

    /// Speak a periodic announcement. Skips if already speaking — the next
    /// scheduled callout will have fresher data instead of queuing stale info.
    private func announce(_ message: String) {
        guard !speech.isBusy else {
            announcerLog("Skip [isSpeaking]: \(message)")
            return
        }
        announcerLog("Speak: \(message)")
        speech.speak(message, interrupt: false)
    }

    /// Immediately speak a critical announcement, cancelling current speech.
    private func announceImmediate(_ message: String) {
        announcerLog("Speak: \(message)")
        speech.speak(message, interrupt: true)
    }

    // MARK: - State Reset

    private func resetFlightState() {
        burnoutAnnounced = false
        apogeeAnnounced = false
        landedAnnounced = false
        lastMaxSpeed = nil
        sawSpeedIncrease = false
        maxSpeedStableCount = 0
        lastAltitudeAnnounceTime = .distantPast
        lastDescentAnnounceTime = .distantPast
        launchLocation = nil
        deployment.reset()
    }
}

// MARK: - Production speech engine

/// AVSpeechSynthesizer + AVAudioSession behind the AnnouncerSpeech seam.
/// Everything platform-audio lives here: ducking category, interruption /
/// route-change / media-services-reset recovery, per-utterance settings.
final class SystemSpeech: NSObject, AnnouncerSpeech, AVSpeechSynthesizerDelegate {

    private let synthesizer = AVSpeechSynthesizer()
    private static let speechRate: Float = 0.52   // AVSpeechUtterance default is 0.5

    private(set) var sessionActive: Bool = false {
        didSet { onStatusChange?() }
    }
    private(set) var lastError: String? {
        didSet { onStatusChange?() }
    }
    var onStatusChange: (() -> Void)?

    private var speaking = false
    private var speakingSince = Date.distantPast
    /// didFinish/didCancel normally clear `speaking`, but if the delegate
    /// callback is lost (audio session dying mid-utterance) a latched value
    /// would make the policy skip every skippable callout for the rest of
    /// the flight.  No utterance runs anywhere near this long — treat an
    /// older claim as stale rather than staying mute.
    private static let busyStaleAfter: TimeInterval = 15
    var isBusy: Bool {
        speaking && Date().timeIntervalSince(speakingSince) < Self.busyStaleAfter
    }

    override init() {
        super.init()
        synthesizer.delegate = self
        configureSession()

        // Recover from interruptions (phone call, Siri) and route changes
        // (BT pair/unpair, headphone unplug). Without these, the session can
        // silently end and the next utterance never reaches the speaker.
        let nc = NotificationCenter.default
        nc.addObserver(self,
                       selector: #selector(handleInterruption(_:)),
                       name: AVAudioSession.interruptionNotification,
                       object: nil)
        nc.addObserver(self,
                       selector: #selector(handleRouteChange(_:)),
                       name: AVAudioSession.routeChangeNotification,
                       object: nil)
        nc.addObserver(self,
                       selector: #selector(handleMediaServicesReset(_:)),
                       name: AVAudioSession.mediaServicesWereResetNotification,
                       object: nil)
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }

    // MARK: AnnouncerSpeech

    func speak(_ text: String, interrupt: Bool) {
        if interrupt {
            synthesizer.stopSpeaking(at: .immediate)
        }
        // Defensive: re-assert category and activate before every speech.
        // Other views (e.g. the keyboard in SimulationView) may have changed
        // the category, and silent route changes can downgrade the session.
        configureSession()
        activate()

        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        utterance.rate = Self.speechRate
        utterance.pitchMultiplier = 1.0
        // volume = 1.0 means "play at full per-utterance gain"; final loudness
        // is then scaled by the iPhone hardware volume buttons. No in-app
        // slider needed — phone volume is the volume.
        utterance.volume = 1.0
        utterance.postUtteranceDelay = 0.1

        speaking = true
        speakingSince = Date()
        synthesizer.speak(utterance)
    }

    func stop() {
        synthesizer.stopSpeaking(at: .immediate)
        speaking = false
    }

    func activate() {
        configureSession()
        do {
            try AVAudioSession.sharedInstance().setActive(true)
            sessionActive = true
            lastError = nil
        } catch {
            sessionActive = false
            lastError = "activate: \(error.localizedDescription)"
            announcerLog("Audio session activate failed: \(error)")
        }
    }

    func deactivate() {
        do {
            try AVAudioSession.sharedInstance().setActive(
                false, options: [.notifyOthersOnDeactivation])
            sessionActive = false
        } catch {
            // Deactivation can fail if speech is mid-utterance — that's fine.
            announcerLog("Audio session deactivate failed: \(error)")
        }
    }

    // MARK: Session plumbing

    private func configureSession() {
        do {
            try AVAudioSession.sharedInstance().setCategory(
                .playback,
                mode: .voicePrompt,
                options: [.mixWithOthers, .duckOthers]
            )
            lastError = nil
        } catch {
            lastError = "config: \(error.localizedDescription)"
            announcerLog("Audio session config failed: \(error)")
        }
    }

    @objc private func handleInterruption(_ note: Notification) {
        guard let info = note.userInfo,
              let raw = info[AVAudioSessionInterruptionTypeKey] as? UInt,
              let type = AVAudioSession.InterruptionType(rawValue: raw) else { return }

        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            switch type {
            case .began:
                self.sessionActive = false
                announcerLog("Audio interrupted")
            case .ended:
                announcerLog("Audio interruption ended — reactivating")
                self.activate()
            @unknown default:
                break
            }
        }
    }

    @objc private func handleRouteChange(_ note: Notification) {
        guard let info = note.userInfo,
              let raw = info[AVAudioSessionRouteChangeReasonKey] as? UInt,
              AVAudioSession.RouteChangeReason(rawValue: raw) != nil else { return }

        DispatchQueue.main.async { [weak self] in
            // Re-assert the category. Some route changes (BT disconnect,
            // headphone unplug) can implicitly downgrade the session back to
            // ambient, which respects the silent switch and would mute us.
            self?.activate()
        }
    }

    @objc private func handleMediaServicesReset(_ note: Notification) {
        DispatchQueue.main.async { [weak self] in
            announcerLog("Media services reset — reconfiguring")
            self?.activate()
        }
    }

    // MARK: AVSpeechSynthesizerDelegate

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didFinish utterance: AVSpeechUtterance) {
        speaking = false
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didCancel utterance: AVSpeechUtterance) {
        speaking = false
    }
}

private func announcerLog(_ message: String) {
    print("[Announcer] \(message)")
}
