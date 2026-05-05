//
//  FlightAnnouncer.swift
//  TinkerRocketApp
//
//  Voice announcements during rocket flight using AVSpeechSynthesizer.
//  Announces burnout speed, altitude, apogee, descent rate/distance, and landing.
//

import Foundation
import AVFoundation
import Combine

class FlightAnnouncer: NSObject, ObservableObject, AVSpeechSynthesizerDelegate {

    @Published var isEnabled: Bool = false

    // Status surface for the dashboard "voice ready" indicator
    // (the toolbar speaker icon colour reads from these).
    @Published private(set) var audioSessionActive: Bool = false
    @Published private(set) var lastSessionError: String?

    // MARK: - Private State

    private let synthesizer = AVSpeechSynthesizer()
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

    // Burnout detection: track consecutive updates where max_speed doesn't increase
    private var lastMaxSpeed: Float = 0
    private var maxSpeedStableCount: Int = 0
    private static let burnoutStableThreshold = 3  // consecutive unchanged updates to confirm burnout

    private var isSpeaking = false

    // MARK: - Constants

    private static let altitudeInterval: TimeInterval = 5.0       // seconds between altitude callouts
    private static let descentInterval: TimeInterval = 10.0       // seconds between descent callouts
    private static let burnoutMinSpeed: Float = 10.0              // ignore burnout below this speed (m/s)
    private static let speechRate: Float = 0.52                   // AVSpeechUtterance default is 0.5

    private static let enabledKey = "voiceAnnouncementsEnabled"

    // MARK: - Init

    override init() {
        super.init()
        synthesizer.delegate = self

        isEnabled = UserDefaults.standard.bool(forKey: Self.enabledKey)

        // Configure (and activate, if enabled) the audio session up-front.
        // Activating is what makes the iPhone hardware volume buttons
        // control *media* volume instead of ringer volume — that's how the
        // operator dials in callout loudness, no in-app slider needed.
        configureAudioSession()
        if isEnabled {
            activateSession()
        }

        $isEnabled
            .dropFirst()
            .sink { [weak self] enabled in
                UserDefaults.standard.set(enabled, forKey: Self.enabledKey)
                guard let self = self else { return }
                if enabled {
                    self.activateSession()
                    // Audible confirmation that voice is alive and at the
                    // current phone volume. Without this the toggle is
                    // silent and the operator can't tell if it's working
                    // until the first in-flight callout fires.
                    self.announceImmediate("Voice ready")
                } else {
                    self.deactivateSession()
                }
            }
            .store(in: &cancellables)

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

    // MARK: - Audio Session

    private func configureAudioSession() {
        do {
            try AVAudioSession.sharedInstance().setCategory(
                .playback,
                mode: .voicePrompt,
                options: [.mixWithOthers, .duckOthers]
            )
            lastSessionError = nil
        } catch {
            lastSessionError = "config: \(error.localizedDescription)"
            announcerLog("Audio session config failed: \(error)")
        }
    }

    private func activateSession() {
        do {
            try AVAudioSession.sharedInstance().setActive(true)
            audioSessionActive = true
            lastSessionError = nil
        } catch {
            audioSessionActive = false
            lastSessionError = "activate: \(error.localizedDescription)"
            announcerLog("Audio session activate failed: \(error)")
        }
    }

    private func deactivateSession() {
        do {
            try AVAudioSession.sharedInstance().setActive(
                false, options: [.notifyOthersOnDeactivation])
            audioSessionActive = false
        } catch {
            // Deactivation can fail if speech is mid-utterance — that's fine.
            announcerLog("Audio session deactivate failed: \(error)")
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
                self.audioSessionActive = false
                announcerLog("Audio interrupted")
            case .ended:
                announcerLog("Audio interruption ended — reactivating")
                self.configureAudioSession()
                if self.isEnabled { self.activateSession() }
            @unknown default:
                break
            }
        }
    }

    @objc private func handleRouteChange(_ note: Notification) {
        guard let info = note.userInfo,
              let raw = info[AVAudioSessionRouteChangeReasonKey] as? UInt,
              let reason = AVAudioSession.RouteChangeReason(rawValue: raw) else { return }

        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            announcerLog("Route change reason=\(reason.rawValue)")
            // Re-assert the category. Some route changes (BT disconnect,
            // headphone unplug) can implicitly downgrade the session back to
            // ambient, which respects the silent switch and would mute us.
            if self.isEnabled {
                self.configureAudioSession()
                self.activateSession()
            }
        }
    }

    @objc private func handleMediaServicesReset(_ note: Notification) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            announcerLog("Media services reset — reconfiguring")
            self.configureAudioSession()
            if self.isEnabled { self.activateSession() }
        }
    }

    // MARK: - Main Entry Point

    /// Called on every BLE telemetry update. Detects events and triggers announcements.
    func processTelemetry(_ telemetry: TelemetryData) {
        guard isEnabled else {
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
        if (telemetry.launch_flag ?? false) && launchLocation == nil {
            if let lat = telemetry.latitude, let lon = telemetry.longitude,
               !lat.isNaN && !lon.isNaN {
                launchLocation = (lat: lat, lon: lon)
            }
        }

        // --- INFLIGHT announcements (before apogee) ---
        if state == "INFLIGHT" && !(telemetry.alt_apo ?? false) {
            checkBurnout(telemetry)
            // Only announce altitude after burnout — during powered flight there
            // is too much happening and the rapidly changing values aren't useful.
            if burnoutAnnounced {
                checkPeriodicAltitude(telemetry)
            }
        }

        // --- Apogee detection ---
        if (telemetry.alt_apo ?? false) && !(prev?.alt_apo ?? false) && !apogeeAnnounced {
            apogeeAnnounced = true
            let alt = telemetry.max_alt_m.map { String(format: "%.0f", $0) } ?? "unknown"
            announceImmediate("Apogee. \(alt) meters")
            // First descent callout 5 seconds after apogee (not the full 10s interval)
            lastDescentAnnounceTime = Date().addingTimeInterval(-(Self.descentInterval - 5.0))
        }

        // --- Descent callouts (after apogee, before landed) ---
        if (telemetry.alt_apo ?? false) && !(telemetry.landed_flag ?? false) && state == "INFLIGHT" {
            checkDescentCallout(telemetry)
        }

        // --- Landing detection ---
        if (telemetry.landed_flag ?? false) && !(prev?.landed_flag ?? false) && !landedAnnounced {
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
        configureAudioSession()
        activateSession()
        announceImmediate("Voice check. Announcements are enabled.")
    }

    /// Reset when disconnecting or starting a new flight
    func reset() {
        resetFlightState()
        synthesizer.stopSpeaking(at: .immediate)
        isSpeaking = false
    }

    // MARK: - Event Detectors

    private func checkBurnout(_ telemetry: TelemetryData) {
        guard !burnoutAnnounced else { return }
        guard let maxSpeed = telemetry.max_speed_mps, maxSpeed > Self.burnoutMinSpeed else { return }

        if maxSpeed > lastMaxSpeed + 0.5 {
            // Speed still increasing — reset counter (0.5 m/s tolerance for telemetry jitter)
            lastMaxSpeed = maxSpeed
            maxSpeedStableCount = 0
        } else {
            // Speed stable or decreasing — increment counter
            maxSpeedStableCount += 1
            if maxSpeedStableCount >= Self.burnoutStableThreshold {
                burnoutAnnounced = true
                let speedStr = String(format: "%.0f", lastMaxSpeed)
                announceImmediate("Burnout. Max speed \(speedStr) meters per second")
            }
        }
    }

    private func checkPeriodicAltitude(_ telemetry: TelemetryData) {
        let now = Date()
        guard now.timeIntervalSince(lastAltitudeAnnounceTime) >= Self.altitudeInterval else { return }
        guard let alt = telemetry.pressure_alt else { return }

        lastAltitudeAnnounceTime = now
        let altStr = String(format: "%.0f", alt)
        // Only include climb rate after burnout — during powered flight the
        // rate changes too rapidly and isn't meaningful to announce.
        if burnoutAnnounced, let rate = telemetry.altitude_rate, abs(rate) > 1.0 {
            let rateStr = String(format: "%.0f", abs(rate))
            announce("\(altStr) meters, climbing \(rateStr) meters per second")
        } else {
            announce("\(altStr) meters")
        }
    }

    private func checkDescentCallout(_ telemetry: TelemetryData) {
        let now = Date()
        guard now.timeIntervalSince(lastDescentAnnounceTime) >= Self.descentInterval else { return }
        guard let alt = telemetry.pressure_alt else { return }

        lastDescentAnnounceTime = now

        let altStr = String(format: "%.0f", alt)
        if let rate = telemetry.altitude_rate, abs(rate) > 1.0 {
            let rateStr = String(format: "%.0f", abs(rate))
            announce("\(altStr) meters, descending \(rateStr) meters per second")
        } else {
            announce("\(altStr) meters")
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
        return String(format: "%.0f meters away", dist)
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

    // MARK: - Speech Engine

    /// Speak a periodic announcement. Skips if already speaking — the next
    /// scheduled callout will have fresher data instead of queuing stale info.
    private func announce(_ message: String) {
        guard !isSpeaking else {
            recordSkip(reason: "isSpeaking", text: message)
            return
        }
        speakNow(message)
    }

    /// Immediately speak a critical announcement, cancelling current speech.
    private func announceImmediate(_ message: String) {
        synthesizer.stopSpeaking(at: .immediate)
        speakNow(message)
    }

    private func speakNow(_ text: String) {
        // Defensive: re-assert category and activate before every speech.
        // Other views (e.g. the keyboard in SimulationView) may have changed
        // the category, and silent route changes can downgrade the session.
        configureAudioSession()
        activateSession()

        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        utterance.rate = Self.speechRate
        utterance.pitchMultiplier = 1.0
        // volume = 1.0 means "play at full per-utterance gain"; final loudness
        // is then scaled by the iPhone hardware volume buttons. No in-app
        // slider needed — phone volume is the volume.
        utterance.volume = 1.0
        utterance.postUtteranceDelay = 0.1

        isSpeaking = true
        announcerLog("Speak: \(text)")
        synthesizer.speak(utterance)
    }

    private func recordSkip(reason: String, text: String) {
        announcerLog("Skip [\(reason)]: \(text)")
    }

    // MARK: - AVSpeechSynthesizerDelegate

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didFinish utterance: AVSpeechUtterance) {
        isSpeaking = false
    }

    func speechSynthesizer(_ synthesizer: AVSpeechSynthesizer,
                           didCancel utterance: AVSpeechUtterance) {
        isSpeaking = false
    }

    // MARK: - State Reset

    private func resetFlightState() {
        burnoutAnnounced = false
        apogeeAnnounced = false
        landedAnnounced = false
        lastMaxSpeed = 0
        maxSpeedStableCount = 0
        lastAltitudeAnnounceTime = .distantPast
        lastDescentAnnounceTime = .distantPast
        launchLocation = nil
    }
}

private func announcerLog(_ message: String) {
    print("[Announcer] \(message)")
}
