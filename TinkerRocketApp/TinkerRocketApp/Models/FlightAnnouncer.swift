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

    // MARK: - Private State

    private let synthesizer = AVSpeechSynthesizer()
    private var enabledCancellable: AnyCancellable?

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

    // MARK: - Init

    override init() {
        super.init()
        isEnabled = UserDefaults.standard.bool(forKey: "voiceAnnouncementsEnabled")
        synthesizer.delegate = self

        // Persist toggle (audio session is configured lazily in speakNow(),
        // NOT here — configuring on toggle was taking over the audio routing
        // immediately, which the user perceived as "turning up the volume").
        enabledCancellable = $isEnabled
            .dropFirst() // skip initial value
            .sink { enabled in
                UserDefaults.standard.set(enabled, forKey: "voiceAnnouncementsEnabled")
            }
    }

    // MARK: - Audio Session

    private func configureAudioSession() {
        do {
            try AVAudioSession.sharedInstance().setCategory(
                .playback,
                mode: .voicePrompt,
                options: [.mixWithOthers]
            )
        } catch {
            print("[Announcer] Audio session config failed: \(error)")
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
        guard !isSpeaking else { return }
        speakNow(message)
    }

    /// Immediately speak a critical announcement, cancelling current speech.
    private func announceImmediate(_ message: String) {
        synthesizer.stopSpeaking(at: .immediate)
        speakNow(message)
    }

    private func speakNow(_ text: String) {
        // Ensure correct audio category before every speech attempt.
        // Other views (e.g. keyboard in SimulationView) may have changed
        // the session category, so we always reconfigure before speaking.
        configureAudioSession()

        do {
            try AVAudioSession.sharedInstance().setActive(true)
        } catch {
            print("[Announcer] Audio activation error: \(error)")
        }

        let utterance = AVSpeechUtterance(string: text)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        utterance.rate = 0.52
        utterance.pitchMultiplier = 1.0
        utterance.volume = 0.9
        utterance.postUtteranceDelay = 0.1

        isSpeaking = true
        synthesizer.speak(utterance)
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
