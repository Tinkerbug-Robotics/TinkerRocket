//
//  PyroTestView.swift
//  TinkerRocketApp
//
//  Test-fire a pyro channel with slow-motion video recording.
//

import SwiftUI
import AVFoundation
import Photos
import Combine
// MARK: - State Machine

enum PyroTestState {
    case idle
    case countdown
    case recording
    case done
}

// MARK: - Tick Counter (ObservableObject so SwiftUI can observe it)

@MainActor
class TickCounter: ObservableObject {
    @Published var tick: UInt = 0
    private var timer: Timer?

    func start() {
        stop()
        tick = 0
        timer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            // Bind an immutable local here (in the timer's nonisolated closure)
            // so the Task captures a `let`, not the mutable `[weak self]` var —
            // referencing the captured var inside the concurrent Task is a Swift 6
            // error.
            guard let self else { return }
            Task { @MainActor in
                self.tick += 1
            }
        }
    }

    func stop() {
        timer?.invalidate()
        timer = nil
    }
}

// MARK: - View

struct PyroTestView: View {
    @ObservedObject var device: BLEDevice
    let channel: Int
    @Environment(\.dismiss) var dismiss

    @StateObject private var ticker = TickCounter()

    @State private var state: PyroTestState = .idle
    @State private var secondsRemaining: Int = 10
    @State private var recordSecondsRemaining: Int = 10
    @State private var cameraRecording = false
    @State private var startedLoggingForTest = false  // #385: only stop what we started
    @State private var videoSaved = false
    @State private var errorMessage: String?
    // True once the fire command actually went out this test — the done-state
    // continuity verdict must not render for a test that never fired (guard
    // failure, video error before T-0).
    @State private var firedThisTest = false
    // BS path: the rocket this test was opened FOR, latched on appear. The
    // #390 focus self-heal can re-pin a stale focus to a DIFFERENT rocket
    // that is still on the air — without the latch, a fire tapped for rocket
    // A could relay to rocket B mid-countdown, and the done-screen verdict
    // could be computed from rocket B's telemetry. The fire guard and the
    // verdict both require focus to still equal this.
    @State private var latchedTargetRID: UInt8?

    @State private var camera = SlowMoCameraManager()

    // Path-aware (#297 fail-safe included): direct link reads the "ps" cont
    // bit; a base-station link reads the relayed sensor-health scorecard —
    // the LoRa downlink carries no pyro_status byte.
    private var continuity: Bool {
        device.pyroContinuityLive(channel: channel)
    }

    var body: some View {
        ZStack {
            // Camera preview (full screen)
            CameraPreviewView(session: camera.session)
                .ignoresSafeArea()

            // Dark overlay for readability
            Color.black.opacity(0.3)
                .ignoresSafeArea()

            VStack {
                // Top bar
                topBar

                Spacer()

                // Center countdown
                centerDisplay

                Spacer()

                // Bottom fire/safe button
                bottomControls
            }
            .padding()
        }
        .onChange(of: ticker.tick) { _ in
            switch state {
            case .countdown:
                secondsRemaining -= 1
                // Start recording + logging at T-5
                if secondsRemaining == 5 {
                    cameraRecording = true
                    // #385: cmd 23 is a TOGGLE — the blind send used to STOP
                    // logging right before the fire when a session was already
                    // running (fire event not recorded), then restart it after.
                    // Only start if the rocket isn't logging, and only stop
                    // afterwards if we were the ones who started it.
                    // setRocketLoggingForTest routes per path: BS links relay
                    // a desired-state cmd 23 to the rocket, because the BS's
                    // own toggle keys off its CSV state and can invert.
                    if !device.telemetry.rocketLoggingActive {
                        device.setRocketLoggingForTest(on: true)
                        startedLoggingForTest = true
                    }
                    camera.startRecording { url, error in
                        if let url = url {
                            saveToPhotoLibrary(url: url)
                        } else {
                            DispatchQueue.main.async {
                                videoSaved = false
                                errorMessage = error?.localizedDescription ?? "Recording failed"
                                state = .done
                                ticker.stop()
                            }
                        }
                    }
                }
                // Fire at T-0
                if secondsRemaining <= 0 {
                    // #292: re-check the link at the fire instant. sendPyroFire
                    // silently no-ops when disconnected, so without this the UI
                    // would advance to recording and report success for a command
                    // that was never sent. On a base-station link the same check
                    // covers the LoRa leg: a stale relay (or no focused rocket)
                    // means the uplink would be queued blind into silence.
                    guard device.pyroCommandPathReady else {
                        errorMessage = device.isBaseStation
                            ? "Rocket LoRa link not ready — no fire command sent"
                            : "Link lost before fire — no fire command sent"
                        state = .done
                        ticker.stop()
                        return
                    }
                    // BS path: the fire relays to whatever rocket is focused
                    // NOW — abort if that is no longer the rocket this test
                    // was opened for (the #390 self-heal can re-pin).
                    if device.isBaseStation,
                       device.focusRocketID != latchedTargetRID {
                        errorMessage = "Focused rocket changed — no fire command sent"
                        state = .done
                        ticker.stop()
                        return
                    }
                    device.sendPyroFire(channel: UInt8(channel))
                    firedThisTest = true
                    state = .recording
                    recordSecondsRemaining = 5
                }
            case .recording:
                recordSecondsRemaining -= 1
                if recordSecondsRemaining <= 0 {
                    ticker.stop()
                    if startedLoggingForTest {
                        device.setRocketLoggingForTest(on: false)
                        startedLoggingForTest = false
                    }
                    camera.stopRecording()
                    // Stand-back confirmation on the LoRa path: the uplink is
                    // blind (#285) and the downlink has no fired bit, so
                    // re-test continuity — a fired charge burns the match
                    // open (CONT → NO CONT). CONT still present is the one
                    // answer that must reach the operator before anyone
                    // approaches. The done screen renders the verdict.
                    // Latch check: if focus re-pinned to a different rocket
                    // since the fire, skip — the re-test would momentarily
                    // ARM that other rocket, and the verdict already renders
                    // could-not-confirm on a focus mismatch.
                    if device.isBaseStation,
                       device.focusRocketID == latchedTargetRID {
                        device.sendPyroContTest(channel: UInt8(channel))
                    }
                }
            default:
                ticker.stop()
            }
        }
        .onAppear {
            camera.configure()
            // Latch which rocket this test is FOR (BS path; nil on direct
            // links) before anything is sent — the fire guard and the done
            // verdict compare live focus against this.
            latchedTargetRID = device.focusRocketID
            device.sendPyroContTest(channel: UInt8(channel))
        }
        .onDisappear {
            cancelAll()
            camera.stop()
        }
        .statusBarHidden()
    }

    // MARK: - Top Bar

    private var topBar: some View {
        HStack {
            // Channel + continuity
            HStack(spacing: 8) {
                Text("Pyro CH \(channel)")
                    .font(.headline.weight(.bold))
                    .foregroundColor(.white)

                // Fire path indicator: this test goes BS → LoRa → rocket, so
                // the operator knows BLE range to the rocket is NOT required.
                if device.isBaseStation {
                    Text("VIA LoRa")
                        .font(.caption2.weight(.bold))
                        .foregroundColor(.white)
                        .padding(.horizontal, 6)
                        .padding(.vertical, 2)
                        .background(Color.blue.opacity(0.8))
                        .cornerRadius(4)
                }

                HStack(spacing: 4) {
                    if device.contTestPending(channel: UInt8(channel)) {
                        ProgressView().controlSize(.mini).tint(.white)
                        Text("TESTING")
                            .font(.caption.weight(.bold))
                            .foregroundColor(.white)
                    } else {
                        Circle()
                            .fill(continuity ? Color.green : Color.red)
                            .frame(width: 10, height: 10)
                        Text(continuity ? "CONT" : "NO CONT")
                            .font(.caption.weight(.bold))
                            .foregroundColor(continuity ? .green : .red)
                    }
                }
            }

            Spacer()

            Button {
                cancelAll()
                dismiss()
            } label: {
                Image(systemName: "xmark.circle.fill")
                    .font(.title2)
                    .foregroundColor(.white.opacity(0.8))
            }
        }
        .padding(.top, 20)
    }

    // MARK: - Center Display

    @ViewBuilder
    private var centerDisplay: some View {
        switch state {
        case .idle:
            if let error = errorMessage {
                Text(error)
                    .font(.headline)
                    .foregroundColor(.red)
                    .multilineTextAlignment(.center)
            }

        case .countdown:
            VStack(spacing: 8) {
                if cameraRecording {
                    HStack(spacing: 6) {
                        Circle()
                            .fill(Color.red)
                            .frame(width: 10, height: 10)
                        Text("REC")
                            .font(.caption.weight(.bold))
                            .foregroundColor(.red)
                    }
                }
                Text("FIRING IN")
                    .font(.title3.weight(.bold))
                    .foregroundColor(.white.opacity(0.7))
                Text("\(secondsRemaining)")
                    .font(.system(size: 120, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
            }

        case .recording:
            VStack(spacing: 8) {
                HStack(spacing: 8) {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 12, height: 12)
                    Text("REC")
                        .font(.title3.weight(.bold))
                        .foregroundColor(.red)
                }
                Text("\(recordSecondsRemaining)s")
                    .font(.system(size: 80, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
            }

        case .done:
            VStack(spacing: 12) {
                Image(systemName: videoSaved ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                    .font(.system(size: 60))
                    .foregroundColor(videoSaved ? .green : .orange)
                Text(videoSaved ? "Video Saved" : "Video save failed")
                    .font(.title2.weight(.bold))
                    .foregroundColor(.white)

                if let error = errorMessage {
                    Text(error)
                        .font(.subheadline)
                        .foregroundColor(.red)
                        .multilineTextAlignment(.center)
                }

                // Stand-back verdict from the post-fire continuity re-test
                // (LoRa path only — direct-link operators are in BLE range
                // and re-test from the settings row as before).
                //
                // The green "charge fired" line requires POSITIVE evidence:
                // a live stream, the same focused rocket the test was opened
                // for, and a measured no-continuity reading (SH_BAD). Absence
                // of data must never read as success — a stale relay, an
                // undelivered re-test (health still DEGRADED/NA), a dropped
                // BLE link, or a swapped focus all collapse `continuity` to
                // false, and rendering green from any of those is exactly
                // the held-over-safe display #297 exists to prevent, in the
                // one screen that gates walking up on a live charge.
                if device.isBaseStation && firedThisTest {
                    if device.contTestPending(channel: UInt8(channel)) {
                        HStack(spacing: 6) {
                            ProgressView().controlSize(.small).tint(.white)
                            Text("Re-testing continuity…")
                                .font(.subheadline)
                                .foregroundColor(.white.opacity(0.8))
                        }
                    } else {
                        pyroFireVerdict
                    }
                }
            }
        }
    }

    /// Post-fire verdict, BS/LoRa path. Only a measured SH_BAD from a live
    /// stream of the latched rocket earns green; every no-data state renders
    /// the explicit treat-as-live warning instead.
    @ViewBuilder
    private var pyroFireVerdict: some View {
        let liveAndSameRocket = device.isConnected
            && device.effectiveDataStatus == .live
            && device.focusRocketID == latchedTargetRID
        if !liveAndSameRocket {
            Text("COULD NOT CONFIRM — rocket link lost. Treat the charge as LIVE; do not approach.")
                .font(.subheadline.weight(.bold))
                .foregroundColor(.orange)
                .multilineTextAlignment(.center)
        } else {
            switch device.telemetry.pyroHealth(channel: channel) {
            case .bad:
                Text("Continuity gone — charge fired")
                    .font(.subheadline.weight(.semibold))
                    .foregroundColor(.green)
                    .multilineTextAlignment(.center)
            case .ok:
                Text("CONTINUITY STILL PRESENT — charge may not have fired. Treat it as LIVE.")
                    .font(.subheadline.weight(.bold))
                    .foregroundColor(.orange)
                    .multilineTextAlignment(.center)
            case .degraded, .na:
                Text("COULD NOT CONFIRM — no fresh continuity reading. Treat the charge as LIVE; do not approach.")
                    .font(.subheadline.weight(.bold))
                    .foregroundColor(.orange)
                    .multilineTextAlignment(.center)
            }
        }
    }

    // MARK: - Bottom Controls

    private var bottomControls: some View {
        VStack(spacing: 16) {
            switch state {
            case .idle, .done:
                Button(action: startCountdown) {
                    Text("FIRE")
                        .font(.title2.weight(.heavy))
                        .foregroundColor(.white)
                        .frame(width: 140, height: 140)
                        .background(Circle().fill(Color.red))
                }

            case .countdown:
                Button(action: safeRocket) {
                    Text("SAFE")
                        .font(.title2.weight(.heavy))
                        .foregroundColor(.white)
                        .frame(width: 140, height: 140)
                        .background(Circle().fill(Color.green))
                }

            case .recording:
                Text("Recording...")
                    .font(.headline)
                    .foregroundColor(.white.opacity(0.7))
            }
        }
        .padding(.bottom, 30)
    }

    // MARK: - Actions

    private func startCountdown() {
        errorMessage = nil
        firedThisTest = false
        state = .countdown
        secondsRemaining = 10
        ticker.start()
    }

    private func safeRocket() {
        ticker.stop()
        if cameraRecording {
            // #560: cmd 23 is a TOGGLE — only stop logging if THIS test started
            // it. Blind-toggling here stopped an operator's pre-existing session
            // (the #385 failure mode, which was fixed on the recording-complete
            // path but missed on the abort paths). Stop the camera regardless.
            if startedLoggingForTest {
                device.setRocketLoggingForTest(on: false)
                startedLoggingForTest = false
            }
            camera.stopRecording()
            cameraRecording = false
        }
        state = .idle
        secondsRemaining = 10
    }

    private func saveToPhotoLibrary(url: URL) {
        PHPhotoLibrary.requestAuthorization(for: .addOnly) { status in
            guard status == .authorized else {
                DispatchQueue.main.async {
                    videoSaved = false
                    errorMessage = "Photo library access denied"
                    state = .done
                }
                return
            }
            PHPhotoLibrary.shared().performChanges {
                PHAssetChangeRequest.creationRequestForAssetFromVideo(atFileURL: url)
            } completionHandler: { success, error in
                try? FileManager.default.removeItem(at: url)
                DispatchQueue.main.async {
                    videoSaved = success
                    if !success {
                        errorMessage = error?.localizedDescription ?? "Save failed"
                    }
                    state = .done
                }
            }
        }
    }

    private func cancelAll() {
        ticker.stop()
        if cameraRecording {
            // #560: only stop logging if THIS test started it (see safeRocket) —
            // otherwise navigating away / dismissing mid-test toggles off an
            // operator's pre-existing log. Stop the camera regardless.
            if startedLoggingForTest {
                device.setRocketLoggingForTest(on: false)
                startedLoggingForTest = false
            }
            camera.stopRecording()
            cameraRecording = false
        }
    }
}

// MARK: - Slow-Mo Camera Manager

class SlowMoCameraManager: NSObject, AVCaptureFileOutputRecordingDelegate {
    let session = AVCaptureSession()
    private let movieOutput = AVCaptureMovieFileOutput()
    private var completion: ((URL?, Error?) -> Void)?
    private var configured = false

    func configure() {
        guard !configured else { return }
        configured = true

        session.beginConfiguration()
        session.sessionPreset = .inputPriority

        // Video input
        guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back),
              let input = try? AVCaptureDeviceInput(device: device),
              session.canAddInput(input) else {
            session.commitConfiguration()
            return
        }
        session.addInput(input)

        // Audio input
        if let audioDevice = AVCaptureDevice.default(for: .audio),
           let audioInput = try? AVCaptureDeviceInput(device: audioDevice),
           session.canAddInput(audioInput) {
            session.addInput(audioInput)
        }

        // Movie output
        if session.canAddOutput(movieOutput) {
            session.addOutput(movieOutput)
        }

        // Configure slow-motion (find highest frame rate format)
        configureSlowMo(device: device)

        session.commitConfiguration()

        DispatchQueue.global(qos: .userInitiated).async {
            self.session.startRunning()
        }
    }

    private func configureSlowMo(device: AVCaptureDevice) {
        var bestFormat: AVCaptureDevice.Format?
        var bestFrameRate: Float64 = 0

        for format in device.formats {
            let dimensions = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            // Prefer 1080p or higher for quality
            guard dimensions.height >= 720 else { continue }

            for range in format.videoSupportedFrameRateRanges {
                if range.maxFrameRate > bestFrameRate {
                    bestFrameRate = range.maxFrameRate
                    bestFormat = format
                }
            }
        }

        guard let format = bestFormat else { return }

        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: CMTimeScale(bestFrameRate))
            device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: CMTimeScale(bestFrameRate))
            device.unlockForConfiguration()
            print("Slow-mo configured: \(bestFrameRate) fps")
        } catch {
            print("Failed to configure slow-mo: \(error)")
        }
    }

    func startRecording(completion: @escaping (URL?, Error?) -> Void) {
        self.completion = completion
        let tempURL = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString)
            .appendingPathExtension("mov")
        movieOutput.startRecording(to: tempURL, recordingDelegate: self)
    }

    func stopRecording() {
        if movieOutput.isRecording {
            movieOutput.stopRecording()
        }
    }

    func stop() {
        if session.isRunning {
            session.stopRunning()
        }
    }

    // AVCaptureFileOutputRecordingDelegate
    func fileOutput(_ output: AVCaptureFileOutput,
                    didFinishRecordingTo outputFileURL: URL,
                    from connections: [AVCaptureConnection],
                    error: Error?) {
        DispatchQueue.main.async {
            self.completion?(error == nil ? outputFileURL : nil, error)
            self.completion = nil
        }
    }
}

// MARK: - Camera Preview (UIViewRepresentable)

struct CameraPreviewView: UIViewRepresentable {
    let session: AVCaptureSession

    func makeUIView(context: Context) -> UIView {
        let view = UIView(frame: .zero)
        let previewLayer = AVCaptureVideoPreviewLayer(session: session)
        previewLayer.videoGravity = .resizeAspectFill
        view.layer.addSublayer(previewLayer)
        context.coordinator.previewLayer = previewLayer
        return view
    }

    func updateUIView(_ uiView: UIView, context: Context) {
        DispatchQueue.main.async {
            context.coordinator.previewLayer?.frame = uiView.bounds
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    class Coordinator {
        var previewLayer: AVCaptureVideoPreviewLayer?
    }
}
