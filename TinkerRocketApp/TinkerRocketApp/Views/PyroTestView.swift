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
            Task { @MainActor in
                self?.tick += 1
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
    @State private var videoSaved = false
    @State private var errorMessage: String?

    @State private var camera = SlowMoCameraManager()

    private var continuity: Bool {
        channel == 1 ? device.telemetry.pyro1_cont : device.telemetry.pyro2_cont
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
                    device.sendToggleLogging()
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
                    device.sendPyroFire(channel: UInt8(channel))
                    state = .recording
                    recordSecondsRemaining = 5
                }
            case .recording:
                recordSecondsRemaining -= 1
                if recordSecondsRemaining <= 0 {
                    ticker.stop()
                    device.sendToggleLogging()
                    camera.stopRecording()
                }
            default:
                ticker.stop()
            }
        }
        .onAppear {
            camera.configure()
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

                HStack(spacing: 4) {
                    Circle()
                        .fill(continuity ? Color.green : Color.red)
                        .frame(width: 10, height: 10)
                    Text(continuity ? "CONT" : "NO CONT")
                        .font(.caption.weight(.bold))
                        .foregroundColor(continuity ? .green : .red)
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
        state = .countdown
        secondsRemaining = 10
        ticker.start()
    }

    private func safeRocket() {
        ticker.stop()
        if cameraRecording {
            device.sendToggleLogging()
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
            device.sendToggleLogging()
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
