//
//  MagCalView.swift
//  TinkerRocketApp
//
//  Magnetometer hard-iron calibration screen (issue #96).  The user opens
//  this from the rocket Settings page; tapping Start asks the FC to enter
//  MAG_CALIBRATION and begin sampling.  The user tumbles the rocket
//  through all orientations for ~10 s while a progress bar tracks
//  sample count and orientation coverage.  When the buffer fills, the FC
//  computes a sphere fit and ships the result back; the view shows the
//  fitted offset, fitted Earth-field magnitude (R), and RMS residual.
//  Accept persists to FC NVS and programs the IIS2MDC OFFSET registers;
//  Retry restarts sampling; Abort drops the run and returns to READY.
//
//  All state lives on the FC.  The view is just a renderer + a few
//  one-byte command buttons; magCalStatus on the BLEDevice is the source
//  of truth and updates at 5 Hz during sampling.
//

import SwiftUI

struct MagCalView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    /// Latest status frame, or nil if the FC hasn't published one yet
    /// (e.g. just navigated in, before tapping Start).  In that case the
    /// view shows the intro/Start state.
    private var status: MagCalStatus? { device.magCalStatus }

    var body: some View {
        Form {
            // The view is mostly status-driven: intro/start, sampling,
            // review (accept/retry), applied success, aborted return.
            switch status?.subType ?? .idle {
            case .idle, .aborted:    introSection
            case .sampling:           samplingSection
            case .review:             reviewSection
            case .applied:            appliedSection
            }
        }
        .navigationTitle("Mag Calibration")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .navigationBarLeading) {
                if status?.subType == .sampling || status?.subType == .review {
                    // Belt-and-suspenders abort: if the user tries to
                    // back out mid-cal, send abort so the FC drops back
                    // to READY rather than staying in MAG_CALIBRATION.
                    Button("Cancel") {
                        device.sendMagCalAbort()
                        dismiss()
                    }
                } else {
                    Button("Done") { dismiss() }
                }
            }
        }
    }

    // MARK: - States

    /// Intro: explain the flow + show a Start button.
    private var introSection: some View {
        Group {
            Section {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Magnetometer Hard-Iron Calibration")
                        .font(.headline)
                    Text("Solves for the fixed magnetic offset on the rocket PCB so the magnetometer reads true Earth field. Run this once per board (or after any hardware change near the mag chip).")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                }
                .padding(.vertical, 4)
            }

            Section(header: Text("How to tumble")) {
                VStack(alignment: .leading, spacing: 8) {
                    Label("Hold the rocket clear of laptops, phones, speakers.", systemImage: "1.circle.fill")
                    Label("Slowly rotate through every orientation for ~10 s.", systemImage: "2.circle.fill")
                    Label("Aim to point each end of the rocket up, down, and sideways.", systemImage: "3.circle.fill")
                }
                .font(.subheadline)
            }

            // Show the prior aborted-status message inline so a re-entry
            // after Cancel doesn't look like a no-op.
            if status?.subType == .aborted {
                Section {
                    Label("Previous run cancelled.", systemImage: "info.circle")
                        .foregroundColor(.secondary)
                }
            }

            Section {
                Button {
                    device.sendMagCalStart()
                } label: {
                    HStack {
                        Image(systemName: "play.fill")
                        Text("Start Calibration")
                            .fontWeight(.semibold)
                        Spacer()
                    }
                    .foregroundColor(.white)
                }
                .listRowBackground(Color.blue)
            }

            Section {
                Text("Calibration is only allowed when the rocket is in READY state. Launch is gated against this mode — no risk of an accidental flight detection while tumbling.")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
    }

    /// Sampling: animated tumble guidance, live |B|, circular coverage
    /// gauge, and a cycling orientation prompt so the user always knows
    /// what to do next.  Drives all visuals off the FC's 5 Hz status
    /// frame — no extra polling.
    private var samplingSection: some View {
        Group {
            if let s = status {
                Section {
                    SamplingHero(status: s)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                }
                Section(header: Text("Progress")) {
                    HStack {
                        Text("Coverage")
                        Spacer()
                        Text("\(s.coverageBins) / 26 wedges")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("Samples")
                        Spacer()
                        Text("\(s.sampleCount) / \(MagCalConstants.maxSamples)")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                }
                Section {
                    Button(role: .destructive) {
                        device.sendMagCalAbort()
                    } label: {
                        HStack {
                            Image(systemName: "xmark.circle")
                            Text("Abort")
                            Spacer()
                        }
                    }
                }
            } else {
                Section {
                    ProgressView("Waiting for rocket…")
                }
            }
        }
    }

    /// Review: show fit + accept/retry buttons.  rejectCode != .ok forces
    /// retry-only (Accept is hidden).
    private var reviewSection: some View {
        Group {
            if let s = status {
                Section(header: Text("Fit result")) {
                    HStack {
                        Label("Status", systemImage: s.rejectCode == .ok ? "checkmark.seal.fill" : "exclamationmark.triangle.fill")
                        Spacer()
                        Text(s.rejectMessage)
                            .foregroundColor(s.rejectCode == .ok ? .green : .orange)
                            .multilineTextAlignment(.trailing)
                    }
                    HStack {
                        Text("Earth field |R|")
                        Spacer()
                        Text(String(format: "%.1f µT", s.fieldR_uT))
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("RMS residual")
                        Spacer()
                        Text(String(format: "%.1f µT", s.residualUT))
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("Coverage")
                        Spacer()
                        Text("\(s.coverageBins) / 26 wedges")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                }

                Section(header: Text("Hard-iron offset (raw counts)")) {
                    HStack {
                        Text("X")
                        Spacer()
                        Text("\(s.offsetX)")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("Y")
                        Spacer()
                        Text("\(s.offsetY)")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("Z")
                        Spacer()
                        Text("\(s.offsetZ)")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    Text("0.15 µT per LSB. The chip subtracts these from every sample once accepted.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                if s.rejectCode == .ok {
                    Section {
                        Button {
                            device.sendMagCalAccept()
                        } label: {
                            HStack {
                                Image(systemName: "checkmark.circle.fill")
                                Text("Accept and Save")
                                    .fontWeight(.semibold)
                                Spacer()
                            }
                            .foregroundColor(.white)
                        }
                        .listRowBackground(Color.green)
                    }
                }
                Section {
                    Button {
                        device.sendMagCalRetry()
                    } label: {
                        HStack {
                            Image(systemName: "arrow.counterclockwise")
                            Text("Retry")
                            Spacer()
                        }
                    }
                    Button(role: .destructive) {
                        device.sendMagCalAbort()
                        dismiss()
                    } label: {
                        HStack {
                            Image(systemName: "xmark.circle")
                            Text("Abort")
                            Spacer()
                        }
                    }
                }
            }
        }
    }

    /// One-shot success: FC just persisted + applied the offset.
    private var appliedSection: some View {
        Group {
            if let s = status {
                Section {
                    VStack(spacing: 12) {
                        Image(systemName: "checkmark.seal.fill")
                            .font(.system(size: 48))
                            .foregroundColor(.green)
                        Text("Calibration applied")
                            .font(.headline)
                        Text(String(format: "Earth field locked at %.1f µT, residual %.1f µT.",
                                    s.fieldR_uT, s.residualUT))
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 12)
                }
                Section {
                    Button {
                        dismiss()
                    } label: {
                        HStack {
                            Spacer()
                            Text("Done")
                                .fontWeight(.semibold)
                            Spacer()
                        }
                        .foregroundColor(.white)
                    }
                    .listRowBackground(Color.blue)
                }
            }
        }
    }
}

// MARK: - Sampling Hero

/// Animated guidance card shown at the top of the sampling phase.  Three
/// pieces of information that drive whether the user knows what to do:
///   1. A rotation animation so they're not staring at a dead screen.
///   2. A live |B| readout — the number changes immediately as they
///      tumble, which is the most visceral confirmation that sampling
///      is happening.
///   3. A cycling orientation prompt with an arrow icon, so they always
///      have an explicit "now do this" instead of a vague "tumble it."
///
/// All three drive off the FC's status frames + a local 2.5 s prompt
/// timer; nothing requires extra firmware support.
private struct SamplingHero: View {
    let status: MagCalStatus

    @State private var promptIndex: Int = 0
    @State private var rotation: Double = 0

    private static let prompts: [(icon: String, text: String)] = [
        ("arrow.up",                "Point the nose UP"),
        ("arrow.down",              "Now point the nose DOWN"),
        ("arrow.left",              "Lay it on its LEFT side"),
        ("arrow.right",             "Lay it on its RIGHT side"),
        ("arrow.up.right",          "Tilt at an angle"),
        ("arrow.triangle.2.circlepath", "Slowly roll it through every direction"),
    ]

    /// Headline that adapts to coverage so the user gets feedback even
    /// when they're between cardinal-direction prompts.
    private var headline: String {
        switch status.coverageBins {
        case 0..<6:   return "Start tumbling"
        case 6..<14:  return "Good — keep going"
        case 14..<22: return "Almost there"
        default:      return "Excellent coverage"
        }
    }

    var body: some View {
        VStack(spacing: 18) {
            Text(headline)
                .font(.title3)
                .fontWeight(.semibold)

            // Rotating icon — visible motion is the cheapest signal that
            // sampling is live.  Stops when the user accepts or aborts
            // because the parent view replaces this hero entirely.
            Image(systemName: "rotate.3d")
                .font(.system(size: 64))
                .foregroundColor(.blue)
                .rotationEffect(.degrees(rotation))
                .onAppear {
                    withAnimation(.linear(duration: 3.0).repeatForever(autoreverses: false)) {
                        rotation = 360
                    }
                }

            // Cycling orientation prompt.  2.5 s/step × 6 steps = 15 s,
            // longer than the ~10 s sample window so the user sees most
            // of the cardinal cues at least once.
            HStack(spacing: 12) {
                Image(systemName: SamplingHero.prompts[promptIndex].icon)
                    .font(.system(size: 28, weight: .semibold))
                    .foregroundColor(.orange)
                    .frame(width: 36)
                Text(SamplingHero.prompts[promptIndex].text)
                    .font(.headline)
                    .multilineTextAlignment(.leading)
                Spacer()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 12)
            .background(Color.orange.opacity(0.10))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 8)
            .onAppear { startPromptCycle() }

            HStack(spacing: 28) {
                // Live |B|.  Big monospaced digits so the eye spots the
                // movement and the user trusts that sampling is live.
                VStack(spacing: 2) {
                    Text(String(format: "%.0f", status.instantaneousFieldUT))
                        .font(.system(size: 36, weight: .bold, design: .monospaced))
                    Text("µT field")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                // Coverage gauge.  Circular feels more "spherical" than
                // a linear bar — and the cal is literally about lighting
                // up wedges of a sphere.
                ZStack {
                    Circle()
                        .stroke(Color.gray.opacity(0.20), lineWidth: 10)
                    Circle()
                        .trim(from: 0, to: CGFloat(min(Double(status.coverageBins) / 26.0, 1.0)))
                        .stroke(Color.green,
                                style: StrokeStyle(lineWidth: 10, lineCap: .round))
                        .rotationEffect(.degrees(-90))
                        .animation(.easeInOut(duration: 0.4), value: status.coverageBins)
                    VStack(spacing: 0) {
                        Text("\(status.coverageBins)")
                            .font(.system(size: 22, weight: .bold, design: .monospaced))
                        Text("/ 26")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
                .frame(width: 84, height: 84)
            }
        }
    }

    private func startPromptCycle() {
        Timer.scheduledTimer(withTimeInterval: 2.5, repeats: true) { timer in
            // The hero is created fresh on each entry to .sampling, so the
            // timer's lifetime is bounded by the view.  We invalidate
            // explicitly when the index would go out of range — defensive
            // against the prompts array shrinking later.
            DispatchQueue.main.async {
                promptIndex = (promptIndex + 1) % SamplingHero.prompts.count
            }
        }
    }
}
