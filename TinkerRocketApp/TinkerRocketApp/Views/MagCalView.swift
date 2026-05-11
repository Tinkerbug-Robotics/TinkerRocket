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

    /// Sampling: orientation-progress hero, live direction bars, Compute
    /// Fit button (user-driven completion — no auto-timeout), and abort.
    /// Orientation detection runs off the low-g accelerometer (gravity
    /// vector), not the magnetometer — the mag is precisely what we're
    /// trying to calibrate, so it's unreliable as an orientation source.
    /// Accel data comes from the regular telemetry stream that runs in
    /// parallel with the cal status frames.
    private var samplingSection: some View {
        Group {
            if let s = status {
                let ax = device.telemetry.low_g_x
                let ay = device.telemetry.low_g_y
                let az = device.telemetry.low_g_z
                Section {
                    SamplingHero(status: s, ax: ax, ay: ay, az: az)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                }
                Section(header: Text("Live accel (gravity)"),
                        footer: Text("These bars show the gravity direction in the rocket's body frame. Whichever axis reads ≈ ±9.8 m/s² is the one pointing up or down. Adjust the rocket so the target axis dominates.")) {
                    DirectionBars(ax: ax, ay: ay, az: az)
                        .padding(.vertical, 4)
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
                // Compute Fit replaces the old auto-completion-at-buffer-fill.
                // Disabled until min samples reached so the user can't ship a
                // bad fit; styled as the primary action once ready.
                let canFit = s.sampleCount >= MagCalConstants.minSamples
                Section(footer: Text(canFit
                    ? "When you're happy with the coverage, tap Compute Fit to run the sphere fit and review the result."
                    : "Need ≥ \(MagCalConstants.minSamples) samples before fitting. Keep tumbling…")) {
                    Button {
                        device.sendMagCalComputeFit()
                    } label: {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                            Text("Compute Fit")
                                .fontWeight(.semibold)
                            Spacer()
                        }
                        .foregroundColor(.white)
                    }
                    .listRowBackground(canFit ? Color.blue : Color.gray)
                    .disabled(!canFit)
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

/// Guidance card for the sampling phase.  Free-form orientation capture
/// driven by the LOW-G ACCELEROMETER (gravity vector) — the user
/// rotates the rocket through 6 unique orientations in whatever order
/// is physically convenient, and each signed-axis (+X / -X / +Y / -Y /
/// +Z / -Z) gets captured independently when the user holds that axis
/// dominant for 1.5 s.  Earlier versions tried a fixed body-frame
/// mapping (nose UP = +X, etc.); on the current rocket that mapping
/// proved fragile (Down never triggered) so we drop the mapping
/// entirely and let the bars + grid teach the user which physical
/// rotation produces which signed axis.
///
/// Per-axis state machine:
///   pending   → no progress yet
///   capturing → axis is currently the dominant one; progress fills
///               toward HOLD_SECONDS, gentle half-rate decay if the
///               user wobbles out of alignment briefly
///   captured  → progress hit HOLD_SECONDS; cell turns green and stays
///
/// Mag-side coverage from the FC is still shown (as the "fit dvs"
/// gauge) — that's the diversity of the sphere fit's input data, and
/// it grows in parallel with the accel-driven capture loop.
private struct SamplingHero: View {
    let status: MagCalStatus
    /// Live low-g accelerometer in body frame, m/s².  Pulled from the
    /// regular telemetry stream by the parent view.  Nil = telemetry
    /// hasn't arrived yet; the hero shows a "waiting" state.
    let ax: Float?
    let ay: Float?
    let az: Float?

    /// Time the user must hold an axis aligned before it captures.
    private static let HOLD_SECONDS: Double = 1.5
    private static let TICK_SECONDS: Double = 0.05

    /// Minimum component magnitude (m/s²) for an axis to count as
    /// "dominant" — filters free-fall, slow tumble, in-between
    /// orientations.  Gravity is ~9.81; 4 m/s² ≈ 24° off pure axis.
    private static let DOMINANT_THRESHOLD_MS2: Float = 4.0

    /// Per-signed-axis hold progress, 0..HOLD_SECONDS.  Captured when ≥
    /// HOLD_SECONDS.  Kept as a dict keyed by SignedAxis so we can
    /// progress multiple axes across the run (each rotation captures
    /// the one currently dominant).
    @State private var captureProgress: [SignedAxis: Double] = [:]

    /// State-machine tick.  Started in onAppear, torn down on disappear.
    @State private var tickTimer: Timer?

    /// Whichever signed axis is currently dominant in the accel reading,
    /// or nil if no axis is above the dominance threshold (free-fall,
    /// rapid tumble, near-zero gravity component).
    private var currentlyAligned: SignedAxis? {
        guard let x = ax, let y = ay, let z = az else { return nil }
        let xa = abs(x), ya = abs(y), za = abs(z)
        let peak = max(xa, max(ya, za))
        guard peak >= SamplingHero.DOMINANT_THRESHOLD_MS2 else { return nil }

        if peak == xa { return SignedAxis(component: .x, positive: x > 0) }
        if peak == ya { return SignedAxis(component: .y, positive: y > 0) }
        return SignedAxis(component: .z, positive: z > 0)
    }

    private var capturedCount: Int {
        SignedAxis.allCases.filter { (captureProgress[$0] ?? 0) >= SamplingHero.HOLD_SECONDS }.count
    }

    /// Adaptive headline based on capture progress.
    private var headline: String {
        switch capturedCount {
        case 6:     return "All six orientations captured"
        case 4...5: return "Almost there"
        case 1...3: return "Keep rotating"
        default:    return "Rotate to any orientation"
        }
    }

    var body: some View {
        VStack(spacing: 18) {
            Text(headline)
                .font(.title3)
                .fontWeight(.semibold)
            Text("\(capturedCount) of 6 directions captured")
                .font(.subheadline)
                .foregroundColor(.secondary)

            currentTargetCard
            orientationGrid

            HStack(spacing: 28) {
                // Live |B| (raw, includes hard-iron bias).  Useful as a
                // sanity check that mag is still sampling.
                VStack(spacing: 2) {
                    Text(String(format: "%.0f", status.instantaneousFieldUT))
                        .font(.system(size: 32, weight: .bold, design: .monospaced))
                    Text("µT raw |B|")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                // Sphere-fit diversity gauge — how many of the 26 wedges
                // of MAGNETIC-space have been sampled.  Independent of
                // the accel-driven capture grid above.  More wedges →
                // better-conditioned sphere fit.
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
                            .font(.system(size: 20, weight: .bold, design: .monospaced))
                        Text("fit dvs")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
                .frame(width: 76, height: 76)
            }
        }
        .onAppear { startTimers() }
        .onDisappear { stopTimers() }
    }

    // MARK: - Pieces

    /// Status card above the grid.  Three visual states:
    ///   - Aligned (not yet captured): blue, progress bar fills toward
    ///     capture; tells the user "you're doing it, keep holding."
    ///   - Aligned (already captured): green check; "Try a different
    ///     orientation."
    ///   - Not aligned: orange; "Rotate the rocket — watch the bars."
    /// All three states use the live accel reading; no fixed body-frame
    /// assumption.
    @ViewBuilder
    private var currentTargetCard: some View {
        if capturedCount == 6 {
            HStack(spacing: 12) {
                Image(systemName: "checkmark.seal.fill")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(.green)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 2) {
                    Text("All six orientations captured")
                        .font(.headline)
                    Text("Tap Compute Fit when ready.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Spacer()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 14)
            .background(Color.green.opacity(0.12))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 8)
        } else if let active = currentlyAligned {
            let progress = captureProgress[active] ?? 0
            let alreadyCaptured = progress >= SamplingHero.HOLD_SECONDS
            HStack(spacing: 12) {
                Image(systemName: alreadyCaptured ? "checkmark.circle.fill" : "scope")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(alreadyCaptured ? .green : .blue)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 4) {
                    Text(alreadyCaptured
                        ? "\(active.label) already captured"
                        : "Capturing \(active.label) — hold steady")
                        .font(.headline)
                    if !alreadyCaptured {
                        ProgressView(value: progress,
                                     total: SamplingHero.HOLD_SECONDS)
                            .progressViewStyle(.linear)
                            .tint(.blue)
                        Text(String(format: "%.0f%%",
                                    100 * progress / SamplingHero.HOLD_SECONDS))
                            .font(.caption)
                            .foregroundColor(.secondary)
                    } else {
                        Text("Rotate to a different orientation")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                Spacer()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 14)
            .background((alreadyCaptured ? Color.green : Color.blue).opacity(0.12))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 8)
            .animation(.easeInOut(duration: 0.20), value: alreadyCaptured)
        } else {
            HStack(spacing: 12) {
                Image(systemName: "arrow.triangle.2.circlepath")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(.orange)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 4) {
                    Text("Rotate the rocket")
                        .font(.headline)
                    Text("Watch the bars below — when one axis reaches ≈ ±9.8 m/s², hold steady.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Spacer()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 14)
            .background(Color.orange.opacity(0.12))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 8)
        }
    }

    /// 6-cell grid showing per-signed-axis capture state.  Labels are
    /// the raw ±X / ±Y / ±Z signed axis (no body-frame guessing) so the
    /// user can match directly against the accel bars.
    private var orientationGrid: some View {
        let cols = [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())]
        return LazyVGrid(columns: cols, spacing: 8) {
            ForEach(SignedAxis.allCases, id: \.self) { axis in
                let progress = captureProgress[axis] ?? 0
                let captured = progress >= SamplingHero.HOLD_SECONDS
                let isActive = currentlyAligned == axis && !captured
                let capturing = progress > 0 && !captured
                VStack(spacing: 4) {
                    Image(systemName: captured ? "checkmark.circle.fill"
                                  : isActive ? "scope"
                                  : "circle")
                        .font(.system(size: 22, weight: .semibold))
                        .foregroundColor(captured ? .green
                                       : isActive ? .blue
                                       : (capturing ? .blue.opacity(0.5) : .gray))
                    Text(axis.label)
                        .font(.system(.headline, design: .monospaced))
                        .foregroundColor(captured ? .primary : .secondary)
                    if capturing {
                        ProgressView(value: progress, total: SamplingHero.HOLD_SECONDS)
                            .progressViewStyle(.linear)
                            .tint(.blue)
                            .frame(height: 3)
                    }
                }
                .frame(maxWidth: .infinity, minHeight: 64)
                .padding(.vertical, 8)
                .background(captured ? Color.green.opacity(0.12)
                          : isActive ? Color.blue.opacity(0.12)
                          : Color.gray.opacity(0.08))
                .clipShape(RoundedRectangle(cornerRadius: 8))
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(isActive ? Color.blue : Color.clear, lineWidth: 2)
                )
            }
        }
        .padding(.horizontal, 8)
    }

    // MARK: - Timers / state-machine

    private func startTimers() {
        tickTimer?.invalidate()
        tickTimer = Timer.scheduledTimer(withTimeInterval: SamplingHero.TICK_SECONDS,
                                          repeats: true) { _ in
            DispatchQueue.main.async { advance() }
        }
    }

    private func stopTimers() {
        tickTimer?.invalidate(); tickTimer = nil
    }

    /// One tick of the state machine.  If an axis is currently dominant
    /// in the accel reading, fill its progress.  Otherwise, gently
    /// decay any in-progress (non-captured) axes so brief wobble
    /// doesn't reset progress on whichever axis the user is working on.
    private func advance() {
        let step = SamplingHero.TICK_SECONDS
        if let active = currentlyAligned {
            let p = captureProgress[active] ?? 0
            if p < SamplingHero.HOLD_SECONDS {
                captureProgress[active] = min(SamplingHero.HOLD_SECONDS, p + step)
            }
        } else {
            for axis in SignedAxis.allCases {
                let p = captureProgress[axis] ?? 0
                if p > 0 && p < SamplingHero.HOLD_SECONDS {
                    captureProgress[axis] = max(0, p - step * 0.5)
                }
            }
        }
    }
}

/// A signed body-frame axis — the six unique directions a rocket's
/// gravity vector can point.  Used as the unit of "captured
/// orientation" in the cal hero.
private struct SignedAxis: Hashable {
    let component: MagCalComponent
    let positive: Bool

    static let allCases: [SignedAxis] = [
        SignedAxis(component: .x, positive: true),
        SignedAxis(component: .x, positive: false),
        SignedAxis(component: .y, positive: true),
        SignedAxis(component: .y, positive: false),
        SignedAxis(component: .z, positive: true),
        SignedAxis(component: .z, positive: false),
    ]

    var label: String {
        "\(positive ? "+" : "-")\(component.rawValue)"
    }
}

// MARK: - Direction Bars

/// Live low-g accelerometer (gravity) display: three signed horizontal
/// bars for X, Y, Z (m/s²).  Drives orientation feedback during the cal
/// because the magnetometer is unreliable as an orientation reference
/// — it's literally what we're trying to calibrate, and at the start of
/// a cal session it can be biased by 30× Earth's field.  Gravity is a
/// dependable ±9.81 m/s² reference vector that's always in the body
/// frame.
///
/// The currently dominant axis (largest |value| above a small dead
/// zone) is highlighted in blue so the user can verify at a glance
/// which body axis is currently pointing up or down.
///
/// nil accel inputs render a "waiting for telemetry" placeholder.
private struct DirectionBars: View {
    let ax: Float?
    let ay: Float?
    let az: Float?

    /// Bar full-scale.  Set just above 1 g so a clean cardinal
    /// orientation reads ~80% of the bar — visually saturated without
    /// being pinned at the edge.
    private static let RANGE_MS2: Float = 12.0

    /// Components below this (m/s²) don't count as "dominant" — avoids
    /// flickering the highlight between near-zero axes when the rocket
    /// is in between orientations or tumbling.
    private static let DEAD_ZONE_MS2: Float = 2.0

    private var hasLiveVector: Bool {
        ax != nil && ay != nil && az != nil
    }

    /// Which axis is currently dominant, or nil if all three are below
    /// the dead zone.
    private var dominantAxis: MagCalComponent? {
        guard let x = ax, let y = ay, let z = az else { return nil }
        let xAbs = abs(x), yAbs = abs(y), zAbs = abs(z)
        let peak = max(xAbs, max(yAbs, zAbs))
        if peak < DirectionBars.DEAD_ZONE_MS2 { return nil }
        if peak == xAbs { return .x }
        if peak == yAbs { return .y }
        return .z
    }

    var body: some View {
        if hasLiveVector, let x = ax, let y = ay, let z = az {
            VStack(spacing: 10) {
                ComponentBar(label: "X", value: x,
                             range: DirectionBars.RANGE_MS2,
                             isDominant: dominantAxis == .x,
                             unitSuffix: " m/s²")
                ComponentBar(label: "Y", value: y,
                             range: DirectionBars.RANGE_MS2,
                             isDominant: dominantAxis == .y,
                             unitSuffix: " m/s²")
                ComponentBar(label: "Z", value: z,
                             range: DirectionBars.RANGE_MS2,
                             isDominant: dominantAxis == .z,
                             unitSuffix: " m/s²")
            }
        } else {
            Text("Waiting for accelerometer telemetry…")
                .font(.caption)
                .foregroundColor(.secondary)
        }
    }
}

private struct ComponentBar: View {
    let label: String
    let value: Float
    let range: Float
    let isDominant: Bool
    /// e.g. " µT" or " m/s²".  Currently unused in the trailing readout
    /// (we keep it compact at "+9.8") but lives here so a future Detail
    /// row can show the full unit.
    let unitSuffix: String

    var body: some View {
        HStack(spacing: 12) {
            Text(label)
                .font(.system(.headline, design: .monospaced))
                .frame(width: 20)
                .foregroundColor(isDominant ? .blue : .secondary)

            // Two-sided bar centered at zero.  GeometryReader lets us
            // size the half-bar in points and place it relative to the
            // centre tick.
            GeometryReader { geo in
                let half = geo.size.width / 2
                let mag = min(abs(value), range)
                let frac = CGFloat(mag) / CGFloat(range)
                let barWidth = frac * half

                ZStack(alignment: .leading) {
                    // Track
                    RoundedRectangle(cornerRadius: 4)
                        .fill(Color.gray.opacity(0.15))
                        .frame(height: 18)
                    // Centre tick
                    Rectangle()
                        .fill(Color.gray.opacity(0.5))
                        .frame(width: 1, height: 24)
                        .offset(x: half - 0.5)
                    // Signed bar
                    RoundedRectangle(cornerRadius: 3)
                        .fill(isDominant ? Color.blue : Color.blue.opacity(0.45))
                        .frame(width: barWidth, height: 16)
                        .offset(x: value >= 0 ? half : (half - barWidth))
                }
            }
            .frame(height: 24)

            Text(String(format: "%+.1f", value))
                .font(.system(size: 14, weight: .semibold, design: .monospaced))
                .frame(width: 56, alignment: .trailing)
                .foregroundColor(isDominant ? .blue : .secondary)
        }
    }
}
