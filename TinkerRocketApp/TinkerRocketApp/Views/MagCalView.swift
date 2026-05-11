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

/// Guidance card for the sampling phase.  Tap-to-capture model:
///
/// - The 6 grid cells correspond to the six signed body-frame axes
///   (+X, -X, +Y, -Y, +Z, -Z) the gravity vector can point to.
/// - The user rotates the rocket to each orientation in turn and
///   TAPS the corresponding cell to mark it captured.  Tapping is
///   the source of truth — sensor weirdness, accel bias, or wedge
///   thresholds can't silently block progress.
/// - The live accel reading still gets shown as a per-cell highlight:
///   whichever axis is currently dominant gets a blue ring + "scope"
///   icon, so the user can see what their current orientation is
///   producing.  That's purely informational, not gating.
///
/// Captures don't drive the firmware-side fit — the FC keeps
/// collecting mag samples continuously while in MAG_CALIBRATION.  The
/// 6-axis grid is the user's mental checklist for "have I shown the
/// rocket in 6 distinct orientations?"
private struct SamplingHero: View {
    let status: MagCalStatus
    /// Live low-g accelerometer in body frame, m/s².  Pulled from the
    /// regular telemetry stream by the parent view.  Used only for the
    /// per-cell "dominant axis" highlight.
    let ax: Float?
    let ay: Float?
    let az: Float?

    /// Threshold (m/s²) for an axis to be considered "dominant" in the
    /// auto-detect highlight.  Below this, no cell is highlighted.
    private static let DOMINANT_THRESHOLD_MS2: Float = 4.0

    /// Which signed axes the user has tapped to mark captured.  Set is
    /// monotonic across the run — once captured, always captured (until
    /// Abort or Retry).  This is the only state the hero owns; the FC
    /// is sampling mag continuously regardless of what's in here.
    @State private var capturedAxes: Set<SignedAxis> = []

    /// Whichever signed axis is currently dominant in the accel reading.
    /// Pure-derived; no state.  Returns nil when no axis is above the
    /// dominance threshold (free-fall, rapid tumble, near-zero gravity).
    private var dominantAxis: SignedAxis? {
        guard let x = ax, let y = ay, let z = az else { return nil }
        let xa = abs(x), ya = abs(y), za = abs(z)
        let peak = max(xa, max(ya, za))
        guard peak >= SamplingHero.DOMINANT_THRESHOLD_MS2 else { return nil }
        if peak == xa { return SignedAxis(component: .x, positive: x > 0) }
        if peak == ya { return SignedAxis(component: .y, positive: y > 0) }
        return SignedAxis(component: .z, positive: z > 0)
    }

    private var capturedCount: Int { capturedAxes.count }

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
                // Live |B| (raw, includes hard-iron bias).  Sanity
                // check that mag is still sampling.
                VStack(spacing: 2) {
                    Text(String(format: "%.0f", status.instantaneousFieldUT))
                        .font(.system(size: 32, weight: .bold, design: .monospaced))
                    Text("µT raw |B|")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                // FC-side sphere-fit diversity gauge.  Tracks how many
                // of the 26 mag-space wedges have been visited; with a
                // large hard-iron bias this can stay near zero even as
                // sample_count grows — that's expected and the fit will
                // still recover the bias.
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
    }

    // MARK: - Pieces

    /// Status card above the grid.  Hints at what the user should do
    /// based on what's currently happening:
    ///   - All 6 captured: "Tap Compute Fit when ready"
    ///   - An axis is dominant + not yet captured: "Tap +X to mark captured"
    ///   - An axis is dominant + already captured: "+X already done,
    ///     rotate to a different orientation"
    ///   - No axis dominant: "Rotate the rocket — watch the bars"
    @ViewBuilder
    private var currentTargetCard: some View {
        if capturedCount == 6 {
            heroCard(
                icon: "checkmark.seal.fill",
                color: .green,
                title: "All six orientations captured",
                subtitle: "Tap Compute Fit when ready."
            )
        } else if let active = dominantAxis {
            let isAlreadyCaptured = capturedAxes.contains(active)
            heroCard(
                icon: isAlreadyCaptured ? "checkmark.circle.fill" : "hand.tap.fill",
                color: isAlreadyCaptured ? .green : .blue,
                title: isAlreadyCaptured
                    ? "\(active.label) already captured"
                    : "Tap the \(active.label) cell to capture",
                subtitle: isAlreadyCaptured
                    ? "Rotate to a different orientation."
                    : "Hold the rocket steady while you tap."
            )
        } else {
            heroCard(
                icon: "arrow.triangle.2.circlepath",
                color: .orange,
                title: "Rotate the rocket",
                subtitle: "Watch the bars below — when one axis reaches ≈ ±9.8 m/s², tap the matching cell."
            )
        }
    }

    private func heroCard(icon: String, color: Color, title: String, subtitle: String) -> some View {
        HStack(spacing: 12) {
            Image(systemName: icon)
                .font(.system(size: 36, weight: .semibold))
                .foregroundColor(color)
                .frame(width: 44)
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.headline)
                Text(subtitle)
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            Spacer()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 14)
        .background(color.opacity(0.12))
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 8)
    }

    /// 6-cell grid.  Each cell is a tap-target — tap toggles capture
    /// for that signed axis so an accidental tap can be undone without
    /// restarting the whole cal.  Auto-detect (live accel dominance)
    /// drives a blue ring around the matching cell as a hint but
    /// doesn't gate capture — the tap is what counts.
    private var orientationGrid: some View {
        let cols = [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())]
        return LazyVGrid(columns: cols, spacing: 8) {
            ForEach(SignedAxis.allCases, id: \.self) { axis in
                let captured = capturedAxes.contains(axis)
                let isActive = dominantAxis == axis && !captured
                Button {
                    // Toggle: tap an unmarked cell to capture, tap a
                    // captured cell to clear it (undo a misclick).
                    if capturedAxes.contains(axis) {
                        capturedAxes.remove(axis)
                    } else {
                        capturedAxes.insert(axis)
                    }
                } label: {
                    VStack(spacing: 4) {
                        Image(systemName: captured ? "checkmark.circle.fill"
                                       : isActive ? "scope"
                                       : "circle")
                            .font(.system(size: 22, weight: .semibold))
                            .foregroundColor(captured ? .green
                                           : isActive ? .blue
                                           : .gray)
                        Text(axis.label)
                            .font(.system(.headline, design: .monospaced))
                            .foregroundColor(captured ? .primary : .secondary)
                        Text(captured ? "tap to undo"
                           : isActive ? "tap to mark"
                           : "")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    .frame(maxWidth: .infinity, minHeight: 72)
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
                .buttonStyle(.plain)
            }
        }
        .padding(.horizontal, 8)
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
