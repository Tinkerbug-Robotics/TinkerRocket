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

/// Guidance card shown at the top of the sampling phase.  Built around
/// the FC's `coverageMask` — bit i is set once the user has dwelt long
/// enough in wedge i for the calibrator to pick up that orientation.
///
/// The hero shows three things, all driven off the live mask:
///   1. A "current target" card — one of six cardinal axes (nose UP,
///      DOWN, LEFT side, RIGHT side, FRONT face, BACK face).  We pick
///      the first uncovered axis in a fixed order and stay on it until
///      the FC reports the corresponding wedge bit is set.  The card
///      flips to a green checkmark for ~0.5 s before advancing — that
///      micro-pause is the user-visible confirmation that the firmware
///      and the app agree this orientation is captured.
///   2. A 6-cell orientation grid showing which cardinal axes are done.
///      Replaces the spinning-globe icon: users can see at a glance
///      exactly how many more orientations are left, and which.
///   3. A live |B| readout in µT so the user trusts that sampling is
///      running.  Big monospaced digits — eye spots the change.
///
/// Old firmware (22-byte payload, coverageMask=0) means no per-axis
/// info; in that case we fall back to a slow timer cycle through all
/// six prompts so the UX still works.
private struct SamplingHero: View {
    let status: MagCalStatus

    /// All six cardinal axes in the order the user is walked through them.
    /// The fixed order means muscle memory carries between runs.
    private static let axisOrder: [MagCalAxis] = [
        .noseUp, .noseDown, .rightSide, .leftSide, .frontFace, .backFace
    ]

    /// Last target we showed — used to detect a "just covered" transition
    /// so we can briefly hold the green-check confirmation.
    @State private var lastSeenTarget: MagCalAxis?

    /// When non-nil, the card shows this axis as just-captured (green
    /// check) for ~0.6 s before the prompt advances.  Bridges the gap
    /// between "wedge bit flipped" and "user has time to register it."
    @State private var heldCoveredAxis: MagCalAxis?

    /// Old-firmware fallback: cycle index for the timer-driven path.
    @State private var fallbackIndex: Int = 0

    /// Are we running against firmware that ships the coverage mask?
    /// True any time the mask carries information OR no samples have
    /// landed yet (so coverage_bins==0 isn't mistaken for "old FC").
    private var hasLiveMask: Bool { status.coverageMask != 0 || status.coverageBins == 0 }

    /// First uncovered axis in fixed order — the "do this now" target.
    /// nil once all six cardinals are covered.
    private var nextUncoveredAxis: MagCalAxis? {
        SamplingHero.axisOrder.first { !status.isAxisCovered($0) }
    }

    /// Adaptive headline so the user gets feedback even between cardinal
    /// transitions.  Computed off the coverage bin count, not the mask,
    /// so this also works on old firmware.
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

            currentTargetCard

            orientationGrid

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

                // Total-coverage gauge.  Circular feels more "spherical"
                // than a linear bar — the cal is literally about lighting
                // up wedges of a sphere.  Counts all 26 wedges, including
                // the diagonals; the six-axis grid above shows only the
                // cardinals.
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
        .onChange(of: status.coverageMask) { _ in
            guard hasLiveMask else { return }
            // Detect the "previous target just got captured" transition:
            // the axis we were prompting for now has its wedge bit set.
            // Flash it green for a beat before the prompt advances.
            if let prev = lastSeenTarget,
               prev != nextUncoveredAxis,
               status.isAxisCovered(prev),
               heldCoveredAxis == nil {
                heldCoveredAxis = prev
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.6) {
                    heldCoveredAxis = nil
                }
            }
            lastSeenTarget = nextUncoveredAxis
        }
        .onAppear {
            lastSeenTarget = nextUncoveredAxis
            if !hasLiveMask { startFallbackCycle() }
        }
    }

    // MARK: - Pieces

    /// Display priority for the prompt card: held-just-captured → next
    /// uncovered → cycle fallback (old firmware).  `covered` is true
    /// only when we're displaying a just-captured axis.
    private var promptDisplay: (axis: MagCalAxis?, covered: Bool) {
        if let held = heldCoveredAxis {
            return (held, true)
        }
        if hasLiveMask {
            return (nextUncoveredAxis, false)
        }
        return (SamplingHero.axisOrder[fallbackIndex % SamplingHero.axisOrder.count], false)
    }

    /// The big "do this now" prompt card.  Stays on the same axis until
    /// the firmware reports its wedge is populated, then flashes a green
    /// "Captured!" for ~0.6 s before moving to the next.
    @ViewBuilder
    private var currentTargetCard: some View {
        let display = promptDisplay
        let axisToShow = display.axis
        let covered = display.covered

        HStack(spacing: 12) {
            if let axis = axisToShow {
                Image(systemName: covered ? "checkmark.circle.fill" : axis.icon)
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(covered ? .green : .orange)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 2) {
                    Text(covered ? "Captured!" : axis.prompt)
                        .font(.headline)
                    Text(covered ? "Moving to next orientation…" : "Hold this position")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Spacer()
            } else {
                // All six cardinal axes done — the FC fit-gate is on 18
                // total wedges, so coverage may still be growing through
                // the diagonals; keep encouraging the user.
                Image(systemName: "checkmark.seal.fill")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(.green)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 2) {
                    Text("All six axes captured")
                        .font(.headline)
                    Text("Keep rolling for full coverage…")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                Spacer()
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 14)
        .background((covered ? Color.green : Color.orange).opacity(0.12))
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 8)
        .animation(.easeInOut(duration: 0.25), value: covered)
    }

    /// 6-cell grid showing which cardinal axes are captured.  Replaces
    /// the spinning globe — gives the user an at-a-glance map of what
    /// orientations remain.
    private var orientationGrid: some View {
        let cols = [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())]
        return LazyVGrid(columns: cols, spacing: 8) {
            ForEach(SamplingHero.axisOrder, id: \.self) { axis in
                let covered = hasLiveMask && status.isAxisCovered(axis)
                VStack(spacing: 4) {
                    Image(systemName: covered ? "checkmark.circle.fill" : axis.icon)
                        .font(.system(size: 22, weight: .semibold))
                        .foregroundColor(covered ? .green : .gray)
                    Text(axis.shortLabel)
                        .font(.caption2)
                        .foregroundColor(covered ? .primary : .secondary)
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 8)
                .background(covered
                    ? Color.green.opacity(0.12)
                    : Color.gray.opacity(0.08))
                .clipShape(RoundedRectangle(cornerRadius: 8))
            }
        }
        .padding(.horizontal, 8)
    }

    // MARK: - Old-firmware fallback (timer cycle)

    private func startFallbackCycle() {
        Timer.scheduledTimer(withTimeInterval: 3.0, repeats: true) { _ in
            DispatchQueue.main.async {
                fallbackIndex = (fallbackIndex + 1) % SamplingHero.axisOrder.count
            }
        }
    }
}
