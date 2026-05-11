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
    /// All driven off the FC's 5 Hz status frame.
    private var samplingSection: some View {
        Group {
            if let s = status {
                Section {
                    SamplingHero(status: s)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                }
                Section(header: Text("Live mag vector"),
                        footer: Text("Use the bars to verify which axis is dominant. The current target's bar is highlighted — adjust the rocket until its bar reaches the target marker.")) {
                    DirectionBars(status: s)
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

/// Guidance card for the sampling phase.  Walks the user through each
/// cardinal orientation one at a time with a deterministic state
/// machine:
///   prompting(axis)  → user matches the axis; the live FC coverageMask
///                       lights up the corresponding wedge bit
///   holding(axis, t) → orientation matched; a visible countdown ticks
///                       down while the user holds it.  Tells the user
///                       "yes, this is working, just keep holding."
///   prompting(next)  → after countdown elapses, advance.
///   allDone          → all 6 cardinals walked; FC continues sampling
///                       through diagonals for the full 2048-sample
///                       window so coverage hits the 18-wedge gate.
///
/// State is local to the view.  Coverage state from the FC drives the
/// prompting→holding transition only; once we've finished a hold for
/// an axis, we advance regardless of whether its bit clears later
/// (which won't happen — bits are sticky on the FC side).
///
/// Old firmware (22-byte payload, coverageMask=0) leaves the state
/// machine stuck in prompting forever, so we additionally drive a
/// slow time-based cycle in that case as a degraded fallback.
private struct SamplingHero: View {
    let status: MagCalStatus

    /// All six cardinal axes in the order the user is walked through them.
    /// Order = nose along the body, then the four sides.  Fixed across
    /// runs so users build muscle memory.
    private static let axisOrder: [MagCalAxis] = [
        .noseUp, .noseDown, .rightSide, .leftSide, .frontFace, .backFace
    ]

    /// Hold duration before advancing — long enough to give the user
    /// satisfying visual confirmation, short enough that 6 × HOLD plus
    /// transition time still fits in the FC's 20 s sample window.
    private static let HOLD_SECONDS: Double = 1.5

    /// State-machine tick — fast enough for a smooth countdown bar.
    private static let TICK_SECONDS: Double = 0.05

    /// Index into axisOrder of the orientation we're currently prompting
    /// for.  Moves forward only — once an axis is held to completion we
    /// don't revisit it (even if its wedge bit somehow clears).
    @State private var currentAxisIndex: Int = 0

    /// Seconds remaining in the current hold.  When this reaches zero
    /// we advance currentAxisIndex.  Zero = not currently holding.
    @State private var holdRemaining: Double = 0

    /// Old-firmware fallback timer — only kicks in when coverageMask is
    /// always zero.  Cycles the prompt every 3 s as a degraded UX.
    @State private var fallbackTimer: Timer?

    /// State-machine driver — fires every TICK_SECONDS while the view
    /// is visible.  Started in onAppear, torn down in onDisappear.
    @State private var tickTimer: Timer?

    /// True any time the FC's wire frame can carry per-wedge info.
    /// Zero-mask + zero-bins (no samples yet) still counts as "live" so
    /// a fresh entry isn't misread as old firmware.
    private var hasLiveMask: Bool { status.coverageMask != 0 || status.coverageBins == 0 }

    /// The axis the user should be aiming for right now (nil = all 6 done).
    private var currentAxis: MagCalAxis? {
        currentAxisIndex < SamplingHero.axisOrder.count
            ? SamplingHero.axisOrder[currentAxisIndex]
            : nil
    }

    /// True if the user is in the right orientation for the current
    /// target — the corresponding wedge bit is set on the FC side.
    private var currentAxisMatched: Bool {
        guard hasLiveMask, let axis = currentAxis else { return false }
        return status.isAxisCovered(axis)
    }

    private var isHolding: Bool { holdRemaining > 0 }

    /// Adaptive headline.  Drives off the local prompt index so it
    /// updates in lock-step with what the user actually sees, rather
    /// than off the FC's coverage count which can include diagonals.
    private var headline: String {
        if currentAxis == nil { return "All orientations captured" }
        switch currentAxisIndex {
        case 0:     return "Let's start"
        case 1, 2:  return "Nice — keep going"
        case 3, 4:  return "Almost there"
        default:    return "Last one"
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
                // Live |B|.  Big monospaced digits so the eye sees motion
                // and trusts that sampling is live.
                VStack(spacing: 2) {
                    Text(String(format: "%.0f", status.instantaneousFieldUT))
                        .font(.system(size: 36, weight: .bold, design: .monospaced))
                    Text("µT field")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                // Total-coverage gauge.  Counts all 26 wedges (including
                // diagonals); the six-axis grid above shows only the
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
        .onAppear { startTimers() }
        .onDisappear { stopTimers() }
    }

    // MARK: - Pieces

    /// The big "do this now" prompt card.  Three visual states:
    ///   - Prompting: orange arrow icon + "Point the nose UP" + caption
    ///   - Holding:   green check + "Captured! Hold steady…" + countdown bar
    ///   - All done:  seal + "All orientations captured" message
    @ViewBuilder
    private var currentTargetCard: some View {
        if let axis = currentAxis {
            let holding = isHolding
            HStack(spacing: 12) {
                Image(systemName: holding ? "checkmark.circle.fill" : axis.icon)
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(holding ? .green : .orange)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 4) {
                    Text(holding ? "Captured — hold steady" : axis.prompt)
                        .font(.headline)
                    if holding {
                        // Countdown bar shrinks from full → empty over
                        // HOLD_SECONDS.  More tangible than a "1.4 s"
                        // number that flickers.
                        ProgressView(value: max(0, holdRemaining),
                                     total: SamplingHero.HOLD_SECONDS)
                            .progressViewStyle(.linear)
                            .tint(.green)
                        Text(String(format: "Advancing in %.1f s", holdRemaining))
                            .font(.caption)
                            .foregroundColor(.secondary)
                    } else {
                        Text("Hold this position until it turns green")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                Spacer()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 14)
            .background((holding ? Color.green : Color.orange).opacity(0.12))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 8)
            .animation(.easeInOut(duration: 0.25), value: holding)
        } else {
            HStack(spacing: 12) {
                Image(systemName: "checkmark.seal.fill")
                    .font(.system(size: 36, weight: .semibold))
                    .foregroundColor(.green)
                    .frame(width: 44)
                VStack(alignment: .leading, spacing: 2) {
                    Text("All orientations captured")
                        .font(.headline)
                    Text("Keep rolling slowly for a clean fit…")
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
        }
    }

    /// 6-cell grid showing per-axis progress.  Each cell turns green
    /// once we've held that axis to completion (currentAxisIndex moved
    /// past it).  The CURRENT target also gets a blue ring + pulse so
    /// it's obvious which one we're prompting for.
    private var orientationGrid: some View {
        let cols = [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())]
        return LazyVGrid(columns: cols, spacing: 8) {
            ForEach(Array(SamplingHero.axisOrder.enumerated()), id: \.offset) { (idx, axis) in
                let done = idx < currentAxisIndex
                let isCurrent = idx == currentAxisIndex
                VStack(spacing: 4) {
                    Image(systemName: done ? "checkmark.circle.fill" : axis.icon)
                        .font(.system(size: 22, weight: .semibold))
                        .foregroundColor(done ? .green : (isCurrent ? .blue : .gray))
                    Text(axis.shortLabel)
                        .font(.caption2)
                        .foregroundColor(done || isCurrent ? .primary : .secondary)
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 8)
                .background(done
                    ? Color.green.opacity(0.12)
                    : (isCurrent ? Color.blue.opacity(0.12) : Color.gray.opacity(0.08)))
                .clipShape(RoundedRectangle(cornerRadius: 8))
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(isCurrent ? Color.blue : Color.clear, lineWidth: 2)
                )
            }
        }
        .padding(.horizontal, 8)
    }

    // MARK: - Timers / state-machine

    private func startTimers() {
        // Main state-machine tick.
        tickTimer?.invalidate()
        tickTimer = Timer.scheduledTimer(withTimeInterval: SamplingHero.TICK_SECONDS,
                                          repeats: true) { _ in
            DispatchQueue.main.async { advance() }
        }
        // Old-firmware fallback: if we never get a real mask, walk the
        // axis cycle on a slow timer so the user still gets prompts.
        // The FC fit will still run, just without per-orientation
        // confirmation.
        fallbackTimer?.invalidate()
        fallbackTimer = Timer.scheduledTimer(withTimeInterval: 3.0,
                                              repeats: true) { _ in
            DispatchQueue.main.async {
                if !hasLiveMask && !isHolding && currentAxisIndex < SamplingHero.axisOrder.count {
                    // No live coverage info — simulate a hold completing.
                    holdRemaining = 0
                    currentAxisIndex += 1
                }
            }
        }
    }

    private func stopTimers() {
        tickTimer?.invalidate(); tickTimer = nil
        fallbackTimer?.invalidate(); fallbackTimer = nil
    }

    /// One tick of the state machine.  Either tick down a running hold,
    /// or start one if the current target's wedge bit is now set.
    private func advance() {
        guard currentAxis != nil else { return }
        if isHolding {
            holdRemaining = max(0, holdRemaining - SamplingHero.TICK_SECONDS)
            if holdRemaining == 0 {
                // Hold done — advance past the current axis.
                currentAxisIndex += 1
            }
        } else if currentAxisMatched {
            // Orientation just matched — kick off the visible hold.
            holdRemaining = SamplingHero.HOLD_SECONDS
        }
    }
}

// MARK: - Direction Bars

/// Live mag-vector display: three signed horizontal bars for X, Y, Z
/// (µT, post-IIS2MDC-OFFSET-subtract).  Centred at zero with a ±100 µT
/// full-scale so an Earth field (~50 µT) reaches roughly half-bar.  The
/// currently dominant component (largest |value|, above a small dead
/// zone) is highlighted in blue so the user can see at a glance which
/// axis their orientation is producing — the key feedback that was
/// missing when the user couldn't tell whether nose-up was actually
/// being detected.
///
/// Falls back to a "waiting for live data" placeholder when running
/// against firmware older than the 32-byte payload (all three components
/// zero).
private struct DirectionBars: View {
    let status: MagCalStatus

    /// Bar full-scale.  Fixed (not auto-scaling) so the bars don't
    /// jitter as the user tumbles.  Earth fields land ~50 µT; the
    /// uncalibrated PCB residual seen on this board is ~1640 µT so we
    /// clamp wider — values past full-scale stay pinned at the end.
    private static let RANGE_UT: Float = 100.0

    /// Components below this (in µT abs) don't count as "dominant" —
    /// avoids flickering the highlight between near-zero axes when the
    /// rocket is between orientations.
    private static let DEAD_ZONE_UT: Float = 5.0

    private var hasLiveVector: Bool {
        status.liveX_uT != 0 || status.liveY_uT != 0 || status.liveZ_uT != 0
    }

    /// Which axis is currently dominant (largest absolute value), or nil
    /// if all three are below the dead zone.
    private var dominantAxis: MagCalComponent? {
        let xAbs = abs(status.liveX_uT)
        let yAbs = abs(status.liveY_uT)
        let zAbs = abs(status.liveZ_uT)
        let peak = max(xAbs, max(yAbs, zAbs))
        if peak < DirectionBars.DEAD_ZONE_UT { return nil }
        if peak == xAbs { return .x }
        if peak == yAbs { return .y }
        return .z
    }

    var body: some View {
        if hasLiveVector {
            VStack(spacing: 10) {
                ForEach(MagCalComponent.allCases, id: \.rawValue) { comp in
                    ComponentBar(label: comp.rawValue,
                                 value: comp.value(in: status),
                                 range: DirectionBars.RANGE_UT,
                                 isDominant: comp == dominantAxis)
                }
            }
        } else {
            Text("Waiting for live mag data… (re-flash firmware if this persists)")
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

            Text(String(format: "%+.0f", value))
                .font(.system(size: 14, weight: .semibold, design: .monospaced))
                .frame(width: 56, alignment: .trailing)
                .foregroundColor(isDominant ? .blue : .secondary)
        }
    }
}
