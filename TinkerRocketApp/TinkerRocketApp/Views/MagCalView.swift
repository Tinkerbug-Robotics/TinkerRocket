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
import Combine

struct MagCalView: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var store: RocketProfileStore
    @Environment(\.dismiss) var dismiss

    /// Latest status frame, or nil if the FC hasn't published one yet
    /// (e.g. just navigated in, before tapping Start).  In that case the
    /// view shows the intro/Start state.
    private var status: MagCalStatus? { device.magCalStatus }

    /// Accel-driven orientation coverage tracker.  Owned at the view
    /// level so both the SamplingHero gauge and the Progress section
    /// read the same value.  Reset whenever a fresh sampling run
    /// starts (sample_count transitions to 0).
    @StateObject private var accelCoverage = AccelCoverageTracker()

    /// #148 verify-window |B| accumulators.  iOS tracks these locally
    /// from the live status frames during VERIFYING so we can show
    /// min/max/spread and a real-time pass/fail readout, all derived
    /// from the same instantaneousFieldUT the firmware uses for its
    /// own gates.  Reset on entry to .verifying and on Retry
    /// verification.  nil means "no samples yet."
    @State private var verifyMinUT: Float? = nil
    @State private var verifyMaxUT: Float? = nil


    var body: some View {
        Form {
            // The view is mostly status-driven: intro/start, sampling,
            // review (accept/retry), applied success, aborted return.
            switch status?.subType ?? .idle {
            case .idle, .aborted:    introSection
            case .sampling:           samplingSection
            case .review:             reviewSection
            case .applied:            appliedSection
            // #206: between Accept and final NVS persist.  FC is sampling
            // the corrected stream to confirm |B| stays in band.
            case .verifying:          verifyingSection
            }
        }
        .navigationTitle("Mag Calibration")
        .navigationBarTitleDisplayMode(.inline)
        // Snapshot a freshly-accepted cal into the active rocket profile (#132),
        // tagged with this board's id so the syncer can re-apply it on connect
        // and warn if a different board is later used.
        .onChange(of: device.magCalStatus) { newStatus in
            // #148 verify-window |B| accumulators.  Update on every
            // status frame received while VERIFYING; reset when we
            // leave VERIFYING so the next entry starts fresh.
            if let s = newStatus, s.subType == .verifying {
                let b = s.instantaneousFieldUT
                if b > 0 {  // FC sends 0 before the first verify sample
                    verifyMinUT = min(verifyMinUT ?? b, b)
                    verifyMaxUT = max(verifyMaxUT ?? b, b)
                }
            } else {
                verifyMinUT = nil
                verifyMaxUT = nil
            }
            // (#132) snapshot a freshly-accepted cal into the active
            // rocket profile, tagged with this board's id so the
            // syncer can re-apply it on connect and warn if a
            // different board is later used.
            guard let s = newStatus, s.subType == .applied,
                  !device.unitID.isEmpty, let id = store.activeId else { return }
            store.update(id) { $0.magCal = MagCalData(status: s, unitID: device.unitID) }
        }
        .toolbar {
            ToolbarItem(placement: .navigationBarLeading) {
                if status?.subType == .sampling || status?.subType == .review ||
                   status?.subType == .verifying {
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

            Section(header: Text("How it works")) {
                VStack(alignment: .leading, spacing: 8) {
                    Label("Hold the rocket clear of laptops, phones, tools, and steel surfaces.", systemImage: "1.circle.fill")
                    Label("Tap Start — you'll see a sphere of red cells with a yellow puck. The puck shows which direction gravity is pulling through the rocket right now.", systemImage: "2.circle.fill")
                    Label("Tilt the rocket so the puck moves into each red cell. Hold each cell for a moment to fill it. Tap Compute Fit once you've cleared most of the sphere.", systemImage: "3.circle.fill")
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

            Section(footer: Text("For the calibration to match what the flight computer sees, ensure the rocket is in the flight configuration before calibration.")) {
                // Start button — greyed out when the rocket isn't in a
                // valid state (in-flight, or telemetry not connected /
                // powered).  No explanatory text below; the visual state
                // of the button itself is the signal.
                let canStart = isStartAllowed
                Button {
                    device.sendMagCalStart()
                } label: {
                    HStack {
                        Image(systemName: "play.fill")
                        Text("Start Calibration")
                            .fontWeight(.semibold)
                        Spacer()
                    }
                    .foregroundColor(canStart ? .white : Color(.systemGray2))
                }
                .listRowBackground(canStart ? Color.blue : Color(.systemGray5))
                .disabled(!canStart)
            }
        }
    }

    /// MAG_CAL_START is refused FC-side from INFLIGHT; we also need the
    /// rocket connected + powered for the cal to do anything useful.
    /// The button is enabled when all three are true.
    private var isStartAllowed: Bool {
        guard device.isConnected, !device.isBaseStation,
              device.telemetry.pwr_pin_on else { return false }
        // Telemetry state string mirrors the FC's rocket_state enum;
        // INFLIGHT is the one truly-forbidden case.
        return device.telemetry.state != "INFLIGHT"
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
                let ax = device.telemetry.low_g_x ?? 0
                let ay = device.telemetry.low_g_y ?? 0
                let az = device.telemetry.low_g_z ?? 0
                // #148: single 3D sphere visualization replaces the old
                // per-axis tap-grid + accel bars.  Capture is fully
                // automatic — the user just tumbles the rocket and the
                // cells fill in green from the firmware's coverage_mask.
                // liveAccel = nil when we have no telemetry yet so the
                // view skips the orientation update and stays at neutral.
                Section {
                    MagCalSphereView(
                        coverageMask: s.coverageMask,
                        partialMask: s.partialMask,
                        liveAccel: (device.telemetry.low_g_x != nil)
                            ? SIMD3<Float>(ax, ay, az) : nil
                    )
                    .frame(height: 360)
                    .listRowInsets(EdgeInsets())
                }
                Section {
                    HStack {
                        Text("Orientation coverage")
                        Spacer()
                        Text("\(s.coverageBins) / 32")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    HStack {
                        Text("Samples")
                        Spacer()
                        Text("\(s.sampleCount)")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                }
                // Compute Fit is user-driven — we DON'T auto-advance even
                // once coverage clears the threshold, so the user can
                // keep rotating to fill remaining cells.  The button
                // enables at the firmware-side coverage floor and turns
                // into the primary action; below it the user knows they
                // can keep going.
                let hasSamples = s.sampleCount >= MagCalConstants.minSamples
                let coverageMet = s.coverageBins >= MagCalConstants.minCoverageBins
                let canFit = hasSamples && coverageMet
                Section(footer: Text(canFit
                    ? "Ready to fit. Keep rotating to cover any remaining cells, or tap Compute Fit when you're satisfied."
                    : !hasSamples
                        ? "Collecting samples — keep rotating. Need at least \(MagCalConstants.minSamples)."
                        : "Need at least \(MagCalConstants.minCoverageBins) of 32 cells covered before the fit can run.")) {
                    Button {
                        device.sendMagCalComputeFit()
                    } label: {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                            Text("Compute Fit")
                                .fontWeight(.semibold)
                            Spacer()
                        }
                        .foregroundColor(canFit ? .white : Color(.systemGray2))
                    }
                    .listRowBackground(canFit ? Color.blue : Color(.systemGray5))
                    .disabled(!canFit)
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
                        Text("\(s.coverageBins) / 32 wedges")
                            .foregroundColor(.secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    // Issue #207 / #148: |c| measures the full PCB
                    // hard-iron.  Almost any value < the chip's ±4915 µT
                    // OFFSET register range subtracts cleanly, so we only
                    // warn near that limit — at typical 1.7 mT this stays
                    // green.  The intro screen carries the "ensure flight
                    // configuration" tip that used to live here, since
                    // that advice applies regardless of |c|.
                    let chipLimitConcern = s.centerWarning != .ok
                    HStack {
                        Text("Offset |c|")
                            .foregroundColor(chipLimitConcern ? .orange : .primary)
                        Spacer()
                        Text(String(format: "%.1f µT", s.centerMagnitudeUT))
                            .foregroundColor(chipLimitConcern ? .orange : .secondary)
                            .font(.system(.body, design: .monospaced))
                    }
                    if chipLimitConcern {
                        Text("Bias is approaching the chip's ±4915 µT OFFSET-register range — the chip may not be able to fully subtract this. Try re-calibrating; if it persists, the board may have unusual magnetics.")
                            .font(.caption)
                            .foregroundColor(.secondary)
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
                    Section(footer: Text("Verify is the recommended path — it programs the offset into the chip and lets you confirm with a live |B| readout. Save and apply skips verification and writes the cal to flight-computer memory immediately.")) {
                        Button {
                            device.sendMagCalAccept()
                        } label: {
                            HStack {
                                Image(systemName: "checkmark.shield")
                                Text("Verify")
                                    .fontWeight(.semibold)
                                Spacer()
                            }
                            .foregroundColor(.white)
                        }
                        .listRowBackground(Color.blue)
                        Button {
                            device.sendMagCalForceApply()
                        } label: {
                            HStack {
                                Image(systemName: "tray.and.arrow.down.fill")
                                Text("Save and apply (skip verify)")
                                Spacer()
                            }
                        }
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

    /// #206 — between Accept and final NVS persist.  The FC has programmed
    /// the new offsets but is sampling for ~5 s to verify the corrected
    /// |B| stays in band.  Show live |B|, a tumbling-progress bar, and
    /// the "rotate slowly" prompt.  No buttons during this window — the
    /// FC drives the transition to APPLIED (success) or back to REVIEW
    /// (fail with VERIFY_FAILED reject code).
    private var verifyingSection: some View {
        Group {
            if let s = status {
                // Pre-compute gate evaluation so the UI can colour rows
                // and the Done button consistently.
                let minB = verifyMinUT
                let maxB = verifyMaxUT
                let spread = (minB != nil && maxB != nil) ? (maxB! - minB!) : 0
                let inBand   = (minB ?? 0) >= MagCalConstants.verifyMinUT &&
                               (maxB ?? 0) <= MagCalConstants.verifyMaxUT
                let tightEnough = spread <= MagCalConstants.verifyRangeUT
                let coverageMet = s.coverageBins >= MagCalConstants.verifyMinCoverageBins
                let samplesMet  = s.sampleCount >= MagCalConstants.verifyMinSamples
                let allGood = (minB != nil) && inBand && tightEnough && coverageMet && samplesMet

                Section {
                    VStack(spacing: 14) {
                        Image(systemName: allGood ? "checkmark.shield.fill" : "checkmark.shield")
                            .font(.system(size: 40))
                            .foregroundColor(allGood ? .green : .blue)
                        Text("Verifying calibration")
                            .font(.headline)
                        Text("Slowly rotate the rocket through every orientation — nose up, nose down, sides, and corners. Watch the live |B| stay between 20 and 70 µT with a tight spread. Tap Done when you've covered enough orientations.")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 12)
                }
                Section(header: Text("Live |B|"),
                        footer: Text("Min and max are tracked from the moment Verify started (or the last Retry). The fit passes when all rows below are green.")) {
                    HStack {
                        Text("Current")
                        Spacer()
                        Text(String(format: "%.1f µT", s.instantaneousFieldUT))
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(.secondary)
                    }
                    HStack {
                        Text("Min observed")
                        Spacer()
                        Text(minB.map { String(format: "%.1f µT", $0) } ?? "—")
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(minB == nil ? .secondary :
                                ((minB ?? 0) >= MagCalConstants.verifyMinUT ? .green : .red))
                    }
                    HStack {
                        Text("Max observed")
                        Spacer()
                        Text(maxB.map { String(format: "%.1f µT", $0) } ?? "—")
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(maxB == nil ? .secondary :
                                ((maxB ?? 0) <= MagCalConstants.verifyMaxUT ? .green : .red))
                    }
                    HStack {
                        Text("Spread")
                        Spacer()
                        Text(minB == nil ? "—" : String(format: "%.1f µT", spread))
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(minB == nil ? .secondary :
                                (tightEnough ? .green : .red))
                    }
                }
                Section(header: Text("Verify progress")) {
                    HStack {
                        Text("Rotation coverage")
                        Spacer()
                        Text("\(s.coverageBins) / 32 wedges (need ≥ \(MagCalConstants.verifyMinCoverageBins))")
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(coverageMet ? .green : .secondary)
                    }
                    HStack {
                        Text("Samples")
                        Spacer()
                        Text("\(s.sampleCount) / \(MagCalConstants.verifyMinSamples)+ needed")
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(samplesMet ? .green : .secondary)
                    }
                }
                verifyButtonsSection(allGood: allGood)
                Section(header: Text("What this does")) {
                    Text("The fit's offset was just programmed into the magnetometer chip. The chip is now subtracting it from every raw sample, which should leave just Earth's magnetic field (~25–65 µT depending on latitude) regardless of which way the rocket is pointed.\n\nVerification confirms that's actually happening: as you rotate, the corrected |B| should stay roughly constant and inside Earth's band. If it wanders out of band or swings widely, the cal absorbed too much (or too little) and likely shouldn't be saved.\n\nAccept and Save commits the cal to flight-computer memory; Retry verification clears the min/max if you want to start fresh without redoing the whole tumble; Re-run calibration throws the proposed fit out and goes back to sampling; Abort restores the previously-saved cal.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .fixedSize(horizontal: false, vertical: true)
                }
            }
        }
    }

    /// Verifying-screen action buttons.  Extracted to keep the SwiftUI
    /// ViewBuilder in verifyingSection from blowing past the type-checker's
    /// expression complexity budget.
    @ViewBuilder
    private func verifyButtonsSection(allGood: Bool) -> some View {
        Section(footer: Text(allGood
            ? "All checks green — tap Accept and Save to commit the cal to flight-computer memory."
            : "Keep rotating until every row above is green. You can still Accept and Save if you want — the gate readouts are advisory, not blocking.")) {
            // Primary action — Accept and Save.  Always tappable; copy
            // and colour change with gate state.  iOS dispatches either
            // VERIFY_DONE (firmware re-checks gates, with the same data
            // so always passes when iOS sees green) or FORCE_APPLY
            // (firmware skips gate check).
            Button {
                if allGood {
                    device.sendMagCalVerifyDone()
                } else {
                    device.sendMagCalForceApply()
                }
            } label: {
                HStack {
                    Image(systemName: allGood ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                    Text(allGood ? "Accept and Save" : "Save anyway")
                        .fontWeight(.semibold)
                    Spacer()
                }
                .foregroundColor(.white)
            }
            .listRowBackground(allGood ? Color.green : Color.orange)

            // Clear iOS + FC verify accumulators; stay in VERIFYING so
            // the proposed cal stays programmed on the chip and the
            // user can rotate again from a clean slate.
            Button {
                verifyMinUT = nil
                verifyMaxUT = nil
                device.sendMagCalVerifyReset()
            } label: {
                HStack {
                    Image(systemName: "arrow.counterclockwise")
                    Text("Retry verification")
                    Spacer()
                }
            }
            // Throw away the proposed cal and go back to SAMPLING for
            // a fresh tumble — useful when verify exposes a clearly-bad
            // fit and the user wants to redo the whole thing.
            Button {
                verifyMinUT = nil
                verifyMaxUT = nil
                device.sendMagCalRetry()
            } label: {
                HStack {
                    Image(systemName: "arrow.uturn.backward.circle")
                    Text("Re-run calibration")
                    Spacer()
                }
            }
            Button(role: .destructive) {
                device.sendMagCalAbort()
                dismiss()
            } label: {
                HStack {
                    Image(systemName: "xmark.circle")
                    Text("Abort (restore prior cal)")
                    Spacer()
                }
            }
        }
    }

    /// One-shot success: FC just persisted + applied the offset.  No
    /// inline action button — the toolbar Done is the only exit, per
    /// user feedback ("leave it to the user to select the done option
    /// on the top").  Banner stays visible until the user dismisses.
    private var appliedSection: some View {
        Group {
            if let s = status {
                Section {
                    VStack(spacing: 12) {
                        Image(systemName: "checkmark.seal.fill")
                            .font(.system(size: 48))
                            .foregroundColor(.green)
                        Text("Saved")
                            .font(.title2)
                            .fontWeight(.semibold)
                        Text(String(format: "Calibration written to flight-computer memory.\nEarth field locked at %.1f µT, residual %.1f µT.",
                                    s.fieldR_uT, s.residualUT))
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
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

    /// Accel-driven orientation coverage tracker, supplied by the parent
    /// view so the gauge here and the row in the Progress section read
    /// the same value.
    @ObservedObject var accelCoverage: AccelCoverageTracker

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
                // Accel-based orientation coverage gauge.  Counts how
                // many of 26 wedges of the gravity-direction sphere the
                // user has rotated through.  Drives the meaningful
                // "have I rotated through enough orientations?" signal
                // during sampling, in place of the mag-side coverage
                // that's biased and useless until the fit runs.
                ZStack {
                    Circle()
                        .stroke(Color.gray.opacity(0.20), lineWidth: 10)
                    Circle()
                        .trim(from: 0, to: CGFloat(min(Double(accelCoverage.coverageCount) / 26.0, 1.0)))
                        .stroke(Color.green,
                                style: StrokeStyle(lineWidth: 10, lineCap: .round))
                        .rotationEffect(.degrees(-90))
                        .animation(.easeInOut(duration: 0.4), value: accelCoverage.coverageCount)
                    VStack(spacing: 0) {
                        Text("\(accelCoverage.coverageCount)")
                            .font(.system(size: 20, weight: .bold, design: .monospaced))
                        Text("/ 26")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
                .frame(width: 76, height: 76)
            }
        }
        // Reset accel coverage whenever the user starts a fresh
        // sampling run (Start / Retry both increment sample_count from
        // 0, so a drop to zero is the signal).
        .onChange(of: status.sampleCount) { newValue in
            if newValue == 0 {
                accelCoverage.reset()
                capturedAxes.removeAll()
            }
        }
        // Feed live accel to the tracker on each new reading.  Watching
        // all three components catches every refresh — the regular
        // telemetry stream delivers low_g_x/y/z together, so this fires
        // at the telemetry rate (~10 Hz).
        .onChange(of: ax) { _ in accelCoverage.update(ax: ax, ay: ay, az: az) }
        .onChange(of: ay) { _ in accelCoverage.update(ax: ax, ay: ay, az: az) }
        .onChange(of: az) { _ in accelCoverage.update(ax: ax, ay: ay, az: az) }
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

// MARK: - Accel coverage tracker

/// Live orientation-coverage counter for the mag-cal sampling phase.
/// Bins the gravity-direction unit vector into the same 3³ = 27-cell
/// grid the firmware uses for its post-fit mag coverage (the (0,0,0)
/// centre cell is unreachable for a unit vector → 26 reachable wedges).
/// Each .update call OR's the current wedge bit into the mask; popcount
/// is the count displayed in the UI.
///
/// Kept as an ObservableObject because @State + struct-view + Timer
/// combinations have repeatedly bitten us on this screen (stale-self
/// captures, missed updates).  Reference-type state behaves predictably
/// across SwiftUI re-renders.
@MainActor
final class AccelCoverageTracker: ObservableObject {
    @Published private(set) var coverageMask: UInt32 = 0

    /// How many wedges have been visited (0..26).
    var coverageCount: Int { coverageMask.nonzeroBitCount }

    /// Components below this magnitude (m/s²) don't bin — avoids
    /// counting a free-fall blip as a unique orientation.
    private static let MIN_MAGNITUDE_MS2: Float = 5.0

    /// Same threshold the firmware's directionWedge uses (0.4 of the
    /// unit vector).  Keeps the wedge encoding bit-identical so the
    /// post-fit FC coverage and this live counter use the same scheme.
    private static let T: Float = 0.4

    func update(ax: Float?, ay: Float?, az: Float?) {
        guard let x = ax, let y = ay, let z = az else { return }
        let mag = sqrtf(x * x + y * y + z * z)
        guard mag >= AccelCoverageTracker.MIN_MAGNITUDE_MS2 else { return }
        let ux = x / mag
        let uy = y / mag
        let uz = z / mag

        func bin(_ v: Float) -> Int {
            return v < -AccelCoverageTracker.T ? 0
                 : v >  AccelCoverageTracker.T ? 2
                 : 1
        }
        let wedge = bin(ux) * 9 + bin(uy) * 3 + bin(uz)
        // wedge ∈ [0, 26]; 13 = (1,1,1) is the centre cell and never
        // sets for a unit vector, but we don't special-case it here —
        // it just stays at 0 in the mask.
        if wedge >= 0 && wedge < 27 {
            coverageMask |= (UInt32(1) << wedge)
        }
    }

    func reset() {
        coverageMask = 0
    }
}
