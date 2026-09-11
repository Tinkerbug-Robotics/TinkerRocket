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

    /// #148 verify-window |B| accumulators.  iOS tracks these locally
    /// from the live status frames during VERIFYING so we can show
    /// min/max/spread and a real-time pass/fail readout, all derived
    /// from the same instantaneousFieldUT the firmware uses for its
    /// own gates.  Reset on entry to .verifying and on Retry
    /// verification.  nil means "no samples yet."
    @State private var verifyMinUT: Float? = nil
    @State private var verifyMaxUT: Float? = nil

    /// True once the user has actually run the cal flow during THIS view
    /// session (we've observed a live SAMPLING/REVIEW/VERIFYING frame).
    /// The FC rests in APPLIED whenever a calibration exists — it reports
    /// sub_type=APPLIED on every connect-time MAG_CAL_READ (see
    /// BLEDevice.sendMagCalRead / FC main.cpp "leave the calibrator in
    /// APPLIED").  So APPLIED on entry means "this board is already
    /// calibrated", NOT "you just finished" — without this flag the view
    /// opened straight into the terminal Saved banner and the user could
    /// never start a fresh run.  Reset per navigation (fresh @State).
    @State private var ranCalThisSession = false

    /// #1037: the last cal sub-state actually seen, and a once-guard for the
    /// teardown abort.
    ///
    /// iOS sent MAG_CAL_ABORT only from three deliberate taps and had no
    /// teardown hook. This view is a NavigationLink push inside the Settings
    /// sheet and nothing sets interactiveDismissDisabled, so a swipe-down on
    /// that sheet destroyed it without telling the rocket — leaving the FC in
    /// MAG_CALIBRATION with the IIS2MDC OFFSET registers ZEROED, launch detect
    /// inhibited, the EKF unable to initialise and pyro servicing dead until a
    /// reboot or until the operator reopened the screen and tapped Cancel.
    /// Android has aborted on every teardown since its first version
    /// (DisposableEffect onDispose + BackHandler).
    ///
    /// Gated on these remembered values rather than on live `status`, for the
    /// reason MagCalScreen.kt records: the live status comes from 5 Hz FC
    /// telemetry and can be nil or stale at exactly the moment teardown
    /// happens.
    @State private var lastSeenSubType: MagCalSubType? = nil
    @State private var teardownAbortSent = false


    var body: some View {
        Form {
            // The view is mostly status-driven: intro/start, sampling,
            // review (accept/retry), applied success, aborted return.
            switch status?.subType ?? .idle {
            case .idle, .aborted:    introSection
            case .sampling:           samplingSection
            case .review:             reviewSection
            // APPLIED just after a run this session → the one-shot Saved
            // banner.  APPLIED on entry (the FC's resting "already
            // calibrated" state, e.g. the connect-time read reply) → the
            // intro, so the user can start a fresh run instead of being
            // stuck on a Done-only screen.
            case .applied:            ranCalThisSession ? AnyView(appliedSection)
                                                        : AnyView(introSection)
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
            // Mark that a real run happened this session the moment we see
            // a live cal frame, so a later APPLIED shows the Saved banner
            // (not the intro).  A bare APPLIED on entry never sets this.
            if let st = newStatus?.subType, st == .sampling || st == .review || st == .verifying {
                ranCalThisSession = true
            }
            if let st = newStatus?.subType { lastSeenSubType = st }   // #1037
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
        .onDisappear {
            // #1037: the uncontrolled exits — a swipe-down on the hosting
            // Settings sheet above all — do not run the toolbar buttons. The
            // buttons stay the fast path; this is the net under them.
            //
            // VERIFYING is deliberately included even though it self-resolves
            // (the proposed offsets are already on the chip and the FC's 60 s
            // timer either commits or falls back to REVIEW): aborting there is
            // harmless and the flag cannot always tell the states apart at
            // teardown. SAMPLING and REVIEW are the ones with no auto-exit,
            // and they are where the rocket sits for the whole tumble.
            guard ranCalThisSession, !teardownAbortSent,
                  lastSeenSubType != .applied else { return }
            teardownAbortSent = true
            device.sendMagCalAbort()
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
                    Label("Tap Start — you'll see a translucent sphere of red cells with a red ball inside, like iOS's own compass calibration.  The ball shows the direction gravity is pulling through the rocket.", systemImage: "2.circle.fill")
                    Label("Tilt the rocket to roll the ball into each red cell. Hold each cell for a moment to fill it. Tap Compute Fit once you've cleared most of the sphere.", systemImage: "3.circle.fill")
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

            // Board already has a saved calibration (the FC rests in
            // APPLIED and reports it on connect).  Surface it so Start
            // reads as an intentional re-run, not a first-time cal.
            if let s = status, s.subType == .applied {
                Section {
                    Label(String(format: "Already calibrated — Earth field %.1f µT, residual %.1f µT.",
                                 s.fieldR_uT, s.residualUT),
                          systemImage: "checkmark.seal.fill")
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
                        Text(status?.subType == .applied ? "Recalibrate" : "Start Calibration")
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
                    Text("\(String(format: "%.4g", s.utPerLsb)) µT per LSB. The chip subtracts these from every sample once accepted.")
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
