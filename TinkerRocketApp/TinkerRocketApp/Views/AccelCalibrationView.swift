//
//  AccelCalibrationView.swift
//  TinkerRocketApp
//
//  The low-g accelerometer's six-face calibration (#1629), opened from
//  Settings → General → Calibration. On a still pad the flight computer finds
//  level from gravity, and a small accelerometer offset looks just like a
//  small tilt; with GNSS correcting the attitude in flight, that false tilt
//  becomes a heading error in the boost. Turning each board axis straight up
//  and then straight down separates the two.
//
//  The flow is tools/accel_cal_six_face.py's, kept in AccelCalFlow: read the
//  stored offset (77), measure the faces in any order (78: a 3 s window each,
//  the face told from the reading), solve, save (76) and read it back (77).
//  This view sends what the flow asks for, arms its timeouts and hands it the
//  0xD1 frames. The offset is a board property that only this screen writes:
//  it is in no rocket profile and no connect-time sync.
//
//  The same windows carry the high-g's reading (a flight and out computer
//  that send it): the screen fits it to the low-g (AccelSixFace.solveHighG),
//  shows its scale, offset, cross-axis, fit and its own gravity fit's
//  agreement, and saves it after the offset (80, read back with 77). The
//  pad calibration then keeps the alignment and re-measures only the
//  high-g's bias; a rocket profile holding this board's sensor cal gets the
//  new bias, so its connect-time push does not put the old one back.
//
//  While the board turns, a live line names the face it is on now (from
//  telemetry's lx/ly/lz turned back through the mounting, BLEDevice
//  .accelLiveFace), tints that row and names it on the Measure button. It is
//  only a hint: the measurement names its face from its own reading. Tapping
//  a row shows how to set the board on that face.
//

import SwiftUI

struct AccelCalibrationView: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var store: RocketProfileStore

    @State private var flow = AccelCalFlow()
    /// When the open face window's MEASURING frame came: drives the 3 s bar.
    @State private var windowOpenedAt: Date?
    @State private var confirmClear = false

    private typealias Face = AccelSixFace.Face

    /// `initialFlow`: where the screen starts (a fresh flow; the tests' screen
    /// renders start from captured faces).
    init(device: BLEDevice, initialFlow: AccelCalFlow = AccelCalFlow()) {
        _device = ObservedObject(wrappedValue: device)
        _flow = State(initialValue: initialFlow)
    }

    private var status: AccelCalStatus? { device.accelCalStatus }

    /// Why the flight computer cannot be calibrated now, if it cannot. Like
    /// the pad calibration it needs the rocket itself, connected and powered;
    /// and its flight computer running and on the ground: it refuses these
    /// commands in flight, during a magnetometer calibration and after a
    /// flight (the post-flight lockout reads LANDED until it restarts), and
    /// measures no face during a sim.
    private var blockedReason: String? {
        if device.isBaseStation { return "Connect to the rocket itself to calibrate it." }
        guard device.isConnected, device.telemetry.pwr_pin_on else {
            return "Connect to the rocket and power it on to calibrate it."
        }
        if device.telemetry.state == "INITIALIZATION" || device.telemetry.fc_boot_step != nil {
            return "Wait for the flight computer to finish starting."
        }
        switch device.telemetry.state {
        case "INFLIGHT":
            return "Not while the rocket is flying."
        case "LANDED":
            return "After a flight the flight computer takes no calibration until it restarts: power the rocket off and on."
        case "MAG_CAL":
            return "Not while a magnetometer calibration is running."
        default:
            break
        }
        if device.simLaunched { return "Not while a simulation is running." }
        return nil
    }

    private var canRun: Bool { blockedReason == nil }

    /// A high-g alignment is stored on the board.
    private var highGStored: Bool { status?.highG?.isStored == true }

    var body: some View {
        Form {
            whySection
            storedSection
            facesSection
            if let solution = flow.captures.solution { resultSection(solution) }
            if let highG = flow.captures.highG { highGSection(highG) }
            clearSection
        }
        .navigationTitle("Accelerometer Calibration")
        .navigationBarTitleDisplayMode(.inline)
        .onAppear {
            // The faces take a minute or two of handling the rocket: keep the
            // screen awake, as the pad calibration does. Connecting already
            // did and the last disconnect re-arms auto-lock.
            if device.isConnected { UIApplication.shared.isIdleTimerDisabled = true }
            if canRun { run(flow.read()) }
        }
        // The rocket came up with the screen open: read what it has stored.
        .onChange(of: canRun) { ok in
            if ok, status == nil { run(flow.read()) }
        }
        .onReceive(device.accelCalFrames) { frame in
            let wasOpen = flow.windowOpen
            let next = flow.take(frame)
            if flow.windowOpen, !wasOpen { windowOpenedAt = Date() }
            run(next)
        }
        // A high-g save's report, whether its read-back came or timed out.
        .onChange(of: flow.lastHighGWrite) { w in
            if let w { highGWritten(w) }
        }
        .confirmationDialog(highGStored ? "Clear the stored calibration?" : "Clear the stored offset?",
                            isPresented: $confirmClear, titleVisibility: .visible) {
            Button(highGStored ? "Clear Calibration" : "Clear Offset", role: .destructive) {
                run(flow.clear(highG: highGStored))
            }
            Button("Cancel", role: .cancel) { }
        } message: {
            Text(highGStored
                 ? "The flight computer goes back to the low-g reading as it is and to the high-g without its alignment (it keeps the high-g's bias until the next pad calibration). You can measure and save again at any time."
                 : "The flight computer goes back to the accelerometer's reading as it is. You can measure and save again at any time.")
        }
    }

    // MARK: - Sections

    private var whySection: some View {
        Section {
            VStack(alignment: .leading, spacing: 8) {
                Text("Six-Face Accelerometer Calibration")
                    .font(.headline)
                Text("On the pad the flight computer finds level from gravity, and a small accelerometer offset looks just like a small tilt. In flight, with GNSS correcting the attitude, that false tilt becomes a heading error in the boost.")
                    .font(.subheadline).foregroundColor(.secondary)
                Text("Turning each axis straight up and then straight down separates the two: gravity flips sign, the offset does not. Do it once per flight computer; the offset is stored on the board.")
                    .font(.subheadline).foregroundColor(.secondary)
            }
            .padding(.vertical, 4)
        }
    }

    private var storedSection: some View {
        Section("Stored Offset") {
            if let s = status {
                if s.isStored {
                    ForEach(0..<3, id: \.self) { i in
                        LabeledContent(AccelSixFace.axisNames[i],
                                       value: AccelCalStatus.mg(Double(s.offset[i])))
                            .monospacedDigit()
                    }
                }
                Label(s.useText, systemImage: s.isStored ? "checkmark.seal.fill" : "circle.dashed")
                    .foregroundColor(.secondary)
                if let h = s.highG {
                    Label(h.alignmentText, systemImage: h.isStored ? "checkmark.seal.fill" : "circle.dashed")
                        .foregroundColor(.secondary)
                    if h.hasBias {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("High-g bias: \(h.biasText)").monospacedDigit()
                            Text(h.sourceText.prefix(1).uppercased() + h.sourceText.dropFirst())
                        }
                        .font(.caption).foregroundColor(.secondary)
                    }
                } else {
                    Label("This firmware sends no high-g calibration: update the flight computer and the out computer to calibrate the high-g.",
                          systemImage: "info.circle")
                        .font(.caption).foregroundColor(.secondary)
                }
            } else if flow.isReading {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Reading the flight computer…")
                }
            } else if flow.readAnswered == false {
                Label("No answer. This flight computer's firmware may not have the six-face calibration: update it, or read again.",
                      systemImage: "exclamationmark.triangle.fill")
                    .foregroundColor(.orange)
                Button("Read Again") { run(flow.read()) }
                    .disabled(!canRun || flow.isBusy)
            } else {
                Text("Not read yet").foregroundColor(.secondary)
            }
            if let reason = blockedReason {
                Label(reason, systemImage: "bolt.slash")
                    .font(.caption).foregroundColor(.orange)
            }
        }
    }

    private var facesSection: some View {
        // From the newest telemetry frame: a hint that follows the board as it
        // turns. The measurement still names its face from its own reading.
        let live = device.accelLiveFace
        return Section("Six Faces") {
            VStack(alignment: .leading, spacing: 6) {
                Text("Board axes: +X points forward (to the nose when the board is mounted nose-forward), +Y to the left, +Z up out of the component side. Set the board, or the whole rocket, on each face and hold it still for 3 s. Any order: the app tells the face from the reading.")
                Text("You don't pick the face: set the board on a side, and the app sees which one it is.")
            }
            .font(.caption).foregroundColor(.secondary)
            liveRow(live)
            ForEach(Face.allCases) { face in faceRow(face, isLive: live.face == face) }
            if let face = flow.placementFace {
                Label(face.placementText, systemImage: "info.circle")
                    .font(.callout).foregroundColor(.secondary)
            }
            measureRow(live)
            if let report = flow.lastFace, !flow.isMeasuring { faceReportRow(report) }
            if !flow.captures.faces.isEmpty {
                Button("Start Over") { flow.startOver() }
                    .font(.callout)
                    .disabled(flow.isBusy)
            }
        }
    }

    /// The live line at the top of the checklist.
    private func liveRow(_ live: AccelSixFace.LiveFace) -> some View {
        let icon: String
        let color: Color
        switch live {
        case .on:      icon = "cube.fill"; color = .accentColor
        case .askew:   icon = "exclamationmark.triangle.fill"; color = .orange
        case .moving:  icon = "hand.raised.fill"; color = .orange
        case .waiting: icon = "hourglass"; color = .secondary
        }
        return Label(live.text, systemImage: icon).foregroundColor(color)
    }

    /// A checklist row. Tapping it shows how to set the board on that face;
    /// the live face's row is tinted and tagged "Now".
    private func faceRow(_ face: Face, isLive: Bool) -> some View {
        let capture = flow.captures.faces[face]
        return Button(action: { flow.showPlacement(face) }) {
            HStack(spacing: 12) {
                Image(systemName: capture == nil ? "circle" : "checkmark.circle.fill")
                    .foregroundColor(capture == nil ? .secondary : .green)
                VStack(alignment: .leading, spacing: 2) {
                    Text(face.name).foregroundColor(.primary)
                    Text(face.howTo).font(.caption).foregroundColor(.secondary)
                }
                Spacer()
                if let c = capture {
                    Text(String(format: "%.1f° off", c.tiltDeg))
                        .font(.caption).monospacedDigit().foregroundColor(.secondary)
                }
                if isLive { nowTag }
            }
            .contentShape(Rectangle())
        }
        .listRowBackground(isLive ? Color.accentColor.opacity(0.12) : nil)
        .accessibilityHint("Shows how to place the board for this face")
    }

    private var nowTag: some View {
        Text("Now")
            .font(.caption.weight(.semibold))
            .foregroundColor(.white)
            .padding(.horizontal, 8)
            .padding(.vertical, 2)
            .background(Capsule().fill(Color.accentColor))
    }

    @ViewBuilder private func measureRow(_ live: AccelSixFace.LiveFace) -> some View {
        if flow.isMeasuring {
            if flow.windowOpen, let start = windowOpenedAt {
                TimelineView(.periodic(from: start, by: 0.1)) { context in
                    let elapsed = min(max(context.date.timeIntervalSince(start), 0), 3)
                    ProgressView(value: elapsed, total: 3) {
                        Text(elapsed < 3 ? "Hold still…" : "Finishing…")
                    }
                }
            } else {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Starting…")
                }
            }
        } else {
            // Named for the live face as a hint; enabled whatever the hint
            // says, since the measurement judges the face itself.
            Button(action: { run(flow.measure()) }) {
                Label(AccelSixFace.measureLabel(live: live, allCaptured: flow.captures.isComplete),
                      systemImage: "scope")
            }
            .disabled(!canRun || flow.isBusy)
        }
    }

    private func faceReportRow(_ report: AccelCalFlow.FaceReport) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Label(report.text, systemImage: report.isAccepted
                      ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                .foregroundColor(report.isAccepted ? .green : .orange)
            if let detail = report.detail {
                Text(detail).font(.caption).monospacedDigit().foregroundColor(.secondary)
            }
        }
    }

    private func resultSection(_ s: AccelSixFace.Solution) -> some View {
        Section {
            ForEach(0..<3, id: \.self) { i in
                HStack {
                    Text(AccelSixFace.axisNames[i])
                    Spacer()
                    VStack(alignment: .trailing, spacing: 2) {
                        Text(AccelCalStatus.mg(s.offset[i])).monospacedDigit()
                        Text("scale \(s.scaleText(i))")
                            .font(.caption).monospacedDigit().foregroundColor(.secondary)
                    }
                }
            }
            Text(AccelSixFace.Solution.solvedTogetherText)
                .font(.caption).foregroundColor(.secondary)
            if let text = s.cannotSaveText {
                Label(text, systemImage: "xmark.octagon.fill").foregroundColor(.red)
            }
            if case .save(_)? = flow.writing {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Saving…")
                }
            } else {
                Button(action: { run(flow.save()) }) {
                    Label("Save to the Flight Computer", systemImage: "square.and.arrow.down")
                }
                .disabled(!canRun || flow.isBusy || !s.canSave)
            }
            if let w = flow.lastWrite, w.write != .clear { writeReportRows(w) }
        } header: {
            Text("Offset")
        } footer: {
            Text("The scale is shown for information; only the offset is saved.")
        }
    }

    private var clearSection: some View {
        Section {
            if flow.writing == .clear || flow.writingHighG == .clear {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Clearing…")
                }
            } else {
                Button(highGStored ? "Clear the Stored Calibration…" : "Clear the Stored Offset…",
                       role: .destructive) { confirmClear = true }
                    .disabled(!canRun || flow.isBusy || (status?.isStored != true && !highGStored))
            }
            if let w = flow.lastWrite, w.write == .clear { writeReportRows(w) }
            if let w = flow.lastHighGWrite, w.write == .clear { highGWriteReportRows(w) }
        } footer: {
            Text(highGStored
                 ? "Removes the offset and the high-g alignment from the flight computer, which then uses the low-g reading as it is and the high-g with its bias alone."
                 : "Removes the offset from the flight computer, which then uses the accelerometer's reading as it is.")
        }
    }

    // MARK: - The high-g

    private func highGSection(_ result: AccelSixFace.HighGResult) -> some View {
        Section {
            switch result {
            case .solved(let h):
                ForEach(0..<3, id: \.self) { i in
                    HStack {
                        Text(AccelSixFace.axisNames[i])
                        Spacer()
                        VStack(alignment: .trailing, spacing: 2) {
                            Text("scale \(h.scaleText(i))").monospacedDigit()
                            Text("offset \(h.offsetText(i))")
                                .font(.caption).monospacedDigit().foregroundColor(.secondary)
                        }
                    }
                }
                VStack(alignment: .leading, spacing: 4) {
                    Text(h.crossText)
                    Text(h.residualText)
                }
                .font(.caption).monospacedDigit().foregroundColor(.secondary)
                Label(h.agreementText, systemImage: h.disagreeAxes.isEmpty ? "checkmark.circle" : "info.circle")
                    .font(.callout)
                    .foregroundColor(h.disagreeAxes.isEmpty ? .secondary : .orange)
                if let problem = h.problemText {
                    Label("Not saved: \(problem)", systemImage: "xmark.octagon.fill").foregroundColor(.red)
                } else {
                    Label("Passed: saved with the offset", systemImage: "checkmark.seal.fill")
                        .foregroundColor(.green)
                }
            case .noReading, .singular:
                Label(result.unavailableText ?? "", systemImage: "exclamationmark.triangle.fill")
                    .foregroundColor(.orange)
            }
            if case .save? = flow.writingHighG {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Saving the high-g…")
                }
            }
            if let w = flow.lastHighGWrite, w.write != .clear { highGWriteReportRows(w) }
        } header: {
            Text("High-g Accelerometer")
        } footer: {
            Text("The high-g is fitted to the low-g, the more accurate channel on the bench, so the flight computer reads the same acceleration when it switches to the high-g near 16 g. ± is one standard deviation from these faces. The pad calibration keeps this alignment and measures the high-g's offset again on the day.")
        }
    }

    @ViewBuilder private func highGWriteReportRows(_ w: AccelCalFlow.HighGWriteReport) -> some View {
        Label(w.text, systemImage: w.confirmed ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
            .foregroundColor(w.confirmed ? .green : .orange)
        if w.readBack == nil {
            Button("Read It Back") { run(flow.readBackAgain()) }
                .disabled(!canRun || flow.isBusy)
        }
    }

    /// A confirmed high-g save: the active profile's sensor cal (this
    /// board's) takes the new bias, so the connect-time push keeps it.
    private func highGWritten(_ w: AccelCalFlow.HighGWriteReport) {
        guard w.confirmed, case .save(_, let bias) = w.write, let id = store.activeId,
              let updated = ActiveRocketSyncer.sensorCalAfterHighGSave(
                profileCal: store.activeProfile?.sensorCal, deviceUnitID: device.unitID, bias: bias)
        else { return }
        store.update(id) { $0.sensorCal = updated }
    }

    /// A save's or a clear's outcome; with no answer, a way to read it back.
    @ViewBuilder private func writeReportRows(_ w: AccelCalFlow.WriteReport) -> some View {
        Label(w.text, systemImage: w.confirmed ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
            .foregroundColor(w.confirmed ? .green : .orange)
        if let back = w.readBack, !w.confirmed {
            Text("Read back: \(back.offsetsText)")
                .font(.caption).monospacedDigit().foregroundColor(.secondary)
        }
        if w.readBack == nil {
            Button("Read It Back") { run(flow.readBackAgain()) }
                .disabled(!canRun || flow.isBusy)
        }
    }

    // MARK: - Commands

    /// Send what the flow asked for and arm its timeout.
    private func run(_ command: AccelCalFlow.Command?) {
        guard let command else { return }
        // Taken before the send: an answer delivered during it moves the flow on.
        let gen = flow.generation
        let timeout = flow.timeout
        switch command {
        case .read:
            device.sendAccelCalRead()
        case .measureFace:
            windowOpenedAt = nil
            device.sendAccelFaceMeasure()
        case .apply(let o):
            device.sendAccelCalApply(x: o[0], y: o[1], z: o[2])
        case .applyHighG(let w):
            device.sendHighGCalApply(op: w.op, matrix: w.matrix, bias: w.bias)
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + timeout) {
            run(flow.timedOut(gen))
        }
    }
}
