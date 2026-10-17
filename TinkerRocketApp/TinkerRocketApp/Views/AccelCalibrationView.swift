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

import SwiftUI

struct AccelCalibrationView: View {
    @ObservedObject var device: BLEDevice

    @State private var flow = AccelCalFlow()
    /// When the open face window's MEASURING frame came: drives the 3 s bar.
    @State private var windowOpenedAt: Date?
    @State private var confirmClear = false

    private typealias Face = AccelSixFace.Face

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

    var body: some View {
        Form {
            whySection
            storedSection
            facesSection
            if let solution = flow.captures.solution { resultSection(solution) }
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
        .confirmationDialog("Clear the stored offset?", isPresented: $confirmClear,
                            titleVisibility: .visible) {
            Button("Clear Offset", role: .destructive) { run(flow.clear()) }
            Button("Cancel", role: .cancel) { }
        } message: {
            Text("The flight computer goes back to the accelerometer's reading as it is. You can measure and save again at any time.")
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
        Section("Six Faces") {
            Text("Board axes: +X points forward (to the nose when the board is mounted nose-forward), +Y to the left, +Z up out of the component side. Set the board, or the whole rocket, on each face and hold it still for 3 s. Any order: the app tells the face from the reading.")
                .font(.caption).foregroundColor(.secondary)
            ForEach(Face.allCases) { face in faceRow(face) }
            measureRow
            if let report = flow.lastFace, !flow.isMeasuring { faceReportRow(report) }
            if !flow.captures.faces.isEmpty {
                Button("Start Over") { flow.startOver() }
                    .font(.callout)
                    .disabled(flow.isBusy)
            }
        }
    }

    private func faceRow(_ face: Face) -> some View {
        let capture = flow.captures.faces[face]
        return HStack(spacing: 12) {
            Image(systemName: capture == nil ? "circle" : "checkmark.circle.fill")
                .foregroundColor(capture == nil ? .secondary : .green)
            VStack(alignment: .leading, spacing: 2) {
                Text(face.name)
                Text(face.howTo).font(.caption).foregroundColor(.secondary)
            }
            Spacer()
            if let c = capture {
                Text(String(format: "%.1f° off", c.tiltDeg))
                    .font(.caption).monospacedDigit().foregroundColor(.secondary)
            }
        }
    }

    @ViewBuilder private var measureRow: some View {
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
            Button(action: { run(flow.measure()) }) {
                Label(flow.captures.isComplete ? "Measure a Face Again" : "Measure This Face",
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
            residualRow(s)
            if let text = s.tooLargeText {
                Label(text, systemImage: "xmark.octagon.fill").foregroundColor(.red)
            }
            if let text = s.residualNoteText {
                Label(text, systemImage: "info.circle")
                    .font(.caption).foregroundColor(.orange)
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

    /// Each face after the correction, off one g, in two columns: up, down.
    private func residualRow(_ s: AccelSixFace.Solution) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Each face after the correction, off one g")
                .font(.caption).foregroundColor(.secondary)
            Grid(alignment: .leading, horizontalSpacing: 10, verticalSpacing: 4) {
                ForEach(0..<3, id: \.self) { axis in
                    GridRow {
                        ForEach([true, false], id: \.self) { up in
                            let face = Face(axis: axis, up: up)
                            Text(face.name).font(.caption)
                            Text(AccelCalStatus.mg(s.residual[face] ?? 0))
                                .font(.caption).monospacedDigit()
                                .gridColumnAlignment(.trailing)
                        }
                    }
                }
            }
        }
    }

    private var clearSection: some View {
        Section {
            if flow.writing == .clear {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Clearing…")
                }
            } else {
                Button("Clear the Stored Offset…", role: .destructive) { confirmClear = true }
                    .disabled(!canRun || flow.isBusy || status?.isStored != true)
            }
            if let w = flow.lastWrite, w.write == .clear { writeReportRows(w) }
        } footer: {
            Text("Removes the offset from the flight computer, which then uses the accelerometer's reading as it is.")
        }
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
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + timeout) {
            run(flow.timedOut(gen))
        }
    }
}
