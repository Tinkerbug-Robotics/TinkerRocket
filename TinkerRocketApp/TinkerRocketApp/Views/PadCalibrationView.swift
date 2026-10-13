//
//  PadCalibrationView.swift
//  TinkerRocketApp
//
//  Pad calibration (#1600 phase 5), opened from Settings → General →
//  Calibration; it replaces "Gyro & Accel Calibration". One button starts it:
//  command 75 with the broadcast ionosphere the phone holds. The flight
//  computer runs the gyro/accel window command 21 always ran, stores the
//  ionosphere for the raw-GNSS filter and restarts its GNSS pad survey.
//
//  The rocket reports in 0xD0 frames once a second on the ground, so this is a
//  live display, not a timer: determinate progress through the ~10 s window,
//  then its result; and on a board with the raw-GNSS filter, the survey —
//  satellites, ephemeris, corrections, ionosphere, pad average — which keeps
//  going until launch.
//
//  Older firmware ignores 75 and never sends 0xD0. BLEDevice.startPadCalibration
//  then sends 21 after 3 s, and this screen runs the flow it always ran for
//  21: a 12 s spinner and a gravity check from telemetry. Either way the 0xCB
//  readback at the window's close is snapshotted into the active profile
//  (#132), as it always was.
//

import SwiftUI

struct PadCalibrationView: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var store: RocketProfileStore
    @ObservedObject private var ionosphere = IonosphereStore.shared
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

    /// The phone's side of a start, until the rocket's frames take over.
    private enum Start: Equatable {
        case idle
        /// The held ionosphere is not today's: up to 5 s of a refresh first.
        case fetchingIonosphere
        /// 75 is out; waiting for the frame that shows the window open.
        case sent
        /// Older firmware: 21 went out instead and the old 12 s spinner runs.
        case legacy
    }
    @State private var start: Start = .idle
    /// Which start the timers belong to, so a stale one cannot end a newer one.
    @State private var startGeneration = 0
    /// Set when the user kicks off a cal so the next sensor-cal readback gets
    /// snapshotted into the active profile (#132) — not arbitrary frames.
    @State private var pendingCalSnapshot = false
    @State private var showGravityWarning = false
    @State private var gravityMag: Float = 0.0
    @State private var gravityError: Float = 0.0

    private var status: PadCalStatus? { device.padCalStatus }

    /// Calibrations run on the rocket: connected directly and powered on.
    private var canRun: Bool {
        device.isConnected && !device.isBaseStation && device.telemetry.pwr_pin_on
    }

    private var busy: Bool { start != .idle || status?.state == .running }

    var body: some View {
        Form {
            startSection
            if let s = status {
                gyroAccelSection(s)
                if s.runsRawGnss { gnssSection(s) }
            } else if start != .idle {
                Section("Gyro & Accel") {
                    HStack(spacing: 10) {
                        ProgressView()
                        Text(start == .legacy ? "Calibrating gyro & accel (about 10 s)…" : "Starting…")
                    }
                }
            }
            phoneSection
        }
        .navigationTitle("Pad Calibration")
        .navigationBarTitleDisplayMode(.inline)
        .onAppear {
            ionosphere.refreshIfStale()
            // The frames keep coming on the ground, so this screen is a live
            // display: keep it awake. Connecting already did (BLEDevice
            // .onConnect) and the last disconnect re-arms auto-lock; this
            // restates it for this screen and leaves the release to them.
            if device.isConnected { UIApplication.shared.isIdleTimerDisabled = true }
        }
        // The legacy flow's check (#1059); frames carry their own (below).
        .alert("Accelerometer Warning", isPresented: $showGravityWarning) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("Low-G accelerometer magnitude (\(UnitFormatter.acceleration(Double(gravityMag), system: unitSystem))) differs from expected gravity (\(UnitFormatter.acceleration(9.80665, system: unitSystem))) by \(String(format: "%.1f", gravityError))%. Consider running a bench calibration before flight.")
        }
        // Snapshot the freshly-run sensor cal into the active profile (#132),
        // tagged with this board's id so the syncer re-applies it on connect.
        // The 0xCB frame comes at the window's close for 75 and 21 alike.
        .onChange(of: device.sensorCalStatus) { newStatus in
            guard pendingCalSnapshot, let s = newStatus, s.valid,
                  !device.unitID.isEmpty, let id = store.activeId else { return }
            pendingCalSnapshot = false
            store.update(id) { $0.sensorCal = SensorCalData(status: s, unitID: device.unitID) }
        }
        .onChange(of: device.padCalStatus) { newStatus in
            guard let s = newStatus else { return }
            // Frames flowing drive the display: a legacy spinner started for
            // a rocket that was slow to report gives way to them.
            if start == .legacy { start = .idle }
            if start == .sent, s.state == .running || s.state == .refused { start = .idle }
        }
    }

    // MARK: - Sections

    private var startSection: some View {
        Section {
            Button(action: startCalibration) {
                HStack {
                    Label(buttonTitle, systemImage: "gyroscope")
                    Spacer()
                    if busy { ProgressView() }
                }
            }
            .disabled(busy || !canRun)
            Text("Place the rocket on its pad and keep it still. The gyro & accel calibration takes about 10 s; the GNSS survey keeps going until launch.")
                .font(.caption).foregroundColor(.secondary)
            if !canRun {
                Label(device.isBaseStation
                          ? "Connect to the rocket itself to calibrate it."
                          : "Connect to the rocket and power it on to calibrate it.",
                      systemImage: "bolt.slash")
                    .font(.caption).foregroundColor(.orange)
            }
        }
    }

    private var buttonTitle: String {
        switch start {
        case .fetchingIonosphere: return "Getting the ionosphere…"
        case .sent: return "Starting…"
        case .legacy: return "Calibrating…"
        case .idle: return status?.state == .running ? "Calibrating…" : "Start Pad Calibration"
        }
    }

    private func gyroAccelSection(_ s: PadCalStatus) -> some View {
        Section("Gyro & Accel") {
            if start == .fetchingIonosphere || start == .sent {
                // Not the last calibration's result while this one starts.
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Starting…")
                }
            } else if s.state == .running {
                ProgressView(value: Double(min(s.calProgress, 100)), total: 100) {
                    Text(s.gyroAccelText)
                }
            } else {
                Label(s.gyroAccelText, systemImage: Self.icon(s.state))
                    .foregroundColor(Self.color(s.state))
                if let warning = s.gravityWarningText {
                    Text(warning).font(.caption).foregroundColor(.orange)
                }
            }
        }
    }

    private func gnssSection(_ s: PadCalStatus) -> some View {
        Section("GNSS Pad Survey") {
            if s.padMismatch {
                Label(PadCalStatus.padMismatchText, systemImage: "exclamationmark.triangle.fill")
                    .foregroundColor(.orange)
            }
            ForEach(s.gnssRows) { row in
                LabeledContent(row.label, value: row.value)
            }
        }
    }

    private var phoneSection: some View {
        Section {
            VStack(alignment: .leading, spacing: 2) {
                Text("Ionosphere on this phone")
                Text(IonosphereStore.phoneText(ionosphere.held, now: Date()))
                    .font(.caption).foregroundColor(.secondary)
            }
            if ionosphere.isRefreshing {
                HStack(spacing: 10) {
                    ProgressView()
                    Text("Fetching today's set from NOAA…")
                        .font(.caption).foregroundColor(.secondary)
                }
            }
        } footer: {
            Text("Read from NOAA's daily GPS navigation file and kept on the phone, so it works at a field with no signal.")
        }
    }

    private static func icon(_ state: PadCalStatus.CalState?) -> String {
        switch state {
        case .done?: return "checkmark.circle.fill"
        case .rejected?, .refused?: return "exclamationmark.triangle.fill"
        default: return "circle.dashed"
        }
    }

    private static func color(_ state: PadCalStatus.CalState?) -> Color {
        switch state {
        case .done?: return .green
        case .rejected?, .refused?: return .orange
        default: return .secondary
        }
    }

    // MARK: - Actions

    private func startCalibration() {
        startGeneration += 1
        let gen = startGeneration
        pendingCalSnapshot = true
        start = ionosphere.isCurrent ? .sent : .fetchingIonosphere
        Task {
            let iono = await ionosphere.setForPadCalibration()
            // Sent whatever this screen does meanwhile: the operator asked.
            device.startPadCalibration(iono) { runLegacyFlow(gen) }
            guard gen == startGeneration else { return }
            if start == .fetchingIonosphere { start = .sent }
            // The net under "Starting…" if the frame that shows the start
            // never comes (a lost write; a refusal arrives as `refused`).
            DispatchQueue.main.asyncAfter(deadline: .now() + 4) {
                if gen == startGeneration, start == .sent { start = .idle }
            }
        }
    }

    /// Older firmware: BLEDevice has sent 21; run the flow 21 always had.
    private func runLegacyFlow(_ gen: Int) {
        guard gen == startGeneration else { return }
        start = .legacy
        // Calibration takes ~10 seconds (1000 samples x 10ms)
        DispatchQueue.main.asyncAfter(deadline: .now() + 12.0) {
            // Frames took over: their result stands, not this blind check.
            guard gen == startGeneration, start == .legacy else { return }
            start = .idle

            // Check gravity magnitude from low-g accelerometer
            let lx = device.telemetry.low_g_x ?? 0
            let ly = device.telemetry.low_g_y ?? 0
            let lz = device.telemetry.low_g_z ?? 0
            let mag = sqrtf(lx * lx + ly * ly + lz * lz)
            let expected: Float = 9.80665
            let errorPct = fabsf(mag - expected) / expected * 100.0

            // Only alert if we actually have accel data (mag > 0)
            if mag > 0.1 && errorPct > 3.0 {
                gravityMag = mag
                gravityError = errorPct
                showGravityWarning = true
            }
        }
    }
}
