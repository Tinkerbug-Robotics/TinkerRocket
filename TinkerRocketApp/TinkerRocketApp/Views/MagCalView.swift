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

    /// Sampling: progress bar + live count + coverage + |B|.
    private var samplingSection: some View {
        Group {
            Section(header: Text("Tumble the rocket")) {
                if let s = status {
                    let progress = s.samplingProgress(
                        targetSamples: MagCalConstants.maxSamples,
                        minCoverage: 26  // visualise approach to full coverage; FC uses 18 as the gate
                    )
                    VStack(alignment: .leading, spacing: 12) {
                        ProgressView(value: progress)
                            .progressViewStyle(.linear)
                        HStack {
                            Text("Samples")
                            Spacer()
                            Text("\(s.sampleCount) / \(MagCalConstants.maxSamples)")
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
                        HStack {
                            Text("|B|")
                            Spacer()
                            Text(String(format: "%.1f µT", s.instantaneousFieldUT))
                                .foregroundColor(.secondary)
                                .font(.system(.body, design: .monospaced))
                        }
                    }
                    .padding(.vertical, 4)
                } else {
                    ProgressView("Waiting for rocket…")
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
