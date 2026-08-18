//
//  FinLayoutView.swift
//  TinkerRocketApp
//
//  Rocket-settings editor for the servo→fin mapping + ring orientation (FinConfigData,
//  cmd 66).  A nose-down ring (looking from the nose aft: +Z up, +Y right) where each
//  servo sits at a fin position; the ring is "+" (on-axis) or "×" (45°); each fin can be
//  reversed; and a servo jog (reuses the servo-test path) confirms deflection direction
//  on the bench.  Pure presentation: values in, edit closures out — the parent owns the
//  profile and sends the config.
//

import SwiftUI

struct FinLayoutView: View {
    let device: BLEDevice
    let ringMode: UInt8            // 0 = "+", 1 = "×"
    let servoAtSlot: [Int]         // servo (1-4) at ring slot 0..3
    let reverse: [Bool]            // per-servo (1-4) tilt (pitch/yaw) reverse
    let rollReverse: [Bool]        // per-servo (1-4) roll reverse (independent)
    let canJog: Bool
    // Fin-angle endpoints the jog may command, from the active profile's fin
    // calibration (#407) — same clamp ServoTestView uses, so a narrow-travel
    // model never jogs past its calibrated deflection. Defaults to ±20°.
    var finMinDeg: Double = -20
    var finMaxDeg: Double = 20
    let onSetRingMode: (UInt8) -> Void
    let onSetServoAtSlot: ([Int]) -> Void
    let onSetReverse: ([Bool]) -> Void
    let onSetRollReverse: ([Bool]) -> Void

    @State private var selectedSlot: Int = 0

    /// Fin-position azimuth (deg) of each ring slot in the firmware convention
    /// (θ=0 = +Z/top slot, 90 = −Y slot, … — positions, NOT force directions:
    /// the FC maps position→tangential force itself, sin/cos in
    /// TR_ControlMixer::setFinLayout); the nose-down view is rendering only.
    private var slotAzimuths: [Double] {
        ringMode == 1 ? [45, 135, 225, 315] : [0, 90, 180, 270]
    }
    private var selectedServo: Int {
        servoAtSlot.indices.contains(selectedSlot) ? servoAtSlot[selectedSlot] : 1
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            Picker("Orientation", selection: ringModeBinding) {
                Text("+ on axes").tag(0)
                Text("× 45°").tag(1)
            }
            .pickerStyle(.segmented)

            ring
                .frame(height: 230)
                .frame(maxWidth: .infinity)

            Text("Viewed from the nose, looking aft · +Z up, +Y right · tap a fin")
                .font(.caption2).foregroundColor(.secondary)
                .frame(maxWidth: .infinity, alignment: .center)

            Divider()

            Toggle("Reverse roll direction", isOn: rollDirectionBinding)
            Text("Use this when the rocket rolls the wrong way — it flips roll on all four fins at once. The per-fin Reverse roll below is only for a single mis-linked servo: flipping one fin makes it fight the other three (weaker roll, same direction) rather than reversing roll.")
                .font(.caption2).foregroundColor(.secondary)

            Divider()
            selectedFinEditor
        }
        .padding(.vertical, 4)
    }

    // MARK: Ring

    private var ring: some View {
        GeometryReader { geo in
            let cx = geo.size.width / 2
            let cy = geo.size.height / 2
            let r = min(cx, cy) - 26
            ZStack {
                Circle()
                    .stroke(Color.secondary.opacity(0.6), lineWidth: 1.5)
                    .frame(width: 2 * r, height: 2 * r)
                    .position(x: cx, y: cy)
                Circle().fill(Color.secondary.opacity(0.5))
                    .frame(width: 7, height: 7).position(x: cx, y: cy)

                // Axis labels: +Z top, +Y right, −Z bottom, −Y left (nose-down).
                ForEach(0..<4, id: \.self) { i in
                    Text(["+Z", "+Y", "−Z", "−Y"][i])
                        .font(.caption2).foregroundColor(.secondary)
                        .position(point(clockwiseDeg: Double(i) * 90, radius: r + 16, cx: cx, cy: cy))
                }

                ForEach(0..<4, id: \.self) { slot in
                    finMarker(slot: slot, r: r, cx: cx, cy: cy)
                }
            }
        }
    }

    @ViewBuilder
    private func finMarker(slot: Int, r: CGFloat, cx: CGFloat, cy: CGFloat) -> some View {
        let scrA = (360 - slotAzimuths[slot]).truncatingRemainder(dividingBy: 360)  // nose-down mirror
        let sel = slot == selectedSlot
        let servo = servoAtSlot.indices.contains(slot) ? servoAtSlot[slot] : slot + 1
        let bladeCenter = point(clockwiseDeg: scrA, radius: r, cx: cx, cy: cy)
        let badgeCenter = point(clockwiseDeg: scrA, radius: r - 22, cx: cx, cy: cy)

        Rectangle()
            .fill(sel ? Color.accentColor : Color.secondary)
            .frame(width: 6, height: 26)
            .rotationEffect(.degrees(scrA))
            .position(bladeCenter)

        Button { selectedSlot = slot } label: {
            Text("\(servo)")
                .font(.headline)
                .foregroundColor(sel ? Color.white : Color.primary)
                .frame(width: 32, height: 32)
                .background(Circle().fill(sel ? Color.accentColor : Color(.secondarySystemBackground)))
                .overlay(Circle().stroke(sel ? Color.accentColor : Color.secondary, lineWidth: 1.5))
        }
        .buttonStyle(.plain)
        .position(badgeCenter)
    }

    // MARK: Selected-fin editor

    private var selectedFinEditor: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text(slotName(selectedSlot)).font(.subheadline).fontWeight(.semibold)
                Spacer()
                Text("drives: \(mixDescription(selectedSlot))")
                    .font(.caption).foregroundColor(.secondary)
            }

            HStack {
                Text("Servo here")
                Spacer()
                Picker("", selection: servoBinding) {
                    ForEach(1...4, id: \.self) { Text("\($0)").tag($0) }
                }
                .pickerStyle(.segmented).frame(width: 170)
            }

            Toggle("Reverse pitch/yaw (tilt)", isOn: reverseBinding)
            Toggle("Reverse roll (this fin only)", isOn: rollReverseBinding)

            VStack(alignment: .leading, spacing: 6) {
                Text("Jog servo \(selectedServo) to confirm direction")
                    .font(.caption).foregroundColor(.secondary)
                HStack(spacing: 8) {
                    Button { jog(-10) } label: { Label("−10°", systemImage: "minus") }
                    Button { jogStop() } label: { Label("Stop", systemImage: "stop.fill") }
                    Button { jog(10) } label: { Label("+10°", systemImage: "plus") }
                }
                .buttonStyle(.bordered)
                .disabled(!canJog)
                if !canJog {
                    Text("Connect and power on the rocket to jog the servos.")
                        .font(.caption2).foregroundColor(.secondary)
                }
            }
        }
    }

    // MARK: Bindings

    private var ringModeBinding: Binding<Int> {
        Binding(get: { Int(ringMode) }, set: { onSetRingMode(UInt8($0)) })
    }
    private var servoBinding: Binding<Int> {
        Binding(get: { selectedServo }, set: { newServo in
            guard servoAtSlot.indices.contains(selectedSlot) else { return }
            var slots = servoAtSlot
            if let other = slots.firstIndex(of: newServo) { slots[other] = slots[selectedSlot] }
            slots[selectedSlot] = newServo
            onSetServoAtSlot(slots)
        })
    }
    private var reverseBinding: Binding<Bool> {
        Binding(get: { reverse.indices.contains(selectedServo - 1) ? reverse[selectedServo - 1] : false },
                set: { newVal in
            guard reverse.indices.contains(selectedServo - 1) else { return }
            var rev = reverse
            rev[selectedServo - 1] = newVal
            onSetReverse(rev)
        })
    }
    /// Whole-airframe roll direction — every servo's roll bit at once.  This is
    /// the common case by far: a mirrored horn or fin convention reverses all
    /// four together, and reversing roll from the per-fin toggles means getting
    /// all four right.  Reads ON only when every bit is set; a mixed state (one
    /// fin overridden below) reads OFF, and switching it on normalises all four.
    /// Same wire field as the per-fin toggle — FinConfigData.roll_reverse_mask.
    private var rollDirectionBinding: Binding<Bool> {
        Binding(get: { rollReverse.count == 4 && rollReverse.allSatisfy { $0 } },
                set: { newVal in onSetRollReverse([Bool](repeating: newVal, count: 4)) })
    }
    private var rollReverseBinding: Binding<Bool> {
        Binding(get: { rollReverse.indices.contains(selectedServo - 1) ? rollReverse[selectedServo - 1] : false },
                set: { newVal in
            guard rollReverse.indices.contains(selectedServo - 1) else { return }
            var rev = rollReverse
            rev[selectedServo - 1] = newVal
            onSetRollReverse(rev)
        })
    }

    // MARK: Jog (reuses the servo-test path)

    private func jog(_ deg: Double) {
        let idx = selectedServo - 1
        guard idx >= 0 && idx < 4 else { return }
        var angles = [0.0, 0.0, 0.0, 0.0]
        // Apply the per-servo reverse so the jog reflects the controller's "+".
        let target = (reverse.indices.contains(idx) && reverse[idx]) ? -deg : deg
        // Clamp the commanded angle to the profile's fin-cal range (#407) so the
        // jog can't over-deflect on a narrow-travel model. The FC clamps too, but
        // matching it here keeps the app honest about what it sends.
        let lo = Swift.min(finMinDeg, finMaxDeg)
        let hi = Swift.max(finMinDeg, finMaxDeg)
        angles[idx] = Swift.min(Swift.max(target, lo), hi)
        device.sendServoTestAngles(angles)
    }
    private func jogStop() { device.sendServoTestStop() }

    // MARK: Geometry + labels

    private func point(clockwiseDeg deg: Double, radius: CGFloat, cx: CGFloat, cy: CGFloat) -> CGPoint {
        let a = deg * .pi / 180
        return CGPoint(x: cx + radius * CGFloat(sin(a)), y: cy - radius * CGFloat(cos(a)))
    }
    private func slotName(_ slot: Int) -> String {
        let plus  = ["Top · +Z", "Left · −Y", "Bottom · −Z", "Right · +Y"]
        let cross = ["Upper-left", "Lower-left", "Lower-right", "Upper-right"]
        let names = ringMode == 1 ? cross : plus
        return names.indices.contains(slot) ? names[slot] : "Fin"
    }
    private func mixDescription(_ slot: Int) -> String {
        let th = slotAzimuths[slot] * .pi / 180
        let servo = servoAtSlot.indices.contains(slot) ? servoAtSlot[slot] : slot + 1
        let tilt = (reverse.indices.contains(servo - 1) && reverse[servo - 1]) ? -1.0 : 1.0
        let rollS = (rollReverse.indices.contains(servo - 1) && rollReverse[servo - 1]) ? -1.0 : 1.0
        // Position→force: a fin's tangential lift is perpendicular to its
        // radial arm, so pitch = sin(az), yaw = -cos(az) — matches
        // TR_ControlMixer::setFinLayout (top/bottom fins are yaw rudders,
        // side fins are pitch elevators).
        let p = sin(th) * tilt, y = -cos(th) * tilt
        var parts: [String] = []
        if abs(p) > 0.05 { parts.append("pitch \(p > 0 ? "+" : "−")") }
        if abs(y) > 0.05 { parts.append("yaw \(y > 0 ? "+" : "−")") }
        parts.append("roll \(rollS > 0 ? "+" : "−")")
        return parts.joined(separator: " · ")
    }
}
