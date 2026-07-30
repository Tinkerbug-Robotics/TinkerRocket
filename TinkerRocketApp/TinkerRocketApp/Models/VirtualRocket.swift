import Foundation

/// Virtual Rocket — the iOS twin of Android's demo mode (user request
/// 2026-07-30): a peripheral-less `BLEDevice` driven by a scripted flight
/// through the REAL parse/dispatch/UI stack.  No CoreBluetooth involved, so
/// it runs in the Simulator too.
///
/// Behavior converged with Android's `DemoMode.kt`, frame for frame: the
/// virtual base station relays a "Booster" that idles in READY like a real
/// rocket and flies ONE canned ~40 s flight per SIM_START — the standard
/// Simulation sheet is the start control, exactly the ritual a real rocket
/// uses — honoring SIM_STOP mid-flight.  A second relayed rocket sits on the
/// pad so the roster and focus-switching are demonstrable.
///
/// Nothing is recorded, and Files stays real-hardware-only on iOS (Android's
/// demo serves a canned log through its transport fake; iOS has no transport
/// seam — ledger-documented divergence).
@MainActor
final class VirtualRocketDriver {

    let device: BLEDevice
    private var task: Task<Void, Never>?
    private var startRequested = false
    private var stopRequested = false

    /// Test hook: scales every sleep so the whole flight runs in tens of
    /// milliseconds under XCTest (same pattern as OTASession.timeScale).
    var timeScale: Double = 1.0

    init() {
        device = BLEDevice(peripheral: nil, name: "TR-B-Virtual")
        device.commandTap = { [weak self] cmd, payload in
            guard let self else { return }
            // On a base-station link the UI wraps rocket commands in the
            // cmd-50 relay envelope [rid][inner cmd][payload...] (#390) —
            // and the virtual device IS a base station, so unwrap first.
            var effective = Int(cmd)
            if cmd == 50, payload.count >= 2 {
                effective = Int(payload[payload.index(payload.startIndex, offsetBy: 1)])
            }
            if effective == 6 { self.startRequested = true }   // SIM_START
            if effective == 7 { self.stopRequested = true }    // SIM_STOP
        }
    }

    func start() {
        device.onConnect()
        // Identity through the same demux a real readback uses.
        feed("""
            {"type":"config_identity","uid":"virtual1","un":"Virtual Base Station",\
            "nid":7,"dt":"B","fw":"demo+sim"}
            """)
        task = Task { [weak self] in await self?.run() }
    }

    func stop() {
        task?.cancel()
        task = nil
        device.onDisconnect()
    }

    // MARK: - Script (mirror of Android DemoMode.kt)

    private func feed(_ json: String) {
        device.parseTelemetryData(json.data(using: .utf8))
    }

    private func sleep(_ seconds: Double) async {
        try? await Task.sleep(nanoseconds: UInt64(max(0, seconds * timeScale) * 1_000_000_000))
    }

    // OK (01) at baro/imu/ekf/mag/gnss/batt/storage shifts; pyro NA (hidden).
    private let health = 0x100555

    private func run() async {
        while !Task.isCancelled {
            // READY idle at 1 Hz until the Simulation sheet starts a flight.
            startRequested = false
            while !startRequested && !Task.isCancelled {
                feed("""
                    {"rid":1,"run":"Booster","st":"READY","fs":16,"ps":\(0x00A | 0x080),\
                    "h":\(health),"nsat":9,"vol":8.20,"soc":87.0,"palt":0.2,\
                    "lat":34.6572,"lon":-118.2015}
                    """)
                feed("""
                    {"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,\
                    "h":\(health),"nsat":8,"vol":8.31,"palt":0.4}
                    """)
                await sleep(1.0)
            }
            if Task.isCancelled { return }

            // One flight: 200 ticks at 5 Hz (12 s pad countdown, boost,
            // coast, apogee, descent, landed) — SIM_STOP aborts to READY.
            stopRequested = false
            var maxAlt: Double = 0
            for tick in 0..<200 {
                if Task.isCancelled { return }
                if stopRequested { break }
                let t = Double(tick) / 5.0
                let phase: String
                switch t {
                case ..<12: phase = "READY"
                case ..<22: phase = "INFLIGHT"
                case ..<36: phase = "DESCENT"
                default:    phase = "LANDED"
                }
                let alt: Double
                switch t {
                case ..<12: alt = 0
                case ..<22:
                    let tt = t - 12
                    alt = max(0, 80 * tt - 4 * tt * tt)
                case ..<36: alt = max(0, 400 - (t - 22) * 28)
                default:    alt = 0
                }
                maxAlt = max(maxAlt, alt)
                var fs = 0x10 | 0x40                      // pwr on + logging
                if t >= 12 { fs |= 0x01 }                 // launch
                if t >= 14.5 { fs |= 0x200 }              // burnout
                if t >= 22 { fs |= 0x02 | 0x04 }          // apogee votes
                if t >= 36 { fs |= 0x08 }                 // landed
                let rate: Double
                switch t {
                case 12..<14.5: rate = 95
                case 14.5..<22: rate = 30 - (t - 14.5) * 4
                case 22..<36:   rate = -28
                default:        rate = 0
                }
                let frame = String(
                    format: """
                        {"rid":1,"run":"Booster","st":"%@","fs":%d,"ps":%d,"h":%d,\
                        "nsat":9,"vol":%.2f,"cur":%.1f,"soc":%.1f,"palt":%.1f,\
                        "malt":%.1f,"arate":%.1f,"lat":34.6572,"lon":-118.2015}
                        """,
                    phase, fs, 0x00A | 0x080, health,
                    8.2 - t * 0.004,
                    120 + (phase == "INFLIGHT" ? 900.0 : 0.0),
                    87 - t * 0.1, alt, maxAlt, rate
                )
                feed(frame)
                if tick % 5 == 0 {
                    feed("""
                        {"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,\
                        "h":\(health),"nsat":8,"vol":8.31,"palt":0.4}
                        """)
                }
                await sleep(0.2)
            }
            // Flight over or aborted: loop back to the READY idle, like a
            // real rocket after touchdown + reset.
        }
    }
}
