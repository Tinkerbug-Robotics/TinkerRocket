import Foundation

/// Every wait in the OTA flow, in one table.
///
/// Split out of OTASession so it can be tested without CoreBluetooth: the
/// session itself reaches a live BLEDevice, but the timing contract is pure
/// data and the part most likely to drift from Android.
///
/// The contract these numbers have to satisfy is `crossesRelay`. A stage that
/// waits on the FC answering over the OC relay MUST get a longer window on the
/// FC path; a stage that only waits on the phone's own link to the OC MUST get
/// the same one. Bench 2026-07-28 found `finish` violating that — flat 15 s on
/// both paths, which expired while a 591.7 kB FC flash was still running, so
/// the app reported failure on a transfer that went on to succeed. See #627.
///
/// The same table lives in Kotlin as `OtaTimeouts`, and both suites check
/// themselves against `tests_cpp/fixtures/app_behavior/ota_timeouts.json` —
/// so changing one platform without the other fails that platform's build.
enum OTAStage: String, CaseIterable {
    case begin
    case finish
    case fwPublish
    case disconnect
    case reconnect

    /// Does finishing this stage depend on the FC answering over the relay?
    var crossesRelay: Bool {
        switch self {
        case .begin, .finish, .fwPublish: return true
        case .disconnect, .reconnect:     return false
        }
    }
}

enum OTATimeouts {

    /// Poll interval for every await in the flow.
    static let pollSeconds: TimeInterval = 0.05

    /// Ceiling on the chunk pump for FC-relay OTAs only — a local OC OTA stays
    /// uncapped (straight to flash, no relay).
    ///
    /// #627: on the relay path the OC re-sends every chunk to the FC over I2S.
    /// Outrun that drain and the OC's 16-deep feed queue fills, receive buffers
    /// stop being retired, NimBLE's ACL mbuf pool exhausts and the link wedges —
    /// the OC stops receiving entirely (bench 2026-07-28: `enq` frozen at 3418
    /// for 135 s amid thousands of `ACL buf alloc failed`). The FC then sees a
    /// hole and, being forward-only, discards everything after it.
    ///
    /// Measured on one board, one image: iOS pumps 11.4 kB/s and the relay
    /// works; Android pumps ~68 kB/s and it fails. iOS was never *correct*
    /// here, only slow enough to stay under a limit nobody had written down.
    /// This writes it down — nearly a no-op on iOS, the actual fix on Android.
    static let fcRelayMaxBytesPerSec: Double = 12_000

    /// Seconds to wait before the next chunk to hold the FC-relay pump under
    /// `fcRelayMaxBytesPerSec`.
    ///
    /// Paced against total bytes sent rather than per-chunk, so the time the
    /// write itself took is credited instead of added — a fixed per-chunk sleep
    /// would drift the effective rate well below the cap.
    static func fcRelayPaceDelay(bytesSent: Int, elapsed: TimeInterval) -> TimeInterval {
        let target = Double(bytesSent) / fcRelayMaxBytesPerSec
        return target > elapsed ? target - elapsed : 0
    }

    static func seconds(_ stage: OTAStage, targetIsFC: Bool) -> TimeInterval {
        switch stage {
        // FC erases its OTA slot before it can answer ready.
        case .begin:      return targetIsFC ? 20.0 : 5.0
        // FC drains the I2S ring, writes the tail, SHA-256s the image and
        // sets the boot partition — every status hop crossing the relay.
        case .finish:     return targetIsFC ? 60.0 : 15.0
        // FC reboots, then the OC re-queries its identity before the new
        // version can reach the app.
        case .fwPublish:  return targetIsFC ? 120.0 : 10.0
        // Local link events — the relay plays no part, so no FC stretch.
        case .disconnect: return 5.0
        case .reconnect:  return 60.0
        }
    }
}
