import Foundation

/// OTA receiver-side status, parsed from `ota_status` JSON notifications on
/// the file-ops characteristic (#8 phase 2). The firmware sends one of these
/// on every state transition (begin / verify_failed / ready_to_boot / aborted)
/// plus rate-limited ~2 Hz "writing" updates during the chunk pump.
struct OTAStatusUpdate: Equatable {
    enum State: String {
        case ready              // OTA_BEGIN accepted, awaiting chunks
        case writing            // chunks landing; `bytes` is cumulative
        // FINISH accepted; the device is doing the terminal work (validating
        // the staged image, setting the boot partition) and is blocked while
        // it does. Followed by readyToBoot or verifyFailed. Firmware older
        // than 2026-09-10 never sends it — writing -> silence -> terminal —
        // so the finish wait must still work when it never arrives.
        case verifying
        case readyToBoot        = "ready_to_boot"   // finish OK; device about to esp_restart
        case verifyFailed       = "verify_failed"   // terminal — see `err`
        case aborted            // OTA_ABORT acknowledged
        case idle
        case unknown
    }

    let state: State
    /// Cumulative bytes written so far (0 outside `writing` / `ready` states).
    let bytes: Int
    /// Stable error token from the firmware (`sha_mismatch`, `bad_offset`,
    /// `size_overflow`, `write_failed`, etc.). nil on non-failure states.
    let err: String?
    /// Version stamp of the image currently running on the device. Present on
    /// `ready_to_boot` (= the image about to take over after reboot — used by
    /// the iOS app as a debug anchor while waiting for the post-reboot identity).
    let fw: String?

    static func parse(_ data: Data) -> OTAStatusUpdate? {
        guard
            let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
            json["type"] as? String == "ota_status"
        else { return nil }

        let rawState = json["state"] as? String ?? ""
        let state = State(rawValue: rawState) ?? .unknown
        let bytes = json["bytes"] as? Int ?? 0
        let err = json["err"] as? String
        let fw = json["fw"] as? String
        return OTAStatusUpdate(state: state, bytes: bytes, err: err, fw: fw)
    }
}
