import Foundation

/// The OTA chunk pump's wait for CoreBluetooth's outgoing write buffer to
/// drain, with a bounded timeout (#836 item 7).
///
/// This used to be a `withTimeout` helper that raced two children in a
/// `withTaskGroup`: one running the body, one sleeping. It took
/// `group.next()`, called `cancelAll()` and returned — except **a task group
/// implicitly awaits ALL of its children before returning**, and the body
/// child was parked in a plain `withCheckedContinuation`, which is not
/// cancellable. So when the sleeper won, `withTimeout` did not return. It
/// blocked until something else resumed the continuation, which is exactly
/// the hang the 5-second bound existed to prevent: with a characteristic that
/// does not advertise WRITE_NO_RSP, nothing ever would. The
/// `if !ready { throw }` branch was unreachable while the device stayed
/// connected, `OTASession` sat in `.uploading` with a frozen progress bar,
/// `Task.cancel()` was a no-op against the checked continuation, and only a
/// BLE disconnect could break it out.
///
/// One continuation, one timer, and a single `signal` funnel that clears both
/// before resuming — so a double resume (a trap, not an error) is structurally
/// impossible rather than merely avoided.
final class OtaWriteGate {

    enum Outcome: Equatable {
        /// The buffer drained — send the chunk.
        case ready
        /// Nothing drained it in time. The characteristic most likely does not
        /// advertise WRITE_NO_RSP (a firmware-side bug we hit on bench
        /// 2026-05-28).
        case timedOut
        /// The link dropped while we were parked.
        case disconnected
    }

    private var continuation: CheckedContinuation<Outcome, Never>?
    private var timeout: DispatchWorkItem?

    /// This target builds with SWIFT_DEFAULT_ACTOR_ISOLATION = MainActor
    /// against an iOS 16 deployment target, so a main-actor class gets an
    /// isolated deinit whose back-deploy shim double-frees on iOS 26.2. Three
    /// PRs have been spent on that (#734, #817, #937); this one opts out up
    /// front.
    nonisolated deinit {}

    /// True while a chunk is parked. The pump is serial, so this should never
    /// be true when `wait` is called.
    var isWaiting: Bool { continuation != nil }

    /// Park until `signal` is called or `seconds` elapse.
    ///
    /// `alreadyReady` is evaluated *inside* the continuation rather than by
    /// the caller beforehand: CoreBluetooth can report the buffer drained
    /// between the caller's check and this parking, and that window is a lost
    /// wakeup that hangs the pump for the full timeout.
    func wait(seconds: TimeInterval, alreadyReady: @escaping () -> Bool) async -> Outcome {
        await withCheckedContinuation { (cont: CheckedContinuation<Outcome, Never>) in
            // The pump is serial, so this should not happen — but leaking a
            // parked continuation is an unrecoverable hang, whereas failing
            // the older chunk is a retryable error. Never overwrite.
            if let stale = continuation {
                continuation = nil
                timeout?.cancel()
                timeout = nil
                stale.resume(returning: .timedOut)
            }

            if alreadyReady() {
                cont.resume(returning: .ready)
                return
            }

            continuation = cont
            let work = DispatchWorkItem { [weak self] in self?.signal(.timedOut) }
            timeout = work
            DispatchQueue.main.asyncAfter(deadline: .now() + seconds, execute: work)
        }
    }

    /// Resume a parked waiter. Harmless when nothing is parked — the buffer
    /// drains constantly during a transfer and most of those signals arrive
    /// with no waiter to wake.
    func signal(_ outcome: Outcome) {
        timeout?.cancel()
        timeout = nil
        guard let cont = continuation else { return }
        continuation = nil
        cont.resume(returning: outcome)
    }
}
