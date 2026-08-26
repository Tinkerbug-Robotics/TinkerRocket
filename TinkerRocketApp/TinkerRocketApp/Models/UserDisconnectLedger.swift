import Foundation

/// Which peripherals the USER asked to disconnect, pending their
/// `didDisconnectPeripheral` callback (#836 item 8).
///
/// BLEFleet decides whether a dropped link should reconnect automatically. A
/// link the user tore down must not; a link that failed on its own must. That
/// decision used to be one fleet-wide `Bool`: `disconnectAll()` set it once and
/// then cancelled every connection in a loop, but the FIRST callback consumed
/// it — so with a base station and a rocket both up (an explicitly supported
/// pairing), the second device fell through to the reconnect ladder and came
/// back about a second later. On the resulting `didConnect` the syncer
/// re-attached and re-pushed the whole active profile, `sendPyroConfig`
/// included, to a rocket the user had just disconnected from.
///
/// A value type on purpose: it has no deinit, so it cannot join the
/// main-actor isolated-deinit trap that classes in this target keep hitting.
struct UserDisconnectLedger: Equatable {
    private var pending: Set<UUID> = []

    var isEmpty: Bool { pending.isEmpty }
    var count: Int { pending.count }

    /// The user asked for this peripheral to go away. Idempotent — a second
    /// tap before the callback arrives must not queue a second suppression.
    mutating func mark(_ id: UUID) {
        pending.insert(id)
    }

    /// Answer for the callback that just fired, and clear it. True means the
    /// user asked for this one, so do NOT reconnect.
    ///
    /// Consuming per-peripheral is the whole point: every `mark` is matched by
    /// exactly the callback it caused, and a callback for a peripheral nobody
    /// marked reads as the dropout it is.
    mutating func consume(_ id: UUID) -> Bool {
        pending.remove(id) != nil
    }

    /// Drop a marker without answering — the peripheral connected again, or
    /// its connection attempt failed, so any pending marker is stale and the
    /// NEXT disconnect is a real dropout that must be allowed to reconnect.
    mutating func forget(_ id: UUID) {
        pending.remove(id)
    }
}
