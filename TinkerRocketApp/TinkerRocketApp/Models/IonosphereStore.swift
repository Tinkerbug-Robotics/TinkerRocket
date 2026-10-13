//
//  IonosphereStore.swift
//  TinkerRocketApp
//
//  The broadcast ionosphere a pad calibration hands the flight computer
//  (#1600 phase 5): found on NOAA, kept on the phone, sent with command 75.
//  The Android app follows the same policy.
//
//  WHERE FROM. NOAA's CORS archive (Klobuchar.noaaUrl) — public, HTTPS, no
//  login, so an anonymous read is all a phone ever needs. Today's file is up
//  from early in the UTC day and grows through it; until it is, or with no
//  signal, yesterday's is next. A file's GPS day is the UTC date it is named
//  for, and that is what the rocket ages the set by.
//
//  KEPT, BECAUSE LAUNCH SITES HAVE NO SIGNAL. The last good set is stored with
//  its GPS day and when it was fetched — Application Support, excluded from
//  backup, for FirmwareCache's reasons. A refresh runs in the background when
//  the app starts and when the pad calibration screen opens, and only when the
//  held set is from before today; nothing on screen waits for it.
//
//  AT A CALIBRATION, a set from before today gets up to 5 s of a refresh, which
//  carries on afterwards for the next one; then what is held is sent, else
//  none. Without one the rocket uses its own pad survey, so a missing set
//  costs accuracy, never the calibration.
//

import Combine
import Foundation

/// A set as the phone holds it.
nonisolated struct HeldIonosphere: Equatable {
    let klobuchar: Klobuchar
    /// When it was fetched; the file's own date is `klobuchar.gpsDay`.
    let fetchedAt: Date
}

/// Where the held set lives between launches of the app.
nonisolated struct IonosphereCache {

    private let file: URL

    init(file: URL) {
        self.file = file
    }

    static func defaultFile() -> URL {
        let base = FileManager.default
            .urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        var dir = base.appendingPathComponent("Ionosphere")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        var values = URLResourceValues()
        values.isExcludedFromBackup = true
        try? dir.setResourceValues(values)
        return dir.appendingPathComponent("broadcast.json")
    }

    /// The file Android's IonosphereCache writes too: one small JSON object.
    private struct Stored: Codable {
        let raw: [Int]
        let gpsDay: Int
        let fetchedAtMs: Int64

        enum CodingKeys: String, CodingKey {
            case raw
            case gpsDay = "gps_day"
            case fetchedAtMs = "fetched_at_ms"
        }
    }

    /// The held set, read back through Klobuchar's own checks so a damaged
    /// file reads as none rather than as a wrong set.
    func load() -> HeldIonosphere? {
        guard let data = try? Data(contentsOf: file),
              let s = try? JSONDecoder().decode(Stored.self, from: data),
              let k = Klobuchar(raw: s.raw, gpsDay: s.gpsDay) else { return nil }
        return HeldIonosphere(klobuchar: k,
                              fetchedAt: Date(timeIntervalSince1970: Double(s.fetchedAtMs) / 1000))
    }

    @discardableResult
    func store(_ held: HeldIonosphere) -> Bool {
        do {
            try FileManager.default.createDirectory(at: file.deletingLastPathComponent(),
                                                    withIntermediateDirectories: true)
            let s = Stored(raw: held.klobuchar.raw, gpsDay: held.klobuchar.gpsDay,
                           fetchedAtMs: Int64((held.fetchedAt.timeIntervalSince1970 * 1000).rounded()))
            // .atomic: a killed app leaves the old set, not half of a new one.
            try JSONEncoder().encode(s).write(to: file, options: .atomic)
            return true
        } catch {
            return false
        }
    }
}

/// Fetching, with the network injected so the policy is testable without one.
nonisolated struct IonosphereRepository {
    /// Returns the body, or nil when the request failed.
    let fetch: @Sendable (String) async -> Data?

    /// The default: URLSession, no credentials of any kind.
    static let urlSessionFetch: @Sendable (String) async -> Data? = { urlString in
        guard let url = URL(string: urlString) else { return nil }
        var req = URLRequest(url: url)
        req.timeoutInterval = 30
        // Today's file grows through the day: always the server's copy.
        req.cachePolicy = .reloadIgnoringLocalCacheData
        guard let (data, resp) = try? await URLSession.shared.data(for: req),
              let http = resp as? HTTPURLResponse, (200..<300).contains(http.statusCode)
        else { return nil }
        return data
    }

    /// Today's set, else yesterday's: the first file that downloads, inflates
    /// and carries both ionosphere lines. Nil when neither does.
    func latest(now: Date) async -> Klobuchar? {
        for date in [now, now.addingTimeInterval(-86_400)] {
            if Task.isCancelled { return nil }
            if let body = await fetch(Klobuchar.noaaUrl(date)),
               let k = Self.parse(body, gpsDay: Klobuchar.gpsDay(date)) {
                return k
            }
        }
        return nil
    }

    /// A day's file is 30-70 KB of gzip and only its header is read: nothing
    /// past this much of a body is looked at (Android caps its download here).
    static let maxBodyBytes = 1 << 20

    /// The parameters in a downloaded file: gzip as NOAA serves it, or plain
    /// text if something on the way (a Content-Encoding) already inflated it.
    static func parse(_ body: Data, gpsDay: Int) -> Klobuchar? {
        let head = Data(body.prefix(maxBodyBytes))
        let text = head.starts(with: [0x1F, 0x8B]) ? Gzip.decompress(head) : head
        guard let text else { return nil }
        return Klobuchar.fromRinexHeader(String(decoding: text, as: UTF8.self), gpsDay: gpsDay)
    }
}

/// The phone's ionosphere for pad calibrations: what it holds, and keeping
/// that current. One per app — the screens observe `shared`; tests build their
/// own with a fake fetch, a scratch cache and a fixed clock.
@MainActor
final class IonosphereStore: ObservableObject {

    // The swiftlang/swift#87316 workaround the other main-actor stores carry:
    // an isolated deinit aborts on iOS 26.2 at this deployment target when a
    // test releases an instance. Nothing to do at deinit, so nothing is lost.
    nonisolated deinit {}

    static let shared = IonosphereStore()

    /// How long a calibration waits on a refresh of a set from before today.
    nonisolated static let calibrationRefreshTimeout: TimeInterval = 5

    /// The last good set, or nil when this phone has never fetched one.
    @Published private(set) var held: HeldIonosphere?
    /// A refresh is in flight.
    @Published private(set) var isRefreshing = false

    private let repository: IonosphereRepository
    private let cache: IonosphereCache?
    private let now: @Sendable () -> Date
    private var background: Task<Void, Never>? {
        didSet { isRefreshing = background != nil }
    }

    init(repository: IonosphereRepository = IonosphereRepository(
            fetch: IonosphereRepository.urlSessionFetch),
         cache: IonosphereCache? = IonosphereCache(file: IonosphereCache.defaultFile()),
         now: @escaping @Sendable () -> Date = { Date() }) {
        self.repository = repository
        self.cache = cache
        self.now = now
        self.held = cache?.load()
    }

    /// The held set is today's file (UTC) — or a later one, on a phone whose
    /// clock is behind.
    var isCurrent: Bool {
        guard let held else { return false }
        return held.klobuchar.gpsDay >= Klobuchar.gpsDay(now())
    }

    /// At app start and when the pad calibration screen opens: fetch in the
    /// background if the held set is from before today. Returns at once; one
    /// refresh runs at a time.
    func refreshIfStale() {
        guard !isCurrent, background == nil else { return }
        background = Task {
            await refresh()
            background = nil
        }
    }

    /// The set a pad calibration sends: if the held set is from before today,
    /// after waiting up to `timeout` on a refresh (the running one, if there
    /// is one), whatever is held; nil when the phone has none. The refresh is
    /// not cut off at the timeout: it lands for the next calibration.
    func setForPadCalibration(timeout: TimeInterval = IonosphereStore.calibrationRefreshTimeout)
        async -> Klobuchar? {
        refreshIfStale()
        // Polled: awaiting the task itself would wait it out.
        let deadline = Date().addingTimeInterval(timeout)
        while background != nil, Date() < deadline {
            do { try await Task.sleep(nanoseconds: 50_000_000) } catch { break }
        }
        return held?.klobuchar
    }

    /// Fetch today's (else yesterday's) set and keep it. Nothing changes when
    /// nothing usable arrives: an old set beats none.
    func refresh() async {
        guard let k = await repository.latest(now: now()) else { return }
        // Never backwards: a refresh that reached only yesterday's file must
        // not replace today's set.
        if let held, held.klobuchar.gpsDay > k.gpsDay { return }
        let entry = HeldIonosphere(klobuchar: k, fetchedAt: now())
        cache?.store(entry)
        held = entry
    }

    /// "set of Oct 13, 2026 (fetched 2 hours ago)", or what the rocket does
    /// without one.
    nonisolated static func phoneText(_ held: HeldIonosphere?, now: Date) -> String {
        guard let held else { return "none — the rocket will use its pad survey" }
        let rel = RelativeDateTimeFormatter()
        rel.dateTimeStyle = .named
        return "set of \(PadCalStatus.day(held.klobuchar.gpsDay)) "
            + "(fetched \(rel.localizedString(for: held.fetchedAt, relativeTo: now)))"
    }
}
