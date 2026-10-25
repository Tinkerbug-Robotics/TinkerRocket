//
//  FileCache.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Manage cached CSV files in app's documents directory
//

import Foundation

/// #1077: the four top-level summary numbers, decoded independently of the
/// settings block so a stale nested schema cannot blank the row.
private struct FlightSummaryHeadline: Decodable {
    let max_altitude_m: Double?
    let max_speed_mps: Double?
    let burnout_time_s: Double?
    let apogee_time_s: Double?
}

/// #1658: where the flight came from, decoded on its own like the headline,
/// so neither block can blank the other or the row.
private struct FlightSummaryIdentity: Decodable {
    struct SettingsBoard: Decodable { let board: String? }
    let source: FlightSource?
    let settings: SettingsBoard?
}

// MARK: - CachedFlight Model

enum CachedFlightType {
    case rocket
    case loraLog
}

struct CachedFlight: Identifiable {
    let name: String          // Original filename in cache (e.g., "flight_20260224_021306.csv")
    let type: CachedFlightType
    let csvURL: URL
    let binaryURL: URL?       // Rocket flights, and base-station logs since #850
    let summaryURL: URL?      // Flight summary JSON (rocket flights only)
    let size: UInt64
    let modificationDate: Date?
    let maxAltitudeM: Double? // From flight summary JSON
    let maxSpeedMps: Double?  // From flight summary JSON
    let burnoutTimeS: Double? // From flight summary JSON (seconds after launch)
    let apogeeTimeS: Double?  // From flight summary JSON (seconds after launch)
    /// Base-station logs: the rocket a per-rocket CSV holds (FileCache splits
    /// a log that carries more than one rocket at download). nil for the
    /// combined log and for everything else.
    var bsLogRocketID: UInt8? = nil
    /// #1658: the rocket and computer it was downloaded from (summary
    /// `source`); nil for flights downloaded before the app recorded it.
    var source: FlightSource? = nil
    /// #1658: the FC's board from the log (summary `settings.board`). nil when
    /// the summary predates the field; FlightDetailView then reads the log.
    var fcBoard: String? = nil

    var id: String { name }

    /// The name the list shows beside the date: the rocket profile, else the
    /// computer's unit name.
    var rocketLabel: String? {
        source?.rocket_profile ?? source?.unit_name
    }

    // Reusable formatters — DateFormatter is expensive to allocate
    private static let utcDateFormatter: DateFormatter = {
        let f = DateFormatter()
        // #1091 item 2: a fixed-format parse must not follow the user's
        // locale or calendar. Without en_US_POSIX the 12/24-hour override
        // makes "HH" return nil and both lists fall back to raw filenames;
        // without .gregorian a Buddhist or Japanese region calendar parses
        // a Date off by the era. Android pins both (Locale.ROOT, STRICT).
        f.locale = Locale(identifier: "en_US_POSIX")
        f.calendar = Calendar(identifier: .gregorian)
        f.dateFormat = "yyyyMMddHHmmss"
        f.timeZone = TimeZone(identifier: "UTC")
        return f
    }()

    private static let localDisplayFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateStyle = .short
        f.timeStyle = .short
        f.timeZone = .current
        // #1091 item 2: display stays localized, but on the Gregorian
        // calendar the parsed Date is on — the locale's own calendar is
        // for rendering, not for round-tripping a filename timestamp.
        f.calendar = Calendar(identifier: .gregorian)
        return f
    }()

    /// Friendly display title — localized date/time for flights with timestamps, raw name otherwise.
    /// One rocket's CSV of a shared base-station log says which rocket.
    var displayTitle: String {
        if let rid = bsLogRocketID {
            return baseDisplayTitle + " · Rocket \(rid)"
        }
        return baseDisplayTitle
    }

    private var baseDisplayTitle: String {
        if let date = flightDate {
            return Self.localDisplayFormatter.string(from: date)
        }
        // Fallback: strip .csv extension for cleaner display
        if name.hasSuffix(".csv") {
            return String(name.dropLast(4))
        }
        return name
    }

    /// Parse UTC timestamp from filename
    /// Supports: "flight_YYYYMMDD_HHMMSS.csv" (or legacy .bin.csv) and "lora_YYYYMMDD_HHMMSS.csv"
    var flightDate: Date? {
        var base = name
        if base.hasSuffix(".bin.csv") {
            base = String(base.dropLast(8)) // legacy format
        } else if base.hasSuffix(".csv") {
            base = String(base.dropLast(4))
        }

        // Determine prefix length: "flight_" (7) or "lora_" (5)
        let prefixLen: Int
        if base.hasPrefix("flight_") {
            prefixLen = 7
        } else if base.hasPrefix("lora_") {
            prefixLen = 5
        } else {
            return nil
        }

        // Need at least prefix + 8 (date) + 1 (underscore) + 6 (time) = prefix + 15
        guard base.count >= prefixLen + 15 else { return nil }

        let startDate = base.index(base.startIndex, offsetBy: prefixLen)
        let endDate = base.index(startDate, offsetBy: 8)
        let dateStr = String(base[startDate..<endDate])

        let startTime = base.index(endDate, offsetBy: 1) // skip underscore
        let endTime = base.index(startTime, offsetBy: 6)
        let timeStr = String(base[startTime..<endTime])

        guard dateStr.allSatisfy(\.isNumber), timeStr.allSatisfy(\.isNumber) else { return nil }

        return Self.utcDateFormatter.date(from: dateStr + timeStr)
    }
}

/// nonisolated: FileCache is accessed from background queues during CSV generation.
nonisolated class FileCache {

    static let shared = FileCache()

    /// #1658: the FC's board for a flight whose summary predates
    /// `settings.board`, read from the first flight-settings frame in its log.
    /// nil when the log predates the field (v9, #413) or cannot be read. It
    /// reads the whole .bin, so call it off the main thread.
    static func boardFromLog(_ binaryURL: URL) -> String? {
        guard let frame = try? MessageParser().firstFrame(
                  ofType: MessageType.flightSettings.rawValue, in: binaryURL),
              let settings = try? FlightSettingsData(from: frame.payload)
        else { return nil }
        return settings.boardRevName
    }

    private let documentsURL: URL
    private let csvCacheURL: URL  // Documents/CSVCache/
    private let binaryCacheURL: URL  // Documents/BinaryCache/

    convenience init() {
        self.init(documentsURL: FileManager.default.urls(
            for: .documentDirectory,
            in: .userDomainMask
        )[0])
    }

    /// `documentsURL` is a seam for tests, which must not touch the app's
    /// own cache; the app uses `shared`.
    init(documentsURL: URL) {
        self.documentsURL = documentsURL

        // Create cache subdirectory paths
        csvCacheURL = documentsURL.appendingPathComponent("CSVCache")
        binaryCacheURL = documentsURL.appendingPathComponent("BinaryCache")

        // Create both cache directories if they don't exist
        try? FileManager.default.createDirectory(
            at: csvCacheURL,
            withIntermediateDirectories: true,
            attributes: nil
        )
        try? FileManager.default.createDirectory(
            at: binaryCacheURL,
            withIntermediateDirectories: true,
            attributes: nil
        )
    }

    // MARK: - Device-scoped path helpers

    /// Get the CSV cache directory for a given device (creates if needed).
    /// If unitID is empty, returns the root CSVCache directory (legacy compat).
    func csvDir(for unitID: String) -> URL {
        if unitID.isEmpty { return csvCacheURL }
        let dir = csvCacheURL.appendingPathComponent(unitID)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    /// Get the binary cache directory for a given device (creates if needed).
    func binaryDir(for unitID: String) -> URL {
        if unitID.isEmpty { return binaryCacheURL }
        let dir = binaryCacheURL.appendingPathComponent(unitID)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    /// Convert a binary filename to its CSV counterpart
    /// e.g. "flight_20260224_021306.bin" → "flight_20260224_021306.csv"
    private func csvName(for binaryFilename: String) -> String {
        if binaryFilename.hasSuffix(".bin") {
            return String(binaryFilename.dropLast(4)) + ".csv"
        }
        return binaryFilename + ".csv"
    }

    /// Convert a binary filename to its summary JSON counterpart
    /// e.g. "flight_20260224_021306.bin" → "flight_20260224_021306.json"
    private func summaryName(for binaryFilename: String) -> String {
        if binaryFilename.hasSuffix(".bin") {
            return String(binaryFilename.dropLast(4)) + ".json"
        }
        return binaryFilename + ".json"
    }

    /// Get cached CSV file URL for a given binary filename
    /// - Parameter binaryFilename: The name of the binary file (e.g., "flight_001.bin")
    /// - Returns: URL of cached CSV if it exists, nil otherwise
    func getCachedCSV(for binaryFilename: String) -> URL? {
        let csvFilename = csvName(for: binaryFilename)
        let url = csvCacheURL.appendingPathComponent(csvFilename)

        // Also check legacy .bin.csv name for backwards compatibility
        if !FileManager.default.fileExists(atPath: url.path) {
            let legacyURL = csvCacheURL.appendingPathComponent(binaryFilename + ".csv")
            return FileManager.default.fileExists(atPath: legacyURL.path) ? legacyURL : nil
        }
        return url
    }

    /// Cache a CSV file by copying it to the cache directory
    /// - Parameters:
    ///   - sourceURL: URL of the CSV file to cache
    ///   - binaryFilename: Name of the original binary file
    /// - Returns: URL of the cached CSV file
    /// - Throws: File system errors
    func cacheCSV(at sourceURL: URL, for binaryFilename: String) throws -> URL {
        let csvFilename = csvName(for: binaryFilename)
        let destURL = csvCacheURL.appendingPathComponent(csvFilename)

        // Remove existing cached file if present
        if FileManager.default.fileExists(atPath: destURL.path) {
            try FileManager.default.removeItem(at: destURL)
        }

        // Copy to cache
        try FileManager.default.copyItem(at: sourceURL, to: destURL)

        return destURL
    }

    /// Remove cached CSV for a specific binary file
    /// - Parameter binaryFilename: Name of the binary file
    func removeCachedCSV(for binaryFilename: String) {
        guard let cachedURL = getCachedCSV(for: binaryFilename) else {
            return
        }

        try? FileManager.default.removeItem(at: cachedURL)
    }

    // MARK: - Summary File Caching

    /// Get cached summary file URL for a given binary filename
    func getCachedSummary(for binaryFilename: String) -> URL? {
        let name = summaryName(for: binaryFilename)
        let url = csvCacheURL.appendingPathComponent(name)
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    /// Cache a summary file by copying it to the cache directory
    func cacheSummary(at sourceURL: URL, for binaryFilename: String) throws -> URL {
        let name = summaryName(for: binaryFilename)
        let destURL = csvCacheURL.appendingPathComponent(name)

        if FileManager.default.fileExists(atPath: destURL.path) {
            try FileManager.default.removeItem(at: destURL)
        }

        try FileManager.default.copyItem(at: sourceURL, to: destURL)
        return destURL
    }

    /// Remove cached summary for a specific binary file
    func removeCachedSummary(for binaryFilename: String) {
        guard let cachedURL = getCachedSummary(for: binaryFilename) else {
            return
        }
        try? FileManager.default.removeItem(at: cachedURL)
    }

    // MARK: - Binary File Caching

    /// Get cached binary file URL for a given binary filename
    /// - Parameter binaryFilename: The name of the binary file (e.g., "flight_001.bin")
    /// - Returns: URL of cached binary if it exists, nil otherwise
    func getCachedBinary(for binaryFilename: String) -> URL? {
        let url = binaryCacheURL.appendingPathComponent(binaryFilename)
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    /// Cache a binary file by copying it to the cache directory
    /// - Parameters:
    ///   - sourceURL: URL of the binary file to cache
    ///   - binaryFilename: Name of the binary file
    /// - Returns: URL of the cached binary file
    /// - Throws: File system errors
    func cacheBinary(at sourceURL: URL, for binaryFilename: String) throws -> URL {
        let destURL = binaryCacheURL.appendingPathComponent(binaryFilename)

        // Remove existing cached file if present
        if FileManager.default.fileExists(atPath: destURL.path) {
            try FileManager.default.removeItem(at: destURL)
        }

        // Copy to cache
        try FileManager.default.copyItem(at: sourceURL, to: destURL)

        // #832: the CSV is DERIVED from this binary, and generateAndCacheCSV
        // returns a cached CSV before it ever looks at the binary it was
        // handed.  Without this, re-downloading a file that came through
        // truncated the first time overwrote the binary and then re-served the
        // truncated CSV — the retry looked like it worked and changed nothing.
        if let staleCSV = getCachedCSV(for: binaryFilename) {
            try? FileManager.default.removeItem(at: staleCSV)
        }

        return destURL
    }

    /// Remove cached binary for a specific binary file
    /// - Parameter binaryFilename: Name of the binary file
    func removeCachedBinary(for binaryFilename: String) {
        guard let cachedURL = getCachedBinary(for: binaryFilename) else {
            return
        }

        try? FileManager.default.removeItem(at: cachedURL)
    }

    // MARK: - Base-station logs: the combined CSV + one per rocket

    /// The base station writes ONE log for every rocket it hears (each frame
    /// carries its rocket id). A log holding more than one rocket is also
    /// rendered as one CSV per rocket beside the combined one:
    /// "lora_X.csv" → "lora_X_rocket2.csv". The date parsers read only the
    /// fixed-width timestamp, so the suffix keeps the log's date.
    static func bsRocketCSVName(combined: String, rocketID: Int) -> String {
        let stem = combined.hasSuffix(".csv") ? String(combined.dropLast(4)) : combined
        return "\(stem)_rocket\(rocketID).csv"
    }

    private static let bsRocketSplitRegex = try! NSRegularExpression(
        pattern: #"^(lora_.+)_rocket([0-9]{1,3})\.csv$"#)

    /// For a per-rocket CSV's name: the combined CSV it was split from and its
    /// rocket. nil for any other name.
    static func bsRocketSplit(of name: String) -> (combined: String, rocketID: UInt8)? {
        let whole = NSRange(name.startIndex..., in: name)
        guard let m = bsRocketSplitRegex.firstMatch(in: name, range: whole),
              let stem = Range(m.range(at: 1), in: name),
              let rid = Range(m.range(at: 2), in: name),
              let id = UInt8(name[rid])
        else { return nil }
        return (String(name[stem]) + ".csv", id)
    }

    /// Render a base-station log into the CSV cache: the combined CSV under
    /// the usual name and, when the log holds more than one rocket, one CSV
    /// per rocket ID beside it. A single-rocket log gets no split — it would
    /// be the combined file again. Splits left by an earlier rendering of
    /// the same log are removed first, so a re-download cannot leave a rocket
    /// that is no longer in it. Returns the combined CSV.
    @discardableResult
    func cacheBaseStationLogCSVs(bytes: [UInt8], binaryFilename: String) throws -> URL {
        let (combined, byRocket) = try BSLogCSVGenerator.writeCSVs(bytes)
        let combinedName = csvName(for: binaryFilename)
        let combinedURL = csvCacheURL.appendingPathComponent(combinedName)
        try combined.write(to: combinedURL, atomically: true, encoding: .utf8)

        removeBaseStationSplits(ofCombined: combinedName)
        if byRocket.count > 1 {
            for (rid, csv) in byRocket {
                let url = csvCacheURL.appendingPathComponent(
                    Self.bsRocketCSVName(combined: combinedName, rocketID: rid))
                try csv.write(to: url, atomically: true, encoding: .utf8)
            }
        }
        return combinedURL
    }

    private func removeBaseStationSplits(ofCombined combinedName: String) {
        let names = (try? FileManager.default.contentsOfDirectory(atPath: csvCacheURL.path)) ?? []
        for name in names where Self.bsRocketSplit(of: name)?.combined == combinedName {
            try? FileManager.default.removeItem(at: csvCacheURL.appendingPathComponent(name))
        }
    }

    // MARK: - Direct CSV Caching (for base station LoRa logs that are already CSV)

    /// Cache a CSV file using its original filename (no .csv suffix appended)
    /// - Parameters:
    ///   - sourceURL: URL of the CSV file to cache
    ///   - filename: Original filename (e.g., "lora_001.csv")
    /// - Returns: URL of the cached file
    /// - Throws: File system errors
    func cacheDirectCSV(at sourceURL: URL, filename: String) throws -> URL {
        let destURL = csvCacheURL.appendingPathComponent(filename)

        // Remove existing cached file if present
        if FileManager.default.fileExists(atPath: destURL.path) {
            try FileManager.default.removeItem(at: destURL)
        }

        // Copy to cache
        try FileManager.default.copyItem(at: sourceURL, to: destURL)
        return destURL
    }

    /// Get cached direct CSV file URL
    /// - Parameter filename: Original filename (e.g., "lora_001.csv")
    /// - Returns: URL of cached file if it exists, nil otherwise
    func getCachedDirectCSV(_ filename: String) -> URL? {
        let url = csvCacheURL.appendingPathComponent(filename)
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    /// Check if a direct CSV file is cached
    /// - Parameter filename: Original filename (e.g., "lora_001.csv")
    /// - Returns: True if cached
    func isDirectCSVCached(_ filename: String) -> Bool {
        return getCachedDirectCSV(filename) != nil
    }

    /// #854 item 2: a direct CSV is transferred byte-for-byte, so "cached"
    /// means present AT THE SIZE THE BOARD ADVERTISES. Existence alone let a
    /// short transfer render as complete until the cache was cleared.
    func isDirectCSVCached(_ filename: String, expectedSize: UInt32) -> Bool {
        guard let url = getCachedDirectCSV(filename) else { return false }
        let attrs = try? FileManager.default.attributesOfItem(atPath: url.path)
        let cachedSize = (attrs?[.size] as? UInt64) ?? 0
        return cachedSize == UInt64(expectedSize)
    }

    // MARK: - Flight Caching Status

    /// Check if a flight is fully cached (both binary and CSV exist)
    /// - Parameter filename: Name of the binary file
    /// - Returns: True if both binary and CSV are cached
    func isFlightCached(_ filename: String) -> Bool {
        return getCachedBinary(for: filename) != nil &&
               getCachedCSV(for: filename) != nil
    }

    /// Check if a flight is cached AND matches the device-reported file size.
    /// Legacy firmware reuses sequential names (rocket_data_000.bin), so a
    /// filename match alone is not enough — the size must also match to
    /// distinguish a new flight from a previously downloaded one.
    func isFlightCached(_ filename: String, expectedSize: UInt32) -> Bool {
        guard let binaryURL = getCachedBinary(for: filename),
              getCachedCSV(for: filename) != nil else {
            return false
        }
        let attrs = try? FileManager.default.attributesOfItem(atPath: binaryURL.path)
        let cachedSize = (attrs?[.size] as? UInt64) ?? 0
        return Self.cachedSizeSatisfiesListing(cachedSize: cachedSize, listedSize: expectedSize,
                                               shortRecord: shortDownloadRecord(for: filename))
    }

    // MARK: - Board-confirmed short downloads

    private static let shortDownloadsKey = "FileCache.shortDownloads"

    /// A download the board itself ended (EOF, its byte count matching what
    /// arrived) short of the size it listed — out-computer firmware before the
    /// byte-exact stream left out stored bytes that were not part of a whole
    /// log frame. BLEDevice keeps such a file; this remembers the two sizes so
    /// the row reads as downloaded for THAT listing, instead of offering the
    /// same short download forever. A different listing (a new flight reusing
    /// the name) does not match the record.
    func recordShortDownload(_ filename: String, listedSize: UInt32, receivedSize: UInt32) {
        var all = UserDefaults.standard.dictionary(forKey: Self.shortDownloadsKey) as? [String: [Int]] ?? [:]
        all[filename] = [Int(listedSize), Int(receivedSize)]
        UserDefaults.standard.set(all, forKey: Self.shortDownloadsKey)
    }

    func shortDownloadRecord(for filename: String) -> (listed: UInt32, received: UInt32)? {
        guard let all = UserDefaults.standard.dictionary(forKey: Self.shortDownloadsKey) as? [String: [Int]],
              let r = all[filename], r.count == 2, r[0] >= 0, r[1] >= 0 else { return nil }
        return (UInt32(r[0]), UInt32(r[1]))
    }

    /// The cached binary counts as this listing's file when it is the listed
    /// size, or when the board confirmed a short download of exactly this
    /// listing and the cached file is what it sent.
    static func cachedSizeSatisfiesListing(cachedSize: UInt64, listedSize: UInt32,
                                           shortRecord: (listed: UInt32, received: UInt32)?) -> Bool {
        if cachedSize == UInt64(listedSize) { return true }
        guard let r = shortRecord else { return false }
        return r.listed == listedSize && UInt64(r.received) == cachedSize
    }

    /// Clear all cached CSV files
    func clearAllCache() {
        guard let fileURLs = try? FileManager.default.contentsOfDirectory(
            at: csvCacheURL,
            includingPropertiesForKeys: nil
        ) else {
            return
        }

        for fileURL in fileURLs {
            try? FileManager.default.removeItem(at: fileURL)
        }
    }

    /// Get total size of all cached CSV files in bytes
    func getCacheSize() -> Int64 {
        guard let fileURLs = try? FileManager.default.contentsOfDirectory(
            at: csvCacheURL,
            includingPropertiesForKeys: [.fileSizeKey]
        ) else {
            return 0
        }

        var totalSize: Int64 = 0
        for fileURL in fileURLs {
            guard let resourceValues = try? fileURL.resourceValues(forKeys: [.fileSizeKey]),
                  let fileSize = resourceValues.fileSize else {
                continue
            }
            totalSize += Int64(fileSize)
        }

        return totalSize
    }

    /// Get number of cached CSV files
    func getCachedFileCount() -> Int {
        guard let fileURLs = try? FileManager.default.contentsOfDirectory(
            at: csvCacheURL,
            includingPropertiesForKeys: nil
        ) else {
            return 0
        }

        return fileURLs.count
    }

    // MARK: - Delete Cached Flight

    /// Delete all cached files for a given flight (CSV + binary + summary if present)
    func deleteCachedFlight(_ flight: CachedFlight) {
        // Remove CSV file
        try? FileManager.default.removeItem(at: flight.csvURL)

        // One rocket's CSV of a shared base-station log: the .bin (and the
        // combined CSV) belong to every rocket in it — leave them.
        if flight.bsLogRocketID != nil { return }
        // The combined log takes its per-rocket CSVs with it.
        if flight.type == .loraLog {
            removeBaseStationSplits(ofCombined: flight.name)
        }

        // Remove binary file if present (rocket flights, and base-station logs)
        if let binaryURL = flight.binaryURL {
            try? FileManager.default.removeItem(at: binaryURL)
        }

        // Remove summary file if present
        if let summaryURL = flight.summaryURL {
            try? FileManager.default.removeItem(at: summaryURL)
        }
    }

    // MARK: - List All Cached Flights

    /// Enumerate cached CSV files and return structured results for the Flight Logs page
    func listCachedFlights() -> [CachedFlight] {
        guard let csvFiles = try? FileManager.default.contentsOfDirectory(
            at: csvCacheURL,
            includingPropertiesForKeys: [.fileSizeKey, .contentModificationDateKey]
        ) else {
            return []
        }

        var results: [CachedFlight] = []

        for csvURL in csvFiles {
            let filename = csvURL.lastPathComponent

            // Skip summary JSON files — they are associated with flights, not standalone
            if filename.hasSuffix(".json") { continue }

            // Determine type by filename pattern
            let type: CachedFlightType
            var binaryURL: URL? = nil
            var summaryURL: URL? = nil
            var split: (combined: String, rocketID: UInt8)? = nil

            if filename.hasPrefix("flight_") && (filename.hasSuffix(".csv") || filename.hasSuffix(".bin.csv")) && !filename.hasPrefix("lora_") {
                type = .rocket
                // Check for matching binary file
                let binName: String
                let baseName: String
                if filename.hasSuffix(".bin.csv") {
                    binName = String(filename.dropLast(4)) // legacy: "flight_xxx.bin.csv" → "flight_xxx.bin"
                    baseName = String(filename.dropLast(8)) // "flight_xxx"
                } else {
                    binName = String(filename.dropLast(4)) + ".bin" // new: "flight_xxx.csv" → "flight_xxx.bin"
                    baseName = String(filename.dropLast(4)) // "flight_xxx"
                }
                let binURL = binaryCacheURL.appendingPathComponent(binName)
                if FileManager.default.fileExists(atPath: binURL.path) {
                    binaryURL = binURL
                }
                // Check for matching summary JSON file
                let jsonURL = csvCacheURL.appendingPathComponent(baseName + ".json")
                if FileManager.default.fileExists(atPath: jsonURL.path) {
                    summaryURL = jsonURL
                }
            } else if filename.hasPrefix("lora_") && filename.hasSuffix(".csv") {
                type = .loraLog
                // #850: base-station logs are binary now and the CSV beside
                // them is a rendering. Associate the raw .bin so it rides the
                // Share sheet — the post-flight tools read the binary directly,
                // the same way they read the rocket computer's log, and a CSV
                // cannot be converted back into one. A per-rocket CSV shares
                // the combined log's .bin.
                split = Self.bsRocketSplit(of: filename)
                let combined = split?.combined ?? filename
                let binURL = binaryCacheURL
                    .appendingPathComponent(String(combined.dropLast(4)) + ".bin")
                if FileManager.default.fileExists(atPath: binURL.path) {
                    binaryURL = binURL
                }
            } else {
                // Unknown file pattern — skip
                continue
            }

            // Get file attributes. #1091 item 1: the displayed size is the
            // .bin's — what came off the board — falling back to the CSV
            // only when the binary is gone, matching Android's
            // FlightCache.listSavedFlights. It used to be the CSV rendering,
            // 5-8x larger, so the same flight read as a different size on
            // each phone.
            let resourceValues = try? csvURL.resourceValues(forKeys: [.fileSizeKey, .contentModificationDateKey])
            // A per-rocket CSV shows its own size: the shared .bin's would
            // read as if each rocket had the whole log.
            let binSize = split == nil
                ? binaryURL.flatMap { try? $0.resourceValues(forKeys: [.fileSizeKey]).fileSize }
                : nil
            let size = UInt64(binSize ?? resourceValues?.fileSize ?? 0)
            let modDate = resourceValues?.contentModificationDate

            // Parse flight summary JSON if available
            var maxAlt: Double? = nil
            var maxSpd: Double? = nil
            var burnoutT: Double? = nil
            var apogeeT: Double? = nil
            // #1077: decode the four headline numbers on their own. The
            // nested `settings` block has grown fields over time, and a
            // sidecar written before a field existed used to take the whole
            // decode down with it — so every flight cached between
            // 2026-05-22 and 07-27 showed no altitude, speed or apogee.
            // The headline struct never touches `settings`.
            if let sURL = summaryURL,
               let data = try? Data(contentsOf: sURL),
               let head = try? JSONDecoder().decode(FlightSummaryHeadline.self, from: data) {
                maxAlt = head.max_altitude_m
                maxSpd = head.max_speed_mps
                burnoutT = head.burnout_time_s
                apogeeT = head.apogee_time_s
            }
            let identity = summaryURL
                .flatMap { try? Data(contentsOf: $0) }
                .flatMap { try? JSONDecoder().decode(FlightSummaryIdentity.self, from: $0) }

            results.append(CachedFlight(
                name: filename,
                type: type,
                csvURL: csvURL,
                binaryURL: binaryURL,
                summaryURL: summaryURL,
                size: size,
                modificationDate: modDate,
                maxAltitudeM: maxAlt,
                maxSpeedMps: maxSpd,
                burnoutTimeS: burnoutT,
                apogeeTimeS: apogeeT,
                bsLogRocketID: split?.rocketID,
                source: identity?.source,
                fcBoard: identity?.settings?.board
            ))
        }

        // Newest first by modification date, with each base-station log's
        // per-rocket CSVs right under it in rocket order (they are written a
        // moment after the combined one, so their own dates would put them
        // above it).
        let dates = Dictionary(results.map { ($0.name, $0.modificationDate ?? .distantPast) },
                               uniquingKeysWith: { a, _ in a })
        func group(_ f: CachedFlight) -> (Date, String, Int) {
            guard let rid = f.bsLogRocketID,
                  let combined = Self.bsRocketSplit(of: f.name)?.combined
            else { return (dates[f.name] ?? .distantPast, f.name, -1) }
            return (dates[combined] ?? dates[f.name] ?? .distantPast, combined, Int(rid))
        }
        results.sort { a, b in
            let (da, ga, ra) = group(a), (db, gb, rb) = group(b)
            if da != db { return da > db }
            if ga != gb { return ga < gb }
            return ra < rb
        }

        return results
    }
}
