//
//  FileCache.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Manage cached CSV files in app's documents directory
//

import Foundation

// MARK: - CachedFlight Model

enum CachedFlightType {
    case rocket
    case loraLog
}

struct CachedFlight: Identifiable {
    let name: String          // Original filename in cache (e.g., "flight_20260224_021306.csv")
    let type: CachedFlightType
    let csvURL: URL
    let binaryURL: URL?       // Only for rocket flights
    let summaryURL: URL?      // Flight summary JSON (rocket flights only)
    let size: UInt64
    let modificationDate: Date?
    let maxAltitudeM: Double? // From flight summary JSON
    let maxSpeedMps: Double?  // From flight summary JSON
    let burnoutTimeS: Double? // From flight summary JSON (seconds after launch)
    let apogeeTimeS: Double?  // From flight summary JSON (seconds after launch)

    var id: String { name }

    // Reusable formatters — DateFormatter is expensive to allocate
    private static let utcDateFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyyMMddHHmmss"
        f.timeZone = TimeZone(identifier: "UTC")
        return f
    }()

    private static let localDisplayFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateStyle = .short
        f.timeStyle = .short
        f.timeZone = .current
        return f
    }()

    /// Friendly display title — localized date/time for flights with timestamps, raw name otherwise
    var displayTitle: String {
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

    private let documentsURL: URL
    private let csvCacheURL: URL  // Documents/CSVCache/
    private let binaryCacheURL: URL  // Documents/BinaryCache/

    init() {
        // Get documents directory
        documentsURL = FileManager.default.urls(
            for: .documentDirectory,
            in: .userDomainMask
        )[0]

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
        return cachedSize == UInt64(expectedSize)
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

        // Remove binary file if present (rocket flights)
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
            } else {
                // Unknown file pattern — skip
                continue
            }

            // Get file attributes
            let resourceValues = try? csvURL.resourceValues(forKeys: [.fileSizeKey, .contentModificationDateKey])
            let size = UInt64(resourceValues?.fileSize ?? 0)
            let modDate = resourceValues?.contentModificationDate

            // Parse flight summary JSON if available
            var maxAlt: Double? = nil
            var maxSpd: Double? = nil
            var burnoutT: Double? = nil
            var apogeeT: Double? = nil
            if let sURL = summaryURL,
               let data = try? Data(contentsOf: sURL),
               let summary = try? JSONDecoder().decode(FlightSummary.self, from: data) {
                maxAlt = summary.max_altitude_m
                maxSpd = summary.max_speed_mps
                burnoutT = summary.burnout_time_s
                apogeeT = summary.apogee_time_s
            }

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
                apogeeTimeS: apogeeT
            ))
        }

        // Sort newest first by modification date
        results.sort { ($0.modificationDate ?? .distantPast) > ($1.modificationDate ?? .distantPast) }

        return results
    }
}
