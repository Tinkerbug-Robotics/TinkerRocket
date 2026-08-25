//
//  FileManagerView.swift
//  TinkerRocketApp
//
//  File browser for rocket data files
//

import SwiftUI
import UIKit
import UniformTypeIdentifiers

struct FileManagerView: View {
    @ObservedObject var device: BLEDevice
    @State private var fileToDelete: FileInfo?
    @State private var showDeleteConfirm = false

    // Multi-select for bulk delete. `selection` holds file names (FileInfo.id),
    // so it survives paging — you can select across pages and the count in the
    // action bar reflects the full set even when some aren't on the visible page.
    @State private var isSelecting = false
    @State private var selection: Set<String> = []
    @State private var showBulkDeleteConfirm = false

    // ESP32 already excludes recovery files from the BLE file list
    private var displayedFiles: [FileInfo] {
        device.files
    }

    // True when every file on the current page is selected (drives Select-all vs
    // Deselect-all). False for an empty page so the toggle reads "Select all".
    private var allOnPageSelected: Bool {
        !displayedFiles.isEmpty && displayedFiles.allSatisfy { selection.contains($0.name) }
    }

    // Both firmwares paginate the BLE file list at 5 entries/page
    // (FILES_PER_PAGE in out_computer + base_station config).
    private static let filesPerPage = 5

    /// Authoritative total page count when we know the full file count, else nil.
    /// The rocket's storage stats report `flightCount` (== the flight-log index
    /// size, which is exactly what the file list paginates over), so we can show
    /// a fixed numbered navigator. The base station reports no count, so its
    /// navigator is discovered page-by-page instead (see FilePageNavigator).
    private var totalFilePages: Int? {
        guard !device.isBaseStation,
              let s = device.rocketStorage, s.initialized else { return nil }
        return FilePageNavigator.totalPages(forFileCount: s.flightCount,
                                            pageSize: Self.filesPerPage)
    }

    /// Show the navigator whenever there's more than one page. With a known
    /// total this also hides it (and the phantom "next") for an exactly-full
    /// single page, which the count==pageSize "hasMore" heuristic can't.
    private var showPagination: Bool {
        if let total = totalFilePages { return total > 1 }
        return device.currentPage > 0 || device.hasMoreFiles
    }

    var body: some View {
        VStack(spacing: 20) {
            if !device.isConnected {
                Text("Not connected")
                    .foregroundColor(.secondary)
                    .padding()
            } else {
                // Flash-space usage for this device's log storage (rocket NAND
                // on a rocket link, base-station filesystem on a BS link).
                StorageBarView(device: device)

                // Header with refresh button
                HStack {
                    VStack(alignment: .leading) {
                        Text(device.isBaseStation ? "LoRa Logs" : "Flights")
                            .font(.headline)
                        if let total = totalFilePages, total > 1 {
                            Text("Page \(Int(device.currentPage) + 1) of \(total)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        } else if device.currentPage > 0 || device.hasMoreFiles {
                            Text("Page \(Int(device.currentPage) + 1)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        } else {
                            Text("\(displayedFiles.count) \(device.isBaseStation ? "logs" : "flights")")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    Spacer()
                    if isSelecting {
                        Button("Cancel") { exitSelection() }
                            .foregroundColor(.blue)
                    } else {
                        if !displayedFiles.isEmpty {
                            Button("Select") { isSelecting = true }
                                .foregroundColor(.blue)
                                .padding(.trailing, 4)
                        }
                        Button(action: {
                            print("Requesting file list...")
                            device.requestFileList(page: device.currentPage)
                        }) {
                            HStack {
                                Image(systemName: "arrow.clockwise")
                                Text("Refresh")
                            }
                            .padding(.horizontal, 12)
                            .padding(.vertical, 6)
                            .background(Color.blue)
                            .foregroundColor(.white)
                            .cornerRadius(8)
                        }
                    }
                }
                .padding()

                // Download progress
                if device.isDownloading {
                    VStack(spacing: 8) {
                        HStack {
                            Text("Downloading \(device.downloadingFilename ?? "file")...")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            Spacer()
                            Text("\(Int(device.downloadProgress * 100))%")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                        }
                        ProgressView(value: device.downloadProgress)
                            .progressViewStyle(LinearProgressViewStyle())
                    }
                    .padding(.horizontal)
                    .padding(.bottom, 8)
                }

                // CSV generation progress
                if device.downloadStates.values.contains(.generatingCSV) {
                    VStack(spacing: 8) {
                        HStack {
                            Text("Generating CSV...")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            Spacer()
                            Text("\(Int(device.csvGenerationProgress * 100))%")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                        }
                        ProgressView(value: device.csvGenerationProgress)
                            .progressViewStyle(LinearProgressViewStyle())
                            .tint(.orange)
                    }
                    .padding(.horizontal)
                    .padding(.bottom, 8)
                }

                // A CSV that could not be generated must SAY so. The bytes are
                // cached either way, so the data is safe — but silence here
                // reads as success, and the file then simply never appears in
                // Saved Files (which lists the CSV cache, not the .bin cache).
                if let csvError = device.csvGenerationError {
                    HStack(alignment: .top, spacing: 8) {
                        Image(systemName: "exclamationmark.triangle.fill")
                            .foregroundColor(.orange)
                        VStack(alignment: .leading, spacing: 2) {
                            Text(csvError)
                                .font(.caption)
                            Text("The .bin was saved — only the CSV step failed.")
                                .font(.caption2)
                                .foregroundColor(.secondary)
                        }
                        Spacer()
                    }
                    .padding(.horizontal)
                    .padding(.bottom, 8)
                }

                // File list
                if displayedFiles.isEmpty {
                    VStack(spacing: 10) {
                        Image(systemName: "doc.text")
                            .font(.system(size: 48))
                            .foregroundColor(.secondary)
                        Text(device.isBaseStation ? "No LoRa logs found" : "No flights found")
                            .foregroundColor(.secondary)
                        Text("Tap Refresh to load files")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                    .padding()
                } else {
                    List {
                        ForEach(displayedFiles) { file in
                            if isSelecting {
                                Button {
                                    toggleSelection(file.name)
                                } label: {
                                    FileRow(file: file, onDelete: {}, onDownload: {},
                                            device: device, selectionMode: true,
                                            isSelected: selection.contains(file.name))
                                }
                                .buttonStyle(.plain)
                            } else {
                                FileRow(
                                    file: file,
                                    onDelete: {
                                        fileToDelete = file
                                        showDeleteConfirm = true
                                    },
                                    onDownload: {
                                        print("Download button tapped for: \(file.name)")

                                        device.downloadAndCacheFlight(file.name) { success in
                                            if success {
                                                print("[DOWNLOAD] Success: \(file.name)")
                                            } else {
                                                print("[DOWNLOAD] Failed: \(file.name)")
                                            }
                                        }
                                    },
                                    device: device
                                )
                            }
                        }
                    }
                    .listStyle(InsetGroupedListStyle())
                    .alert(device.isBaseStation ? "Delete LoRa Log?" : "Delete Flight?",
                           isPresented: $showDeleteConfirm) {
                        Button("Delete", role: .destructive) {
                            if let file = fileToDelete {
                                device.deleteFile(file.name)
                            }
                            fileToDelete = nil
                        }
                        Button("Cancel", role: .cancel) {
                            fileToDelete = nil
                        }
                    } message: {
                        if let file = fileToDelete {
                            Text("Are you sure you want to delete \"\(file.displayTitle)\" from the \(device.isBaseStation ? "base station" : "rocket")? This cannot be undone.")
                        }
                    }
                    .alert("Delete \(selection.count) \(device.isBaseStation ? "LoRa Log" : "Flight")\(selection.count == 1 ? "" : "s")?",
                           isPresented: $showBulkDeleteConfirm) {
                        Button("Delete", role: .destructive) { deleteSelected() }
                        Button("Cancel", role: .cancel) {}
                    } message: {
                        Text("Are you sure you want to delete \(selection.count) \(device.isBaseStation ? "log" : "flight")\(selection.count == 1 ? "" : "s") from the \(device.isBaseStation ? "base station" : "rocket")? This cannot be undone.")
                    }

                    // Pagination — numbered page navigator so many saved files
                    // are easy to jump through (not just Prev/Next one at a time).
                    if showPagination {
                        FilePageNavigator(
                            currentPage: Int(device.currentPage),
                            totalPages: totalFilePages,
                            hasMore: device.hasMoreFiles,
                            onSelect: { device.requestFileList(page: UInt8(clamping: $0)) }
                        )
                        .padding(.horizontal)
                        .padding(.bottom, isSelecting ? 4 : 10)
                    }

                    // Multi-select action bar.
                    if isSelecting {
                        HStack {
                            Button(allOnPageSelected ? "Deselect all" : "Select all") {
                                if allOnPageSelected {
                                    displayedFiles.forEach { selection.remove($0.name) }
                                } else {
                                    displayedFiles.forEach { selection.insert($0.name) }
                                }
                            }
                            .foregroundColor(.blue)

                            Spacer()

                            Button(action: { showBulkDeleteConfirm = true }) {
                                HStack(spacing: 6) {
                                    Image(systemName: "trash")
                                    Text("Delete\(selection.isEmpty ? "" : " (\(selection.count))")")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(selection.isEmpty ? Color.gray : Color.red)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(selection.isEmpty)
                        }
                        .padding(.horizontal)
                        .padding(.bottom, 10)
                    }
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
        .navigationTitle(device.isBaseStation ? "LoRa Logs" : "Flights")
        .brandedToolbar()
        .onAppear {
            // Auto-load file list when view appears
            print("FileManagerView appeared, connected: \(device.isConnected)")
            if device.isConnected {
                print("Auto-requesting file list...")
                device.requestFileList()
            }
        }
        .onChange(of: device.files) { files in
            // Rebuild download states from cache when file list updates
            device.rebuildDownloadStates(for: files.map { $0.name })
        }
    }

    private func toggleSelection(_ name: String) {
        if selection.contains(name) { selection.remove(name) } else { selection.insert(name) }
    }

    private func exitSelection() {
        isSelecting = false
        selection.removeAll()
    }

    private func deleteSelected() {
        let names = Array(selection)
        device.deleteFiles(names)
        exitSelection()
        // Resync from the device: a bulk delete can empty the current page or
        // shift the total, so jump back to the first page.
        device.requestFileList(page: 0)
    }
}

// Numbered page bar for the saved-files list. Tapping a number jumps straight
// to that page; chevrons step one at a time. Two modes:
//   • totalPages known (rocket, from flightCount) → fixed pills 1…N, so you can
//     jump to any page including the last.
//   • totalPages nil (base station reports no count) → pills are discovered as
//     you page forward (1…current, plus a "…" while more remain).
// The row scrolls horizontally and keeps the current page centered.
struct FilePageNavigator: View {
    let currentPage: Int          // 0-based
    let totalPages: Int?          // nil when the device reports no total
    let hasMore: Bool             // used only in the discovered (nil-total) mode
    let onSelect: (Int) -> Void

    // 0-based page indices to render.
    private var pageIndices: [Int] {
        Self.pageIndices(currentPage: currentPage, totalPages: totalPages, hasMore: hasMore)
    }

    private var canGoNext: Bool {
        Self.canGoNext(currentPage: currentPage, totalPages: totalPages, hasMore: hasMore)
    }

    // --- Pure paging logic (unit-tested; see FilePageNavigatorTests) ---

    /// Total pages for a known file count, paginated at `pageSize`. At least 1.
    static func totalPages(forFileCount count: Int, pageSize: Int) -> Int {
        guard pageSize > 0 else { return 1 }
        return max(1, (count + pageSize - 1) / pageSize)
    }

    static func pageIndices(currentPage: Int, totalPages: Int?, hasMore: Bool) -> [Int] {
        if let total = totalPages, total > 0 { return Array(0..<total) }
        let last = currentPage + (hasMore ? 1 : 0)
        return Array(0...max(0, last))
    }

    static func canGoNext(currentPage: Int, totalPages: Int?, hasMore: Bool) -> Bool {
        if let total = totalPages { return currentPage < total - 1 }
        return hasMore
    }

    var body: some View {
        HStack(spacing: 12) {
            chevron("chevron.left", enabled: currentPage > 0) {
                onSelect(currentPage - 1)
            }

            ScrollViewReader { proxy in
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack(spacing: 8) {
                        ForEach(pageIndices, id: \.self) { p in
                            pagePill(p).id(p)
                        }
                        if totalPages == nil && hasMore {
                            Text("…")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                                .padding(.horizontal, 2)
                        }
                    }
                    .padding(.horizontal, 2)
                }
                .onChange(of: currentPage) { p in
                    withAnimation { proxy.scrollTo(p, anchor: .center) }
                }
                .onAppear { proxy.scrollTo(currentPage, anchor: .center) }
            }

            chevron("chevron.right", enabled: canGoNext) {
                onSelect(currentPage + 1)
            }
        }
    }

    private func pagePill(_ p: Int) -> some View {
        let isCurrent = (p == currentPage)
        return Button {
            if !isCurrent { onSelect(p) }
        } label: {
            Text("\(p + 1)")
                .font(.subheadline.weight(isCurrent ? .semibold : .regular))
                .frame(minWidth: 34, minHeight: 34)
                .background(isCurrent ? Color.blue : Color(.systemGray5))
                .foregroundColor(isCurrent ? .white : .primary)
                .clipShape(Circle())
        }
        .disabled(isCurrent)
        .accessibilityLabel("Page \(p + 1)")
    }

    private func chevron(_ system: String, enabled: Bool,
                         action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: system)
                .frame(width: 34, height: 34)
                .background(enabled ? Color.blue : Color(.systemGray4))
                .foregroundColor(.white)
                .clipShape(Circle())
        }
        .disabled(!enabled)
    }
}

struct FileRow: View {
    let file: FileInfo
    let onDelete: () -> Void
    let onDownload: () -> Void
    @ObservedObject var device: BLEDevice
    // In selection mode the per-row Download/Delete buttons give way to a leading
    // checkmark and the whole row acts as a selection toggle (handled by the
    // caller wrapping this in a Button).
    var selectionMode: Bool = false
    var isSelected: Bool = false

    var body: some View {
        HStack {
            if selectionMode {
                Image(systemName: isSelected ? "checkmark.circle.fill" : "circle")
                    .font(.title3)
                    .foregroundColor(isSelected ? .blue : .secondary)
            }

            VStack(alignment: .leading, spacing: 4) {
                Text(file.displayTitle)
                    .font(.body)

                Text(formatFileSize(file.size))
                    .font(.caption)
                    .foregroundColor(.secondary)
            }

            Spacer()

            if !selectionMode {
                // Download button
                Button(action: onDownload) {
                    downloadButtonContent
                }
                .buttonStyle(BorderlessButtonStyle())
                .disabled(downloadButtonDisabled)

                // Delete button
                Button(action: onDelete) {
                    Image(systemName: "trash")
                        .foregroundColor(.red)
                }
                .buttonStyle(BorderlessButtonStyle())
            }
        }
        .padding(.vertical, 4)
        .contentShape(Rectangle())
    }

    // Download button content based on state
    @ViewBuilder
    private var downloadButtonContent: some View {
        let state = device.getDownloadState(for: file.name)

        switch state {
        case .notDownloaded:
            Image(systemName: "arrow.down.circle")
                .foregroundColor(.blue)
        case .downloading:
            ProgressView()
                .progressViewStyle(CircularProgressViewStyle())
        case .generatingCSV:
            ProgressView()
                .progressViewStyle(CircularProgressViewStyle())
                .tint(.orange)
        case .completed:
            Image(systemName: "checkmark.circle.fill")
                .foregroundColor(.green)
        case .failed:
            Image(systemName: "exclamationmark.circle")
                .foregroundColor(.red)
        }
    }

    private var downloadButtonDisabled: Bool {
        if !device.isConnected { return true }
        let state = device.getDownloadState(for: file.name)
        return state == .downloading || state == .generatingCSV || state == .completed
    }

    private func formatFileSize(_ bytes: UInt32) -> String {
        let kb = Double(bytes) / 1024.0
        if kb < 1024 {
            return String(format: "%.1f KB", kb)
        } else {
            let mb = kb / 1024.0
            return String(format: "%.2f MB", mb)
        }
    }
}

// Preview requires a connected BLEDevice — omitted for now.

// MARK: - Share Helper (presents UIActivityViewController via UIKit to avoid SwiftUI .sheet blank screen)

struct ShareHelper {
    /// Share items. File URLs (usually from Documents/CSVCache or
    /// Documents/BinaryCache) are first copied into the app's
    /// temporaryDirectory — LaunchServices can't reliably access
    /// app-private cache subdirectories (Code=256 / Code=-10814), which
    /// otherwise stalls the share sheet and spams the log. See #67.
    ///
    /// Each URL is then wrapped in a `FlightFileItemSource` that:
    ///  - declares an explicit UTType (skips LaunchServices'
    ///    extension-based type inference)
    ///  - returns a nil Quick Look thumbnail (suppresses preview
    ///    generation for the multi-MB CSV, which dominates first-share
    ///    latency after a fresh install)
    ///  - supplies a Mail-friendly subject derived from the filename
    ///
    /// Non-URL items pass through untouched.
    static func share(items: [Any]) {
        guard let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
              let rootVC = windowScene.windows.first?.rootViewController else { return }

        // Walk up to the topmost presented controller
        var topVC = rootVC
        while let presented = topVC.presentedViewController {
            topVC = presented
        }

        let preparedItems: [Any] = items.map { item -> Any in
            guard let url = item as? URL, url.isFileURL else { return item }
            let shareURL = copyToTemp(url) ?? url
            return FlightFileItemSource(fileURL: shareURL,
                                        utType: utType(for: shareURL.pathExtension))
        }

        let activityVC = UIActivityViewController(
            activityItems: preparedItems,
            applicationActivities: nil
        )
        topVC.present(activityVC, animated: true)
    }

    /// Copy a source file URL into the app's temporaryDirectory, preserving
    /// filename + extension. Returns the temp URL on success, or nil on
    /// failure (in which case callers fall back to the original URL).
    private static func copyToTemp(_ src: URL) -> URL? {
        let fm = FileManager.default
        let dst = fm.temporaryDirectory.appendingPathComponent(src.lastPathComponent)
        do {
            if fm.fileExists(atPath: dst.path) {
                try fm.removeItem(at: dst)
            }
            try fm.copyItem(at: src, to: dst)
            return dst
        } catch {
            print("ShareHelper.copyToTemp failed for \(src.lastPathComponent): \(error)")
            return nil
        }
    }

    /// Map a filename extension to a UTType. Declared explicitly for the
    /// flight-data file types so LaunchServices doesn't need to infer them.
    private static func utType(for pathExtension: String) -> UTType {
        switch pathExtension.lowercased() {
        case "csv":  return .commaSeparatedText
        case "json": return .json
        case "bin":  return .data
        default:     return UTType(filenameExtension: pathExtension) ?? .data
        }
    }
}

/// UIActivityItemSource wrapper used by ShareHelper. See the doc comment on
/// ShareHelper.share for rationale. Minimal overrides — just enough to
/// declare a UTType up front and suppress the Quick Look thumbnail that
/// otherwise blocks first-share latency while iOS renders the full CSV.
final class FlightFileItemSource: NSObject, UIActivityItemSource {
    let fileURL: URL
    let declaredUTType: UTType

    init(fileURL: URL, utType: UTType) {
        self.fileURL = fileURL
        self.declaredUTType = utType
    }

    // Placeholder shown while the activity picker is being built. Returning
    // the file URL lets the picker know what kind of thing is being shared
    // without triggering a full file read.
    func activityViewControllerPlaceholderItem(_ activityViewController: UIActivityViewController) -> Any {
        return fileURL
    }

    // Actual item once an activity is selected.
    func activityViewController(_ activityViewController: UIActivityViewController,
                                itemForActivityType activityType: UIActivity.ActivityType?) -> Any? {
        return fileURL
    }

    // Pre-declared UTI — skips LaunchServices' extension-based inference,
    // which is what's logging the Code=-10814 noise.
    func activityViewController(_ activityViewController: UIActivityViewController,
                                dataTypeIdentifierForActivityType activityType: UIActivity.ActivityType?) -> String {
        return declaredUTType.identifier
    }

    // Default email subject — Mail uses this when the share destination is Mail.
    func activityViewController(_ activityViewController: UIActivityViewController,
                                subjectForActivityType activityType: UIActivity.ActivityType?) -> String {
        return fileURL.deletingPathExtension().lastPathComponent
    }

    // Return nil thumbnail — iOS otherwise reads + renders the multi-MB CSV
    // for a preview, which is the bulk of first-share latency.
    func activityViewController(_ activityViewController: UIActivityViewController,
                                thumbnailImageForActivityType activityType: UIActivity.ActivityType?,
                                suggestedSize size: CGSize) -> UIImage? {
        return nil
    }
}
