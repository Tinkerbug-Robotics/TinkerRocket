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

    // ESP32 already excludes recovery files from the BLE file list
    private var displayedFiles: [FileInfo] {
        device.files
    }

    var body: some View {
        VStack(spacing: 20) {
            if !device.isConnected {
                Text("Not connected")
                    .foregroundColor(.secondary)
                    .padding()
            } else {
                // Header with refresh button
                HStack {
                    VStack(alignment: .leading) {
                        Text(device.isBaseStation ? "LoRa Logs" : "Flights")
                            .font(.headline)
                        if device.currentPage > 0 || device.hasMoreFiles {
                            Text("Page \(device.currentPage + 1)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        } else {
                            Text("\(displayedFiles.count) \(device.isBaseStation ? "logs" : "flights")")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    Spacer()
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

                    // Pagination controls
                    if device.currentPage > 0 || device.hasMoreFiles {
                        HStack(spacing: 20) {
                            Button(action: {
                                device.previousPage()
                            }) {
                                HStack {
                                    Image(systemName: "chevron.left")
                                    Text("Previous")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(device.currentPage > 0 ? Color.blue : Color.gray)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(device.currentPage == 0)

                            Spacer()

                            Button(action: {
                                device.nextPage()
                            }) {
                                HStack {
                                    Text("Next")
                                    Image(systemName: "chevron.right")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(device.hasMoreFiles ? Color.blue : Color.gray)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(!device.hasMoreFiles)
                        }
                        .padding(.horizontal)
                        .padding(.bottom, 10)
                    }
                }
            }
        }
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
}

struct FileRow: View {
    let file: FileInfo
    let onDelete: () -> Void
    let onDownload: () -> Void
    @ObservedObject var device: BLEDevice

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                Text(file.displayTitle)
                    .font(.body)

                Text(formatFileSize(file.size))
                    .font(.caption)
                    .foregroundColor(.secondary)
            }

            Spacer()

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
        .padding(.vertical, 4)
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
