//
//  FileManagerView.swift
//  TinkerRocketApp
//
//  File browser for rocket data files
//

import SwiftUI

struct FileManagerView: View {
    @ObservedObject var bleManager: BLEManager
    @State private var fileToDelete: FileInfo?
    @State private var showDeleteConfirm = false

    // ESP32 already excludes recovery files from the BLE file list
    private var displayedFiles: [FileInfo] {
        bleManager.files
    }

    var body: some View {
        VStack(spacing: 20) {
            if !bleManager.isConnected {
                Text("Not connected")
                    .foregroundColor(.secondary)
                    .padding()
            } else {
                // Header with refresh button
                HStack {
                    VStack(alignment: .leading) {
                        Text(bleManager.isBaseStation ? "LoRa Logs" : "Flights")
                            .font(.headline)
                        if bleManager.currentPage > 0 || bleManager.hasMoreFiles {
                            Text("Page \(bleManager.currentPage + 1)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        } else {
                            Text("\(displayedFiles.count) \(bleManager.isBaseStation ? "logs" : "flights")")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    Spacer()
                    Button(action: {
                        print("Requesting file list...")
                        bleManager.requestFileList(page: bleManager.currentPage)
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
                if bleManager.isDownloading {
                    VStack(spacing: 8) {
                        HStack {
                            Text("Downloading \(bleManager.downloadingFilename ?? "file")...")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            Spacer()
                            Text("\(Int(bleManager.downloadProgress * 100))%")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                        }
                        ProgressView(value: bleManager.downloadProgress)
                            .progressViewStyle(LinearProgressViewStyle())
                    }
                    .padding(.horizontal)
                    .padding(.bottom, 8)
                }

                // CSV generation progress
                if bleManager.downloadStates.values.contains(.generatingCSV) {
                    VStack(spacing: 8) {
                        HStack {
                            Text("Generating CSV...")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            Spacer()
                            Text("\(Int(bleManager.csvGenerationProgress * 100))%")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                        }
                        ProgressView(value: bleManager.csvGenerationProgress)
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
                        Text(bleManager.isBaseStation ? "No LoRa logs found" : "No flights found")
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

                                    bleManager.downloadAndCacheFlight(file.name) { success in
                                        if success {
                                            print("[DOWNLOAD] Success: \(file.name)")
                                        } else {
                                            print("[DOWNLOAD] Failed: \(file.name)")
                                        }
                                    }
                                },
                                bleManager: bleManager
                            )
                        }
                    }
                    .listStyle(InsetGroupedListStyle())
                    .alert(bleManager.isBaseStation ? "Delete LoRa Log?" : "Delete Flight?",
                           isPresented: $showDeleteConfirm) {
                        Button("Delete", role: .destructive) {
                            if let file = fileToDelete {
                                bleManager.deleteFile(file.name)
                            }
                            fileToDelete = nil
                        }
                        Button("Cancel", role: .cancel) {
                            fileToDelete = nil
                        }
                    } message: {
                        if let file = fileToDelete {
                            Text("Are you sure you want to delete \"\(file.displayTitle)\" from the \(bleManager.isBaseStation ? "base station" : "rocket")? This cannot be undone.")
                        }
                    }

                    // Pagination controls
                    if bleManager.currentPage > 0 || bleManager.hasMoreFiles {
                        HStack(spacing: 20) {
                            Button(action: {
                                bleManager.previousPage()
                            }) {
                                HStack {
                                    Image(systemName: "chevron.left")
                                    Text("Previous")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(bleManager.currentPage > 0 ? Color.blue : Color.gray)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(bleManager.currentPage == 0)

                            Spacer()

                            Button(action: {
                                bleManager.nextPage()
                            }) {
                                HStack {
                                    Text("Next")
                                    Image(systemName: "chevron.right")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(bleManager.hasMoreFiles ? Color.blue : Color.gray)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(!bleManager.hasMoreFiles)
                        }
                        .padding(.horizontal)
                        .padding(.bottom, 10)
                    }
                }
            }
        }
        .navigationTitle(bleManager.isBaseStation ? "LoRa Logs" : "Flights")
        .brandedToolbar()
        .onAppear {
            // Auto-load file list when view appears
            print("FileManagerView appeared, connected: \(bleManager.isConnected)")
            if bleManager.isConnected {
                print("Auto-requesting file list...")
                bleManager.requestFileList()
            }
        }
        .onChange(of: bleManager.files) { files in
            // Rebuild download states from cache when file list updates
            bleManager.rebuildDownloadStates(for: files.map { $0.name })
        }
    }
}

struct FileRow: View {
    let file: FileInfo
    let onDelete: () -> Void
    let onDownload: () -> Void
    @ObservedObject var bleManager: BLEManager

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
        let state = bleManager.getDownloadState(for: file.name)

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
        if !bleManager.isConnected { return true }
        let state = bleManager.getDownloadState(for: file.name)
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

struct FileManagerView_Previews: PreviewProvider {
    static var previews: some View {
        NavigationView {
            FileManagerView(bleManager: BLEManager())
        }
    }
}

// MARK: - Share Helper (presents UIActivityViewController via UIKit to avoid SwiftUI .sheet blank screen)

struct ShareHelper {
    static func share(items: [Any]) {
        guard let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
              let rootVC = windowScene.windows.first?.rootViewController else { return }

        // Walk up to the topmost presented controller
        var topVC = rootVC
        while let presented = topVC.presentedViewController {
            topVC = presented
        }

        let activityVC = UIActivityViewController(
            activityItems: items,
            applicationActivities: nil
        )
        topVC.present(activityVC, animated: true)
    }
}
