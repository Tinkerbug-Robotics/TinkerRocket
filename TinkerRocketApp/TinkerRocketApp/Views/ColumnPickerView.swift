//
//  ColumnPickerView.swift
//  TinkerRocketApp
//
//  Grouped column picker for selecting CSV columns to plot
//

import SwiftUI

struct ColumnPickerView: View {
    let availableColumns: [String]
    let columnGroups: [(name: String, columns: [String])]
    @Binding var selectedColumns: Set<String>
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationView {
            List {
                ForEach(columnGroups, id: \.name) { group in
                    let groupColumns = group.columns.filter { availableColumns.contains($0) }
                    if !groupColumns.isEmpty {
                        Section(header: Text(group.name)) {
                            ForEach(groupColumns, id: \.self) { column in
                                Button {
                                    if selectedColumns.contains(column) {
                                        selectedColumns.remove(column)
                                    } else {
                                        selectedColumns.insert(column)
                                    }
                                } label: {
                                    HStack {
                                        Text(shortLabel(for: column))
                                            .foregroundColor(.primary)
                                        Spacer()
                                        if selectedColumns.contains(column) {
                                            Image(systemName: "checkmark")
                                                .foregroundColor(.blue)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if !selectedColumns.isEmpty {
                    Section {
                        Button("Clear All", role: .destructive) {
                            selectedColumns.removeAll()
                        }
                    }
                }
            }
            .navigationTitle("Select Columns")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
        }
    }

    /// Strip the unit suffix for cleaner display
    /// e.g. "Low-G Acceleration X (m/s2)" -> "Low-G Acceleration X"
    private func shortLabel(for column: String) -> String {
        if let range = column.range(of: " (", options: .backwards) {
            return String(column[..<range.lowerBound])
        }
        return column
    }
}
