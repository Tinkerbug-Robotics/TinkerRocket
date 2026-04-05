//
//  FlightChartView.swift
//  TinkerRocketApp
//
//  Plot CSV flight data columns against time using Swift Charts.
//  Supports pinch-to-zoom, drag-to-pan, and double-tap to reset.
//

import SwiftUI
import Charts

struct FlightChartView: View {
    let flight: CachedFlight

    @State private var csvData: FlightCSVData?
    @State private var isLoading = true
    @State private var errorMessage: String?
    @State private var selectedColumns: Set<String> = []
    @State private var showColumnPicker = false
    @State private var chartSeries: [ChartSeries] = []
    @State private var fullSeriesData: [FullColumnData] = []

    // MARK: - Zoom & Pan State

    /// The full X-axis range from the data (set once when chartSeries changes)
    @State private var fullXRange: ClosedRange<Double> = 0...1

    /// The currently visible X-axis window
    @State private var visibleXRange: ClosedRange<Double> = 0...1

    /// Anchor X range captured at the start of a pinch gesture
    @State private var gestureStartXRange: ClosedRange<Double>?

    /// Anchor X range captured at the start of a drag gesture
    @State private var dragStartXRange: ClosedRange<Double>?

    private let maxPointsPerSeries = 2000
    private let seriesColors: [Color] = [.blue, .red, .green, .orange, .purple, .cyan]

    struct ChartSeries: Identifiable {
        let id: String  // column name
        let points: [(x: Double, y: Double)]
    }

    /// Full-resolution clean data per series (no NaN, sorted by time).
    /// Kept in memory so we can re-decimate for any visible window.
    struct FullColumnData: Identifiable {
        let id: String   // column name
        let x: [Double]  // time in seconds, sorted ascending
        let y: [Double]  // corresponding values
    }

    // MARK: - Zoom Helpers

    /// Whether the chart is currently zoomed in beyond the full view
    private var isZoomed: Bool {
        let fullSpan = fullXRange.upperBound - fullXRange.lowerBound
        let visibleSpan = visibleXRange.upperBound - visibleXRange.lowerBound
        guard fullSpan > 0 else { return false }
        return visibleSpan < fullSpan * 0.99  // small tolerance for floating point
    }

    /// Y-axis range that fits only the data visible in the current X window
    private var visibleYRange: ClosedRange<Double> {
        var minY = Double.infinity
        var maxY = -Double.infinity

        for series in chartSeries {
            for point in series.points {
                if point.x >= visibleXRange.lowerBound && point.x <= visibleXRange.upperBound {
                    if point.y < minY { minY = point.y }
                    if point.y > maxY { maxY = point.y }
                }
            }
        }

        guard minY.isFinite && maxY.isFinite else { return 0...1 }

        // Add 5% vertical padding so lines don't touch the axis edges
        let span = maxY - minY
        if span < 1e-9 {
            // Flat data — give it a ±1 range
            return (minY - 1)...(maxY + 1)
        }
        let padding = span * 0.05
        return (minY - padding)...(maxY + padding)
    }

    // MARK: - Body

    var body: some View {
        VStack(spacing: 0) {
            if isLoading {
                ProgressView("Parsing flight data...")
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else if let error = errorMessage {
                VStack(spacing: 12) {
                    Image(systemName: "exclamationmark.triangle")
                        .font(.system(size: 48))
                        .foregroundColor(.orange)
                    Text(error)
                        .font(.body)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
                .padding()
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else if chartSeries.isEmpty {
                VStack(spacing: 12) {
                    Image(systemName: "chart.xyaxis.line")
                        .font(.system(size: 48))
                        .foregroundColor(.secondary)
                    Text("Select a column to plot")
                        .font(.headline)
                        .foregroundColor(.secondary)
                    Button("Choose Columns") { showColumnPicker = true }
                        .buttonStyle(.borderedProminent)
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else {
                chartView
            }
        }
        .navigationTitle("Flight Data")
        .navigationBarTitleDisplayMode(.inline)
        .brandedToolbar()
        .toolbar {
            ToolbarItem(placement: .navigationBarTrailing) {
                Button {
                    showColumnPicker = true
                } label: {
                    Image(systemName: "line.3.horizontal.decrease.circle")
                }
                .disabled(isLoading)
            }
        }
        .sheet(isPresented: $showColumnPicker) {
            ColumnPickerView(
                availableColumns: csvData?.headers.filter { !isTimeColumn($0) } ?? [],
                columnGroups: FlightCSVData.columnGroups(for: csvData?.headers ?? []),
                selectedColumns: $selectedColumns
            )
        }
        .onChange(of: selectedColumns) { newSelection in
            updateChartSeries(for: newSelection)
        }
        .task {
            await loadCSV()
        }
    }

    // MARK: - Chart View

    @ViewBuilder
    private var chartView: some View {
        VStack(spacing: 0) {
            // Zoom indicator bar (only shown when zoomed)
            if isZoomed {
                zoomIndicatorBar
            }

            GeometryReader { geometry in
                Chart {
                    ForEach(chartSeries) { series in
                        ForEach(Array(series.points.enumerated()), id: \.offset) { _, point in
                            LineMark(
                                x: .value("Time (s)", point.x),
                                y: .value("Value", point.y),
                                series: .value("Series", series.id)
                            )
                            .foregroundStyle(by: .value("Series", series.id))
                        }

                        // Only show point markers when zoomed in enough that
                        // individual data points are meaningful (raw 1kHz data)
                        if series.points.count <= 500 {
                            ForEach(Array(series.points.enumerated()), id: \.offset) { _, point in
                                PointMark(
                                    x: .value("Time (s)", point.x),
                                    y: .value("Value", point.y)
                                )
                                .foregroundStyle(by: .value("Series", series.id))
                                .symbolSize(12)
                            }
                        }
                    }
                }
                .chartXScale(domain: visibleXRange)
                .chartYScale(domain: visibleYRange)
                .chartForegroundStyleScale(
                    domain: chartSeries.map(\.id),
                    range: Array(seriesColors.prefix(chartSeries.count))
                )
                .chartXAxisLabel("Time (s)")
                .chartLegend(chartSeries.count > 1 ? .visible : .hidden)
                .padding()
                .contentShape(Rectangle())
                .highPriorityGesture(pinchGesture(chartWidth: geometry.size.width))
                .gesture(panGesture(chartWidth: geometry.size.width))
                .gesture(doubleTapGesture)
            }
        }
    }

    // MARK: - Zoom Indicator

    private var zoomIndicatorBar: some View {
        HStack(spacing: 8) {
            Image(systemName: "magnifyingglass")
                .font(.caption2)
                .foregroundColor(.secondary)

            Text(String(format: "%.1fs – %.1fs", visibleXRange.lowerBound, visibleXRange.upperBound))
                .font(.caption2.monospacedDigit())
                .foregroundColor(.secondary)

            Spacer()

            Button {
                withAnimation(.easeInOut(duration: 0.3)) {
                    visibleXRange = fullXRange
                }
                redecimateForVisibleRange()
            } label: {
                Text("Reset")
                    .font(.caption2)
                    .foregroundColor(.blue)
            }
        }
        .padding(.horizontal)
        .padding(.vertical, 4)
        .background(Color(.systemGray6))
        .transition(.move(edge: .top).combined(with: .opacity))
    }

    // MARK: - Gestures

    private func pinchGesture(chartWidth: CGFloat) -> some Gesture {
        MagnificationGesture()
            .onChanged { scale in
                if gestureStartXRange == nil {
                    gestureStartXRange = visibleXRange
                }
                guard let startRange = gestureStartXRange else { return }

                let startSpan = startRange.upperBound - startRange.lowerBound
                let fullSpan = fullXRange.upperBound - fullXRange.lowerBound
                guard fullSpan > 0 else { return }

                // scale > 1 = fingers spreading = zoom in = smaller visible span
                // Min span ~30ms (fullSpan/1000) — enough to see individual 1ms samples
                let newSpan = max(fullSpan / 1000.0, min(fullSpan, startSpan / scale))

                // Anchor zoom to center of the starting visible range
                let center = (startRange.lowerBound + startRange.upperBound) / 2.0
                var newLower = center - newSpan / 2.0
                var newUpper = center + newSpan / 2.0

                // Clamp to full data range
                if newLower < fullXRange.lowerBound {
                    newLower = fullXRange.lowerBound
                    newUpper = newLower + newSpan
                }
                if newUpper > fullXRange.upperBound {
                    newUpper = fullXRange.upperBound
                    newLower = newUpper - newSpan
                }

                visibleXRange = max(newLower, fullXRange.lowerBound)...min(newUpper, fullXRange.upperBound)
            }
            .onEnded { _ in
                gestureStartXRange = nil
                redecimateForVisibleRange()
            }
    }

    private func panGesture(chartWidth: CGFloat) -> some Gesture {
        DragGesture(minimumDistance: 20)
            .onChanged { value in
                guard isZoomed else { return }

                if dragStartXRange == nil {
                    dragStartXRange = visibleXRange
                }
                guard let startRange = dragStartXRange else { return }

                let visibleSpan = startRange.upperBound - startRange.lowerBound

                // Convert pixel drag to data-space offset.
                // Approximate effective chart width (total minus padding and axis labels)
                let effectiveWidth = max(chartWidth - 64, 1)
                let dataPerPoint = visibleSpan / Double(effectiveWidth)

                // Dragging right shows earlier data (negative offset)
                let dataOffset = -Double(value.translation.width) * dataPerPoint

                var newLower = startRange.lowerBound + dataOffset
                var newUpper = startRange.upperBound + dataOffset

                // Clamp to full data range
                if newLower < fullXRange.lowerBound {
                    newLower = fullXRange.lowerBound
                    newUpper = newLower + visibleSpan
                }
                if newUpper > fullXRange.upperBound {
                    newUpper = fullXRange.upperBound
                    newLower = newUpper - visibleSpan
                }

                visibleXRange = newLower...newUpper
            }
            .onEnded { _ in
                dragStartXRange = nil
                redecimateForVisibleRange()
            }
    }

    private var doubleTapGesture: some Gesture {
        TapGesture(count: 2)
            .onEnded {
                withAnimation(.easeInOut(duration: 0.3)) {
                    visibleXRange = fullXRange
                }
                redecimateForVisibleRange()
            }
    }

    // MARK: - Data Loading

    private func loadCSV() async {
        do {
            let url = flight.csvURL
            let data = try await Task.detached(priority: .userInitiated) {
                try CSVParser.parse(url: url)
            }.value

            await MainActor.run {
                self.csvData = data
                self.isLoading = false

                // Auto-select a sensible default column
                if data.columns["Pressure Altitude (m)"] != nil {
                    // Rocket on-board CSV
                    self.selectedColumns = ["Pressure Altitude (m)"]
                } else if data.columns["pressure_alt"] != nil {
                    // LoRa base-station CSV
                    self.selectedColumns = ["pressure_alt"]
                }
            }
        } catch {
            await MainActor.run {
                self.errorMessage = error.localizedDescription
                self.isLoading = false
            }
        }
    }

    // MARK: - Chart Series Update

    /// Check whether a column name is the time axis (should be excluded from plotting)
    private func isTimeColumn(_ name: String) -> Bool {
        name == "Time (ms)" || name == "time_ms"
    }

    private func updateChartSeries(for selection: Set<String>) {
        guard let data = csvData else {
            chartSeries = []
            fullSeriesData = []
            return
        }

        // Support both rocket ("Time (ms)") and LoRa ("time_ms") time columns
        guard let timeMs = data.columns["Time (ms)"] ?? data.columns["time_ms"] else {
            chartSeries = []
            fullSeriesData = []
            return
        }

        let timeSeconds = timeMs.map { $0 / 1000.0 }

        // Build full-resolution clean data for each selected column
        let newFullData: [FullColumnData] = selection.sorted().compactMap { columnName in
            guard let values = data.columns[columnName] else { return nil }

            var cleanX: [Double] = []
            var cleanY: [Double] = []
            for i in 0..<min(timeSeconds.count, values.count) {
                if !timeSeconds[i].isNaN && !values[i].isNaN {
                    cleanX.append(timeSeconds[i])
                    cleanY.append(values[i])
                }
            }

            guard !cleanX.isEmpty else { return nil }
            return FullColumnData(id: columnName, x: cleanX, y: cleanY)
        }

        fullSeriesData = newFullData

        // Initial decimation from full range
        chartSeries = newFullData.map { col in
            let decimated = CSVParser.lttbDecimate(
                x: col.x, y: col.y, targetCount: maxPointsPerSeries
            )
            return ChartSeries(id: col.id, points: decimated)
        }

        // Initialize zoom ranges from the full data
        if let firstCol = newFullData.first,
           let minX = firstCol.x.first,
           let maxX = newFullData.last?.x.last,
           minX < maxX {
            // Use actual min/max across all series
            let allMinX = newFullData.compactMap { $0.x.first }.min() ?? minX
            let allMaxX = newFullData.compactMap { $0.x.last }.max() ?? maxX
            fullXRange = allMinX...allMaxX
            visibleXRange = allMinX...allMaxX
        } else {
            fullXRange = 0...1
            visibleXRange = 0...1
        }
    }

    // MARK: - Progressive LOD Re-decimation

    /// Re-decimate all series for the current visible time window.
    /// Called on gesture end so the chart always has up to `maxPointsPerSeries`
    /// points covering just the visible range.  At deep zoom (≤2s at 1kHz),
    /// raw data is shown with no decimation.
    private func redecimateForVisibleRange() {
        guard !fullSeriesData.isEmpty else { return }

        let lower = visibleXRange.lowerBound
        let upper = visibleXRange.upperBound

        chartSeries = fullSeriesData.map { col in
            // Binary search for the first index where x[i] >= lower
            let startIdx = lowerBound(col.x, lower)
            // Binary search for the last index where x[i] <= upper
            let endIdx = upperBound(col.x, upper)

            guard startIdx <= endIdx else {
                return ChartSeries(id: col.id, points: [])
            }

            // Include 1 extra point on each side for smooth line edge connection
            let safeStart = max(0, startIdx - 1)
            let safeEnd = min(col.x.count - 1, endIdx + 1)
            let rangeCount = safeEnd - safeStart + 1

            if rangeCount <= maxPointsPerSeries {
                // Show all raw data points — true 1kHz resolution!
                return ChartSeries(
                    id: col.id,
                    points: (safeStart...safeEnd).map { (x: col.x[$0], y: col.y[$0]) }
                )
            } else {
                // LTTB decimate the visible subset
                let subX = Array(col.x[safeStart...safeEnd])
                let subY = Array(col.y[safeStart...safeEnd])
                let decimated = CSVParser.lttbDecimate(
                    x: subX, y: subY, targetCount: maxPointsPerSeries
                )
                return ChartSeries(id: col.id, points: decimated)
            }
        }
    }

    // MARK: - Binary Search Helpers

    /// First index where arr[i] >= target (sorted ascending).
    /// Returns arr.count if all elements are < target.
    private func lowerBound(_ arr: [Double], _ target: Double) -> Int {
        var lo = 0, hi = arr.count
        while lo < hi {
            let mid = (lo + hi) / 2
            if arr[mid] < target { lo = mid + 1 } else { hi = mid }
        }
        return lo
    }

    /// Last index where arr[i] <= target (sorted ascending).
    /// Returns -1 if all elements are > target.
    private func upperBound(_ arr: [Double], _ target: Double) -> Int {
        var lo = 0, hi = arr.count
        while lo < hi {
            let mid = (lo + hi) / 2
            if arr[mid] <= target { lo = mid + 1 } else { hi = mid }
        }
        return lo - 1
    }
}
