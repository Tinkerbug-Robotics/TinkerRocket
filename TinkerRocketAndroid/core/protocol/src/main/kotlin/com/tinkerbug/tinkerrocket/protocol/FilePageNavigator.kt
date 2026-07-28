package com.tinkerbug.tinkerrocket.protocol

/**
 * Pure paging logic behind the saved-files page navigator — a
 * semantics-exact port of the static helpers on the iOS `FilePageNavigator`
 * SwiftUI view (FileManagerView.swift; the pill/chevron rendering stays in
 * the UI layer).
 *
 * Two modes:
 *  - totalPages known (rocket: the storage stats report `flightCount`, which
 *    is exactly what the BLE file list paginates over) → fixed pills 1…N, so
 *    the user can jump to any page including the last.  A known total also
 *    suppresses the phantom "next" page that the count==pageSize "hasMore"
 *    heuristic would otherwise show for an exactly-full single page.
 *  - totalPages null (the base station reports no count) → pages are
 *    discovered as you page forward (1…current, plus one more while the wire
 *    "hasMore" flag remains set).
 */
public object FilePageNavigator {

    /**
     * Both firmwares paginate the BLE file list at 5 entries/page
     * (FILES_PER_PAGE in out_computer + base_station config).  iOS keeps
     * this as a private constant on FileManagerView; hoisted here so the
     * session/UI layers share one definition.
     */
    public const val FILES_PER_PAGE: Int = 5

    /** Total pages for a known file count, paginated at [pageSize]. At least 1. */
    public fun totalPages(fileCount: Int, pageSize: Int): Int {
        if (pageSize <= 0) return 1
        return maxOf(1, (fileCount + pageSize - 1) / pageSize)
    }

    /** 0-based page indices to render. */
    public fun pageIndices(currentPage: Int, totalPages: Int?, hasMore: Boolean): List<Int> {
        // NOTE: a non-null totalPages of 0 falls through to discovered mode,
        // exactly like the iOS `if let total = totalPages, total > 0` guard.
        if (totalPages != null && totalPages > 0) return (0 until totalPages).toList()
        val last = currentPage + (if (hasMore) 1 else 0)
        return (0..maxOf(0, last)).toList()
    }

    public fun canGoNext(currentPage: Int, totalPages: Int?, hasMore: Boolean): Boolean {
        if (totalPages != null) return currentPage < totalPages - 1
        return hasMore
    }
}
