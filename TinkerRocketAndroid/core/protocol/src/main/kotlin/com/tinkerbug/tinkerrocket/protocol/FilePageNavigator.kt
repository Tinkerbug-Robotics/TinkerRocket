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

    // ── #1144: the page size is negotiated, not assumed ──────────────────
    //
    // 5 was hardcoded here, on iOS and in three firmwares, with no reference
    // to the link MTU. A five-entry page is ~281 B worst case; an ATT MTU of
    // 185 allows 182. The firmware used to let NimBLE trim the page (so the
    // app parsed garbage) and since #1284 refuses to send it at all — which
    // means a phone at 185 with four or more flights got NO list.
    //
    // cmd 2 now carries an optional per_page byte derived from the MTU this
    // app negotiated. These constants mirror
    // tr_flightlog::wire_format in the firmware and must move with it.

    /**
     * Worst-case bytes for one encoded entry:
     * `{"name":"` 9 + filename ≤ 27 + `","size":` 9 + up to 10 digits + `}` 1.
     * The filename bound is the firmware's `FlightIndexEntry::filename[28]`.
     */
    public const val ENTRY_MAX_BYTES: Int = 56

    /** Never ask for more than this, whatever the MTU suggests. */
    public const val MAX_PER_PAGE: Int = 16

    /** Entries that fit a notification budget: `n*E + (n-1) + 2 <= budget`. */
    public fun entriesThatFit(budgetBytes: Int): Int {
        if (budgetBytes < ENTRY_MAX_BYTES + 2) return 1
        return (budgetBytes - 1) / (ENTRY_MAX_BYTES + 1)
    }

    /**
     * Page size to request for a negotiated ATT MTU, or [FILES_PER_PAGE] when
     * the MTU is unknown (0/negative — no MtuChanged seen yet).
     *
     * A notification carries `mtu - 3`. Asking for MORE than fits is not a
     * silent degradation: the firmware clamps down, but the app's own
     * `hasMore` would then disagree with the page it received, so ask for
     * exactly what the link can carry.
     */
    public fun perPageForMtu(negotiatedMtu: Int): Int {
        // 23 is the BLE default ATT MTU and the value the session holds before
        // any MtuChanged arrives, so it means "not negotiated yet", not "this
        // link carries 20 bytes". Asking for 1 entry on that basis would make
        // paging crawl on every link for the window before negotiation lands —
        // and a link genuinely stuck at 23 cannot carry even one 56-byte entry,
        // so the firmware refuses the page either way and nothing is gained by
        // guessing low. Unknown therefore means the historical default.
        if (negotiatedMtu <= BLE_DEFAULT_MTU) return FILES_PER_PAGE
        return entriesThatFit(negotiatedMtu - 3).coerceIn(1, MAX_PER_PAGE)
    }

    /** The pre-negotiation ATT MTU. Treated as "unknown" by [perPageForMtu]. */
    public const val BLE_DEFAULT_MTU: Int = 23

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

    /**
     * #1144: learn the device's page size from what it actually serves.
     *
     * The per_page byte in cmd 2 is a request. An older firmware ignores it
     * and serves 5; a newer one clamps to its own notification budget and may
     * serve fewer than asked. Judging "is this page full" against our own
     * request is wrong in both directions — it stopped paging at page 0
     * against an old device in testing — so take the largest page seen so far
     * as the device's size.
     *
     * Monotonic on purpose: a partial LAST page must not shrink the yardstick
     * and make the next request look full.
     */
    public fun observePageSize(previous: Int, servedCount: Int): Int =
        maxOf(previous, servedCount)
}
