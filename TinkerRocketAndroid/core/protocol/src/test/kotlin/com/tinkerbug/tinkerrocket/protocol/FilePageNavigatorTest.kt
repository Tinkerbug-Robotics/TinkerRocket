package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Pure paging logic behind the saved-files page navigator — port of the iOS
 * FilePageNavigatorTests (every case). The rocket knows its total file count
 * (→ fixed 1…N pills); the base station doesn't (→ pages discovered as you
 * advance). These guard the off-by-one-prone arithmetic.
 */
class FilePageNavigatorTest {

    // MARK: total pages from a known file count (rocket)

    @Test
    fun `total pages ceil division`() {
        val ps = 5
        assertEquals(1, FilePageNavigator.totalPages(fileCount = 0, pageSize = ps))
        assertEquals(1, FilePageNavigator.totalPages(fileCount = 1, pageSize = ps))
        assertEquals(1, FilePageNavigator.totalPages(fileCount = 5, pageSize = ps))
        assertEquals(2, FilePageNavigator.totalPages(fileCount = 6, pageSize = ps))
        assertEquals(2, FilePageNavigator.totalPages(fileCount = 10, pageSize = ps))
        assertEquals(3, FilePageNavigator.totalPages(fileCount = 11, pageSize = ps))
        assertEquals(6, FilePageNavigator.totalPages(fileCount = 26, pageSize = ps))
    }

    @Test
    fun `total pages guards zero page size`() {
        assertEquals(1, FilePageNavigator.totalPages(fileCount = 10, pageSize = 0))
    }

    // MARK: known-total mode (rocket) — all pages shown, jump anywhere

    @Test
    fun `known total shows all pages`() {
        assertEquals(
            listOf(0, 1, 2, 3),
            FilePageNavigator.pageIndices(currentPage = 2, totalPages = 4, hasMore = false),
        )
    }

    @Test
    fun `known total next disabled on last page`() {
        assertTrue(FilePageNavigator.canGoNext(currentPage = 0, totalPages = 3, hasMore = true))
        assertTrue(FilePageNavigator.canGoNext(currentPage = 1, totalPages = 3, hasMore = false))
        assertFalse(FilePageNavigator.canGoNext(currentPage = 2, totalPages = 3, hasMore = true))
    }

    @Test
    fun `known total ignores hasMore heuristic`() {
        // Exactly-full single page: count==pageSize makes the wire "hasMore"
        // heuristic true, but a known total of 1 must not offer a phantom page 2.
        assertEquals(
            listOf(0),
            FilePageNavigator.pageIndices(currentPage = 0, totalPages = 1, hasMore = true),
        )
        assertFalse(FilePageNavigator.canGoNext(currentPage = 0, totalPages = 1, hasMore = true))
    }

    // MARK: discovered mode (base station) — grows with hasMore

    @Test
    fun `discovered mode reveals next when more`() {
        // On page 0 with more to come: show pages 1 and 2 (0-based 0,1).
        assertEquals(
            listOf(0, 1),
            FilePageNavigator.pageIndices(currentPage = 0, totalPages = null, hasMore = true),
        )
        assertTrue(FilePageNavigator.canGoNext(currentPage = 0, totalPages = null, hasMore = true))
    }

    @Test
    fun `discovered mode stops when no more`() {
        // On page 2 (0-based) with nothing more: pages 1…3 known, no next.
        assertEquals(
            listOf(0, 1, 2),
            FilePageNavigator.pageIndices(currentPage = 2, totalPages = null, hasMore = false),
        )
        assertFalse(FilePageNavigator.canGoNext(currentPage = 2, totalPages = null, hasMore = false))
    }
}
