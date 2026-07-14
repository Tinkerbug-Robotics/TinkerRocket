import XCTest
@testable import TinkerRocketApp

/// Pure paging logic behind the saved-files page navigator. The rocket knows its
/// total file count (→ fixed 1…N pills); the base station doesn't (→ pages
/// discovered as you advance). These guard the off-by-one-prone arithmetic.
final class FilePageNavigatorTests: XCTestCase {

    // MARK: total pages from a known file count (rocket)

    func testTotalPagesCeilDivision() {
        let ps = 5
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 0, pageSize: ps), 1)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 1, pageSize: ps), 1)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 5, pageSize: ps), 1)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 6, pageSize: ps), 2)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 10, pageSize: ps), 2)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 11, pageSize: ps), 3)
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 26, pageSize: ps), 6)
    }

    func testTotalPagesGuardsZeroPageSize() {
        XCTAssertEqual(FilePageNavigator.totalPages(forFileCount: 10, pageSize: 0), 1)
    }

    // MARK: known-total mode (rocket) — all pages shown, jump anywhere

    func testKnownTotalShowsAllPages() {
        XCTAssertEqual(
            FilePageNavigator.pageIndices(currentPage: 2, totalPages: 4, hasMore: false),
            [0, 1, 2, 3])
    }

    func testKnownTotalNextDisabledOnLastPage() {
        XCTAssertTrue(FilePageNavigator.canGoNext(currentPage: 0, totalPages: 3, hasMore: true))
        XCTAssertTrue(FilePageNavigator.canGoNext(currentPage: 1, totalPages: 3, hasMore: false))
        XCTAssertFalse(FilePageNavigator.canGoNext(currentPage: 2, totalPages: 3, hasMore: true))
    }

    func testKnownTotalIgnoresHasMoreHeuristic() {
        // Exactly-full single page: count==pageSize makes the wire "hasMore"
        // heuristic true, but a known total of 1 must not offer a phantom page 2.
        XCTAssertEqual(
            FilePageNavigator.pageIndices(currentPage: 0, totalPages: 1, hasMore: true),
            [0])
        XCTAssertFalse(FilePageNavigator.canGoNext(currentPage: 0, totalPages: 1, hasMore: true))
    }

    // MARK: discovered mode (base station) — grows with hasMore

    func testDiscoveredModeRevealsNextWhenMore() {
        // On page 0 with more to come: show pages 1 and 2 (0-based 0,1).
        XCTAssertEqual(
            FilePageNavigator.pageIndices(currentPage: 0, totalPages: nil, hasMore: true),
            [0, 1])
        XCTAssertTrue(FilePageNavigator.canGoNext(currentPage: 0, totalPages: nil, hasMore: true))
    }

    func testDiscoveredModeStopsWhenNoMore() {
        // On page 2 (0-based) with nothing more: pages 1…3 known, no next.
        XCTAssertEqual(
            FilePageNavigator.pageIndices(currentPage: 2, totalPages: nil, hasMore: false),
            [0, 1, 2])
        XCTAssertFalse(FilePageNavigator.canGoNext(currentPage: 2, totalPages: nil, hasMore: false))
    }
}
