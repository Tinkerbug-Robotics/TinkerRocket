package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.async
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertTrue

/**
 * DeviceSession download engine vs the scripted [FakeFirmware] chunk server —
 * pins the EXACT iOS `handleFileChunk` semantics: append-in-ARRIVAL-order,
 * the 3 s stall timer, EOF|ABORT (#526), duplicate-EOF tolerance, and the
 * legacy no-EOF stall-complete path.
 */
class DownloadEngineTest {

    private class Harness(val session: DeviceSession, val fw: FakeFirmware)

    private fun TestScope.startedSession(configure: FakeFirmware.() -> Unit = {}): Harness {
        val fw = FakeFirmware(backgroundScope).apply(configure)
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-R-Test",
            initialDeviceType = BleDeviceType.ROCKET,
            clock = { currentTime },
        )
        session.start()
        runCurrent()
        return Harness(session, fw)
    }

    /** Load page 0 so the session knows the expected file size. */
    private fun TestScope.loadFileList(h: Harness) {
        h.session.requestFileList(0)
        advanceTimeBy(500)
        runCurrent()
    }

    private fun payload(n: Int): ByteArray = ByteArray(n) { (it * 31 + 7).toByte() }

    // ── Happy path ───────────────────────────────────────────────────────

    @Test
    fun happyPath_assemblesAllBytes_progressReachesOne() = runTest {
        val bytes = payload(1234)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 500)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        val success = assertIs<DownloadResult.Success>(result.await())
        assertContentEquals(bytes, success.bytes)
        assertEquals(1.0, h.session.downloadState.value.progress)
        assertFalse(h.session.downloadState.value.active)
        assertTrue(h.fw.ops.contains("write:COMMAND:4"))
    }

    // ── Stall timer ──────────────────────────────────────────────────────

    @Test
    fun stall_firesAtExactlyThreeSeconds_shortfallIsIncomplete() = runTest {
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", payload(300))
            // One 100-byte chunk arrives, then the transfer wedges forever.
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, stallAfterChunks = 1)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()   // chunk 0 arrives; stall timer armed
        assertTrue(h.session.downloadState.value.active)

        advanceTimeBy(2999)
        runCurrent()
        assertFalse(result.isCompleted)   // 2.999 s: not yet

        advanceTimeBy(1)
        runCurrent()                      // exactly 3.000 s after the last chunk
        assertEquals(DownloadResult.Incomplete, result.await())
        assertFalse(h.session.downloadState.value.active)
    }

    @Test
    fun stall_withUnknownExpectedSize_completesWithWhatArrived() = runTest {
        val bytes = payload(300)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, stallAfterChunks = 2)
        }
        // NO file-list load → expectedSize 0 → the stall cannot detect the
        // shortfall and completes with the 200 bytes that arrived (iOS
        // parity: the stall-shortfall check needs a known expected size).
        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        advanceTimeBy(3000)
        runCurrent()
        val success = assertIs<DownloadResult.Success>(result.await())
        assertContentEquals(bytes.copyOfRange(0, 200), success.bytes)
    }

    @Test
    fun legacyNoEof_stallIsTheCompletionPath() = runTest {
        val bytes = payload(250)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, legacyNoEof = true)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()   // all 3 chunks arrive, no EOF flag anywhere
        assertFalse(result.isCompleted)

        advanceTimeBy(2999)
        runCurrent()
        assertFalse(result.isCompleted)
        advanceTimeBy(1)
        runCurrent()
        // Full byte count → the stall completes the download (legacy path).
        val success = assertIs<DownloadResult.Success>(result.await())
        assertContentEquals(bytes, success.bytes)
    }

    // ── EOF / ABORT flags ────────────────────────────────────────────────

    @Test
    fun duplicateEof_isIgnored_andSessionStaysUsable() = runTest {
        val bytes = payload(300)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, duplicateEof = true)
        }
        loadFileList(h)

        val first = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()   // chunks + EOF + the duplicate EOF all delivered
        val success = assertIs<DownloadResult.Success>(first.await())
        assertContentEquals(bytes, success.bytes)
        assertFalse(h.session.downloadState.value.active)

        // The stray EOF did not corrupt anything — a second download works.
        val second = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        assertContentEquals(bytes, assertIs<DownloadResult.Success>(second.await()).bytes)
    }

    @Test
    fun abort_discardsPartialBytes() = runTest {
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", payload(300))
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, abortAfterChunks = 1)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()   // one data chunk, then EOF|ABORT (#526)
        assertEquals(DownloadResult.Aborted, result.await())
        assertFalse(h.session.downloadState.value.active)
    }

    // ── Arrival-order append (iOS bug-compat pins) ───────────────────────

    @Test
    fun droppedChunkThenEof_bugCompat_succeedsTruncated() = runTest {
        val bytes = payload(300)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, dropChunkIndex = 1)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        // iOS parity pin: the EOF path never checks the byte count (only the
        // stall path does), so a dropped middle chunk still "succeeds" — with
        // chunks 0 and 2 concatenated in ARRIVAL order (offsets unused).
        val success = assertIs<DownloadResult.Success>(result.await())
        assertEquals(200, success.bytes.size)
        assertContentEquals(
            bytes.copyOfRange(0, 100) + bytes.copyOfRange(200, 300),
            success.bytes,
        )
    }

    @Test
    fun scrambledOffsetFields_areIgnored_appendIsArrivalOrder() = runTest {
        val bytes = payload(450)
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", bytes)
            // Every chunk claims offset 0 — reassembly must not care.
            downloadScript = FakeFirmware.DownloadScript(chunkSize = 100, scrambleOffsets = true)
        }
        loadFileList(h)

        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        assertContentEquals(bytes, assertIs<DownloadResult.Success>(result.await()).bytes)
    }

    // ── Malformed frames ─────────────────────────────────────────────────

    @Test
    fun malformedFrames_shortIgnored_lyingLengthSkippedButResetsTimer() = runTest {
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", payload(300))
            // The firmware sends nothing on its own — frames are manual below.
            downloadScript = FakeFirmware.DownloadScript(stallAfterChunks = 0)
        }
        // (No file list → expectedSize 0, so the final stall would complete.)
        val result = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()

        // A short (<7 B) frame is ignored entirely — it does NOT arm the
        // stall timer (iOS parity: no timer exists before the first chunk).
        h.fw.emitFileTransferFrame(byteArrayOf(1, 2, 3))
        runCurrent()
        advanceTimeBy(10_000)
        runCurrent()
        assertFalse(result.isCompleted)

        // A valid 10-byte chunk lands and arms the timer.
        val good = ByteArray(10) { it.toByte() }
        h.fw.emitFileTransferFrame(h.fw.chunkFrame(0, good, 0))
        runCurrent()
        advanceTimeBy(2000)
        runCurrent()

        // A frame whose length field LIES (claims 50, carries 10): nothing is
        // appended, but the stall timer still resets (iOS behavior).
        val liar = h.fw.chunkFrame(10, good, 0).also { it[4] = 50 }
        h.fw.emitFileTransferFrame(liar)
        runCurrent()
        advanceTimeBy(2000)   // 4 s after the good chunk — would have stalled
        runCurrent()          // without the liar's timer reset
        assertFalse(result.isCompleted)

        // Zero-length EOF completes with only the good chunk's bytes.
        h.fw.emitFileTransferFrame(h.fw.chunkFrame(10, ByteArray(0), FakeFirmware.FLAG_EOF))
        runCurrent()
        assertContentEquals(good, assertIs<DownloadResult.Success>(result.await()).bytes)
    }

    // ── Concurrency / connection failures ────────────────────────────────

    @Test
    fun secondConcurrentDownload_isBusy() = runTest {
        val h = startedSession {
            deviceFiles += FakeFirmware.FakeFile("flight_1.bin", payload(300))
            downloadScript = FakeFirmware.DownloadScript(stallAfterChunks = 0)
        }
        val first = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        val second = async { h.session.downloadFile("flight_1.bin") }
        runCurrent()
        assertEquals(DownloadResult.Busy, second.await())
        assertFalse(first.isCompleted)

        // The wedged first download fails typed on disconnect.
        h.fw.fireDisconnect()
        runCurrent()
        assertEquals(DownloadResult.Disconnected, first.await())
        assertFalse(h.session.downloadState.value.active)
    }

    @Test
    fun downloadBeforeConnect_isNotConnected() = runTest {
        val fw = FakeFirmware(backgroundScope)
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-R-Test",
            clock = { currentTime },
        )
        // start() not driven — the session never connected.
        assertEquals(DownloadResult.NotConnected, session.downloadFile("flight_1.bin"))
    }
}
