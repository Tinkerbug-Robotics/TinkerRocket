package com.tinkerbug.tinkerrocket.app

import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.MagCalSubType
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportEvent
import java.util.concurrent.CopyOnWriteArrayList
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The mag cal's verify step: its Save button reads "Save" (dark blue) once
 * coverage and samples are met, and "Save anyway" (orange) before.  It sends
 * VERIFY_DONE only when every row is green, |B| included, and FORCE_APPLY
 * otherwise.  These compose the real screen over a session on a recording
 * link and play the FC's VERIFYING frames.
 */
@RunWith(AndroidJUnit4::class)
class MagCalVerifySaveTest {

    @get:Rule val compose = createComposeRule()

    /** A link that records the single-byte commands written to it. */
    private class Link : BleTransport {
        private val fromFc = MutableSharedFlow<TransportEvent>(extraBufferCapacity = 64)
        override val events: Flow<TransportEvent> = fromFc
        val commands = CopyOnWriteArrayList<Int>()

        override suspend fun connect() = Unit
        override suspend fun requestMtu(target: Int): Int = 185
        override suspend fun enableNotifications(char: TrCharacteristic) = Unit
        override suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean) {
            if (char == TrCharacteristic.COMMAND && bytes.size == 1) commands += bytes[0].toInt() and 0xFF
        }
        override suspend fun read(char: TrCharacteristic): ByteArray = ByteArray(0)
        override suspend fun readRssi(): Int = -60
        override fun disconnect() = Unit

        /** The saves sent: VERIFY_DONE or FORCE_APPLY. */
        val saves: List<Int>
            get() = commands.filter {
                it == BleCommandId.MAG_CAL_VERIFY_DONE_OC || it == BleCommandId.MAG_CAL_FORCE_APPLY_OC
            }

        /**
         * The FC's 0xCA status frame while verifying: sub-type [4], coverage
         * [5], samples [6..7] and the live |B| x10 [8..9].
         */
        fun verifying(coverage: Int, samples: Int, fieldUt: Float) {
            val frame = ByteArray(23)
            frame[0] = 0xCA.toByte()
            frame[1 + 4] = MagCalSubType.VERIFYING.raw.toByte()
            frame[1 + 5] = coverage.toByte()
            frame[1 + 6] = samples.toByte()
            frame[1 + 7] = (samples ushr 8).toByte()
            val b = Math.round(fieldUt * 10)
            frame[1 + 8] = b.toByte()
            frame[1 + 9] = (b ushr 8).toByte()
            fromFc.tryEmit(TransportEvent.Notification(TrCharacteristic.FILE_OPS, frame))
        }
    }

    private val sessions = mutableListOf<DeviceSession>()

    @After fun closeSessions() = sessions.forEach { it.close() }

    /** The screen over a new session on [link]. */
    private fun open(link: Link) {
        val session = DeviceSession(
            scope = CoroutineScope(Dispatchers.Unconfined),
            transport = link,
            connectedDeviceName = "TR-R-Test",
            initialDeviceType = BleDeviceType.ROCKET,
        ).also { sessions += it; it.start() }
        compose.setContent { MagCalScreen(session, "rocket", syncer = null, onBack = {}) }
        compose.waitForIdle()
    }

    private fun play(link: Link, coverage: Int, samples: Int, fieldUt: Float) {
        link.verifying(coverage, samples, fieldUt)
        compose.waitForIdle()
    }

    private fun tap(text: String) {
        compose.onNodeWithText(text).performScrollTo().performClick()
        compose.waitForIdle()
    }

    @Test
    fun beforeEnoughPoints_itIsSaveAnyway_andForces() {
        val link = Link()
        open(link)
        play(link, coverage = 3, samples = 40, fieldUt = 45f)
        compose.onNodeWithText("Save").assertDoesNotExist()
        compose.onNodeWithText(
            "Keep rotating until coverage and samples are green. Save anyway saves now, without the checks.",
        ).assertExists()
        tap("Save anyway")
        assertEquals(listOf(BleCommandId.MAG_CAL_FORCE_APPLY_OC), link.saves)
    }

    @Test
    fun withEnoughPointsAndEveryRowGreen_itIsSave_andTheFcChecks() {
        val link = Link()
        open(link)
        play(link, coverage = 3, samples = 40, fieldUt = 45f)
        play(link, coverage = 10, samples = 150, fieldUt = 48f)
        compose.onNodeWithText("Save anyway").assertDoesNotExist()
        compose.onNodeWithText("All checks green — tap Save to commit the cal to flight-computer memory.")
            .assertExists()
        tap("Save")
        assertEquals(listOf(BleCommandId.MAG_CAL_VERIFY_DONE_OC), link.saves)
    }

    @Test
    fun withEnoughPointsAndARedField_itIsStillSave_andForces() {
        val link = Link()
        open(link)
        play(link, coverage = 10, samples = 150, fieldUt = 45f)
        play(link, coverage = 11, samples = 160, fieldUt = 82f)   // past 70 µT: Max observed is red
        compose.onNodeWithText("Save anyway").assertDoesNotExist()
        compose.onNodeWithText(
            "Enough points, but a |B| row is not green. Red means the field left Earth's band or swung " +
                "widely: something magnetic nearby, or a fit to redo. Save keeps this cal anyway; Retry " +
                "verification starts the readings over.",
        ).assertExists()
        tap("Save")
        assertEquals(listOf(BleCommandId.MAG_CAL_FORCE_APPLY_OC), link.saves)
    }
}
