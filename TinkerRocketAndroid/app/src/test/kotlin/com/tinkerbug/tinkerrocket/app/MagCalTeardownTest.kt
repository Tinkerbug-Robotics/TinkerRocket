package com.tinkerbug.tinkerrocket.app

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
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
 * The mag cal screen's teardown abort, when the link under it is replaced.
 *
 * Leaving the screen mid-cal has to send MAG_CAL_ABORT, or the FC stays in
 * MAG_CALIBRATION with its magnetometer offsets zeroed (the comment over
 * `needsAbort` in MagCalScreen). The fleet builds a new DeviceSession on every
 * reconnect, and while the app is stopped the drop and the reconnect can both
 * land without the dashboard drawing the disconnected state in between, so the
 * screen stays up and is just handed the new session. Its one-shot latch was
 * not keyed on the session: the old link's teardown spent it on a send that
 * went nowhere, and every way out after that sent nothing over the live link.
 *
 * These compose the real screen over real sessions on recording links, and
 * play the FC's status frames.
 */
@RunWith(AndroidJUnit4::class)
class MagCalTeardownTest {

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

        /** The FC's 0xCA status frame, the 22-byte payload (sub-type at [4]). */
        fun fcReports(subType: MagCalSubType) {
            val frame = ByteArray(23)
            frame[0] = 0xCA.toByte()
            frame[1 + 4] = subType.raw.toByte()
            fromFc.tryEmit(TransportEvent.Notification(TrCharacteristic.FILE_OPS, frame))
        }

        val aborts: Int get() = commands.count { it == BleCommandId.MAG_CAL_ABORT_OC }
    }

    private val sessions = mutableListOf<DeviceSession>()

    /** What the fleet does on a connect: a new session, started. */
    private fun connect(link: Link): DeviceSession = DeviceSession(
        scope = CoroutineScope(Dispatchers.Unconfined),
        transport = link,
        connectedDeviceName = "TR-R-Test",
        initialDeviceType = BleDeviceType.ROCKET,
    ).also { sessions += it; it.start() }

    @After fun closeSessions() = sessions.forEach { it.close() }

    /** The screen as the dashboard routes it, over whichever session is current. */
    private inner class Screen(first: DeviceSession, deviceKey: String) {
        var session by mutableStateOf(first)
        var deviceKey by mutableStateOf(deviceKey)
        var open by mutableStateOf(true)

        init {
            compose.setContent {
                if (open) MagCalScreen(session, this.deviceKey, syncer = null, onBack = { open = false })
            }
            compose.waitForIdle()
        }
    }

    // MARK: - Tests

    /** THE regression: the same rocket, mid-cal, comes back on a new link. */
    @Test
    fun aReconnectMidCal_leavingStillAbortsOverTheLiveLink() {
        val first = Link()
        val screen = Screen(connect(first), deviceKey = "rocket")
        first.fcReports(MagCalSubType.SAMPLING)
        compose.waitForIdle()

        // The drop and the reconnect, with the screen still up.
        screen.session.close()
        val second = Link()
        screen.session = connect(second)
        compose.waitForIdle()

        screen.open = false   // every way out runs this teardown
        compose.waitForIdle()

        assertEquals("The FC is still in MAG_CALIBRATION: the abort must go over the live link",
                     1, second.aborts)
    }

    /** Cancel after the reconnect: one abort over the new link, and no second
     *  one when the screen then tears down. */
    @Test
    fun aReconnectMidCal_cancelAbortsOnce() {
        val first = Link()
        val screen = Screen(connect(first), deviceKey = "rocket")
        first.fcReports(MagCalSubType.SAMPLING)
        compose.waitForIdle()

        screen.session.close()
        val second = Link()
        screen.session = connect(second)
        second.fcReports(MagCalSubType.REVIEW)   // the FC republishes on the new link
        compose.waitForIdle()

        compose.onNodeWithText("✕ Cancel").performClick()
        compose.waitForIdle()

        assertEquals(false, screen.open)
        assertEquals(1, second.aborts)
    }

    /** Another rocket takes the screen while the first is mid-cal and still
     *  connected. The cal left behind is aborted over its own link, and the
     *  new rocket, which was not calibrating, is sent nothing. */
    @Test
    fun anotherRocketTakesTheScreen_theCalLeftBehindIsAborted_theNewRocketIsNot() {
        val first = Link()
        val screen = Screen(connect(first), deviceKey = "rocket-a")
        first.fcReports(MagCalSubType.SAMPLING)
        compose.waitForIdle()

        val other = Link()
        screen.session = connect(other)
        screen.deviceKey = "rocket-b"
        compose.waitForIdle()
        assertEquals("The cal left running on the first rocket must be ended", 1, first.aborts)

        screen.open = false
        compose.waitForIdle()
        assertEquals("A stray abort on a rocket that was not calibrating", 0, other.aborts)
        assertEquals(1, first.aborts)
    }

    /** Nothing running: the FC rests in APPLIED, and a visit must not abort it. */
    @Test
    fun noCalRunning_leavingSendsNothing() {
        val link = Link()
        val screen = Screen(connect(link), deviceKey = "rocket")
        link.fcReports(MagCalSubType.APPLIED)
        compose.waitForIdle()

        screen.open = false
        compose.waitForIdle()
        assertEquals(0, link.aborts)
    }
}
