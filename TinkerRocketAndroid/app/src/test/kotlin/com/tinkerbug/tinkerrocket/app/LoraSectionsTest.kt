package com.tinkerbug.tinkerrocket.app

import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithText
import androidx.test.ext.junit.runners.AndroidJUnit4
import android.app.Application
import androidx.test.core.app.ApplicationProvider
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.KnownDeviceStorage
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportEvent
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * #624/#150: the base-station radio sections, and specifically that a refused
 * push says WHY.
 *
 * The rules themselves are pinned without a device in
 * `LoraAutoApplyTest` (`:core:session`). What needs a rendered screen is the
 * part that bit us before: a control that goes dead in silence. On a simulator
 * or a bench phone with no base station these sections are permanently
 * refused, so the footer is the only thing standing between the operator and
 * "the app is broken".
 */
@RunWith(AndroidJUnit4::class)
class LoraSectionsTest {

    @get:Rule val compose = createComposeRule()

    /** No radio, no events — enough to construct a session that never connects. */
    private class DeadTransport : BleTransport {
        override val events: Flow<TransportEvent> = MutableSharedFlow()
        override suspend fun connect() = Unit
        override suspend fun requestMtu(target: Int): Int = 23
        override suspend fun enableNotifications(char: TrCharacteristic) = Unit
        override suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean) = Unit
        override suspend fun read(char: TrCharacteristic): ByteArray = ByteArray(0)
        override suspend fun readRssi(): Int = -60
        override fun disconnect() = Unit
    }

    /** Registry with nothing in it; enough to satisfy the section's writer. */
    private fun knownDevices() = KnownDeviceStore(object : KnownDeviceStorage {
        private var json: String? = null
        override fun loadDevicesJson(): String? = json
        override fun saveDevicesJson(json: String) { this.json = json }
        override fun loadLegacyKnownIds(): List<String>? = null
        override fun removeLegacyKnownIds() = Unit
    })

    private fun appNetwork(name: String) =
        AppNetworkStore(ApplicationProvider.getApplicationContext<Application>())
            .also { it.setNetwork(name) }

    private fun session(type: BleDeviceType) = DeviceSession(
        scope = CoroutineScope(Dispatchers.Unconfined),
        transport = DeadTransport(),
        connectedDeviceName = "TR-Bench",
        initialDeviceType = type,
    )

    @Test
    fun all_three_radio_sections_render() {
        // The whole point of the change: this branch used to stop before them.
        compose.setContent { LoraSections(session(BleDeviceType.BASE_STATION)) }
        compose.onNodeWithText("Link Mode").assertIsDisplayed()
        compose.onNodeWithText("LoRa Frequency").assertIsDisplayed()
        compose.onNodeWithText("LoRa TX Power").assertIsDisplayed()
    }

    @Test
    fun each_refused_control_explains_itself_where_it_is() {
        // Twice, not once, and deliberately: Link Mode and TX Power are
        // separate controls that both went dead, and a reason printed under
        // only one of them leaves the other unexplained. iOS prints it in both
        // footers for the same reason.
        compose.setContent { LoraSections(session(BleDeviceType.BASE_STATION)) }
        compose.onAllNodesWithText("Base station is not connected over BLE.")
            .assertCountEquals(2)
    }

    @Test
    fun a_rocket_link_names_the_base_station_as_the_fix() {
        // Cmd 17 is rejected by rocket-side firmware, so "connect to the base
        // station" is the actionable sentence, not "unavailable".
        compose.setContent { LoraSections(session(BleDeviceType.ROCKET)) }
        compose.onAllNodesWithText("Connect to the base station first.")
            .assertCountEquals(2)
    }

    @Test
    fun a_refused_link_cannot_push_transmit_power() {
        // The session refuses the write regardless, but a live-looking Apply
        // button on a dead link invites the operator to tap it and believe
        // something happened.
        compose.setContent { LoraSections(session(BleDeviceType.BASE_STATION)) }
        compose.onNodeWithText("Apply 17 dBm").assertIsNotEnabled()
    }

    @Test
    fun an_unprovisioned_device_is_not_flagged_as_a_mismatch() {
        // A device reporting network ID 0 has not been told its network yet;
        // the provisioning dialog owns that, and warning here would put a
        // scary orange line on every single first connect.
        compose.setContent {
            NetworkSection(
                session(BleDeviceType.BASE_STATION),
                appNetwork("Bench"),
                knownDevices(),
            )
        }
        compose.onNodeWithText("Devices only hear each other on the same network ID.")
            .assertIsDisplayed()
    }

    @Test
    fun frequency_reads_as_unknown_rather_than_as_a_number_nobody_measured() {
        // No readback yet. Rendering a plausible default here would assert a
        // channel the base station never reported.
        compose.setContent { LoraSections(session(BleDeviceType.BASE_STATION)) }
        compose.onNodeWithText("—").assertIsDisplayed()
    }
}
