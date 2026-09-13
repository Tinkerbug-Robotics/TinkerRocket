package com.tinkerbug.tinkerrocket.app

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithContentDescription
import androidx.compose.ui.test.onNodeWithContentDescription
import androidx.compose.ui.test.onNodeWithText
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.DiscoveredDevice
import com.tinkerbug.tinkerrocket.session.PreflightAutoStatus
import com.tinkerbug.tinkerrocket.session.PreflightItem
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * #624: which status icons a screen reader should announce, and which it
 * should pass over in silence.
 *
 * There is one rule and it cuts both ways. An icon gets a contentDescription
 * when it is the ONLY thing on its row carrying that fact, and `null` when the
 * row already says it in words -- because a labelled-but-redundant icon makes
 * TalkBack announce the same thing twice, which is worse than silence.
 *
 * PR #1442 applied that rule by reading the source, could not run it, and got
 * two of four wrong: it labelled the scanner's device-type glyph and the
 * preflight complete tick, both of which sit beside text that already says the
 * same thing. Those two are now `null` again, and the cases below pin each
 * direction so the next reader does not have to re-derive it.
 */
@RunWith(AndroidJUnit4::class)
class StatusIconLabelTest {

    @get:Rule val compose = createComposeRule()

    private fun device(name: String, type: BleDeviceType?) =
        DiscoveredDevice(deviceId = "AA:BB:CC:DD:EE:FF", name = name, rssi = -55, knownType = type)

    // ---- silent: the row already says it ----

    @Test
    fun scanner_row_states_the_device_kind_in_words_and_the_glyph_stays_silent() {
        // The failure this guards against is the one #1442 shipped: the icon
        // labelled "Rocket" immediately before a text line reading "Rocket".
        var type by mutableStateOf(BleDeviceType.ROCKET)
        compose.setContent { DeviceRow(device("TR-Bench", type)) {} }

        for ((t, word) in listOf(
            BleDeviceType.ROCKET to "Rocket",
            BleDeviceType.BASE_STATION to "Base Station",
            BleDeviceType.UNKNOWN to "TinkerRocket device",
        )) {
            type = t
            compose.waitForIdle()
            // Said once, in words, by the subtitle.
            compose.onNodeWithText(word).assertIsDisplayed()
            // And not a second time by the icon.
            compose.onAllNodesWithContentDescription(word).assertCountEquals(0)
        }
    }

    @Test
    fun scanner_row_still_reads_out_the_device_name() {
        // Silencing the icon must not silence the row: the name and the kind
        // are both still reachable as text.
        compose.setContent { DeviceRow(device("TR-Bench", BleDeviceType.BASE_STATION)) {} }
        compose.onNodeWithText("TR-Bench").assertIsDisplayed()
        compose.onNodeWithText("Base Station").assertIsDisplayed()
    }

    // ---- labelled: the glyph is the only signal ----

    @Test
    fun manual_checklist_tick_says_whether_the_step_is_done() {
        // The only visual cue for a done step is a strikethrough on the title,
        // and strikethrough carries no semantics at all. Without this label a
        // checklist reads aloud identically whether or not anything is ticked.
        val item = PreflightItem(title = "Igniter continuity", detail = "beep test")
        var checked by mutableStateOf(false)
        compose.setContent { PreflightManualRunRow(item, checked) { checked = it } }

        compose.onNodeWithContentDescription("Not checked").assertIsDisplayed()

        checked = true
        compose.waitForIdle()
        compose.onNodeWithContentDescription("Checked").assertIsDisplayed()

        // The words never moved; only the tick did.
        compose.onNodeWithText("Igniter continuity").assertIsDisplayed()
    }

    @Test
    fun auto_step_status_names_each_state_it_can_report() {
        val item = PreflightItem(title = "GNSS fix")
        var status by mutableStateOf<PreflightAutoStatus>(PreflightAutoStatus.Satisfied)
        compose.setContent { PreflightAutoRunRow(item, status) }

        compose.onNodeWithContentDescription("Satisfied").assertIsDisplayed()

        status = PreflightAutoStatus.Pending("waiting for 4 sats")
        compose.waitForIdle()
        compose.onNodeWithContentDescription("Pending").assertIsDisplayed()

        status = PreflightAutoStatus.NotApplicable("no camera configured")
        compose.waitForIdle()
        compose.onNodeWithContentDescription("Not applicable").assertIsDisplayed()
    }

    @Test
    fun satisfied_is_the_one_auto_state_with_no_words_on_the_row() {
        // Pending and NotApplicable both print their `reason` underneath, so a
        // screen reader gets something either way. Satisfied prints only the
        // item title, so its label is the sole evidence the step passed. This
        // is the case that makes the whole auto-status label load-bearing.
        val item = PreflightItem(title = "GNSS fix")
        var status by mutableStateOf<PreflightAutoStatus>(PreflightAutoStatus.Satisfied)
        compose.setContent { PreflightAutoRunRow(item, status) }

        compose.onNodeWithContentDescription("Satisfied").assertIsDisplayed()
        compose.onAllNodesWithContentDescription("Pending").assertCountEquals(0)

        status = PreflightAutoStatus.Pending("waiting for 4 sats")
        compose.waitForIdle()
        compose.onNodeWithText("waiting for 4 sats").assertIsDisplayed()
    }

    @Test
    fun the_auto_bolt_is_labelled_because_nothing_else_marks_a_step_automatic() {
        // Manual and auto steps are told apart by this bolt alone -- no text
        // on either row says which kind it is. An earlier draft of this suite
        // assumed the bolt was decorative and failed here, which is the point:
        // the rule is "is this fact anywhere else on the row", not a guess.
        compose.setContent {
            PreflightAutoRunRow(
                PreflightItem(title = "GNSS fix"),
                PreflightAutoStatus.Pending("waiting for 4 sats"),
            )
        }
        compose.onNodeWithContentDescription("Auto").assertIsDisplayed()
    }
}
