package com.tinkerbug.tinkerrocket.app

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.semantics.SemanticsProperties
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performImeAction
import androidx.compose.ui.test.performTextInput
import androidx.compose.ui.test.performTextReplacement
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.protocol.AntennaOffset
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The GNSS antenna offset section's commits (cmd 81). Each one is written to
 * the rocket, and a Measured offset of zeros tells the navigation filter the
 * antenna sits on the IMU, with tight doubt. So the switch to Measured sends
 * nothing, the three fields go out only together, and leaving the screen
 * sends only what the user typed.
 *
 * The section is composed over a rocket that reports the profile's value,
 * and a commit lands in the profile at once, as the screen's `edit` does.
 */
@RunWith(AndroidJUnit4::class)
class AntennaOffsetSectionTest {

    @get:Rule val compose = createComposeRule()

    private val commits = mutableListOf<AntennaOffset>()
    private var profile by mutableStateOf(AntennaOffset.UNKNOWN)
    private var shown by mutableStateOf(true)

    private fun section(start: AntennaOffset) {
        profile = start
        compose.setContent {
            if (shown) {
                AntennaOffsetSection(
                    offset = profile,
                    connected = true,
                    haveConfig = true,
                    reported = start,
                    reportedByFlightComputer = true,
                    receiver = "mosaic-G5",
                    inflight = false,
                ) {
                    commits += it
                    profile = it
                }
            }
        }
    }

    /** Take the section off the screen, as leaving Settings does. */
    private fun leave() {
        shown = false
        compose.waitForIdle()
    }

    private fun field(label: String) = compose.onNodeWithText(label)

    private fun fieldText(label: String): String =
        field(label).fetchSemanticsNode().config[SemanticsProperties.EditableText].text

    private fun errorsShown(): Int =
        compose.onAllNodesWithText("enter whole millimeters", substring = true)
            .fetchSemanticsNodes().size

    @Test
    fun switching_to_measured_starts_empty_and_leaving_sends_nothing() {
        section(AntennaOffset.UNKNOWN)
        compose.onNodeWithText("Measured").performClick()

        assertEquals(listOf("", "", ""), listOf(fieldText("Board X"), fieldText("Board Y"), fieldText("Board Z")))
        // Nothing typed yet: the empty fields are no mistake, and the caption
        // says what to do.
        assertEquals(0, errorsShown())
        compose.onAllNodesWithText("Not sent yet: enter all three, then tap Done.", substring = true)
            .assertCountEquals(1)

        leave()
        assertEquals(emptyList<AntennaOffset>(), commits)
    }

    @Test
    fun one_field_alone_is_not_sent() {
        section(AntennaOffset.UNKNOWN)
        compose.onNodeWithText("Measured").performClick()
        field("Board X").performTextInput("312")
        field("Board X").performImeAction()   // Done: the field loses focus

        assertEquals(emptyList<AntennaOffset>(), commits)
        compose.onAllNodesWithText(
            "Board Y, Z: enter whole millimeters from -2000 to 2000. Nothing is sent until all three are.",
        ).assertCountEquals(1)

        leave()
        assertEquals(emptyList<AntennaOffset>(), commits)
    }

    @Test
    fun all_three_then_done_is_one_commit() {
        section(AntennaOffset.UNKNOWN)
        compose.onNodeWithText("Measured").performClick()
        // Each field takes the focus from the one before, which tries a
        // commit with the rest still empty.
        field("Board X").performTextInput("312")
        field("Board Y").performTextInput("-25")
        field("Board Z").performTextInput("4")
        assertEquals(emptyList<AntennaOffset>(), commits)
        field("Board Z").performImeAction()

        assertEquals(listOf(AntennaOffset(true, 312, -25, 4)), commits)
        assertEquals(0, errorsShown())

        leave()
        assertEquals(listOf(AntennaOffset(true, 312, -25, 4)), commits)
    }

    @Test
    fun a_measured_profile_left_untouched_is_not_sent_again() {
        section(AntennaOffset(true, 312, -25, 4))
        assertEquals(listOf("312", "-25", "4"), listOf(fieldText("Board X"), fieldText("Board Y"), fieldText("Board Z")))
        // Into a field and out again, without typing.
        field("Board Y").performImeAction()

        leave()
        assertEquals(emptyList<AntennaOffset>(), commits)
    }

    @Test
    fun an_edit_not_yet_committed_goes_out_when_the_screen_closes() {
        // The NumField rule: a tab change or the access gate closing must not
        // drop what was typed.
        section(AntennaOffset(true, 312, -25, 4))
        field("Board X").performTextReplacement("320")

        leave()
        assertEquals(listOf(AntennaOffset(true, 320, -25, 4)), commits)
    }

    @Test
    fun switching_back_to_unknown_sends_unknown_and_nothing_else() {
        section(AntennaOffset(true, 312, -25, 4))
        field("Board X").performTextReplacement("320")
        compose.onNodeWithText("Unknown").performClick()

        leave()
        assertEquals(listOf(AntennaOffset.UNKNOWN), commits)
    }
}
