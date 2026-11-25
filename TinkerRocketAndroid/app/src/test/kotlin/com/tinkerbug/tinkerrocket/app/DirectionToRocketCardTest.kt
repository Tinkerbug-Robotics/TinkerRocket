package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.app.Activity
import android.app.Application
import android.content.Intent
import android.content.pm.PackageManager
import androidx.activity.ComponentActivity
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createAndroidComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotSame
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.Shadows.shadowOf
import org.robolectric.shadows.ShadowActivity

/**
 * #1616: the bearing card's permission states, and what its button asks for.
 *
 * Android 12+ lets the user answer a location prompt with Approximate, which
 * grants COARSE and not FINE. The arrow still needs FINE, and the card used
 * to read that answer as no answer, so it kept asking for "permission". Its
 * button also asked for FINE alone, which some Android 12 builds ignore. No
 * device was on hand to show either, so the cases below pin both.
 */
@RunWith(AndroidJUnit4::class)
class DirectionToRocketCardTest {

    @get:Rule val compose = createAndroidComposeRule<ComponentActivity>()

    private val app: Application get() = ApplicationProvider.getApplicationContext()
    private val fine = Manifest.permission.ACCESS_FINE_LOCATION
    private val coarse = Manifest.permission.ACCESS_COARSE_LOCATION

    private fun showCard() {
        val manager = PhoneLocationManager(app)
        compose.setContent { DirectionToRocketCard(manager, fix = null, rocketAltM = null) }
    }

    // ---- what the card shows for each answer ----

    @Test
    fun no_answer_yet_asks_for_location() {
        showCard()
        compose.onNodeWithText("Grant location").assertIsDisplayed()
        compose.onAllNodesWithText("Use precise location").assertCountEquals(0)
    }

    @Test
    fun approximate_only_asks_for_precise_rather_than_for_permission() {
        shadowOf(app).grantPermissions(coarse)
        showCard()
        compose.onNodeWithText("Use precise location").assertIsDisplayed()
        compose.onAllNodesWithText("Grant location").assertCountEquals(0)
    }

    @Test
    fun precise_gets_past_the_prompt() {
        shadowOf(app).grantPermissions(fine, coarse)
        showCard()
        compose.onNodeWithText("Getting phone location…").assertIsDisplayed()
        compose.onAllNodesWithText("Grant location").assertCountEquals(0)
        compose.onAllNodesWithText("Use precise location").assertCountEquals(0)
    }

    // ---- what the button asks for, through the real result plumbing ----

    @Test
    fun the_button_asks_for_fine_and_coarse_together_and_then_for_the_upgrade() {
        showCard()
        compose.onNodeWithText("Grant location").performClick()
        val first = lastRequest()
        assertEquals(setOf(fine, coarse), first.requestedPermissions.toSet())

        // The user picks Approximate: COARSE only.
        answer(coarse)
        compose.onNodeWithText("Use precise location").assertIsDisplayed()

        // Asking again sends the same pair, which is what gets Android's
        // approximate-to-precise prompt once COARSE is held.
        compose.onNodeWithText("Use precise location").performClick()
        val second = lastRequest()
        assertNotSame(first, second)
        assertEquals(setOf(fine, coarse), second.requestedPermissions.toSet())

        answer(fine, coarse)
        compose.onNodeWithText("Getting phone location…").assertIsDisplayed()
    }

    private fun lastRequest(): ShadowActivity.PermissionsRequest =
        shadowOf(compose.activity).lastRequestedPermission

    /**
     * The system's answer to the open request, sent back the way the
     * permission controller's is: through the activity, which clears its
     * request-in-flight flag before the registry calls the card.
     */
    private fun answer(vararg granted: String) {
        val activity = shadowOf(compose.activity)
        val request = activity.nextStartedActivityForResult.intent
        val names = lastRequest().requestedPermissions
        shadowOf(app).grantPermissions(*granted)
        val data = Intent()
            .putExtra(EXTRA_NAMES, names)
            .putExtra(
                EXTRA_RESULTS,
                IntArray(names.size) { i ->
                    if (names[i] in granted) {
                        PackageManager.PERMISSION_GRANTED
                    } else {
                        PackageManager.PERMISSION_DENIED
                    }
                },
            )
        compose.runOnUiThread { activity.receiveResult(request, Activity.RESULT_OK, data) }
        compose.waitForIdle()
    }

    private companion object {
        // PackageManager's keys for the permission controller's result. They
        // are hidden from the SDK stubs, so they are spelled out here.
        const val EXTRA_NAMES = "android.content.pm.extra.REQUEST_PERMISSIONS_NAMES"
        const val EXTRA_RESULTS = "android.content.pm.extra.REQUEST_PERMISSIONS_RESULTS"
    }
}
