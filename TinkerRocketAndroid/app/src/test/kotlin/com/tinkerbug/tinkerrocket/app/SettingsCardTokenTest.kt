package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Column
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.app.theme.TinkerRocketTheme
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import org.junit.Rule
import org.junit.Test
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.runner.RunWith

/**
 * #624: does the Settings screen actually need a "design-token re-skin"?
 *
 * The parity ledger has carried Settings since 2026-08-09 as "still
 * functional-Material: 4 `Tr*` component uses against 12 raw Material colors".
 * The screen contains **zero** raw `Color(0x…)` literals — the 17 references it
 * does have are `MaterialTheme.colorScheme` ROLES, which is precisely what the
 * design language asks Android screens to reach for.
 *
 * The remaining question could not be settled by reading either file, because
 * it depends on which role Material3's `Card` resolves to in this Compose
 * version. So measure it: if the card a `Section` draws is already the
 * TinkerRocket card token, there is nothing to re-skin.
 */
@RunWith(AndroidJUnit4::class)
class SettingsCardTokenTest {

    @get:Rule val compose = createComposeRule()

    private fun underTheme(body: @Composable () -> Unit) {
        compose.setContent { TinkerRocketTheme { Column { body() } } }
    }

    @Test
    fun a_settings_card_is_already_painted_with_the_tinkerrocket_card_token() {
        // `Section` in SettingsScreen is a Material `Card`. Whatever role
        // CardDefaults picks, the theme maps every surface-container role onto
        // a TinkerRocket token, so the fill must be one of them.
        var cardFill: Color? = null
        var tokens: List<Color> = emptyList()
        underTheme {
            Card {
                cardFill = CardDefaults.cardColors().containerColor
                tokens = listOf(TrTheme.colors.card, TrTheme.colors.cardSecondary)
            }
        }
        assertTrue(
            "a Settings card renders $cardFill, which is neither card nor cardSecondary — " +
                "that WOULD be a re-skin gap",
            cardFill in tokens,
        )
    }

    @Test
    fun every_surface_container_role_resolves_to_a_tinkerrocket_token() {
        // This is the mechanism the whole "screens reach for roles" rule rests
        // on. If a role ever goes back to a stock Material value, every screen
        // using it silently drifts, and the ledger's complaint becomes true.
        var scheme: androidx.compose.material3.ColorScheme? = null
        var tr: com.tinkerbug.tinkerrocket.app.theme.TrColors? = null
        underTheme { scheme = MaterialTheme.colorScheme; tr = TrTheme.colors }
        val s = requireNotNull(scheme)
        val t = requireNotNull(tr)
        assertEquals(t.background, s.surfaceContainerLowest)
        assertEquals(t.card, s.surfaceContainerLow)
        assertEquals(t.card, s.surfaceContainer)
        assertEquals(t.cardSecondary, s.surfaceContainerHigh)
        assertEquals(t.cardSecondary, s.surfaceContainerHighest)
        assertEquals(t.card, s.surfaceVariant)
    }
}
