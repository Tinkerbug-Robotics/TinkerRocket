package com.tinkerbug.tinkerrocket.app.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color

/**
 * TinkerRocket theme: the design-token palette (iOS reference values) wired
 * into Material 3 in place of the baseline purple scheme — the piece that
 * made the two apps look unrelated (design pass 2026-07-30).  Material
 * components pick up the iOS surface family (white/black screens, systemGray6
 * cards) and blue accent; screens reach for semantic roles via [TrTheme].
 *
 * Deliberately NOT tokens: navigation mechanics, ripple feedback, fonts
 * (SF Pro is Apple-licensed; Roboto at matched sizes/weights instead).
 */
val LocalTrColors = staticCompositionLocalOf { TrColorsLight }

object TrTheme {
    val colors: TrColors
        @Composable get() = LocalTrColors.current
}

@Composable
fun TinkerRocketTheme(content: @Composable () -> Unit) {
    val dark = isSystemInDarkTheme()
    val tr = if (dark) TrColorsDark else TrColorsLight
    val scheme = if (dark) {
        darkColorScheme(
            primary = tr.savedFlights,
            onPrimary = Color.White,
            secondary = tr.myDevices,
            onSecondary = Color.White,
            tertiary = tr.servoTest,
            error = tr.voiceError,
            background = tr.background,
            onBackground = Color.White,
            surface = tr.background,
            onSurface = Color.White,
            surfaceVariant = tr.card,
            onSurfaceVariant = tr.statusIdle,
            outline = tr.statusIdle,
            surfaceContainerLowest = tr.background,
            surfaceContainerLow = tr.card,
            surfaceContainer = tr.card,
            surfaceContainerHigh = tr.cardSecondary,
            surfaceContainerHighest = tr.cardSecondary,
        )
    } else {
        lightColorScheme(
            primary = tr.savedFlights,
            onPrimary = Color.White,
            secondary = tr.myDevices,
            onSecondary = Color.White,
            tertiary = tr.servoTest,
            error = tr.voiceError,
            background = tr.background,
            onBackground = Color.Black,
            surface = tr.background,
            onSurface = Color.Black,
            surfaceVariant = tr.card,
            onSurfaceVariant = tr.statusIdle,
            outline = tr.statusIdle,
            surfaceContainerLowest = tr.background,
            surfaceContainerLow = tr.card,
            surfaceContainer = tr.card,
            surfaceContainerHigh = tr.cardSecondary,
            surfaceContainerHighest = tr.cardSecondary,
        )
    }
    // The one TinkerRocket radius (tokens: 10) with the chip radius below it
    // and the sheet radius above — mirrors the iOS cornerRadius distribution.
    val shapes = Shapes(
        extraSmall = RoundedCornerShape(TrShape.radiusChip),
        small = RoundedCornerShape(TrShape.radiusChip),
        medium = RoundedCornerShape(TrShape.radiusButton),
        large = RoundedCornerShape(TrShape.radiusSheet),
        extraLarge = RoundedCornerShape(TrShape.radiusSheet),
    )
    CompositionLocalProvider(LocalTrColors provides tr) {
        MaterialTheme(colorScheme = scheme, shapes = shapes, content = content)
    }
}
