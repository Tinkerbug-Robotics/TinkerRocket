package com.tinkerbug.tinkerrocket.app.theme

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowDropDown
import androidx.compose.material.icons.filled.Check
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * The TinkerRocket component vocabulary — Compose renderings of the shapes
 * the iOS app is built from (design pass 2026-07-30, iOS = reference):
 * full-width color-filled action buttons, the compact scan button, the
 * dot-status pill, and the systemGray6 section card.  All values come from
 * the generated design tokens; screens compose these instead of styling ad
 * hoc, which is what keeps the two apps looking like one product.
 */

/** iOS top-screen action button: full width, semantic fill, white label. */
@Composable
fun TrActionButton(
    text: String,
    color: Color,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    icon: ImageVector? = null,
) {
    Row(
        modifier
            .fillMaxWidth()
            .background(color, RoundedCornerShape(TrShape.radiusButton))
            .clickable(onClick = onClick)
            .padding(vertical = TrSpacing.buttonPadV),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        icon?.let {
            Icon(it, contentDescription = null, tint = Color.White)
            Box(Modifier.width(8.dp))
        }
        Text(text, color = Color.White, fontSize = 17.sp, fontWeight = FontWeight.Medium)
    }
}

/** iOS scan-style compact button (tighter padding, semantic fill). */
@Composable
fun TrCompactButton(
    text: String,
    color: Color,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
) {
    Box(
        modifier
            .background(
                if (enabled) color else color.copy(alpha = 0.5f),
                RoundedCornerShape(TrShape.radiusButton),
            )
            .clickable(enabled = enabled, onClick = onClick)
            .padding(horizontal = TrSpacing.compactPadH, vertical = TrSpacing.compactPadV),
        contentAlignment = Alignment.Center,
    ) {
        Text(text, color = Color.White, fontSize = 17.sp, fontWeight = FontWeight.SemiBold)
    }
}

/** iOS ConnectionStatusView: colored dot + status text on a card fill. */
@Composable
fun TrStatusPill(
    dotColor: Color,
    text: String,
    modifier: Modifier = Modifier,
) {
    Row(
        modifier
            .background(TrTheme.colors.card, RoundedCornerShape(TrShape.radiusButton))
            .padding(horizontal = TrSpacing.rowSpacing, vertical = TrSpacing.compactPadV),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(
            Modifier
                .size(TrSpacing.statusDotSize)
                .background(dotColor, CircleShape),
        )
        Box(Modifier.width(8.dp))
        Text(text, style = MaterialTheme.typography.bodyLarge)
    }
}

/** iOS section card: systemGray6 fill, the standard radius, inner padding. */
@Composable
fun TrCard(
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit,
) {
    Column(
        modifier
            .fillMaxWidth()
            .background(TrTheme.colors.card, RoundedCornerShape(TrShape.radiusButton))
            .padding(TrSpacing.screenPadding),
        verticalArrangement = Arrangement.spacedBy(TrSpacing.cardInnerSpacing),
        content = content,
    )
}

/**
 * Map chrome plate — the solid backing every floating control on a map screen
 * sits on.  Text drawn straight onto tile imagery is a coin flip: the old
 * outlined source chip was blue-on-imagery and became unreadable over pale
 * desert satellite (Pixel 8, 2026-08-10), while the attribution bar two
 * corners away was perfectly legible for exactly one reason — it had a plate.
 * So nothing on a map renders without one of these behind it.
 *
 * iOS uses `.ultraThinMaterial` for the same job; Compose has no live-blur
 * primitive, so the systemGray6 card fill stands in — opaque rather than
 * translucent, which is the stronger choice over imagery anyway.
 */
@Composable
fun TrMapPlate(
    modifier: Modifier = Modifier,
    shape: Shape = RoundedCornerShape(TrShape.radiusChip),
    onClick: (() -> Unit)? = null,
    content: @Composable BoxScope.() -> Unit,
) {
    Box(
        modifier
            .shadow(2.dp, shape)
            .background(TrTheme.colors.card, shape)
            .then(if (onClick != null) Modifier.clickable(onClick = onClick) else Modifier),
        content = content,
    )
}

/**
 * iOS floating map control (MapView.swift): a 44pt square plate carrying one
 * glyph.  44 is the Apple touch-target minimum and comfortably above the
 * Android 48dp guidance once the 12dp gutter around it is counted.
 */
@Composable
fun TrMapIconButton(
    icon: ImageVector,
    contentDescription: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    tint: Color? = null,
) {
    TrMapPlate(
        modifier = modifier,
        shape = RoundedCornerShape(TrShape.radiusButton),
        onClick = onClick,
    ) {
        Icon(
            icon,
            contentDescription = contentDescription,
            tint = tint ?: MaterialTheme.colorScheme.onSurface,
            // 20 matches the iOS glyph inside the same 44pt plate.
            modifier = Modifier.align(Alignment.Center).size(20.dp),
        )
    }
}

/**
 * A "this opens a list" affordance: label + trailing chevron on a plate.  The
 * chevron is the whole point — the control it replaces was an outlined button
 * that looked like it performed an action and in fact cycled through five
 * hidden options one tap at a time.
 */
@Composable
fun TrPickerButton(
    label: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    icon: ImageVector? = null,
) {
    TrMapPlate(modifier = modifier, onClick = onClick) {
        Row(
            Modifier.padding(start = 10.dp, end = 6.dp, top = 7.dp, bottom = 7.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            icon?.let {
                Icon(
                    it,
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.onSurface,
                    modifier = Modifier.size(16.dp),
                )
                Box(Modifier.width(6.dp))
            }
            Text(label, style = MaterialTheme.typography.labelLarge)
            Icon(
                Icons.Filled.ArrowDropDown,
                contentDescription = null,
                tint = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.size(20.dp),
            )
        }
    }
}

/**
 * One row of a Tr picker menu: leading glyph, label, and a trailing check on
 * the active entry — the iOS `Menu { Picker }` row, which gets its checkmark
 * from the framework.  Compose's DropdownMenuItem has no selected state, so
 * the check is drawn here.
 */
@Composable
fun TrMenuRow(
    label: String,
    icon: ImageVector,
    selected: Boolean,
    onClick: () -> Unit,
) {
    DropdownMenuItem(
        text = {
            Text(
                label,
                style = MaterialTheme.typography.bodyLarge,
                fontWeight = if (selected) FontWeight.SemiBold else FontWeight.Normal,
            )
        },
        onClick = onClick,
        leadingIcon = {
            Icon(
                icon,
                contentDescription = null,
                tint = if (selected) {
                    TrTheme.colors.mapControl
                } else {
                    MaterialTheme.colorScheme.onSurfaceVariant
                },
            )
        },
        trailingIcon = {
            if (selected) {
                Icon(
                    Icons.Filled.Check,
                    contentDescription = "Active",
                    tint = TrTheme.colors.mapControl,
                )
            }
        },
    )
}

/** iOS BLESignalIndicator: four ascending bars, green up to strength. */
@Composable
fun TrSignalBars(rssi: Int, modifier: Modifier = Modifier) {
    val bars = when {
        rssi > -50 -> 4
        rssi > -65 -> 3
        rssi > -80 -> 2
        else -> 1
    }
    Row(modifier, horizontalArrangement = Arrangement.spacedBy(2.dp), verticalAlignment = Alignment.Bottom) {
        repeat(4) { i ->
            Box(
                Modifier
                    .width(4.dp)
                    .height((6 + i * 4).dp)
                    .background(
                        if (i < bars) TrTheme.colors.statusConnected else TrTheme.colors.cardSecondary,
                        RoundedCornerShape(1.dp),
                    ),
            )
        }
    }
}
