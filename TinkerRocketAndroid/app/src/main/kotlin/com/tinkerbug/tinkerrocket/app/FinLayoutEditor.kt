package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Servo→fin mapping + ring orientation editor — port of iOS FinLayoutView
 * (FinConfigData, cmd 66).  Nose-down ring (looking from the nose aft: +Z up,
 * +Y right); "+" (on-axis) or "×" (45°); per-SERVO tilt + roll reverses; a
 * servo jog (servo-test path, cmd 24/25) confirms direction on the bench.
 * Pure presentation: values in, edit closures out — the parent owns the
 * profile and sends the config.
 */
@Composable
fun FinLayoutEditor(
    ringMode: Int,
    servoAtSlot: List<Int>,
    reverse: List<Boolean>,
    rollReverse: List<Boolean>,
    canJog: Boolean,
    finMinDeg: Float,
    finMaxDeg: Float,
    onSetRingMode: (Int) -> Unit,
    onSetServoAtSlot: (List<Int>) -> Unit,
    onSetReverse: (List<Boolean>) -> Unit,
    onSetRollReverse: (List<Boolean>) -> Unit,
    onJogAngles: (List<Double>) -> Unit,
    onJogStop: () -> Unit,
) {
    var selectedSlot by remember { mutableIntStateOf(0) }
    // Control azimuth (deg) of each ring slot — the firmware convention
    // (θ=0 top/+pitch, 90 +yaw, …); the nose-down view is rendering only.
    val slotAzimuths =
        if (ringMode == 1) listOf(45.0, 135.0, 225.0, 315.0) else listOf(0.0, 90.0, 180.0, 270.0)
    val selectedServo = servoAtSlot.getOrElse(selectedSlot) { 1 }

    Column(verticalArrangement = Arrangement.spacedBy(10.dp)) {
        // Orientation picker
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            SegBtn("+ on axes", ringMode == 0) { onSetRingMode(0) }
            SegBtn("× 45°", ringMode == 1) { onSetRingMode(1) }
        }

        // Ring
        BoxWithConstraints(Modifier.fillMaxWidth().height(230.dp)) {
            val cx = maxWidth / 2
            val cy = 115.dp
            val r = min(cx.value, cy.value).dp - 26.dp
            val ringColor = MaterialTheme.colorScheme.onSurfaceVariant

            Canvas(Modifier.fillMaxWidth().height(230.dp)) {
                drawCircle(
                    color = ringColor.copy(alpha = 0.6f),
                    radius = r.toPx(),
                    center = Offset(cx.toPx(), cy.toPx()),
                    style = Stroke(width = 1.5.dp.toPx()),
                )
                drawCircle(
                    color = ringColor.copy(alpha = 0.5f),
                    radius = 3.5.dp.toPx(),
                    center = Offset(cx.toPx(), cy.toPx()),
                )
            }

            // Axis labels: +Z top, +Y right, −Z bottom, −Y left (nose-down).
            listOf("+Z", "+Y", "−Z", "−Y").forEachIndexed { i, label ->
                val p = ringPoint(i * 90.0, r + 16.dp, cx, cy)
                Text(
                    label,
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.offset(p.first - 8.dp, p.second - 8.dp),
                )
            }

            for (slot in 0 until 4) {
                // Screen angle = nose-down mirror of the control azimuth.
                val scrA = (360.0 - slotAzimuths[slot]).mod(360.0)
                val sel = slot == selectedSlot
                val servo = servoAtSlot.getOrElse(slot) { slot + 1 }
                val blade = ringPoint(scrA, r, cx, cy)
                val badge = ringPoint(scrA, r - 22.dp, cx, cy)

                Box(
                    Modifier
                        .offset(blade.first - 3.dp, blade.second - 13.dp)
                        .size(6.dp, 26.dp)
                        .rotate(scrA.toFloat())
                        .background(
                            if (sel) MaterialTheme.colorScheme.primary
                            else MaterialTheme.colorScheme.onSurfaceVariant,
                        ),
                )
                Box(
                    Modifier
                        .offset(badge.first - 16.dp, badge.second - 16.dp)
                        .size(32.dp)
                        .background(
                            if (sel) MaterialTheme.colorScheme.primary
                            else MaterialTheme.colorScheme.surfaceVariant,
                            CircleShape,
                        )
                        .clickable { selectedSlot = slot },
                    contentAlignment = Alignment.Center,
                ) {
                    Text(
                        "$servo",
                        style = MaterialTheme.typography.titleMedium,
                        color = if (sel) Color.White else MaterialTheme.colorScheme.onSurface,
                    )
                }
            }
        }
        Text(
            "Viewed from the nose, looking aft · +Z up, +Y right · tap a fin",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            modifier = Modifier.fillMaxWidth(),
        )

        // Selected-fin editor
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                slotName(ringMode, selectedSlot),
                style = MaterialTheme.typography.titleSmall,
            )
            Text(
                "drives: ${mixDescription(slotAzimuths[selectedSlot], selectedServo, reverse, rollReverse)}",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }

        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Servo here", style = MaterialTheme.typography.bodyLarge)
            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                for (s in 1..4) {
                    SegBtn("$s", s == selectedServo) {
                        // Assigning a servo already on another slot SWAPS them.
                        val slots = servoAtSlot.toMutableList()
                        if (selectedSlot !in slots.indices) return@SegBtn
                        val other = slots.indexOf(s)
                        if (other >= 0) slots[other] = slots[selectedSlot]
                        slots[selectedSlot] = s
                        onSetServoAtSlot(slots)
                    }
                }
            }
        }

        ToggleRowSmall("Reverse pitch/yaw (tilt)", reverse.getOrElse(selectedServo - 1) { false }) { v ->
            if (selectedServo - 1 in reverse.indices) {
                onSetReverse(reverse.toMutableList().also { it[selectedServo - 1] = v })
            }
        }
        ToggleRowSmall("Reverse roll", rollReverse.getOrElse(selectedServo - 1) { false }) { v ->
            if (selectedServo - 1 in rollReverse.indices) {
                onSetRollReverse(rollReverse.toMutableList().also { it[selectedServo - 1] = v })
            }
        }

        Text(
            "Jog servo $selectedServo to confirm direction",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(enabled = canJog, onClick = { jog(-10.0, selectedServo, reverse, finMinDeg, finMaxDeg, onJogAngles) }) {
                Text("−10°")
            }
            OutlinedButton(enabled = canJog, onClick = onJogStop) { Text("Stop") }
            OutlinedButton(enabled = canJog, onClick = { jog(10.0, selectedServo, reverse, finMinDeg, finMaxDeg, onJogAngles) }) {
                Text("+10°")
            }
        }
        if (!canJog) {
            Text(
                "Connect and power on the rocket to jog the servos.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

/** Screen point for a clockwise-from-top angle at [radius] from (cx, cy). */
private fun ringPoint(clockwiseDeg: Double, radius: Dp, cx: Dp, cy: Dp): Pair<Dp, Dp> {
    val a = Math.toRadians(clockwiseDeg)
    return (cx + (radius.value * sin(a)).dp) to (cy - (radius.value * cos(a)).dp)
}

private fun slotName(ringMode: Int, slot: Int): String {
    val plus = listOf("Top · +Z", "Left · −Y", "Bottom · −Z", "Right · +Y")
    val cross = listOf("Upper-left", "Lower-left", "Lower-right", "Upper-right")
    return (if (ringMode == 1) cross else plus).getOrElse(slot) { "Fin" }
}

/** iOS mixDescription verbatim: signs of the pitch/yaw/roll contributions. */
private fun mixDescription(
    azimuthDeg: Double,
    servo: Int,
    reverse: List<Boolean>,
    rollReverse: List<Boolean>,
): String {
    val th = Math.toRadians(azimuthDeg)
    val tilt = if (reverse.getOrElse(servo - 1) { false }) -1.0 else 1.0
    val rollS = if (rollReverse.getOrElse(servo - 1) { false }) -1.0 else 1.0
    val c = cos(th) * tilt
    val s = sin(th) * tilt
    val parts = mutableListOf<String>()
    if (abs(c) > 0.05) parts += "pitch ${if (c > 0) "+" else "−"}"
    if (abs(s) > 0.05) parts += "yaw ${if (s > 0) "+" else "−"}"
    parts += "roll ${if (rollS > 0) "+" else "−"}"
    return parts.joinToString(" · ")
}

/**
 * Jog with the per-servo reverse applied (so the jog reflects the
 * controller's "+") and clamped to the profile's fin-cal range (#407) — the
 * FC clamps too, but matching it keeps the app honest about what it sends.
 */
private fun jog(
    deg: Double,
    servo: Int,
    reverse: List<Boolean>,
    finMinDeg: Float,
    finMaxDeg: Float,
    onJogAngles: (List<Double>) -> Unit,
) {
    val idx = servo - 1
    if (idx !in 0 until 4) return
    val target = if (reverse.getOrElse(idx) { false }) -deg else deg
    val lo = min(finMinDeg, finMaxDeg).toDouble()
    val hi = max(finMinDeg, finMaxDeg).toDouble()
    val angles = MutableList(4) { 0.0 }
    angles[idx] = target.coerceIn(lo, hi)
    onJogAngles(angles)
}

@Composable
private fun SegBtn(label: String, selected: Boolean, onClick: () -> Unit) {
    if (selected) {
        Button(onClick = onClick) { Text(label) }
    } else {
        OutlinedButton(onClick = onClick) { Text(label) }
    }
}

@Composable
private fun ToggleRowSmall(label: String, value: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, style = MaterialTheme.typography.bodyLarge)
        androidx.compose.material3.Switch(checked = value, onCheckedChange = onChange)
    }
}
