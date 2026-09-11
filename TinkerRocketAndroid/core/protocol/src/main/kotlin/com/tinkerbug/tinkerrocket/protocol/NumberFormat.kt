package com.tinkerbug.tinkerrocket.protocol

import java.math.BigDecimal
import java.math.RoundingMode

/**
 * Fixed-decimal formatter matching C/Swift `printf("%.Nf")` semantics.
 *
 * ROUNDING POLICY (the single place every display and CSV float column rounds
 * through): Swift `String(format:)` is C printf — it rounds the EXACT binary
 * value of the double to N decimals, ties-to-even. Kotlin `String.format("%.Nf")`
 * uses HALF_UP instead, so it is NOT used here; `BigDecimal(value)` (the exact
 * binary expansion) + `setScale(HALF_EVEN)` reproduces printf. printf also keeps
 * the sign on negative values that round to zero (`-0.0` → `"-0.00"`), which
 * BigDecimal drops (it has no signed zero) — restored explicitly. Locale note:
 * BigDecimal.toPlainString / Int.toString are locale-independent ('.' decimal
 * separator always) — equivalent to Locale.ROOT, with no formatter that could
 * inject a locale-dependent comma.
 *
 * #1086: lifted out of CsvGenerator so UnitFormatter's display path uses the
 * same tie-to-even rounding as the CSV and as iOS. Before this the display
 * twin went through `String.format` (HALF_UP), so a frame carrying an exact
 * .5 (e.g. palt = 124.5, ~1 frame in 20 at one decimal) printed 125 m on
 * Android and 124 m on iOS — two phones on the pad disagreeing by 1 m.
 */
public fun formatFixed(value: Double, decimals: Int): String {
    if (value.isNaN()) return "nan"
    if (value.isInfinite()) return if (value > 0) "inf" else "-inf"
    var s = BigDecimal(value).setScale(decimals, RoundingMode.HALF_EVEN).toPlainString()
    val negative = value < 0.0 || (value == 0.0 && 1.0 / value < 0.0)
    if (negative && !s.startsWith("-")) s = "-$s"
    return s
}
