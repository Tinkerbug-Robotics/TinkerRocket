package com.tinkerbug.tinkerrocket.protocol

/**
 * Quality band for the dashboard's LoRa / GNSS / BLE indicators — a direct port
 * of the iOS `loraColor` / `gnssColor` / `bleColor` computed properties
 * (DashboardView.swift:2899, :2918, :2938).
 *
 * Android rendered all three bars in one hardcoded amber and varied only the
 * bar HEIGHT, so a dying link and a perfect one were the same colour and the
 * grading iOS has always shown was simply absent.
 *
 * The bands live here rather than in the Composable because they are the whole
 * substance of the feature: a boundary off by one reads as a healthy link on a
 * marginal one, which is the failure that matters at a launch site.
 */
public enum class SignalQuality {
    /** Better than "good" — GNSS only, where more satellites keeps helping. */
    BEST,
    GOOD,
    FAIR,
    WEAK,
    BAD,

    /** No value at all: link down, or the metric is not reported on this path. */
    UNKNOWN,
    ;

    public companion object {
        /**
         * LoRa RSSI in dBm; null when the base station has heard nothing.
         *
         * iOS bands, ported verbatim including the strict `>` at every edge:
         * `> -70` green, `> -90` yellow, `> -110` orange, else red.
         */
        public fun forLoraRssi(rssi: Float?): SignalQuality = when {
            rssi == null -> UNKNOWN
            rssi > -70f -> GOOD
            rssi > -90f -> FAIR
            rssi > -110f -> WEAK
            else -> BAD
        }

        /**
         * GNSS satellite count.
         *
         * The odd one out, and ported as-is: it has a fourth, BETTER band above
         * 15 satellites, and its lower edges are `>=` where the RSSI bands are
         * `>`.  iOS: `> 15` blue, `>= 11` green, `>= 6` yellow, else red — with
         * no unknown state, because zero satellites is a real reading rather
         * than a missing one.
         */
        public fun forSatCount(sats: Int): SignalQuality = when {
            sats > 15 -> BEST
            sats >= 11 -> GOOD
            sats >= 6 -> FAIR
            else -> BAD
        }

        /**
         * BLE RSSI in dBm; null before the first read of the connected link.
         *
         * iOS: `> -50` green, `> -65` yellow, `> -80` orange, else red.  Tighter
         * than LoRa because it is a metres-range radio, not a kilometres one.
         */
        public fun forBleRssi(rssi: Int?): SignalQuality = when {
            rssi == null -> UNKNOWN
            rssi > -50 -> GOOD
            rssi > -65 -> FAIR
            rssi > -80 -> WEAK
            else -> BAD
        }

        /** iOS loraFill: (rssi + 130) / 100, clamped. */
        public fun loraFill(rssi: Float?): Float =
            rssi?.let { ((it + 130f) / 100f).coerceIn(0f, 1f) } ?: 0f

        /** iOS gnssFill: sats / 30, clamped. */
        public fun satFill(sats: Int): Float = (sats / 30f).coerceIn(0f, 1f)

        /** iOS bleFill: (rssi + 100) / 70, clamped. */
        public fun bleFill(rssi: Int?): Float =
            rssi?.let { ((it + 100f) / 70f).coerceIn(0f, 1f) } ?: 0f
    }
}
