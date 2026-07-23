package com.tinkerbug.tinkerrocket.protocol

/** One RSSI sample from a LoRa frequency scan, with its computed frequency. */
public data class FrequencyScanSample(
    val freqMhz: Float,
    val rssiDbm: Int,
)

/**
 * FreqScanResult — the 0xAA scan-results frame on the file_ops
 * characteristic.  Unlike the 0xCA/0xCB/0xCC/0xCD siblings, [decode] takes
 * the FULL frame INCLUDING the 0xAA discriminator, mirroring the iOS
 * parseScanResult which reads its floats at unaligned frame offsets 1 and 5.
 *
 * Wire (10 + n bytes, LE):
 *   [0]     u8  0xAA discriminator
 *   [1..4]  f32 start frequency, MHz (unaligned)
 *   [5..8]  f32 step, kHz (unaligned)
 *   [9]     u8  n = sample count
 *   [10..]  i8 × n  RSSI, dBm
 *
 * Per-sample frequency mirrors the iOS float math exactly:
 * freq_i = start + (step_khz * i) / 1000.
 */
public data class FreqScanResult(
    val startMhz: Float,
    val stepKhz: Float,
    val samples: List<FrequencyScanSample>,
) {
    public companion object {
        public const val DISCRIMINATOR: Int = 0xAA
        public const val HEADER_SIZE: Int = 10

        /**
         * Returns null on a malformed frame (short header, wrong
         * discriminator, or truncated sample list), mirroring the iOS
         * bounds-checked bail-outs.
         */
        public fun decode(frame: ByteArray): FreqScanResult? {
            if (frame.size < HEADER_SIZE) return null
            val b = LeBuffer(frame)
            if (b.u8() != DISCRIMINATOR) return null
            val startMhz = b.f32()      // offset 1, unaligned
            val stepKhz = b.f32()       // offset 5, unaligned
            val n = b.u8()              // offset 9
            if (frame.size < HEADER_SIZE + n) return null
            val samples = ArrayList<FrequencyScanSample>(n)
            for (i in 0 until n) {
                val rssi = b.i8()
                val freq = startMhz + (stepKhz * i.toFloat()) / 1000.0f
                samples.add(FrequencyScanSample(freqMhz = freq, rssiDbm = rssi))
            }
            return FreqScanResult(startMhz = startMhz, stepKhz = stepKhz, samples = samples)
        }
    }
}
