package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals

/**
 * Band edges checked against the iOS source (DashboardView.swift `loraColor`,
 * `gnssColor`, `bleColor`) one value either side of every boundary.
 *
 * The edges are the point of these tests.  A boundary off by one renders a
 * marginal link in the colour of a healthy one, which is exactly the reading an
 * operator trusts when deciding whether to launch.
 */
class SignalQualityTest {

    // ---- LoRa: > -70 good, > -90 fair, > -110 weak, else bad ----

    @Test
    fun `lora bands match iOS at every edge`() {
        assertEquals(SignalQuality.GOOD, SignalQuality.forLoraRssi(-30f))
        assertEquals(SignalQuality.GOOD, SignalQuality.forLoraRssi(-69.9f))
        // -70 exactly is NOT good: iOS uses a strict `>`.
        assertEquals(SignalQuality.FAIR, SignalQuality.forLoraRssi(-70f))
        assertEquals(SignalQuality.FAIR, SignalQuality.forLoraRssi(-89.9f))
        assertEquals(SignalQuality.WEAK, SignalQuality.forLoraRssi(-90f))
        assertEquals(SignalQuality.WEAK, SignalQuality.forLoraRssi(-109.9f))
        assertEquals(SignalQuality.BAD, SignalQuality.forLoraRssi(-110f))
        assertEquals(SignalQuality.BAD, SignalQuality.forLoraRssi(-140f))
    }

    @Test
    fun `lora with no reading is unknown, not bad`() {
        // A silent link and a terrible one are different facts; iOS greys the
        // first and reds the second.
        assertEquals(SignalQuality.UNKNOWN, SignalQuality.forLoraRssi(null))
        assertEquals(0f, SignalQuality.loraFill(null), 0f)
    }

    // ---- GNSS: > 15 best, >= 11 good, >= 6 fair, else bad ----

    @Test
    fun `gnss bands match iOS at every edge`() {
        assertEquals(SignalQuality.BEST, SignalQuality.forSatCount(16))
        // 15 is good, not best — `>` at the top edge, unlike the two below it.
        assertEquals(SignalQuality.GOOD, SignalQuality.forSatCount(15))
        assertEquals(SignalQuality.GOOD, SignalQuality.forSatCount(11))
        assertEquals(SignalQuality.FAIR, SignalQuality.forSatCount(10))
        assertEquals(SignalQuality.FAIR, SignalQuality.forSatCount(6))
        assertEquals(SignalQuality.BAD, SignalQuality.forSatCount(5))
        assertEquals(SignalQuality.BAD, SignalQuality.forSatCount(0))
    }

    @Test
    fun `zero satellites is a real reading, never unknown`() {
        // No null overload exists on purpose: the frame always carries a count.
        assertEquals(SignalQuality.BAD, SignalQuality.forSatCount(0))
    }

    // ---- BLE: > -50 good, > -65 fair, > -80 weak, else bad ----

    @Test
    fun `ble bands match iOS at every edge`() {
        assertEquals(SignalQuality.GOOD, SignalQuality.forBleRssi(-49))
        assertEquals(SignalQuality.FAIR, SignalQuality.forBleRssi(-50))
        assertEquals(SignalQuality.FAIR, SignalQuality.forBleRssi(-64))
        assertEquals(SignalQuality.WEAK, SignalQuality.forBleRssi(-65))
        assertEquals(SignalQuality.WEAK, SignalQuality.forBleRssi(-79))
        assertEquals(SignalQuality.BAD, SignalQuality.forBleRssi(-80))
        assertEquals(SignalQuality.UNKNOWN, SignalQuality.forBleRssi(null))
    }

    // ---- Fills: iOS scales, clamped ----

    @Test
    fun `fills use the iOS scales and clamp at both ends`() {
        // LoRa spans -130..-30, not the -100..-30 this code used to assume.
        assertEquals(0f, SignalQuality.loraFill(-130f), 1e-6f)
        assertEquals(0.5f, SignalQuality.loraFill(-80f), 1e-6f)
        assertEquals(1f, SignalQuality.loraFill(-30f), 1e-6f)
        assertEquals(0f, SignalQuality.loraFill(-200f), 0f)
        assertEquals(1f, SignalQuality.loraFill(0f), 0f)

        // Sats span 0..30, not 0..12 — 12 sats used to peg the bar full.
        assertEquals(0.4f, SignalQuality.satFill(12), 1e-6f)
        assertEquals(1f, SignalQuality.satFill(30), 1e-6f)
        assertEquals(1f, SignalQuality.satFill(40), 0f)
        assertEquals(0f, SignalQuality.satFill(0), 0f)

        assertEquals(0f, SignalQuality.bleFill(-100), 1e-6f)
        assertEquals(1f, SignalQuality.bleFill(-30), 1e-6f)
        assertEquals(0f, SignalQuality.bleFill(null), 0f)
    }

    @Test
    fun `a full bar always means a good link, on every metric`() {
        // Guards the pairing rather than either half: fill and colour are
        // computed separately, so nothing else stops them drifting apart into
        // a full bar rendered red.
        for (rssi in -130..-30) {
            if (SignalQuality.loraFill(rssi.toFloat()) >= 1f) {
                assertEquals(SignalQuality.GOOD, SignalQuality.forLoraRssi(rssi.toFloat()))
            }
        }
        for (rssi in -100..-30) {
            if (SignalQuality.bleFill(rssi) >= 1f) {
                assertEquals(SignalQuality.GOOD, SignalQuality.forBleRssi(rssi))
            }
        }
        for (sats in 0..40) {
            if (SignalQuality.satFill(sats) >= 1f) {
                assertEquals(SignalQuality.BEST, SignalQuality.forSatCount(sats))
            }
        }
    }
}
