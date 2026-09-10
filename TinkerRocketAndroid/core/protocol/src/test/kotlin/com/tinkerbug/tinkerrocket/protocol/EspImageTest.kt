package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue
import kotlin.test.assertIs

/**
 * #773: the header values here are taken from real images built out of this
 * repo on 2026-09-09, so the offsets are pinned against the actual ESP-IDF
 * layout rather than against my reading of it:
 *
 *   flight_computer      chip 0x0012 (ESP32-P4)  "537dc3ff-dirty-v9+20260909-1835"
 *   out_computer         chip 0x0009 (ESP32-S3)  "7410cdc6-dirty-v9+20260909-2056"
 *   rocket_computer_mini chip 0x0009 (ESP32-S3)  "7410cdc6-dirty+20260909-2043"
 */
class EspImageTest {

    private fun image(
        project: String,
        version: String,
        chipId: Int,
        magic: Int = 0xE9,
        appDescMagic: Long = 0xABCD5432L,
        size: Int = 4096,
    ): ByteArray {
        val b = ByteArray(size)
        b[0] = magic.toByte()
        b[12] = (chipId and 0xFF).toByte()
        b[13] = ((chipId shr 8) and 0xFF).toByte()
        for (i in 0 until 4) b[32 + i] = ((appDescMagic shr (8 * i)) and 0xFF).toByte()
        fun put(off: Int, s: String, len: Int) {
            val bytes = s.encodeToByteArray()
            for (i in 0 until minOf(bytes.size, len - 1)) b[off + i] = bytes[i]
        }
        put(48, version, 32)
        put(80, project, 32)
        put(112, "14:35:47", 16)
        put(128, "Sep  9 2026", 16)
        put(144, "v6.0.1-dirty", 32)
        return b
    }

    @Test
    fun `parses a real flight computer header`() {
        val img = EspImage.parse(
            image("flight_computer", "537dc3ff-dirty-v9+20260909-1835", 0x0012)
        )!!
        assertEquals("flight_computer", img.projectName)
        assertEquals("537dc3ff-dirty-v9+20260909-1835", img.version)
        assertEquals(0x0012, img.chipId)
        assertEquals("ESP32-P4", img.chipName)
        assertEquals("v6.0.1-dirty", img.idfVersion)
        assertEquals("Sep  9 2026", img.buildDate)
        assertEquals("v9", img.boardSuffix)
    }

    @Test
    fun `the mini's flight computer image is labelled, not universal`() {
        // The suffix letter is not always `v`. This parsed as "no board" until
        // 2026-09-10, so the catalog read the one image that fits exactly one
        // board as the image that fits every board — and offered it as the
        // default to an out-of-the-box V7/V8/V9 whose revision is unknown.
        // The firmware never had the bug: TR_OTA_Receiver strstr's "-m1" and
        // refused, so the app proposed a flash the board then rejected.
        val img = EspImage.parse(
            image("flight_computer", "7410cdc6-dirty-m1+20260909-2043", 0x0009)
        )!!
        assertEquals("m1", img.boardSuffix)
    }

    @Test
    fun `a base station image carries its revision`() {
        // Two base-station images ship in every release and they are not
        // interchangeable — V2 is an 8 MB board, V3 a 16 MB one with a
        // different partition table. Neither carried a suffix until
        // base_station/CMakeLists.txt started stamping one.
        val img = EspImage.parse(
            image("base_station", "abc1234-v3+20260910-0950", 0x0009)
        )!!
        assertEquals("v3", img.boardSuffix)
    }

    @Test
    fun `the mini's own project labels itself with -b`() {
        val img = EspImage.parse(
            image("rocket_computer_mini", "7410cdc6-b1+20260909-2043", 0x0009)
        )!!
        assertEquals("b1", img.boardSuffix)
    }

    @Test
    fun `the date tail is not mistaken for a board suffix`() {
        // The build date is "+20260909-2043"; a laxer pattern reads "-2043".
        val img = EspImage.parse(
            image("rocket_computer_mini", "7410cdc6-dirty+20260909-2043", 0x0009)
        )!!
        assertNull(img.boardSuffix)
    }

    @Test
    fun `a build with no board suffix reports none`() {
        val img = EspImage.parse(
            image("rocket_computer_mini", "7410cdc6-dirty+20260909-2043", 0x0009)
        )!!
        assertNull(img.boardSuffix)
        assertEquals("ESP32-S3", img.chipName)
    }

    @Test
    fun `refuses anything that is not an ESP-IDF image`() {
        assertNull(EspImage.parse(ByteArray(16)))                               // too short
        assertNull(EspImage.parse(image("x", "y", 9, magic = 0x50)))            // not 0xE9
        assertNull(EspImage.parse(image("x", "y", 9, appDescMagic = 0L)))       // no app desc
        val v = EspImage.check(ByteArray(16), EspImage.PROJECT_FC)
        assertIs<EspImageVerdict.Refuse>(v)
        assertTrue(v.reason.contains("not an ESP-IDF firmware image"))
    }

    @Test
    fun `the case this exists for - a base station image aimed at the out computer`() {
        // Both are ESP32-S3 with byte-identical app slots, so nothing downstream
        // separates them: TR_OTA checks size and SHA-256 only.
        val bs = image("base_station", "abc1234-v5+20260909-1200", 0x0009)
        val v = EspImage.check(bs, EspImage.PROJECT_OC, expectedChipId = 0x0009)
        assertIs<EspImageVerdict.Refuse>(v)
        assertTrue(v.reason.contains("base_station"))
        assertTrue(v.reason.contains("out_computer"))
    }

    @Test
    fun `the right image for the right unit passes clean`() {
        val oc = image("out_computer", "7410cdc6-dirty-v9+20260909-2056", 0x0009)
        val v = EspImage.check(oc, EspImage.PROJECT_OC, expectedChipId = 0x0009,
                               runningVersion = "0000000-v9+20260901-0900")
        assertIs<EspImageVerdict.Ok>(v)
        assertEquals("out_computer", v.image.projectName)
    }

    @Test
    fun `a chip mismatch warns rather than refuses`() {
        // The flight computer is an ESP32-P4 on V9 and an ESP32-S3 on the mini,
        // so this cannot be a hard gate without blocking a legitimate update.
        val fc = image("flight_computer", "abc1234+20260909-1200", 0x0009)
        val v = EspImage.check(fc, EspImage.PROJECT_FC, expectedChipId = 0x0012)
        assertIs<EspImageVerdict.Warn>(v)
        assertTrue(v.reason.contains("ESP32-S3"))
        assertTrue(v.reason.contains("ESP32-P4"))
    }

    @Test
    fun `a board suffix disagreeing with the running firmware warns`() {
        val fc = image("flight_computer", "abc1234-v8+20260909-1200", 0x0012)
        val v = EspImage.check(fc, EspImage.PROJECT_FC, expectedChipId = 0x0012,
                               runningVersion = "0000000-v9+20260901-0900")
        assertIs<EspImageVerdict.Warn>(v)
        assertTrue(v.reason.contains("v8"))
        assertTrue(v.reason.contains("v9"))
    }

    @Test
    fun `an unnamed program is refused by name-free wording`() {
        val v = EspImage.check(image("", "abc+1", 0x0009), EspImage.PROJECT_BS)
        assertIs<EspImageVerdict.Refuse>(v)
        assertTrue(v.reason.contains("unnamed program"))
    }

    @Test
    fun `fields stop at the first NUL and never run into the next one`() {
        val img = EspImage.parse(image("out_computer", "v1", 0x0009))!!
        assertEquals("out_computer", img.projectName)
        assertEquals("v1", img.version)
        assertEquals("v6.0.1-dirty", img.idfVersion)
    }

    // ── #773 step 2: the provisioned revision beats the image's own claim ──

    @Test
    fun `the provisioned board wins over the running version`() {
        // The scenario the whole feature exists for: a V9 board that was
        // wrongly flashed with a V8 image reports v8 forever, so comparing the
        // picked image against the RUNNING version would happily agree with
        // the mistake. The board's own answer catches it.
        val v8image = image("flight_computer", "abc-v8+1", 0x0012)
        val v = EspImage.check(
            v8image, EspImage.PROJECT_FC,
            runningVersion = "0000000-v8+20260901-0900",   // the wrong flash, agreeing with itself
            provisionedBoard = "v9",
        )
        assertIs<EspImageVerdict.Warn>(v)
        assertTrue(v.reason.contains("provisioned as v9"))
    }

    @Test
    fun `a provisioned board that agrees is silent`() {
        val v = EspImage.check(
            image("flight_computer", "abc-v9+1", 0x0012),
            EspImage.PROJECT_FC, runningVersion = "0000000-v8+1", provisionedBoard = "V9",
        )
        assertIs<EspImageVerdict.Ok>(v)
    }

    @Test
    fun `without a provisioned board it falls back and says so`() {
        val v = EspImage.check(
            image("flight_computer", "abc-v8+1", 0x0012),
            EspImage.PROJECT_FC, runningVersion = "0000000-v9+1", provisionedBoard = null,
        )
        assertIs<EspImageVerdict.Warn>(v)
        assertTrue(v.reason.contains("not provisioned"))
        assertTrue(v.reason.contains("image's own claim"))
    }

    @Test
    fun `an empty provisioned string is treated as unprovisioned`() {
        // The firmware sends "" rather than omitting the key when a board has
        // never been provisioned; that must not compare as a board named "".
        val v = EspImage.check(
            image("flight_computer", "abc-v8+1", 0x0012),
            EspImage.PROJECT_FC, runningVersion = "0000000-v9+1", provisionedBoard = "  ",
        )
        assertIs<EspImageVerdict.Warn>(v)
        assertTrue(v.reason.contains("not provisioned"))
    }

    @Test
    fun `an image with no board suffix is never board-warned`() {
        // rocket_computer_mini carries no suffix; there is nothing to compare,
        // and inventing a mismatch would be worse than saying nothing.
        val v = EspImage.check(
            image("rocket_computer_mini", "abc+20260909-2043", 0x0009),
            EspImage.PROJECT_MINI, expectedChipId = 0x0009, provisionedBoard = "v9",
        )
        assertIs<EspImageVerdict.Ok>(v)
    }
}
