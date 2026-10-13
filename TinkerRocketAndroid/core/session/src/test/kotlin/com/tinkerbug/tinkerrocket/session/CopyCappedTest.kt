package com.tinkerbug.tinkerrocket.session

import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.InputStream
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals

/**
 * [copyCapped] in place of `InputStream.readNBytes`, which Android 12 does
 * not have: the whole body up to the cap, however small the pieces it
 * arrives in, and nothing past the cap.
 */
class CopyCappedTest {

    /** A body as a socket hands it out: at most 1,500 bytes a read. */
    private class Trickle(body: ByteArray) : ByteArrayInputStream(body) {
        override fun read(b: ByteArray, off: Int, len: Int): Int =
            super.read(b, off, minOf(len, 1_500))
    }

    // One byte past 13 whole reads, so the cap cuts the last read to a byte.
    private val cap = 13 * 1_500 + 1

    private fun capped(input: InputStream): ByteArray =
        ByteArrayOutputStream().also { copyCapped(input, it, cap) }.toByteArray()

    @Test
    fun `a body under the cap is read whole`() {
        val body = Random(1).nextBytes(cap - 1)
        assertContentEquals(body, capped(Trickle(body)))
    }

    @Test
    fun `a body at the cap is read whole`() {
        val body = Random(2).nextBytes(cap)
        assertContentEquals(body, capped(Trickle(body)))
    }

    @Test
    fun `a body over the cap stops at the cap and leaves the rest unread`() {
        val body = Random(3).nextBytes(cap + 1)
        val input = Trickle(body)
        assertContentEquals(body.copyOf(cap), capped(input))
        assertEquals(1, input.available())
    }
}
