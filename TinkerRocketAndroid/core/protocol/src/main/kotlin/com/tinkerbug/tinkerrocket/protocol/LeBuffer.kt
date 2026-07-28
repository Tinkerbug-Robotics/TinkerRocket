package com.tinkerbug.tinkerrocket.protocol

/**
 * Sequential little-endian reader over a [ByteArray].
 *
 * EVERY multi-byte field in the TinkerRocket wire protocol is little-endian
 * (the only exception is the frame CRC16, which [MessageParser] handles
 * itself).  The JVM's ByteBuffer defaults to BIG-endian, and one missed
 * `order()` call silently corrupts every numeric payload — so wire decoding
 * goes through this reader instead, and `java.nio` stays out of the module's
 * public surface (which also keeps the KMP door open, plan §1).
 *
 * Unsigned widths widen: u8→Int, u16→Int, u32→Long.  A u32 like `time_us`
 * exceeds Int.MAX_VALUE ~35.8 min after boot — a pad-wait bug if truncated —
 * which is why there is deliberately no `u32AsInt`.
 */
public class LeBuffer(
    private val bytes: ByteArray,
    public var position: Int = 0,
) {
    public val remaining: Int get() = bytes.size - position

    public fun u8(): Int = (bytes[position++].toInt() and 0xFF)

    public fun i8(): Int = bytes[position++].toInt()

    public fun u16(): Int {
        val v = (bytes[position].toInt() and 0xFF) or
            ((bytes[position + 1].toInt() and 0xFF) shl 8)
        position += 2
        return v
    }

    public fun i16(): Int = u16().toShort().toInt()

    public fun u32(): Long {
        val v = (bytes[position].toLong() and 0xFF) or
            ((bytes[position + 1].toLong() and 0xFF) shl 8) or
            ((bytes[position + 2].toLong() and 0xFF) shl 16) or
            ((bytes[position + 3].toLong() and 0xFF) shl 24)
        position += 4
        return v
    }

    public fun i32(): Int = u32().toInt()

    public fun u64(): Long {
        var v = 0L
        for (i in 0 until 8) {
            v = v or ((bytes[position + i].toLong() and 0xFF) shl (8 * i))
        }
        position += 8
        return v
    }

    public fun f32(): Float = Float.fromBits(i32())

    public fun f64(): Double = Double.fromBits(u64())

    public fun bytes(n: Int): ByteArray {
        val out = bytes.copyOfRange(position, position + n)
        position += n
        return out
    }
}
