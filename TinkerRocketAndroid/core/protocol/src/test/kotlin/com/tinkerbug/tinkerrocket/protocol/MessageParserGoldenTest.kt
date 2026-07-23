package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlin.test.Test
import kotlin.test.assertEquals

/**
 * Frame scanner semantics against the framed/ goldens: happy path,
 * bad-CRC skip-and-resync, truncated-tail stop.  These pin the Kotlin
 * parser to firmware framing (real calcCRC16 output) AND to the iOS
 * parser's documented behavior in one shot.
 */
class MessageParserGoldenTest {

    @Test
    fun `good stream parses every frame with matching types and lengths`() {
        val side = WireFixtures.sidecar("framed/stream_good.bin")
        val frames = MessageParser.parse(WireFixtures.bytes("framed/stream_good.bin"))

        assertEquals(side["frame_count"]!!.jsonPrimitive.int, frames.size)
        val types = side["types"]!!.jsonArray.map { it.jsonPrimitive.int }
        val lens = side["payload_lens"]!!.jsonArray.map { it.jsonPrimitive.int }
        assertEquals(types, frames.map { it.type })
        assertEquals(lens, frames.map { it.payload.size })
    }

    @Test
    fun `timestamp convention - first four payload bytes as LE u32`() {
        val frames = MessageParser.parse(WireFixtures.bytes("framed/stream_good.bin"))
        val gnss = frames.first { it.type == MsgType.GNSS }
        assertEquals(3600123456L, gnss.timestamp)  // canonical GNSS time_us > INT32_MAX
    }

    @Test
    fun `corrupt CRC skips that frame and resyncs on the next`() {
        val side = WireFixtures.sidecar("framed/stream_badcrc_resync.bin")
        val frames = MessageParser.parse(WireFixtures.bytes("framed/stream_badcrc_resync.bin"))

        assertEquals(side["valid_frame_count"]!!.jsonPrimitive.int, frames.size)
        val types = side["valid_types"]!!.jsonArray.map { it.jsonPrimitive.int }
        assertEquals(types, frames.map { it.type })
    }

    @Test
    fun `truncated tail stops cleanly keeping earlier frames`() {
        val side = WireFixtures.sidecar("framed/stream_truncated_tail.bin")
        val frames = MessageParser.parse(WireFixtures.bytes("framed/stream_truncated_tail.bin"))

        assertEquals(side["valid_frame_count"]!!.jsonPrimitive.int, frames.size)
        val types = side["valid_types"]!!.jsonArray.map { it.jsonPrimitive.int }
        assertEquals(types, frames.map { it.type })
    }
}
