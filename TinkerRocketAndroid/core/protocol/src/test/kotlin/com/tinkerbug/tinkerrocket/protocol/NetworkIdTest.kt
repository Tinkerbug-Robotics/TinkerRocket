package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull

/**
 * #150 network-name → network-ID contract — port of iOS NetworkIdTests.
 * Pinned so a refactor can't silently re-map every fielded device.
 */
class NetworkIdTest {

    @Test
    fun `fnv1a8 is deterministic`() {
        assertEquals(NetworkId.fnv1a8("Skyhawks Club"), NetworkId.fnv1a8("Skyhawks Club"))
        assertEquals(NetworkId.fnv1a8(""), NetworkId.fnv1a8(""))
    }

    @Test
    fun `fnv1a8 known vector`() {
        // Golden pin: FNV-1a 32-bit of "a" is 0xE40C292C; XOR-folded to
        // 8 bits: 0x2C ^ 0x29 ^ 0x0C ^ 0xE4 = 0xED.  If this fails, the hash
        // changed and every provisioned device's ID mapping breaks.
        assertEquals(0xED, NetworkId.fnv1a8("a"))
    }

    @Test
    fun `fnv1a8 distinguishes typical names`() {
        // Not a collision-resistance proof (8 bits can't be) — just a sanity
        // check that obvious sibling names don't collide.
        assertNotEquals(NetworkId.fnv1a8("My Backyard"), NetworkId.fnv1a8("Skyhawks Club"))
    }

    @Test
    fun `networkIdForName never returns zero`() {
        // 0 is reserved: firmware factory default AND the app's "unset"
        // sentinel — a name hashing to 0 would silently disable the
        // provisioning push and the mismatch badge.  That 1-in-256 case
        // remaps to 1; everything else passes through.
        assertEquals(NetworkId.fnv1a8("a"), NetworkId.networkIdForName("a"))
        // Brute-force a zero-hash input to prove the remap engages.
        var zeroName: String? = null
        for (i in 0 until 100_000) {
            val candidate = "net$i"
            if (NetworkId.fnv1a8(candidate) == 0) {
                zeroName = candidate
                break
            }
        }
        assertNotNull(zeroName, "expected a zero-hash name within 100k candidates")
        assertEquals(1, NetworkId.networkIdForName(zeroName))
    }
}
