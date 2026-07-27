package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals

/**
 * Cross-platform pins for the #150 name→network_id hash.  Goldens computed
 * independently (Python FNV-1a reference) and matched against the Swift
 * fnv1a8 — a divergence here means a mixed iOS/Android fleet splits into
 * two deaf networks.
 */
class NetworkIdentityTest {

    @Test
    fun fnv1a8_goldenValues() {
        assertEquals(240, NetworkIdentity.fnv1a8("TinkerRocket")) // the bench fleet's nid
        assertEquals(48, NetworkIdentity.fnv1a8("Pedersen Field"))
        assertEquals(237, NetworkIdentity.fnv1a8("a"))
        assertEquals(197, NetworkIdentity.fnv1a8(""))
        assertEquals(142, NetworkIdentity.fnv1a8("Launch Crew 42"))
    }

    @Test
    fun zeroHash_remapsToOne() {
        // "az" is the shortest lowercase string hashing to 0 (found by sweep).
        assertEquals(0, NetworkIdentity.fnv1a8("az"))
        assertEquals(1, NetworkIdentity.networkIdForName("az"))
        // Non-zero hashes pass through unchanged.
        assertEquals(240, NetworkIdentity.networkIdForName("TinkerRocket"))
    }
}
