package com.tinkerbug.tinkerrocket.session

/**
 * #150 network identity — port of iOS OnboardingView.swift's hash pair.
 * The app maps a human network name to the wire network_id; both platforms
 * MUST produce the same id for the same name or a mixed iOS/Android fleet
 * splits into two deaf networks.
 */
public object NetworkIdentity {

    /** FNV-1a truncated to 1 byte: UTF-8 bytes, 32-bit run, XOR-fold to 8. */
    public fun fnv1a8(name: String): Int {
        var hash = 0x811C9DC5.toInt() // 2166136261, FNV offset basis
        for (b in name.encodeToByteArray()) {
            hash = hash xor (b.toInt() and 0xFF)
            hash *= 16777619 // FNV prime (wrapping, same as Swift &*)
        }
        return (hash xor (hash ushr 8) xor (hash ushr 16) xor (hash ushr 24)) and 0xFF
    }

    /**
     * The wire network ID for a name.  0 is reserved — it's both the
     * firmware factory default and the app's "unset" sentinel, so a name
     * that happens to hash to 0 (1-in-256) would silently disable the
     * provisioning push and the mismatch badge.  Remap it to 1.
     */
    public fun networkIdForName(name: String): Int {
        val raw = fnv1a8(name)
        return if (raw == 0) 1 else raw
    }
}
