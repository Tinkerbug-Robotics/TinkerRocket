package com.tinkerbug.tinkerrocket.protocol

/**
 * Network-name → network-ID mapping (#150) — the contract between
 * onboarding (preview), provisioning (push), and the firmware's identity
 * NVS.  Port of the free functions in iOS OnboardingView.swift, pinned by
 * NetworkIdTests: a re-map here silently breaks every fielded device.
 */
public object NetworkId {

    /**
     * FNV-1a 32-bit hash of the name's UTF-8 bytes, XOR-folded to one byte.
     * Golden pin: fnv1a8("a") == 0xED.
     */
    public fun fnv1a8(name: String): Int {
        var hash = 2166136261u   // FNV offset basis
        for (byte in name.toByteArray(Charsets.UTF_8)) {
            hash = hash xor (byte.toUInt() and 0xFFu)
            hash *= 16777619u    // FNV prime
        }
        // Fold 32-bit hash to 8 bits via XOR
        return ((hash xor (hash shr 8) xor (hash shr 16) xor (hash shr 24)) and 0xFFu).toInt()
    }

    /**
     * The wire network ID for a name.  0 is reserved — it's both the firmware
     * factory default and the app's "unset" sentinel, so a name that happens
     * to hash to 0 (1-in-256) would silently disable the provisioning push
     * and the mismatch badge.  Remapped to 1.
     */
    public fun networkIdForName(name: String): Int {
        val raw = fnv1a8(name)
        return if (raw == 0) 1 else raw
    }
}
