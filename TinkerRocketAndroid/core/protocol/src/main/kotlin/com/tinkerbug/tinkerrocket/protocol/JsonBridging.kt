package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlin.math.floor

/**
 * Internal helpers mirroring the iOS `JSONSerialization` + Swift `as?`
 * bridging semantics used by the BLEDevice.swift config/identity readback
 * ladders (NOT the Codable path — that is TelemetryData's strict/flex split).
 *
 * On iOS every JSON number arrives as an NSNumber and the handlers read it
 * with conditional casts, whose bridging rules are:
 *  - `as? Int`    — exact-integral numbers only (5 and 5.0 yes, 5.5 no);
 *                   booleans bridge to 1/0 (CFBoolean is an NSNumber);
 *                   strings never.
 *  - `as? Bool`   — true/false, plus the numbers 0/1 (Bool(exactly:));
 *                   strings never.
 *  - `as? Double` — any JSON number; booleans bridge to 1.0/0.0; strings never.
 *  - `as? String` — JSON strings only.
 *
 * These are deliberately DIFFERENT from TelemetryData's flexInt (#293/#571):
 * that leniency policy is a Codable-side contract; the readback ladders pin
 * the JSONSerialization behavior above instead.
 */
internal object JsonBridging {

    private fun primitive(el: JsonElement?): JsonPrimitive? {
        if (el == null || el is JsonNull) return null
        return el as? JsonPrimitive
    }

    /** Swift `as? String`: JSON strings only. */
    fun nsString(json: JsonObject, key: String): String? {
        val p = primitive(json[key]) ?: return null
        return if (p.isString) p.content else null
    }

    /**
     * Swift `as? Int` (64-bit) on a JSONSerialization value: exact-integral
     * numbers, booleans as 1/0, everything else nil.
     */
    fun nsInt(json: JsonObject, key: String): Long? {
        val p = primitive(json[key]) ?: return null
        if (p.isString) return null
        when (p.content) {
            "true" -> return 1L
            "false" -> return 0L
        }
        p.content.toLongOrNull()?.let { return it }
        val d = p.content.toDoubleOrNull() ?: return null
        // Int(exactly:) — integral value, exactly representable in Int64.
        if (!d.isFinite() || d != floor(d)) return null
        if (d < Long.MIN_VALUE.toDouble() || d >= Long.MAX_VALUE.toDouble()) return null
        val l = d.toLong()
        return if (l.toDouble() == d) l else null
    }

    /**
     * Swift `as? Bool` on a JSONSerialization value: true/false, plus the
     * numbers 0/1 (any numeric width — `Bool(exactly:)` semantics).
     */
    fun nsBool(json: JsonObject, key: String): Boolean? {
        val p = primitive(json[key]) ?: return null
        if (p.isString) return null
        when (p.content) {
            "true" -> return true
            "false" -> return false
        }
        return when (p.content.toDoubleOrNull()) {
            1.0 -> true
            0.0 -> false
            else -> null
        }
    }

    /** Swift `as? Double`: any JSON number; booleans bridge to 1.0/0.0. */
    fun nsDouble(json: JsonObject, key: String): Double? {
        val p = primitive(json[key]) ?: return null
        if (p.isString) return null
        when (p.content) {
            "true" -> return 1.0
            "false" -> return 0.0
        }
        return p.content.toDoubleOrNull()
    }

    /**
     * Port of iOS `BLEDevice.parseFloat(_:)`:
     *   Double → Float, Int → Float (both subsumed by [nsDouble]),
     *   String → Swift `Float(String)`.
     * Swift Float(String) accepts "nan"/"inf"/"infinity" (any case, optional
     * sign) and rejects whitespace and Java-style f/F/d/D suffixes — mirrored
     * here so this port is never MORE lenient than iOS.
     */
    fun parseFloatIos(json: JsonObject, key: String): Float? {
        nsDouble(json, key)?.let { return it.toFloat() }
        val s = nsString(json, key) ?: return null
        return swiftFloat(s)
    }

    private fun swiftFloat(s: String): Float? {
        if (s.isEmpty() || s.trim() != s) return null
        when (s.lowercase()) {
            "nan", "+nan", "-nan" -> return Float.NaN
            "inf", "+inf", "infinity", "+infinity" -> return Float.POSITIVE_INFINITY
            "-inf", "-infinity" -> return Float.NEGATIVE_INFINITY
        }
        // Java's parse accepts trailing f/F/d/D type suffixes; Swift doesn't.
        if (s.last() in "fFdD") return null
        val d = s.toDoubleOrNull() ?: return null
        return d.toFloat()
    }
}
