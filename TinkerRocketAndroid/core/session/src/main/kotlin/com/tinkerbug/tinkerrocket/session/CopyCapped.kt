package com.tinkerbug.tinkerrocket.session

import java.io.ByteArrayOutputStream
import java.io.InputStream

/**
 * At most [maxBytes] of [input] into [out].  `InputStream.readNBytes` would
 * do, but Android has it only from API 33 and the app's minSdk is 31.
 */
internal fun copyCapped(input: InputStream, out: ByteArrayOutputStream, maxBytes: Int) {
    val buf = ByteArray(8 * 1024)
    while (out.size() < maxBytes) {
        val n = input.read(buf, 0, minOf(buf.size, maxBytes - out.size()))
        if (n < 0) break
        out.write(buf, 0, n)
    }
}
