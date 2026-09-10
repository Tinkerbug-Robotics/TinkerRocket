package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.FetchedCatalog
import com.tinkerbug.tinkerrocket.protocol.FirmwareImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareManifest
import com.tinkerbug.tinkerrocket.protocol.FirmwareRelease
import java.io.File
import java.nio.file.Files
import java.security.MessageDigest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #773: firmware kept on the phone so a field with no signal is survivable.
 * iOS twin: `FirmwareCacheTests.swift`.
 */
class FirmwareCacheTest {

    private fun realSha(b: ByteArray): String =
        MessageDigest.getInstance("SHA-256").digest(b).joinToString("") { "%02x".format(it) }

    private fun tmp(): File = Files.createTempDirectory("fwcache").toFile()

    private val bytes = ByteArray(512) { (it % 251).toByte() }
    private val sha = realSha(bytes)

    private fun image(
        file: String = "out_computer-V9.bin",
        s: String = sha,
        size: Long = 512,
    ) = FirmwareImage(file, "out_computer", "abc-v9+1", 9, "ESP32-S3", size, s, "v9")

    private fun cache(dir: File) = FirmwareCache(dir, ::realSha)

    @Test
    fun `an image survives a round trip and is handed back`() {
        val c = cache(tmp())
        assertTrue(c.putImage(image(), bytes))
        assertTrue(c.hasImage(image()))
        assertTrue(c.image(image())!!.contentEquals(bytes))
    }

    @Test
    fun `a corrupted file is refused and deleted rather than served`() {
        // The file name is a CLAIM about the contents, not proof: a
        // half-written file from a killed app carries the right name and the
        // wrong bytes. Flashing a rocket from something nobody re-checked is
        // exactly what the manifest exists to prevent, so every read re-hashes.
        val dir = tmp()
        val c = cache(dir)
        c.putImage(image(), bytes)
        File(dir, "images/$sha").writeBytes(ByteArray(512) { 7 })   // same size, wrong bytes

        assertNull(c.image(image()), "content decides, not the name")
        assertFalse(File(dir, "images/$sha").exists(), "and it is dropped, not left to be retried")
    }

    @Test
    fun `a truncated file is refused too`() {
        val dir = tmp()
        val c = cache(dir)
        c.putImage(image(), bytes)
        File(dir, "images/$sha").writeBytes(ByteArray(100))

        assertNull(c.image(image()))
    }

    @Test
    fun `a half-written file is never visible under a whole file's name`() {
        // putImage writes beside and renames, so a kill mid-write leaves a
        // .part, which nothing reads and heldShas ignores.
        val dir = tmp()
        val c = cache(dir)
        File(dir, "images").mkdirs()
        File(dir, "images/$sha.part").writeBytes(ByteArray(200))

        assertFalse(c.hasImage(image()))
        assertNull(c.image(image()))
        assertTrue(c.heldShas().isEmpty())
    }

    @Test
    fun `the catalog round-trips through the same parsers the network uses`() {
        // One codec for release JSON, not two that can disagree.
        val c = cache(tmp())
        val release = FirmwareRelease(
            tag = "fw-v0.1.0",
            isPrerelease = false,
            assets = mapOf(
                "manifest.json" to "https://x.test/fw-v0.1.0/manifest.json",
                "out_computer-V9.bin" to "https://x.test/fw-v0.1.0/oc.bin",
            ),
        )
        val json = """{"manifest_version":1,"tag":"fw-v0.1.0","images":[
            {"file":"out_computer-V9.bin","project":"out_computer","version":"abc-v9+1",
             "chip_id":9,"chip":"ESP32-S3","size":512,"sha256":"$sha","board":"v9"}]}"""
        val manifest = FirmwareManifest.parse(json)!!

        assertTrue(c.putCatalog(FetchedCatalog(release, manifest, json)))
        val back = c.catalog()
        assertNotNull(back)
        assertEquals("fw-v0.1.0", back.first.tag)
        assertEquals(release.assets, back.first.assets, "the download URLs must survive")
        assertEquals(1, back.second.images.size)
        assertEquals(sha, back.second.images[0].sha256)
    }

    @Test
    fun `no catalog is null rather than a crash`() {
        assertNull(cache(tmp()).catalog())
    }

    @Test
    fun `a prerelease catalog is readable back`() {
        // The cache reads with includePrereleases = true deliberately: it is
        // replaying what was already chosen, not choosing again, and dropping
        // it here would silently empty the cache of an rc a tester fetched.
        val c = cache(tmp())
        val release = FirmwareRelease("fw-v0.2.0-rc1", true,
            mapOf("manifest.json" to "https://x.test/m.json"))
        val json = """{"manifest_version":1,"tag":"fw-v0.2.0-rc1","images":[
            {"file":"o.bin","project":"out_computer","version":"v","chip_id":9,"chip":"ESP32-S3",
             "size":512,"sha256":"$sha"}]}"""
        c.putCatalog(FetchedCatalog(release, FirmwareManifest.parse(json)!!, json))

        assertEquals("fw-v0.2.0-rc1", c.catalog()?.first?.tag)
    }

    @Test
    fun `prune keeps the current release and drops the rest`() {
        val dir = tmp()
        val c = cache(dir)
        val other = ByteArray(64) { 3 }
        c.putImage(image(), bytes)
        c.putImage(image("old.bin", realSha(other), 64), other)
        assertEquals(2, c.heldShas().size)

        c.prune(setOf(sha))

        assertEquals(setOf(sha), c.heldShas())
        assertTrue(c.image(image())!!.contentEquals(bytes), "the kept one is still good")
    }

    @Test
    fun `two images with the same bytes share one file`() {
        // Content addressing: an unchanged image across two releases is stored
        // once, so a phone does not pay twice for the same bytes.
        val dir = tmp()
        val c = cache(dir)
        c.putImage(image("a.bin"), bytes)
        c.putImage(image("b.bin"), bytes)

        assertEquals(1, c.heldShas().size)
        assertEquals(512L, c.bytesHeld())
    }
}
