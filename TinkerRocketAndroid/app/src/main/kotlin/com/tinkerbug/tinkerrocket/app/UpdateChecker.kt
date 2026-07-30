package com.tinkerbug.tinkerrocket.app

import android.app.Application
import android.content.Context
import com.tinkerbug.tinkerrocket.session.UpdateCheck
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.withContext
import java.net.HttpURLConnection
import java.net.URL

/**
 * Thin shell around [UpdateCheck]: fetch GitHub releases, throttle to one
 * network hit per day, persist a found update so a throttled restart still
 * shows the banner. Every failure path is silent-null — no signal at a launch
 * site is NORMAL, and an update check must never cost anything there.
 */
class UpdateChecker(private val app: Application) {

    private val prefs = app.getSharedPreferences("update_check", Context.MODE_PRIVATE)

    // Plain val, declared BEFORE _available: Kotlin runs property initializers
    // in declaration order, and _available's initializer reads this through
    // storedIfStillAhead(). The first cut had it below as a `by lazy` — the
    // delegate object didn't exist yet, and the app crashed at Application
    // onCreate… but ONLY when a stored update existed, so the first launch
    // passed and every launch after a release was found died (bench-caught).
    val currentVersionName: String = try {
        app.packageManager.getPackageInfo(app.packageName, 0).versionName ?: "0.0.0"
    } catch (_: Exception) {
        "0.0.0"
    }

    private val _available = MutableStateFlow(storedIfStillAhead())
    val available: StateFlow<UpdateCheck.ReleaseCandidate?> = _available

    /** Once per app start; hits the network at most every [THROTTLE_MS]. */
    suspend fun checkThrottled() {
        val now = System.currentTimeMillis()
        if (now - prefs.getLong("last_check_ms", 0) < THROTTLE_MS) return
        val body = fetch() ?: return
        // Only a NON-EMPTY release list charges the 24 h throttle. GitHub's
        // unauthenticated API rides a CDN that can serve a stale empty array
        // (observed on the bench minutes after the first release was cut) —
        // burning the throttle on one would hide a real release for a day.
        // The fleet repo always has ≥1 android release, so empty ⇒ retry
        // next app start instead.
        if (UpdateCheck.parseReleases(body).isNotEmpty()) {
            prefs.edit().putLong("last_check_ms", now).apply()
        }

        val update = UpdateCheck.updateAvailable(currentVersionName, body)
        if (update != null) {
            prefs.edit()
                .putString("found_version", update.versionName)
                .putString("found_tag", update.tag)
                .putString("found_url", update.htmlUrl)
                .apply()
        } else {
            prefs.edit()
                .remove("found_version").remove("found_tag").remove("found_url")
                .apply()
        }
        _available.value = update
    }

    /** A previously-found update survives restarts, but never outlives an install that caught up. */
    private fun storedIfStillAhead(): UpdateCheck.ReleaseCandidate? {
        val v = prefs.getString("found_version", null) ?: return null
        val tag = prefs.getString("found_tag", null) ?: return null
        val url = prefs.getString("found_url", null) ?: return null
        return UpdateCheck.ReleaseCandidate(v, tag, url)
            .takeIf { UpdateCheck.compareVersions(it.versionName, currentVersionName) > 0 }
    }

    private suspend fun fetch(): String? = withContext(Dispatchers.IO) {
        try {
            val conn = URL(RELEASES_URL).openConnection() as HttpURLConnection
            conn.connectTimeout = 10_000
            conn.readTimeout = 10_000
            // GitHub rejects UA-less requests outright.
            conn.setRequestProperty("User-Agent", "TinkerRocket-Android")
            conn.setRequestProperty("Accept", "application/vnd.github+json")
            try {
                if (conn.responseCode != 200) return@withContext null
                conn.inputStream.bufferedReader().readText()
            } finally {
                conn.disconnect()
            }
        } catch (_: Exception) {
            null
        }
    }

    companion object {
        private const val THROTTLE_MS = 24L * 60 * 60 * 1000
        private const val RELEASES_URL =
            "https://api.github.com/repos/Tinkerbug-Robotics/TinkerRocket/releases?per_page=30"
    }
}
