// :app — Compose UI + platform glue (android-port plan §1).  Phase 3 slice:
// scanner + dashboard over the real BLE stack.  Manual DI (AppContainer),
// no Hilt (plan §1); simple two-screen state nav until more destinations
// justify NavHost.

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.plugin.compose")
}

// Build provenance (#974).  The firmware stamps its commit into every flight
// record (`fw_git_sha`, e.g. "d7017c0-v8+20260827-1919"), so any board can be
// traced to the source that built it.  The apps could not: versionName is a
// static "1.0.3" that has not moved in months, which made the app build the
// one component of a flight nobody could identify after the fact.
//
// Computed at BUILD time on purpose.  A committed constant is structurally
// unable to name its own commit — it can only ever hold the sha of the commit
// BEFORE the one being built.
val trGitSha: String = run {
    fun git(vararg args: String): String? = try {
        val p = ProcessBuilder(listOf("git", *args))
            .directory(rootDir)
            .redirectErrorStream(true)
            .start()
        val out = p.inputStream.bufferedReader().readText().trim()
        if (p.waitFor() == 0) out else null
    } catch (_: Exception) {
        null
    }
    val sha = git("rev-parse", "--short", "HEAD")
    when {
        sha.isNullOrEmpty() -> "unknown"
        // A dirty tree means the APK does NOT correspond to any commit; say so
        // rather than naming a commit the binary does not match.
        !git("status", "--porcelain").isNullOrBlank() -> "$sha-dirty"
        else -> sha
    }
}

android {
    namespace = "com.tinkerbug.tinkerrocket"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.tinkerbug.tinkerrocket"
        minSdk = 31
        targetSdk = 36
        // versionCode must only ever increase (Play + sideload "update"
        // installs both key on it); bump BOTH before tagging android-v<name>.
        versionCode = 7
        versionName = "1.0.3"

        // Google Map Tiles API key (online-only satellite basemap).  Same
        // env-var discipline as release signing below: never in the repo;
        // forks without it just don't get the Google source in the picker.
        // A client-side Maps key necessarily ships in the APK — it is
        // API-restricted server-side to the Map Tiles API only.
        buildConfigField(
            "String",
            "TR_MAPS_API_KEY",
            "\"${System.getenv("TR_ANDROID_MAPS_API_KEY").orEmpty()}\"",
        )
        // #974: which commit built this APK.  Surfaced in Settings.
        buildConfigField("String", "TR_GIT_SHA", "\"$trGitSha\"")
    }

    buildFeatures {
        compose = true
        // For the Maps key constant below — AGP 9 defaults buildConfig off.
        buildConfig = true
    }

    // Release signing (plan §2.5): source is public, the key is not.  The
    // keystore lives OUTSIDE the repo and reaches builds only via env vars —
    // locally from the operator's shell, in CI from repo secrets (see
    // .github/workflows/android-release.yml and docs/android-release-signing.md).
    // Forks build without any of this set and get a debug-signed release —
    // installable everywhere, but unable to impersonate an UPDATE to a real
    // install, which is the entire security model.
    val keystorePath: String? = System.getenv("TR_ANDROID_KEYSTORE")
    signingConfigs {
        if (keystorePath != null) {
            create("release") {
                storeFile = file(keystorePath)
                storePassword = System.getenv("TR_ANDROID_KEYSTORE_PASSWORD")
                keyAlias = System.getenv("TR_ANDROID_KEY_ALIAS") ?: "tinkerrocket"
                keyPassword = System.getenv("TR_ANDROID_KEY_PASSWORD")
            }
        }
    }

    buildTypes {
        release {
            // Minification stays OFF for v1.0: R8 vs MapLibre/GMS keep-rules is
            // real risk with zero payoff at this APK size and fleet size.
            isMinifyEnabled = false
            signingConfig = if (keystorePath != null) {
                signingConfigs.getByName("release")
            } else {
                signingConfigs.getByName("debug")
            }
        }
        debug {
            // #975: bench installs without editing this file.
            //
            // The app already on a bench phone is signed with a DIFFERENT debug
            // keystore than a fresh local build, so `adb install -r` fails
            // INSTALL_FAILED_UPDATE_INCOMPATIBLE and a plain install would need
            // an uninstall -- which drops that install's flights, offline tiles
            // and settings. A suffixed applicationId coexists instead, and
            // updates in place on repeat installs.
            //
            // This used to be a hand-edit reverted before committing. Two things
            // went wrong with that: it dirties the tree, so #974's build stamp
            // reports `<sha>-dirty` and the bench APK can never name its commit;
            // and "revert before committing" is a step someone eventually
            // forgets. An env var does the same job with neither problem:
            //
            //     TR_ANDROID_APP_ID_SUFFIX=.bench ./gradlew :app:assembleDebug
            //
            // Empty/unset (the default, and what CI sees) leaves the applicationId
            // exactly as it was.
            val suffix = System.getenv("TR_ANDROID_APP_ID_SUFFIX").orEmpty()
            if (suffix.isNotEmpty()) {
                applicationIdSuffix = suffix
            }
        }
    }

    sourceSets {
        getByName("main") {
            // Demo mode serves the emitter-generated synthetic flight as a
            // downloadable device file — read straight from the golden corpus
            // so there is no second copy to drift.
            assets.srcDirs("../../tests_cpp/fixtures/wire/csv")
        }
    }
}

kotlin {
    jvmToolchain(21)
}

dependencies {
    implementation(project(":core:ble"))
    implementation(project(":core:maps"))
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.10.2")

    // Maps: MapLibre GL Native, raster-only over the localhost tile proxy
    // (plan §1 Maps row — Google Maps ToS forbids tile caching; MapLibre's
    // own OfflineManager is evictable SQLite, the silent-data-loss iOS
    // designed around).  Pinned per the dependency policy.
    // Google satellite (Map Tiles API) later joined as an ONLINE-ONLY
    // source riding the same proxy, never cached — offline stays USGS, so
    // the ToS objection above doesn't apply to it.
    implementation("org.maplibre.gl:android-sdk:11.8.8")

    // Phone GPS for direction/distance-to-rocket (fused provider; Pixel/GMS).
    implementation("com.google.android.gms:play-services-location:21.3.0")

    val composeBom = platform("androidx.compose:compose-bom:2025.06.01")
    implementation(composeBom)
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.foundation:foundation")
    implementation("androidx.compose.material3:material3")
    // Icon set for the iOS-parity screens (SF-symbol analogs: RocketLaunch,
    // CellTower, Air, Inventory2, AutoAwesome…). R8 strips the unused bulk.
    implementation("androidx.compose.material:material-icons-extended")
    implementation("androidx.activity:activity-compose:1.10.1")
    // Not used directly (MainActivity is a ComponentActivity) — pins the
    // transitive fragment above 1.3.0 so the ActivityResult APIs are safe on
    // every resolved path; release-only lintVital fails the build without it
    // (InvalidFragmentVersionForActivityResult).
    implementation("androidx.fragment:fragment:1.8.5")
}
