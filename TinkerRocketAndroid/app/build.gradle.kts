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

    // #624: Compose UI tests run on the JVM under Robolectric, in `src/test`,
    // NOT as instrumented tests in `src/androidTest`.
    //
    // The reason is that they have to run in CI, and CI has no emulator:
    // android-tests.yml is `./gradlew test` on a plain ubuntu runner. An
    // instrumented suite would need an emulator step, which is slow and
    // famously flaky, and until someone adds one the tests would run only on
    // whatever phone happened to be on a desk. That is the situation this
    // module was already in -- `:core:ble` has instrumented tests that need
    // the bench hardware, so in practice nothing about `:app` was verified by
    // CI at all, and a four-line accessibility change on #1442 could not be
    // proven without a connected rocket.
    //
    // Robolectric needs the merged manifest and the resource table, which is
    // what this flag hands it.
    testOptions {
        unitTests {
            isIncludeAndroidResources = true
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

    // ---- #624: Compose UI tests on the JVM (see `testOptions` above) ----
    //
    // JUnit 4, not 5, and deliberately: the Compose test harness and
    // Robolectric are both JUnit 4 `TestRule`/`Runner` machinery, so the
    // `:core:*` modules' `useJUnitPlatform()` is not applied here. The
    // repo's own `tools/check_android_tests_ran.py` guard counts `@Test`
    // annotations against JUnit XML cases and is engine-agnostic, so it
    // covers this module exactly as it covers the others.
    testImplementation("junit:junit:4.13.2")
    testImplementation("org.robolectric:robolectric:4.17")
    testImplementation("androidx.test:core-ktx:1.6.1")
    testImplementation("androidx.test.ext:junit-ktx:1.2.1")
    testImplementation(composeBom)
    testImplementation("androidx.compose.ui:ui-test-junit4")

    // Supplies the `ComponentActivity` entry that `createComposeRule()` needs
    // in the merged manifest. It is a debug-only artifact and must never ship
    // in a release build, which is why the release unit-test variant is turned
    // off below rather than this being promoted to `implementation`.
    debugImplementation(composeBom)
    debugImplementation("androidx.compose.ui:ui-test-manifest")
}

// #624: `./gradlew test` would otherwise run this module's suite twice, once
// per build type. The release run would also fail outright -- `ui-test-manifest`
// is debug-only, so `createComposeRule()` finds no ComponentActivity in the
// release manifest. Debug and release differ here only in signing and the
// applicationId suffix, neither of which any UI test observes, so the second
// run costs emulated-Android startup twice over and proves nothing.
tasks.matching { it.name == "testReleaseUnitTest" }.configureEach { enabled = false }

// #624: Robolectric reaches into JDK internals that have been closed to the
// unnamed module since JDK 9, and on JDK 21 the reflection simply throws.
// Emulating SDK 36 walks `ApplicationSharedMemory.create`, which needs the
// raw FileDescriptor interceptor, so without these the very first test dies
// with "Failed to interact with raw FileDescriptor internals". These are
// Robolectric's own documented arguments, not a local workaround.
tasks.withType<Test>().configureEach {
    jvmArgs(
        "--add-opens=java.base/java.lang=ALL-UNNAMED",
        "--add-opens=java.base/java.io=ALL-UNNAMED",
        "--add-opens=java.base/java.util=ALL-UNNAMED",
        "--add-opens=java.base/jdk.internal.access=ALL-UNNAMED",
    )
}
