// :app — Compose UI + platform glue (android-port plan §1).  Phase 3 slice:
// scanner + dashboard over the real BLE stack.  Manual DI (AppContainer),
// no Hilt (plan §1); simple two-screen state nav until more destinations
// justify NavHost.

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.plugin.compose")
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
        versionCode = 4
        versionName = "1.0.0"
    }

    buildFeatures {
        compose = true
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
    implementation("org.maplibre.gl:android-sdk:11.8.8")

    // Phone GPS for direction/distance-to-rocket (fused provider; Pixel/GMS).
    implementation("com.google.android.gms:play-services-location:21.3.0")

    val composeBom = platform("androidx.compose:compose-bom:2025.06.01")
    implementation(composeBom)
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.foundation:foundation")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.activity:activity-compose:1.10.1")
    // Not used directly (MainActivity is a ComponentActivity) — pins the
    // transitive fragment above 1.3.0 so the ActivityResult APIs are safe on
    // every resolved path; release-only lintVital fails the build without it
    // (InvalidFragmentVersionForActivityResult).
    implementation("androidx.fragment:fragment:1.8.5")
}
