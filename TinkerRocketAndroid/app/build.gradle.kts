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
        versionCode = 1
        versionName = "0.1.0-phase3"
    }

    buildFeatures {
        compose = true
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
}
