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
}

kotlin {
    jvmToolchain(21)
}

dependencies {
    implementation(project(":core:ble"))
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.10.2")

    val composeBom = platform("androidx.compose:compose-bom:2025.06.01")
    implementation(composeBom)
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.foundation:foundation")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.activity:activity-compose:1.10.1")
}
