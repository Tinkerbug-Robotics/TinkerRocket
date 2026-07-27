// :core:ble — the ONLY module that touches the Android BLE stack
// (android-port plan §1).  RealBleTransport + AndroidBleScanner implement the
// :core:session seams over raw BluetoothGatt with the serial one-outstanding-op
// GATT queue.  Nothing above the transport contract lives here.

plugins {
    // AGP 9 ships built-in Kotlin support — no kotlin("android") plugin.
    id("com.android.library")
}

android {
    namespace = "com.tinkerbug.tinkerrocket.ble"
    compileSdk = 36

    defaultConfig {
        minSdk = 31   // plan §5: modern BLE permission pair, no legacy path
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }
}

kotlin {
    explicitApi()
    jvmToolchain(21)
}

dependencies {
    api(project(":core:session"))
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.10.2")

    // Bench-seam instrumented test (BenchSeamTest — runs ON the Pixel
    // against a real bench OC, gated behind -Pandroid.testInstrumentationRunnerArguments.bench=1).
    androidTestImplementation("androidx.test:runner:1.6.2")
    androidTestImplementation("androidx.test.ext:junit:1.2.1")
    androidTestImplementation("androidx.test:rules:1.6.1")
    androidTestImplementation(kotlin("test"))
}
