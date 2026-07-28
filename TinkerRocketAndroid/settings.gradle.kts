// TinkerRocket Android — see docs/plans/android-port.md.
//
// Module layout (§1 of the plan):
//   :core:protocol  pure JVM, zero Android deps — wire codecs + firmware-mirrored math
//   :core:session   pure JVM — fleet/device domain over the abstract BleTransport
//   :core:ble       Android library (RealBleTransport)  — added when the SDK lands
//   :app            Compose UI + platform glue          — added when the SDK lands
//
// The two Android modules are deliberately absent until Android Studio / the
// SDK is set up (Phase 2/3); everything committed so far builds and tests on
// a plain JVM: ./gradlew test

pluginManagement {
    repositories {
        google()          // Android Gradle Plugin
        mavenCentral()
        gradlePluginPortal()
    }
}

plugins {
    // Resolves the JVM toolchain (21) from foojay if the local JDK differs.
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        mavenCentral()
        google()  // for the Android modules when they arrive
    }
}

rootProject.name = "TinkerRocketAndroid"

include(":core:protocol")
include(":core:session")
include(":core:maps")    // pure JVM: tile math + offline cache + tile proxy
include(":core:ble")     // Android library: RealBleTransport over BluetoothGatt
include(":app")          // Compose app
