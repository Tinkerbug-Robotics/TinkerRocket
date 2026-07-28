// :core:maps — pure JVM, ZERO Android dependencies (android-port plan §1/§4
// Phase 6).  Tile math, the flat-file offline cache, and the localhost
// read-through tile proxy all live here so the z/y/x-URL vs z/x/y-disk
// transposition and the read-through semantics are JVM-testable without a
// device.  MapLibre itself (Android-only) lives in :app.

plugins {
    kotlin("jvm")
}

kotlin {
    jvmToolchain(21)
    explicitApi()
}

dependencies {
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.10.2")
    // Region manifest JSON — hand-mapped JsonObject (schema-identical to the
    // iOS Codable output: Apple-epoch dates, uppercase UUIDs), same policy
    // as :core:protocol.
    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.9.0")
    testImplementation(kotlin("test"))
    testImplementation("org.jetbrains.kotlinx:kotlinx-coroutines-test:1.10.2")
}

tasks.test {
    useJUnitPlatform()
}
