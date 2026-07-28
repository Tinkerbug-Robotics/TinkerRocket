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
    testImplementation(kotlin("test"))
}

tasks.test {
    useJUnitPlatform()
}
