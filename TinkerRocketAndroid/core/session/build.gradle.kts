// :core:session — pure JVM fleet/device domain over the abstract BleTransport
// (android-port plan §1/§3).  Grows DeviceSession/FleetManager in Phase 2;
// today it holds the transport seam so :core:ble and the replay feeds have a
// stable contract to implement.

plugins {
    kotlin("jvm")
}

kotlin {
    jvmToolchain(21)
    explicitApi()
}

dependencies {
    api(project(":core:protocol"))
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.10.2")
    // KnownDeviceStore's persistence codec (iOS-schema-identical JSON) uses
    // the JsonElement API; :core:protocol keeps the lib `implementation`, so
    // it does not flow through the project dependency.
    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.9.0")
    testImplementation(kotlin("test"))
    // Virtual-time testing for the session layer's timed behavior (connect
    // choreography delays, download stall timers, reconnect backoff).
    testImplementation("org.jetbrains.kotlinx:kotlinx-coroutines-test:1.10.2")
}

tasks.test {
    useJUnitPlatform()
}
