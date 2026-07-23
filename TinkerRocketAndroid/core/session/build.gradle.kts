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
    testImplementation(kotlin("test"))
}

tasks.test {
    useJUnitPlatform()
}
