// Root build file — version pins live here so both JVM modules stay in lockstep.
// Dependency policy (android-port plan §3): pin everything; one scheduled
// upgrade window per year.
plugins {
    kotlin("jvm") version "2.2.20" apply false
    // AGP 9: Kotlin support is built in — kotlin("android") must NOT be applied.
    id("com.android.library") version "9.0.0" apply false
}
