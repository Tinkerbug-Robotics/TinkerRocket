// Root build file — version pins live here so both JVM modules stay in lockstep.
// Dependency policy (android-port plan §3): pin everything; one scheduled
// upgrade window per year.
plugins {
    kotlin("jvm") version "2.2.20" apply false
}
