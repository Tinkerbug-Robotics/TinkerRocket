package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Column
import androidx.compose.ui.semantics.SemanticsProperties
import androidx.compose.ui.test.SemanticsMatcher
import androidx.compose.ui.test.assert
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithText
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import com.tinkerbug.tinkerrocket.protocol.gnssImuHealthDetail
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The GNSS board's IMU dot ("gi") is the one health dot with a plain-language
 * line, and TalkBack reads it as the dot's state. The other dots must stay as
 * they were: name only, no state description.
 */
@RunWith(AndroidJUnit4::class)
class HealthDotTest {

    @get:Rule val compose = createComposeRule()

    private fun telemetry(gi: Int?): TelemetryData {
        // Baro, IMU, EKF, Mag, GNSS, battery and storage OK.
        val h = 1 or (1 shl 2) or (1 shl 4) or (1 shl 6) or (1 shl 8) or (1 shl 10) or (1 shl 20)
        val json = if (gi == null) """{"h":$h}""" else """{"h":$h,"gi":$gi}"""
        return requireNotNull(TelemetryData.decode(json))
    }

    @Test
    fun gnss_imu_dot_reads_out_its_line_and_the_others_do_not() {
        val rows = telemetry(gi = 2).sensorHealthRows
        compose.setContent {
            Column { rows.forEach { HealthDot(it.name, it.state, it.detail) } }
        }
        val detail = requireNotNull(gnssImuHealthDetail(TelemetryData.SensorHealth.DEGRADED))
        compose.onNodeWithText("GNSS IMU")
            .assertIsDisplayed()
            .assert(SemanticsMatcher.expectValue(SemanticsProperties.StateDescription, detail))
        compose.onNodeWithText("Mag")
            .assert(SemanticsMatcher.keyNotDefined(SemanticsProperties.StateDescription))
    }

    @Test
    fun no_gnss_imu_dot_without_the_key() {
        val rows = telemetry(gi = null).sensorHealthRows
        compose.setContent {
            Column { rows.forEach { HealthDot(it.name, it.state, it.detail) } }
        }
        compose.onNodeWithText("Storage").assertIsDisplayed()
        compose.onNodeWithText("GNSS IMU").assertDoesNotExist()
    }
}
