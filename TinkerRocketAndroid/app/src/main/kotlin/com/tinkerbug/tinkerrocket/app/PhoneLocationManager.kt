package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.app.Application
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.GeomagneticField
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Looper
import androidx.core.content.ContextCompat
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs

/**
 * Phone GPS + compass heading for direction/distance to the rocket — port
 * of iOS LocationManager.swift.  Same discipline: updates run ONLY between
 * start() and stop() (continuous published heading updates were a major
 * iOS memory-growth source), heading publishes on ≥1° change only.
 *
 * Heading: rotation-vector sensor (magnetic) + geomagnetic declination
 * from the last fix → true north, the iOS trueHeading analogue.
 * Altitude: MSL when the platform provides it (API 34+); otherwise the
 * GNSS ellipsoid altitude — the ~30 m geoid delta caveat is an open plan
 * decision (EGM96 table vs caveat text).
 *
 * Ref-counted: dashboard, map, and DriftCast each start/stop around their
 * own lifecycles.
 */
class PhoneLocationManager(private val app: Application) {

    data class PhoneFix(val lat: Double, val lon: Double, val altMslM: Double?)

    private val _location = MutableStateFlow<PhoneFix?>(null)
    val location: StateFlow<PhoneFix?> = _location.asStateFlow()

    /** True-north compass heading in degrees. */
    private val _headingDeg = MutableStateFlow(0.0)
    val headingDeg: StateFlow<Double> = _headingDeg.asStateFlow()

    private val fused = LocationServices.getFusedLocationProviderClient(app)
    private val sensors = app.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private var refCount = 0
    private var declinationDeg = 0.0

    fun hasPermission(): Boolean =
        ContextCompat.checkSelfPermission(app, Manifest.permission.ACCESS_FINE_LOCATION) ==
            PackageManager.PERMISSION_GRANTED

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            val loc = result.lastLocation ?: return
            val msl = when {
                Build.VERSION.SDK_INT >= 34 && loc.hasMslAltitude() -> loc.mslAltitudeMeters
                loc.hasAltitude() -> loc.altitude // ellipsoid (geoid caveat)
                else -> null
            }
            _location.value = PhoneFix(loc.latitude, loc.longitude, msl)
            declinationDeg = GeomagneticField(
                loc.latitude.toFloat(), loc.longitude.toFloat(),
                (msl ?: 0.0).toFloat(), loc.time,
            ).declination.toDouble()
        }
    }

    private val headingListener = object : SensorEventListener {
        private val rotation = FloatArray(9)
        private val orientation = FloatArray(3)
        override fun onSensorChanged(event: SensorEvent) {
            SensorManager.getRotationMatrixFromVector(rotation, event.values)
            SensorManager.getOrientation(rotation, orientation)
            val magnetic = Math.toDegrees(orientation[0].toDouble())
            val trueHeading = (magnetic + declinationDeg).mod(360.0)
            // ≥1° guard, wrap-aware (359.5 → 0.5 is a 1° move, not 359°).
            val delta = abs((trueHeading - _headingDeg.value + 180.0).mod(360.0) - 180.0)
            if (delta >= 1.0) _headingDeg.value = trueHeading
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
    }

    /** Start GPS + compass updates (no-op without the runtime permission). */
    fun start() {
        if (refCount++ > 0) return
        if (!hasPermission()) return
        val request = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L)
            .setMinUpdateDistanceMeters(5f) // iOS distanceFilter = 5 m
            .build()
        runCatching {
            fused.requestLocationUpdates(request, locationCallback, Looper.getMainLooper())
        }
        sensors.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)?.let { sensor ->
            sensors.registerListener(headingListener, sensor, SensorManager.SENSOR_DELAY_UI)
        }
    }

    fun stop() {
        if (refCount == 0 || --refCount > 0) return
        fused.removeLocationUpdates(locationCallback)
        sensors.unregisterListener(headingListener)
    }

    /** Re-arm after the user grants permission mid-session. */
    fun restartIfHeld() {
        if (refCount > 0 && hasPermission()) {
            val held = refCount
            refCount = 0
            start() // arms the providers
            refCount = held
        }
    }
}
