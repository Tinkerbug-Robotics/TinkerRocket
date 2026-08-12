package com.tinkerbug.tinkerrocket.app

import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.DriftCast
import com.tinkerbug.tinkerrocket.session.HeadingMath
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

    /**
     * @param hAccM horizontal accuracy in metres, null when the platform
     *   doesn't report one — it rides the phone-fix push so the BS log can
     *   say how good the position it recorded actually was.
     */
    data class PhoneFix(
        val lat: Double,
        val lon: Double,
        val altMslM: Double?,
        val hAccM: Double? = null,
    )

    private val _location = MutableStateFlow<PhoneFix?>(null)
    val location: StateFlow<PhoneFix?> = _location.asStateFlow()

    /** True-north compass heading in degrees. */
    private val _headingDeg = MutableStateFlow(0.0)
    val headingDeg: StateFlow<Double> = _headingDeg.asStateFlow()

    private val fused = LocationServices.getFusedLocationProviderClient(app)
    private val sensors = app.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private var refCount = 0
    private var declinationDeg = 0.0

    // Throttle state for reportFix().  Lives here rather than at the call
    // site so every caller shares one budget (iOS does the same).
    private var lastFixSentAtMs: Long? = null
    private var lastFixSentFrom: PhoneFix? = null

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
            _location.value = PhoneFix(
                loc.latitude, loc.longitude, msl,
                hAccM = if (loc.hasAccuracy()) loc.accuracy.toDouble() else null,
            )
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
            // Math lives in HeadingMath (:core:session) so it is unit-tested —
            // the wrap cases can't be caught by looking at the arrow, since 359°
            // wrong and 1° wrong render identically (Checkpoint A Group 7).
            val trueHeading = HeadingMath.trueHeading(magnetic, declinationDeg)
            if (HeadingMath.shouldPublish(trueHeading, _headingDeg.value)) {
                _headingDeg.value = trueHeading
            }
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

    /**
     * Push the current fix to [session] as cmd 47, throttled.  Port of iOS
     * `LocationManager.reportFix(to:)`.
     *
     * The base station is wherever the phone is, and the BS has no GNSS of
     * its own, so this is the only way the range between the two ends ever
     * reaches a log file.  The BS writes the row only while a logging
     * session is open, so calling this off-session costs one BLE write and
     * nothing else — safe to call on every location update.
     */
    fun reportFix(session: DeviceSession) {
        val fix = _location.value ?: return
        val now = System.currentTimeMillis()
        val sentAt = lastFixSentAtMs
        val from = lastFixSentFrom
        if (sentAt != null && from != null) {
            val moved = DriftCast.haversineM(from.lat, from.lon, fix.lat, fix.lon)
            if (now - sentAt < FIX_MIN_INTERVAL_MS && moved < FIX_FORCE_DISTANCE_M) return
        }
        lastFixSentAtMs = now
        lastFixSentFrom = fix
        session.sendCommandFrame(
            Commands.setPhoneFix(fix.lat, fix.lon, fix.altMslM, fix.hAccM),
        )
    }

    /**
     * Drop the throttle so the next fix is pushed immediately.  Called on
     * (re)connect: the fix at the START of a log is the one that matters,
     * and a timestamp left over from the previous session could suppress it.
     */
    fun resetFixThrottle() {
        lastFixSentAtMs = null
        lastFixSentFrom = null
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

    companion object {
        /** iOS `fixMinInterval` — 30 s between pushes when standing still. */
        const val FIX_MIN_INTERVAL_MS: Long = 30_000

        /** iOS `fixForceDistance` — a move this far pushes regardless. */
        const val FIX_FORCE_DISTANCE_M: Double = 25.0
    }
}
