package com.irurueta.android.navigation.inertial.calibration

import android.content.Context
/*import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.AccelerationTriad*/

class AccelerometerCalibrator(val context: Context) {

    /*private var location: Location? = null

    private var gravityNorm: Double? = null

    private val mode = Mode.GRAVITY_NORM_BASED

    private val knownBias: Boolean = DEFAULT_BIAS_KNOWN

    private val bias: AccelerationTriad? = null

    private val sensorDelay: SensorDelay = SensorDelay.FASTEST


    // TODO: when LOCATION_BASED is used and no location is provided, location needs to be requested
    // TODO: when GRAVITY_NORM_BASED is used and no gravityNorm is provided, gravity needs to be measured before starting
    // TODO: when knownBias = true and no bias is provided, bias needs to be measured before starting.

    private val gravityMeasurementListener = object : GravitySensorCollector.OnMeasurementListener {
        override fun onMeasurement(
            gx: Float,
            gy: Float,
            gz: Float,
            g: Double,
            timestamp: Long,
            accuracy: SensorAccuracy?
        ) {
            TODO("Not yet implemented")
        }
    }

    private val gravityAccuracyChangedListener =
        object : SensorCollector.OnAccuracyChangedListener {
            override fun onAccuracyChanged(accuracy: SensorAccuracy?) {
                TODO("Not yet implemented")
            }

        }

    private val gravityCollector = GravitySensorCollector(
        context,
        sensorDelay,
        gravityMeasurementListener,
        gravityAccuracyChangedListener
    )


    enum class Mode {
        LOCATION_BASED,
        GRAVITY_NORM_BASED
    }

    companion object {
        // When bias is known, a calibrator with known bias values is used
        const val DEFAULT_BIAS_KNOWN = false
    }*/
}