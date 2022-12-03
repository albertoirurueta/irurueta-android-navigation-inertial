package com.irurueta.android.navigation.inertial.test.estimators.attitude

import android.hardware.SensorManager
import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.estimators.attitude.GravityEstimator
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

@RequiresDevice
class GravityEstimatorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Volatile
    private var accelerometerMeasured = 0

    @Volatile
    private var gravityMeasured = 0

    @Before
    fun setUp() {
        completed = 0
        accelerometerMeasured = 0
        gravityMeasured = 0
    }

    @Test
    fun startAndStop_whenAccelerometerNotUsed_estimatesGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GravityEstimator(
            context,
            useAccelerometer = false,
            estimationListener = { _, fx, fy, fz, timestamp ->
                logGravity(fx, fy, fz, timestamp)

                syncHelper.notifyAll { completed++ }
            },
            accelerometerMeasurementListener = { _, _, _, _, _, _, _, _ -> accelerometerMeasured++ },
            gravityMeasurementListener = { _, _, _, _, _, _ -> gravityMeasured++ }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
        assertEquals(0, accelerometerMeasured)
        assertTrue(gravityMeasured > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerUsed_estimatesGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GravityEstimator(
            context,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            estimationListener = { _, fx, fy, fz, timestamp ->
                logGravity(fx, fy, fz, timestamp)

                syncHelper.notifyAll { completed++ }
            },
            accelerometerMeasurementListener = { _, _, _, _, _, _, _, _ -> accelerometerMeasured++ },
            gravityMeasurementListener = { _, _, _, _, _, _ -> gravityMeasured++ }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
        assertTrue(accelerometerMeasured > 0)
        assertEquals(0, gravityMeasured)
    }

    @Test
    fun startAndStop_whenAccelerometerUncalibratedUsed_estimatesGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GravityEstimator(
            context,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimationListener = { _, fx, fy, fz, timestamp ->
                logGravity(fx, fy, fz, timestamp)

                syncHelper.notifyAll { completed++ }
            },
            accelerometerMeasurementListener = { _, _, _, _, _, _, _, _ -> accelerometerMeasured++ },
            gravityMeasurementListener = { _, _, _, _, _, _ -> gravityMeasured++ }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
        assertTrue(accelerometerMeasured > 0)
        assertEquals(0, gravityMeasured)
    }

    private fun logGravity(fx: Double, fy: Double, fz: Double, timestamp: Long) {
        val norm = sqrt(fx.pow(2.0) + fy.pow(2.0) + fz.pow(2.0))
        val diff = abs(norm - SensorManager.GRAVITY_EARTH)
        Log.d(
            "GravityEstimatorTest",
            "onEstimation - fx: $fx, fy: $fy, fz: $fz, timestamp: $timestamp, norm: $norm, diff: $diff"
        )
    }
}