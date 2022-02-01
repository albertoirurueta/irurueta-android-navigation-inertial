package com.irurueta.android.navigation.inertial.test.calibration

import android.content.Context
import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalAccelerometerCalibratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var currentLocation: Location? = null

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.ACCESS_COARSE_LOCATION,
        android.Manifest.permission.ACCESS_FINE_LOCATION,
        android.Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenNoLocationAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, AccelerometerSensorCollector.SensorType.ACCELEROMETER)

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, AccelerometerSensorCollector.SensorType.ACCELEROMETER)
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenNoLocationAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(
                context,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(
                context,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    private fun getCurrentLocation(): Location {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val currentLocationListener =
                    spyk(object : LocationService.OnCurrentLocationListener {
                        override fun onCurrentLocation(location: Location) {
                            currentLocation = location

                            syncHelper.notifyAll { completed++ }
                        }
                    })

                service.getCurrentLocation(currentLocationListener)
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        completed = 0

        val currentLocation = this.currentLocation
        requireNotNull(currentLocation)
        return currentLocation
    }

    private fun buildCalibrator(
        context: Context,
        sensorType: AccelerometerSensorCollector.SensorType
    ): StaticIntervalAccelerometerCalibrator {
        val calibrator = StaticIntervalAccelerometerCalibrator(context,
            sensorType = sensorType,
            initializationStartedListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            unreliableGravityNormEstimationListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Unreliable gravity"
                )
            },
            initialBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            newCalibrationMeasurementAvailableListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "AccelerometerCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "AccelerometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = { Log.i("AccelerometerCalibratorTest", "Calibrator stopped") }
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerCalibrator) {
        Log.i("AccelerometerCalibratorTest", "Result unreliable: ${calibrator.resultUnreliable}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Initial bias. x: ${calibrator.initialBiasX}, y: ${calibrator.initialBiasY}, z: ${calibrator.initialBiasZ} m/s^2"
        )
        Log.i("AccelerometerCalibratorTest", "Result unreliable: ${calibrator.resultUnreliable}")
        Log.i(
            "AccelerometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialBias}"
        )
        Log.i("AccelerometerCalibratorTest", "Location: ${calibrator.location}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i("AccelerometerCalibratorTest", "Gravity sensor: ${calibrator.gravitySensor}")
        Log.i("AccelerometerCalibratorTest", "Window size: ${calibrator.windowSize}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i("AccelerometerCalibratorTest", "Threshold factor: ${calibrator.thresholdFactor}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold}"
        )
        Log.i("AccelerometerCalibratorTest", "Base noise level: ${calibrator.baseNoiseLevel} m/s^2")
        Log.i(
            "AccelerometerCalibratorTest",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i("AccelerometerCalibratorTest", "Threshold: ${calibrator.threshold} m/s^2")
        Log.i(
            "AccelerometerCalibratorTest",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i("AccelerometerCalibratorTest", "Initial sx: ${calibrator.initialSx}")
        Log.i("AccelerometerCalibratorTest", "Initial sy: ${calibrator.initialSy}")
        Log.i("AccelerometerCalibratorTest", "Initial sz: ${calibrator.initialSz}")
        Log.i("AccelerometerCalibratorTest", "Initial mxy: ${calibrator.initialMxy}")
        Log.i("AccelerometerCalibratorTest", "Initial mxz: ${calibrator.initialMxz}")
        Log.i("AccelerometerCalibratorTest", "Initial myx: ${calibrator.initialMyx}")
        Log.i("AccelerometerCalibratorTest", "Initial myz: ${calibrator.initialMyz}")
        Log.i("AccelerometerCalibratorTest", "Initial mzx: ${calibrator.initialMzx}")
        Log.i("AccelerometerCalibratorTest", "Initial mzy: ${calibrator.initialMzy}")
        Log.i("AccelerometerCalibratorTest", "Is common axis used: ${calibrator.isCommonAxisUsed}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i("AccelerometerCalibratorTest", "Robust method: ${calibrator.robustMethod}")
        Log.i("AccelerometerCalibratorTest", "Robust confidence: ${calibrator.robustConfidence}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i("AccelerometerCalibratorTest", "Robust threshold: ${calibrator.robustThreshold}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Average gravity norm: ${calibrator.averageGravityNorm} m/s^2"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Gravity norm variance: ${calibrator.gravityNormVariance} m^2/s^4"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Gravity norm standard deviation: ${calibrator.gravityNormStandardDeviation} m/s^2"
        )
        Log.i("AccelerometerCalibratorTest", "Gravity PSD: ${calibrator.gravityPsd} m^2 * s^-3")
        Log.i(
            "AccelerometerCalibratorTest",
            "Gravity root PSD: ${calibrator.gravityRootPsd} m * s^-1.5"
        )
        Log.i("AccelerometerCalibratorTest", "Estimated sx: ${calibrator.estimatedSx}")
        Log.i("AccelerometerCalibratorTest", "Estimated sy: ${calibrator.estimatedSy}")
        Log.i("AccelerometerCalibratorTest", "Estimated sz: ${calibrator.estimatedSz}")
        Log.i("AccelerometerCalibratorTest", "Estimated mxy: ${calibrator.estimatedMxy}")
        Log.i("AccelerometerCalibratorTest", "Estimated mxz: ${calibrator.estimatedMxz}")
        Log.i("AccelerometerCalibratorTest", "Estimated myx: ${calibrator.estimatedMyx}")
        Log.i("AccelerometerCalibratorTest", "Estimated myz: ${calibrator.estimatedMyz}")
        Log.i("AccelerometerCalibratorTest", "Estimated mzx: ${calibrator.estimatedMzx}")
        Log.i("AccelerometerCalibratorTest", "Estimated mzy: ${calibrator.estimatedMzy}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer}"
        )
        Log.i("AccelerometerCalibratorTest", "Estimated chi sq: ${calibrator.estimatedChiSq}")
        Log.i("AccelerometerCalibratorTest", "Estimated mse: ${calibrator.estimatedMse}")
        Log.i(
            "AccelerometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedBiasX}, y: ${calibrator.estimatedBiasY}, z: ${calibrator.estimatedBiasZ} m/s^2"
        )
        Log.i(
            "AccelerometerCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedBiasStandardDeviationNorm} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }
}