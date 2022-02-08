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
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            unreliableGravityNormEstimationListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Unreliable gravity"
                )
            },
            initialBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            newCalibrationMeasurementAvailableListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerCalibrator) {
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Result unreliable: ${calibrator.resultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Initial bias. x: ${calibrator.initialBiasX}, y: ${calibrator.initialBiasY}, z: ${calibrator.initialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Result unreliable: ${calibrator.resultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialBias}"
        )
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Location: ${calibrator.location}")
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Gravity sensor: ${calibrator.gravitySensor}"
        )
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Window size: ${calibrator.windowSize}")
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level: ${calibrator.baseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sx: ${calibrator.initialSx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sy: ${calibrator.initialSy}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sz: ${calibrator.initialSz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mxy: ${calibrator.initialMxy}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mxz: ${calibrator.initialMxz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial myx: ${calibrator.initialMyx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial myz: ${calibrator.initialMyz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mzx: ${calibrator.initialMzx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mzy: ${calibrator.initialMzy}")
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Is common axis used: ${calibrator.isCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust method: ${calibrator.robustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust confidence: ${calibrator.robustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust threshold: ${calibrator.robustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Average gravity norm: ${calibrator.averageGravityNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Gravity norm variance: ${calibrator.gravityNormVariance} m^2/s^4"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Gravity norm standard deviation: ${calibrator.gravityNormStandardDeviation} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Gravity PSD: ${calibrator.gravityPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Gravity root PSD: ${calibrator.gravityRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated sx: ${calibrator.estimatedSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedBiasX}, y: ${calibrator.estimatedBiasY}, z: ${calibrator.estimatedBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedBiasStandardDeviationNorm} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }
}