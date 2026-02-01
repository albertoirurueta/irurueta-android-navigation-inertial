/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.irurueta.android.navigation.inertial.test.calibration

import android.Manifest
import android.content.Context
import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.SingleSensorStaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.android.testutils.RequiresRealDevice
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class SingleSensorStaticIntervalAccelerometerCalibratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var currentLocation: Location? = null

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        Manifest.permission.ACCESS_COARSE_LOCATION,
        Manifest.permission.ACCESS_FINE_LOCATION,
        Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenNoLocationAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, AccelerometerSensorType.ACCELEROMETER)

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, AccelerometerSensorType.ACCELEROMETER)
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenNoLocationAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(
                context,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(
                context,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
        sensorType: AccelerometerSensorType
    ): SingleSensorStaticIntervalAccelerometerCalibrator {
        val calibrator = SingleSensorStaticIntervalAccelerometerCalibrator(context,
            sensorType = sensorType,
            initializationStartedListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            unreliableGravityNormEstimationListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Unreliable gravity"
                )
            },
            newCalibrationMeasurementAvailableListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
        calibrator.thresholdFactor = THRESHOLD_FACTOR

        return calibrator
    }

    private fun logCalibrationResult(calibrator: SingleSensorStaticIntervalAccelerometerCalibrator) {
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Result unreliable: ${calibrator.resultUnreliable}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial bias. x: ${calibrator.initialBiasX}, y: ${calibrator.initialBiasY}, z: ${calibrator.initialBiasZ} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialBias}"
        )
        Log.i("SingleSensorStaticIntervalAccelerometerCalibratorTest", "Location: ${calibrator.location}")
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Gravity sensor: ${calibrator.gravitySensor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Base noise level: ${calibrator.baseNoiseLevel} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial sx: ${calibrator.initialSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial sy: ${calibrator.initialSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial sz: ${calibrator.initialSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial mxy: ${calibrator.initialMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial mxz: ${calibrator.initialMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial myx: ${calibrator.initialMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial myz: ${calibrator.initialMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial mzx: ${calibrator.initialMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Initial mzy: ${calibrator.initialMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Is common axis used: ${calibrator.isCommonAxisUsed}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust method: ${calibrator.robustMethod}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust confidence: ${calibrator.robustConfidence}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust threshold: ${calibrator.robustThreshold}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Average gravity norm: ${calibrator.averageGravityNorm} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Gravity norm variance: ${calibrator.gravityNormVariance} m^2/s^4"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Gravity norm standard deviation: ${calibrator.gravityNormStandardDeviation} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Gravity PSD: ${calibrator.gravityPsd} m^2 * s^-3"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Gravity root PSD: ${calibrator.gravityRootPsd} m * s^-1.5"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated sx: ${calibrator.estimatedSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer.contentToString()}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedChiSq}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated chi sq degrees of freedom: ${calibrator.estimatedChiSqDegreesOfFreedom}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated reduced chi sq: ${calibrator.estimatedReducedChiSq}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated P: ${calibrator.estimatedP}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated Q: ${calibrator.estimatedQ}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedMse}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedBiasX}, y: ${calibrator.estimatedBiasY}, z: ${calibrator.estimatedBiasZ} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedBiasStandardDeviationNorm} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val THRESHOLD_FACTOR = 3.0
    }
}