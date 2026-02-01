/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.test.old.calibration

import android.Manifest
import android.content.Context
import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.old.calibration.SingleSensorStaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert.*
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

    @RequiresDevice
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

    @RequiresDevice
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

    @RequiresDevice
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

    @RequiresDevice
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
            initialBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "SingleSensorStaticIntervalAccelerometerCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
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
        return calibrator
    }

    private fun logCalibrationResult(calibrator: SingleSensorStaticIntervalAccelerometerCalibrator) {
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Result unreliable: ${calibrator.resultUnreliable}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial bias. x: ${calibrator.initialBiasX}, y: ${calibrator.initialBiasY}, z: ${calibrator.initialBiasZ} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialBias}"
        )
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Location: ${calibrator.location}")
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Gravity sensor: ${calibrator.gravitySensor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Base noise level: ${calibrator.baseNoiseLevel} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial sx: ${calibrator.initialSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial sy: ${calibrator.initialSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial sz: ${calibrator.initialSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial mxy: ${calibrator.initialMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial mxz: ${calibrator.initialMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial myx: ${calibrator.initialMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial myz: ${calibrator.initialMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial mzx: ${calibrator.initialMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Initial mzy: ${calibrator.initialMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Is common axis used: ${calibrator.isCommonAxisUsed}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust method: ${calibrator.robustMethod}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust confidence: ${calibrator.robustConfidence}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust threshold: ${calibrator.robustThreshold}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Average gravity norm: ${calibrator.averageGravityNorm} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Gravity norm variance: ${calibrator.gravityNormVariance} m^2/s^4"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Gravity norm standard deviation: ${calibrator.gravityNormStandardDeviation} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Gravity PSD: ${calibrator.gravityPsd} m^2 * s^-3"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Gravity root PSD: ${calibrator.gravityRootPsd} m * s^-1.5"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated sx: ${calibrator.estimatedSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated sy: ${calibrator.estimatedSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated sz: ${calibrator.estimatedSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated mxy: ${calibrator.estimatedMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated mxz: ${calibrator.estimatedMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated myx: ${calibrator.estimatedMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated myz: ${calibrator.estimatedMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated mzx: ${calibrator.estimatedMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated mzy: ${calibrator.estimatedMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated chi sq: ${calibrator.estimatedChiSq}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated mse: ${calibrator.estimatedMse}"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated bias. x: ${calibrator.estimatedBiasX}, y: ${calibrator.estimatedBiasY}, z: ${calibrator.estimatedBiasZ} m/s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalAccelerometerCalibrator",
            "Estimated bias standard deviation norm: ${calibrator.estimatedBiasStandardDeviationNorm} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L
    }
}