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
import org.junit.Assert
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
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
        Assert.assertNotNull(scenario)

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
            accelerometerSensorType = sensorType,
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
            initialAccelerometerBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
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
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerCalibrator) {
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Result unreliable: ${calibrator.accelerometerResultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Initial bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Result unreliable: ${calibrator.accelerometerResultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isAccelerometerGroundTruthInitialBias}"
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
            "Base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sx: ${calibrator.accelerometerInitialSx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sy: ${calibrator.accelerometerInitialSy}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial sz: ${calibrator.accelerometerInitialSz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mxy: ${calibrator.accelerometerInitialMxy}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mxz: ${calibrator.accelerometerInitialMxz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial myx: ${calibrator.accelerometerInitialMyx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial myz: ${calibrator.accelerometerInitialMyz}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mzx: ${calibrator.accelerometerInitialMzx}")
        Log.i("StaticIntervalAccelerometerCalibratorTest", "Initial mzy: ${calibrator.accelerometerInitialMzy}")
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Is common axis used: ${calibrator.isAccelerometerCommonAxisUsed}"
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
            "Robust method: ${calibrator.accelerometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust confidence: ${calibrator.accelerometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust max iterations: ${calibrator.accelerometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.accelerometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust threshold: ${calibrator.accelerometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust threshold factor: ${calibrator.accelerometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.accelerometerRobustStopThresholdFactor}"
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
            "Estimated sx: ${calibrator.estimatedAccelerometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedAccelerometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedAccelerometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedAccelerometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedAccelerometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedAccelerometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedAccelerometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedAccelerometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedAccelerometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedAccelerometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedAccelerometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedAccelerometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedAccelerometerBiasStandardDeviationNorm} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }
}