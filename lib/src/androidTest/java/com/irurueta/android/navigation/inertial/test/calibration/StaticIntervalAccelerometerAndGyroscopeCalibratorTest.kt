/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerAndGyroscopeCalibrator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.android.testutils.RequiresRealDevice
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalAccelerometerAndGyroscopeCalibratorTest {

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
    fun startAndStop_whenNoLocationGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenNoLocationGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenNoLocationGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenNoLocationGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenLocationGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
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
                Assert.assertTrue(enabled)

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
        Assert.assertEquals(1, completed)
        completed = 0

        val currentLocation = this.currentLocation
        requireNotNull(currentLocation)
        return currentLocation
    }

    private fun buildCalibrator(
        context: Context,
        gyroscopeSensorType: GyroscopeSensorType,
        accelerometerSensorType: AccelerometerSensorType
    ): StaticIntervalAccelerometerAndGyroscopeCalibrator {
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context,
            gyroscopeSensorType = gyroscopeSensorType,
            accelerometerSensorType = accelerometerSensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            generatedAccelerometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "New accelerometer measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "New gyroscope measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
        calibrator.thresholdFactor = THRESHOLD_FACTOR

        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator) {
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Result unreliable: ${calibrator.accelerometerResultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial accelerometer bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial gyroscope bias. x: ${calibrator.gyroscopeInitialBiasX}, y: ${calibrator.gyroscopeInitialBiasY}, z: ${calibrator.gyroscopeInitialBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "is accelerometer ground truth initial bias: ${calibrator.isAccelerometerGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "is gyroscope ground truth initial bias: ${calibrator.isGyroscopeGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "is G-dependant cross biases estimated: ${calibrator.isGDependentCrossBiasesEstimated}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "initial Gg: ${calibrator.gyroscopeInitialGg.buffer.contentToString()}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope sensor: ${calibrator.gyroscopeSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gravity sensor: ${calibrator.gravitySensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope base noise level: ${calibrator.gyroscopeBaseNoiseLevel} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial sx: ${calibrator.accelerometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial sy: ${calibrator.accelerometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial sz: ${calibrator.accelerometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial mxy: ${calibrator.accelerometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial mxz: ${calibrator.accelerometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial myx: ${calibrator.accelerometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial myz: ${calibrator.accelerometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial mzx: ${calibrator.accelerometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer initial mzy: ${calibrator.accelerometerInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial sx: ${calibrator.gyroscopeInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial sy: ${calibrator.gyroscopeInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial sz: ${calibrator.gyroscopeInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial mxy: ${calibrator.gyroscopeInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial mxz: ${calibrator.gyroscopeInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial myx: ${calibrator.gyroscopeInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial myz: ${calibrator.gyroscopeInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial mzx: ${calibrator.gyroscopeInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope initial mzy: ${calibrator.gyroscopeInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Is common axis used: ${calibrator.isAccelerometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Is common axis used: ${calibrator.isGyroscopeCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust method: ${calibrator.accelerometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust confidence: ${calibrator.accelerometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust max iterations: ${calibrator.accelerometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust preliminary subset size: ${calibrator.accelerometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust threshold: ${calibrator.accelerometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust threshold factor: ${calibrator.accelerometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Accelerometer robust stop threshold factor: ${calibrator.accelerometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gyroscope robust method: ${calibrator.gyroscopeRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust confidence: ${calibrator.gyroscopeRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust max iterations: ${calibrator.gyroscopeRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust preliminary subset size: ${calibrator.gyroscopeRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust threshold: ${calibrator.gyroscopeRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust threshold factor: ${calibrator.gyroscopeRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust stop threshold factor: ${calibrator.gyroscopeRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Average gravity norm: ${calibrator.averageGravityNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gravity norm variance: ${calibrator.gravityNormVariance} m^2/s^4"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gravity norm standard deviation: ${calibrator.gravityNormStandardDeviation} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gravity PSD: ${calibrator.gravityPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Gravity root PSD: ${calibrator.gravityRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer sx: ${calibrator.estimatedAccelerometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer sy: ${calibrator.estimatedAccelerometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer sz: ${calibrator.estimatedAccelerometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer mxy: ${calibrator.estimatedAccelerometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer mxz: ${calibrator.estimatedAccelerometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer myx: ${calibrator.estimatedAccelerometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer myz: ${calibrator.estimatedAccelerometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer mzx: ${calibrator.estimatedAccelerometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer mzy: ${calibrator.estimatedAccelerometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer covariance: ${calibrator.estimatedAccelerometerCovariance?.buffer.contentToString()}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer chi sq: ${calibrator.estimatedAccelerometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer chi sq degrees of freedom: ${calibrator.estimatedAccelerometerChiSqDegreesOfFreedom}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer reduced chi sq: ${calibrator.estimatedAccelerometerReducedChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer P: ${calibrator.estimatedAccelerometerP}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer Q: ${calibrator.estimatedAccelerometerQ}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer mse: ${calibrator.estimatedAccelerometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer bias standard deviation norm: ${calibrator.estimatedAccelerometerBiasStandardDeviationNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope sx: ${calibrator.estimatedGyroscopeSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope sy: ${calibrator.estimatedGyroscopeSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope sz: ${calibrator.estimatedGyroscopeSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope mxy: ${calibrator.estimatedGyroscopeMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope mxz: ${calibrator.estimatedGyroscopeMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope myx: ${calibrator.estimatedGyroscopeMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope myz: ${calibrator.estimatedGyroscopeMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope mzx: ${calibrator.estimatedGyroscopeMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope mzy: ${calibrator.estimatedGyroscopeMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope covariance: ${calibrator.estimatedGyroscopeCovariance?.buffer.contentToString()}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope chi sq: ${calibrator.estimatedGyroscopeChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope chi sq degrees of freedom: ${calibrator.estimatedGyroscopeChiSqDegreesOfFreedom}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope reduced chi sq: ${calibrator.estimatedGyroscopeReducedChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope P: ${calibrator.estimatedGyroscopeP}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope Q: ${calibrator.estimatedGyroscopeQ}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope mse: ${calibrator.estimatedGyroscopeMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope bias. x: ${calibrator.estimatedGyroscopeBiasX}, y: ${calibrator.estimatedGyroscopeBiasY}, z: ${calibrator.estimatedGyroscopeBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope bias standard deviation norm: ${calibrator.estimatedGyroscopeBiasStandardDeviationNorm} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial gyroscope Gg: ${calibrator.estimatedGyroscopeGg?.buffer.contentToString()}"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val THRESHOLD_FACTOR = 3.0
    }
}