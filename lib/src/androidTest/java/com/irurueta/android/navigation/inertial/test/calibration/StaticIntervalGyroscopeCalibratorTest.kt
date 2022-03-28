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
import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalGyroscopeCalibrator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.numerical.robust.RobustEstimatorMethod
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalGyroscopeCalibratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

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
    fun startAndStop_whenGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    private fun buildCalibrator(
        context: Context,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType
    ): StaticIntervalGyroscopeCalibrator {
        val calibrator = StaticIntervalGyroscopeCalibrator(context,
            gyroscopeSensorType = gyroscopeSensorType,
            accelerometerSensorType = accelerometerSensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialGyroscopeBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ rad/s"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalGyroscopeCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalGyroscopeCalibrator) {
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "is accelerometer ground truth initial bias: ${calibrator.isAccelerometerGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "is gyroscope ground truth initial bias: ${calibrator.isGyroscopeGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "is G-dependant cross biases estimated: ${calibrator.isGDependentCrossBiasesEstimated}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "initial Gg: ${calibrator.gyroscopeInitialGg.buffer}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Gyroscope sensor: ${calibrator.gyroscopeSensor}"
        )
        Log.i("StaticIntervalGyroscopeCalibratorTest", "Window size: ${calibrator.windowSize}")
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Accelerometer base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Gyroscope base noise level: ${calibrator.gyroscopeBaseNoiseLevel} rad/s"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Accelerometer base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Accelerometer base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial sx: ${calibrator.gyroscopeInitialSx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial sy: ${calibrator.gyroscopeInitialSy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial sz: ${calibrator.gyroscopeInitialSz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial mxy: ${calibrator.gyroscopeInitialMxy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial mxz: ${calibrator.gyroscopeInitialMxz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial myx: ${calibrator.gyroscopeInitialMyx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial myz: ${calibrator.gyroscopeInitialMyz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial mzx: ${calibrator.gyroscopeInitialMzx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Initial mzy: ${calibrator.gyroscopeInitialMzy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Is common axis used: ${calibrator.isGyroscopeCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust method: ${calibrator.gyroscopeRobustMethod}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust confidence: ${calibrator.gyroscopeRobustConfidence}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust max iterations: ${calibrator.gyroscopeRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust preliminary subset size: ${calibrator.gyroscopeRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust threshold: ${calibrator.gyroscopeRobustThreshold}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust threshold factor: ${calibrator.gyroscopeRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Robust stop threshold factor: ${calibrator.gyroscopeRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated sx: ${calibrator.estimatedGyroscopeSx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated sy: ${calibrator.estimatedGyroscopeSy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated sz: ${calibrator.estimatedGyroscopeSz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedGyroscopeMxy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedGyroscopeMxz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated myx: ${calibrator.estimatedGyroscopeMyx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated myz: ${calibrator.estimatedGyroscopeMyz}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedGyroscopeMzx}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedGyroscopeMzy}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedGyroscopeCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedGyroscopeChiSq}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated mse: ${calibrator.estimatedGyroscopeMse}"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated gyroscope bias. x: ${calibrator.estimatedGyroscopeBiasX}, y: ${calibrator.estimatedGyroscopeBiasY}, z: ${calibrator.estimatedGyroscopeBiasZ} rad/s"
        )

        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedGyroscopeBiasStandardDeviationNorm} rad/s"
        )
        Log.i(
            "StaticIntervalGyroscopeCalibratorTest",
            "initial Gg: ${calibrator.estimatedGyroscopeGg?.buffer}"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }
}