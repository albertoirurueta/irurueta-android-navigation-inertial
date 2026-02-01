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
import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.old.calibration.StaticIntervalMagnetometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.numerical.robust.RobustEstimatorMethod
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalMagnetometerCalibratorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

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
    fun startAndStop_whenMagnetometerSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, MagnetometerSensorType.MAGNETOMETER)

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenMagnetometerUncalibratedSensor_completesCalibration() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    private fun buildCalibrator(
        context: Context,
        sensorType: MagnetometerSensorType
    ): StaticIntervalMagnetometerCalibrator {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            magnetometerSensorType = sensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initialization completed"
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialMagnetometerHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initial bias available. x: $hardIronX, y: $hardIronY, z: $hardIronZ µT"
                )
            },
            generatedMagnetometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalMagnetometerCalibrator) {
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial magnetic flux density norm: ${calibrator.initialMagneticFluxDensityNorm} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial hard iron. x: ${calibrator.magnetometerInitialHardIronX}, y: ${calibrator.magnetometerInitialHardIronY}, z: ${calibrator.magnetometerInitialHardIronZ} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "is ground truth initial hard iron: ${calibrator.isMagnetometerGroundTruthInitialHardIron}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Magnetometer sensor: ${calibrator.magnetometerSensor}"
        )
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Window size: ${calibrator.windowSize}")
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level: ${calibrator.magnetometerBaseNoiseLevel} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial sx: ${calibrator.magnetometerInitialSx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial sy: ${calibrator.magnetometerInitialSy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial sz: ${calibrator.magnetometerInitialSz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial mxy: ${calibrator.magnetometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial mxz: ${calibrator.magnetometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial myx: ${calibrator.magnetometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial myz: ${calibrator.magnetometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial mzx: ${calibrator.magnetometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial mzy: ${calibrator.magnetometerInitialMzy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isMagnetometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust method: ${calibrator.magnetometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust confidence: ${calibrator.magnetometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust max iterations: ${calibrator.magnetometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.magnetometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust threshold: ${calibrator.magnetometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust threshold factor: ${calibrator.magnetometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.magnetometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sx: ${calibrator.estimatedMagnetometerSx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedMagnetometerSy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedMagnetometerSz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedMagnetometerMxy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedMagnetometerMxz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedMagnetometerMyx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedMagnetometerMyz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedMagnetometerMzx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedMagnetometerMzy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedMagnetometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedMagnetometerChiSq}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedMagnetometerMse}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated hard iron. x: ${calibrator.estimatedMagnetometerHardIronX}, y: ${calibrator.estimatedMagnetometerHardIronY}, z: ${calibrator.estimatedMagnetometerHardIronZ} T"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L
    }
}