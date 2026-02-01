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
import com.irurueta.android.navigation.inertial.old.calibration.SingleSensorStaticIntervalMagnetometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.numerical.robust.RobustEstimatorMethod
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class SingleSensorStaticIntervalMagnetometerCalibratorTest {

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
    ): SingleSensorStaticIntervalMagnetometerCalibrator {
        val calibrator = SingleSensorStaticIntervalMagnetometerCalibrator(
            context,
            sensorType = sensorType,
            initializationStartedListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Initialization started"
                )

            },
            initializationCompletedListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Initial bias available. x: $hardIronX, y: $hardIronY, z: $hardIronZ T"
                )
            },
            newCalibrationMeasurementAvailableListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "SingleSensorStaticIntervalMagnetometerCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: SingleSensorStaticIntervalMagnetometerCalibrator) {
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial magnetic flux density norm: ${calibrator.initialMagneticFluxDensityNorm} T"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial bias. x: ${calibrator.initialHardIronX}, y: ${calibrator.initialHardIronY}, z: ${calibrator.initialHardIronZ} T"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialHardIron}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.magnetometerSensor}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} T"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Base noise level: ${calibrator.baseNoiseLevel} T"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} T^2 * s"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} T * s^0.5"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Threshold: ${calibrator.threshold} T"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial sx: ${calibrator.initialSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial sy: ${calibrator.initialSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial sz: ${calibrator.initialSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial mxy: ${calibrator.initialMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial mxz: ${calibrator.initialMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial myx: ${calibrator.initialMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial myz: ${calibrator.initialMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial mzx: ${calibrator.initialMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Initial mzy: ${calibrator.initialMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isCommonAxisUsed}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust method: ${calibrator.robustMethod}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust confidence: ${calibrator.robustConfidence}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust threshold: ${calibrator.robustThreshold}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated sx: ${calibrator.estimatedSx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedSy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedSz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedMxy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedMxz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedMyx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedMyz}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedMzx}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedMzy}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedChiSq}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedMse}"
        )
        Log.i(
            "SingleSensorStaticIntervalMagnetometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedHardIronX}, y: ${calibrator.estimatedHardIronY}, z: ${calibrator.estimatedHardIronZ} T"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L
    }
}