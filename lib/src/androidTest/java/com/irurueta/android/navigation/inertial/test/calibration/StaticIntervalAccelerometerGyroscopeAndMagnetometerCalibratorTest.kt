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
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest {

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
    fun startAndStop_whenMagnetometerSensorGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
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
    fun startAndStop_whenMagnetometerSensorGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
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
    fun startAndStop_whenMagnetometerSensorGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
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
    fun startAndStop_whenMagnetometerSensorGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenMagnetometerUncalibratedSensorGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
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
    fun startAndStop_whenMagnetometerUncalibratedSensorGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
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
    fun startAndStop_whenMagnetometerUncalibratedSensorGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
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
    fun startAndStop_whenMagnetometerUncalibratedSensorGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

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
        location: Location,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType
    ): StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator {
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location = location,
            magnetometerSensorType = magnetometerSensorType,
            gyroscopeSensorType = gyroscopeSensorType,
            accelerometerSensorType = accelerometerSensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialMagnetometerHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Initial magnetometer hard iron available. x: $hardIronX, y: $hardIronY, z: $hardIronZ µT"
                )
            },
            initialGyroscopeBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Initial gyroscope bias available. x: $biasX, y: $biasY, z: $biasZ rad/s"
                )
            },
            initialAccelerometerBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Initial accelerometer bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "New accelerometer measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "New gyroscope measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedMagnetometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "New magnetometer measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
                    "Calibrator stopped"
                )
            }
        )
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator) {
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Initial bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Initial gyroscope bias. x: ${calibrator.gyroscopeInitialBiasX}, y: ${calibrator.gyroscopeInitialBiasY}, z: ${calibrator.gyroscopeInitialBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Initial magnetometer hard iron. x: ${calibrator.magnetometerInitialHardIronX}, y: ${calibrator.magnetometerInitialHardIronX}, z: ${calibrator.magnetometerInitialHardIronX} T"
        )

        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "is accelerometer ground truth initial bias: ${calibrator.isAccelerometerGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "is gyroscope ground truth initial bias: ${calibrator.isGyroscopeGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "is magnetometer ground truth initial hard iron: ${calibrator.isMagnetometerGroundTruthInitialHardIron}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "is G-dependant cross biases estimated: ${calibrator.isGDependentCrossBiasesEstimated}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "initial Gg: ${calibrator.gyroscopeInitialGg.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope sensor: ${calibrator.gyroscopeSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Magnetometer sensor: ${calibrator.magnetometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope base noise level: ${calibrator.gyroscopeBaseNoiseLevel} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Magnetometer base noise level: ${calibrator.magnetometerBaseNoiseLevel} T"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial sx: ${calibrator.accelerometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial sy: ${calibrator.accelerometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial sz: ${calibrator.accelerometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial mxy: ${calibrator.accelerometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial mxz: ${calibrator.accelerometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial myx: ${calibrator.accelerometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial myz: ${calibrator.accelerometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial mzx: ${calibrator.accelerometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer initial mzy: ${calibrator.accelerometerInitialMzy}"
        )

        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial sx: ${calibrator.gyroscopeInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial sy: ${calibrator.gyroscopeInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial sz: ${calibrator.gyroscopeInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial mxy: ${calibrator.gyroscopeInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial mxz: ${calibrator.gyroscopeInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial myx: ${calibrator.gyroscopeInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial myz: ${calibrator.gyroscopeInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial mzx: ${calibrator.gyroscopeInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope initial mzy: ${calibrator.gyroscopeInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial sx: ${calibrator.magnetometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial sy: ${calibrator.magnetometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial sz: ${calibrator.magnetometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial mxy: ${calibrator.magnetometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial mxz: ${calibrator.magnetometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial myx: ${calibrator.magnetometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial myz: ${calibrator.magnetometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial mzx: ${calibrator.magnetometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer initial mzy: ${calibrator.magnetometerInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isAccelerometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isGyroscopeCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isMagnetometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust method: ${calibrator.accelerometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust confidence: ${calibrator.accelerometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust max iterations: ${calibrator.accelerometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust preliminary subset size: ${calibrator.accelerometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust threshold: ${calibrator.accelerometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust threshold factor: ${calibrator.accelerometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Accelerometer robust stop threshold factor: ${calibrator.accelerometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust method: ${calibrator.gyroscopeRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust confidence: ${calibrator.gyroscopeRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust max iterations: ${calibrator.gyroscopeRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust preliminary subset size: ${calibrator.gyroscopeRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust threshold: ${calibrator.gyroscopeRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust threshold factor: ${calibrator.gyroscopeRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Gyroscope robust stop threshold factor: ${calibrator.gyroscopeRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust method: ${calibrator.magnetometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust confidence: ${calibrator.magnetometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust max iterations: ${calibrator.magnetometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust preliminary subset size: ${calibrator.magnetometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust threshold: ${calibrator.magnetometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust threshold factor: ${calibrator.magnetometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Magnetometer robust stop threshold factor: ${calibrator.magnetometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer sx: ${calibrator.estimatedAccelerometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer sy: ${calibrator.estimatedAccelerometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer sz: ${calibrator.estimatedAccelerometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer mxy: ${calibrator.estimatedAccelerometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer mxz: ${calibrator.estimatedAccelerometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer myx: ${calibrator.estimatedAccelerometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer myz: ${calibrator.estimatedAccelerometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer mzx: ${calibrator.estimatedAccelerometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer mzy: ${calibrator.estimatedAccelerometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer covariance: ${calibrator.estimatedAccelerometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer chi sq: ${calibrator.estimatedAccelerometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer mse: ${calibrator.estimatedAccelerometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer bias standard deviation norm: ${calibrator.estimatedAccelerometerBiasStandardDeviationNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope sx: ${calibrator.estimatedGyroscopeSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope sy: ${calibrator.estimatedGyroscopeSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope sz: ${calibrator.estimatedGyroscopeSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope mxy: ${calibrator.estimatedGyroscopeMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope mxz: ${calibrator.estimatedGyroscopeMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope myx: ${calibrator.estimatedGyroscopeMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope myz: ${calibrator.estimatedGyroscopeMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope mzx: ${calibrator.estimatedGyroscopeMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope mzy: ${calibrator.estimatedGyroscopeMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope covariance: ${calibrator.estimatedGyroscopeCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope chi sq: ${calibrator.estimatedGyroscopeChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope mse: ${calibrator.estimatedGyroscopeMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope bias. x: ${calibrator.estimatedGyroscopeBiasX}, y: ${calibrator.estimatedGyroscopeBiasY}, z: ${calibrator.estimatedGyroscopeBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Estimated gyroscope bias standard deviation norm: ${calibrator.estimatedGyroscopeBiasStandardDeviationNorm} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest",
            "Initial gyroscope Gg: ${calibrator.estimatedGyroscopeGg?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer sx: ${calibrator.estimatedMagnetometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer sy: ${calibrator.estimatedMagnetometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer sz: ${calibrator.estimatedMagnetometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer mxy: ${calibrator.estimatedMagnetometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer mxz: ${calibrator.estimatedMagnetometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer myx: ${calibrator.estimatedMagnetometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer myz: ${calibrator.estimatedMagnetometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer mzx: ${calibrator.estimatedMagnetometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer mzy: ${calibrator.estimatedMagnetometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer covariance: ${calibrator.estimatedMagnetometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer chi sq: ${calibrator.estimatedMagnetometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer mse: ${calibrator.estimatedMagnetometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated magnetometer hard iron. x: ${calibrator.estimatedMagnetometerHardIronX}, y: ${calibrator.estimatedMagnetometerHardIronY}, z: ${calibrator.estimatedMagnetometerHardIronZ} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L
    }
}