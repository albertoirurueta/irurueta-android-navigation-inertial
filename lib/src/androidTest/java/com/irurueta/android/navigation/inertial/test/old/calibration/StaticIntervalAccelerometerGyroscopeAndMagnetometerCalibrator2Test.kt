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
import com.irurueta.android.navigation.inertial.old.calibration.StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test {

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
    fun startAndStop_whenMagnetometerSensorGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorType.MAGNETOMETER,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER
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
            MagnetometerSensorType.MAGNETOMETER,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
            MagnetometerSensorType.MAGNETOMETER,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER
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
            MagnetometerSensorType.MAGNETOMETER,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER
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
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER
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
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
        magnetometerSensorType: MagnetometerSensorType,
        gyroscopeSensorType: GyroscopeSensorType,
        accelerometerSensorType: AccelerometerSensorType
    ): StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2 {
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2(
            context,
            location = location,
            magnetometerSensorType = magnetometerSensorType,
            gyroscopeSensorType = gyroscopeSensorType,
            accelerometerSensorType = accelerometerSensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Initialization started"
                )
            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialMagnetometerHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Initial magnetometer hard iron available. x: $hardIronX, y: $hardIronY, z: $hardIronZ µT"
                )
            },
            initialGyroscopeBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Initial gyroscope bias available. x: $biasX, y: $biasY, z: $biasZ rad/s"
                )
            },
            initialAccelerometerBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Initial accelerometer bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "New accelerometer measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "New gyroscope measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedMagnetometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "New magnetometer measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            readyToSolveCalibrationListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Ready to solve calibration"
                )
            },
            calibrationSolvingStartedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Calibration solving started"
                )
            },
            calibrationCompletedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
                    "Calibration completed"
                )

                syncHelper.notifyAll { completed++ }
            },
            stoppedListener = {
                Log.i(
                    "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
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

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2) {
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Initial bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Initial gyroscope bias. x: ${calibrator.gyroscopeInitialBiasX}, y: ${calibrator.gyroscopeInitialBiasY}, z: ${calibrator.gyroscopeInitialBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Initial magnetometer hard iron. x: ${calibrator.magnetometerInitialHardIronX}, y: ${calibrator.magnetometerInitialHardIronX}, z: ${calibrator.magnetometerInitialHardIronX} T"
        )

        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "is accelerometer ground truth initial bias: ${calibrator.isAccelerometerGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "is gyroscope ground truth initial bias: ${calibrator.isGyroscopeGroundTruthInitialBias}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "is magnetometer ground truth initial hard iron: ${calibrator.isMagnetometerGroundTruthInitialHardIron}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "is G-dependant cross biases estimated: ${calibrator.isGDependentCrossBiasesEstimated}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "initial Gg: ${calibrator.gyroscopeInitialGg.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer sensor: ${calibrator.accelerometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope sensor: ${calibrator.gyroscopeSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Magnetometer sensor: ${calibrator.magnetometerSensor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Window size: ${calibrator.windowSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Initial static samples: ${calibrator.initialStaticSamples}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Threshold factor: ${calibrator.thresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Instantaneous noise level factor: ${calibrator.instantaneousNoiseLevelFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer base noise level: ${calibrator.accelerometerBaseNoiseLevel} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope base noise level: ${calibrator.gyroscopeBaseNoiseLevel} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Magnetometer base noise level: ${calibrator.magnetometerBaseNoiseLevel} T"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer base noise level PSD: ${calibrator.accelerometerBaseNoiseLevelPsd} m^2 * s^-3"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer base noise level root PSD: ${calibrator.accelerometerBaseNoiseLevelRootPsd} m * s^-1.5"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Threshold: ${calibrator.threshold} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Average time interval: ${calibrator.accelerometerAverageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Time interval variance: ${calibrator.accelerometerTimeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Time interval standard deviation: ${calibrator.accelerometerTimeIntervalStandardDeviation} s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial sx: ${calibrator.accelerometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial sy: ${calibrator.accelerometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial sz: ${calibrator.accelerometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial mxy: ${calibrator.accelerometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial mxz: ${calibrator.accelerometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial myx: ${calibrator.accelerometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial myz: ${calibrator.accelerometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial mzx: ${calibrator.accelerometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer initial mzy: ${calibrator.accelerometerInitialMzy}"
        )

        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial sx: ${calibrator.gyroscopeInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial sy: ${calibrator.gyroscopeInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial sz: ${calibrator.gyroscopeInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial mxy: ${calibrator.gyroscopeInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial mxz: ${calibrator.gyroscopeInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial myx: ${calibrator.gyroscopeInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial myz: ${calibrator.gyroscopeInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial mzx: ${calibrator.gyroscopeInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope initial mzy: ${calibrator.gyroscopeInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial sx: ${calibrator.magnetometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial sy: ${calibrator.magnetometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial sz: ${calibrator.magnetometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial mxy: ${calibrator.magnetometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial mxz: ${calibrator.magnetometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial myx: ${calibrator.magnetometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial myz: ${calibrator.magnetometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial mzx: ${calibrator.magnetometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer initial mzy: ${calibrator.magnetometerInitialMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Is common axis used: ${calibrator.isAccelerometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Is common axis used: ${calibrator.isGyroscopeCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Is common axis used: ${calibrator.isMagnetometerCommonAxisUsed}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Minimum required measurements: ${calibrator.minimumRequiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Required measurements: ${calibrator.requiredMeasurements}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust method: ${calibrator.accelerometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust confidence: ${calibrator.accelerometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust max iterations: ${calibrator.accelerometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust preliminary subset size: ${calibrator.accelerometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust threshold: ${calibrator.accelerometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust threshold factor: ${calibrator.accelerometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Accelerometer robust stop threshold factor: ${calibrator.accelerometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust method: ${calibrator.gyroscopeRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust confidence: ${calibrator.gyroscopeRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust max iterations: ${calibrator.gyroscopeRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust preliminary subset size: ${calibrator.gyroscopeRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust threshold: ${calibrator.gyroscopeRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust threshold factor: ${calibrator.gyroscopeRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Gyroscope robust stop threshold factor: ${calibrator.gyroscopeRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust method: ${calibrator.magnetometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust confidence: ${calibrator.magnetometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust max iterations: ${calibrator.magnetometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust preliminary subset size: ${calibrator.magnetometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust threshold: ${calibrator.magnetometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust threshold factor: ${calibrator.magnetometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Magnetometer robust stop threshold factor: ${calibrator.magnetometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer sx: ${calibrator.estimatedAccelerometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer sy: ${calibrator.estimatedAccelerometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer sz: ${calibrator.estimatedAccelerometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer mxy: ${calibrator.estimatedAccelerometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer mxz: ${calibrator.estimatedAccelerometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer myx: ${calibrator.estimatedAccelerometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer myz: ${calibrator.estimatedAccelerometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer mzx: ${calibrator.estimatedAccelerometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer mzy: ${calibrator.estimatedAccelerometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer covariance: ${calibrator.estimatedAccelerometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer chi sq: ${calibrator.estimatedAccelerometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer mse: ${calibrator.estimatedAccelerometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer bias standard deviation norm: ${calibrator.estimatedAccelerometerBiasStandardDeviationNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope sx: ${calibrator.estimatedGyroscopeSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope sy: ${calibrator.estimatedGyroscopeSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope sz: ${calibrator.estimatedGyroscopeSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope mxy: ${calibrator.estimatedGyroscopeMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope mxz: ${calibrator.estimatedGyroscopeMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope myx: ${calibrator.estimatedGyroscopeMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope myz: ${calibrator.estimatedGyroscopeMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope mzx: ${calibrator.estimatedGyroscopeMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope mzy: ${calibrator.estimatedGyroscopeMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope covariance: ${calibrator.estimatedGyroscopeCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope chi sq: ${calibrator.estimatedGyroscopeChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope mse: ${calibrator.estimatedGyroscopeMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope bias. x: ${calibrator.estimatedGyroscopeBiasX}, y: ${calibrator.estimatedGyroscopeBiasY}, z: ${calibrator.estimatedGyroscopeBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Estimated gyroscope bias standard deviation norm: ${calibrator.estimatedGyroscopeBiasStandardDeviationNorm} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2Test",
            "Initial gyroscope Gg: ${calibrator.estimatedGyroscopeGg?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer sx: ${calibrator.estimatedMagnetometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer sy: ${calibrator.estimatedMagnetometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer sz: ${calibrator.estimatedMagnetometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer mxy: ${calibrator.estimatedMagnetometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer mxz: ${calibrator.estimatedMagnetometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer myx: ${calibrator.estimatedMagnetometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer myz: ${calibrator.estimatedMagnetometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer mzx: ${calibrator.estimatedMagnetometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer mzy: ${calibrator.estimatedMagnetometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer covariance: ${calibrator.estimatedMagnetometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer chi sq: ${calibrator.estimatedMagnetometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer mse: ${calibrator.estimatedMagnetometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibrator2Test",
            "Estimated magnetometer hard iron. x: ${calibrator.estimatedMagnetometerHardIronX}, y: ${calibrator.estimatedMagnetometerHardIronY}, z: ${calibrator.estimatedMagnetometerHardIronZ} m/s^2"
        )
    }

    private companion object {
        const val TIMEOUT = 5000L
    }
}