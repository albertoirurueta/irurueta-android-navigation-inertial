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
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerAndGyroscopeCalibrator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.test.LocationActivity
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
    fun startAndStop_whenNoLocationGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
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
    fun startAndStop_whenNoLocationGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
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
    fun startAndStop_whenNoLocationGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
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
    fun startAndStop_whenNoLocationGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
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

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationGyroscopeSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationGyroscopeSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationGyroscopeUncalibratedSensorAndAccelerometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER
        )
        calibrator.location = location

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLocationGyroscopeUncalibratedSensorAndAccelerometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType
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
            initialGyroscopeBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ rad/s"
                )
            },
            initialAccelerometerBiasAvailableListener = { _, biasX, biasY, biasZ ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "Initial bias available. x: $biasX, y: $biasY, z: $biasZ m/s^2"
                )
            },
            generatedAccelerometerMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
                )
            },
            generatedGyroscopeMeasurementListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
                Log.i(
                    "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
                    "New measurement available. $measurementsFoundSoFar / $requiredMeasurements"
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: StaticIntervalAccelerometerAndGyroscopeCalibrator) {
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Result unreliable: ${calibrator.accelerometerResultUnreliable}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial bias. x: ${calibrator.accelerometerInitialBiasX}, y: ${calibrator.accelerometerInitialBiasY}, z: ${calibrator.accelerometerInitialBiasZ} m/s^2"
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
            "initial Gg: ${calibrator.gyroscopeInitialGg.buffer}"
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
            "Initial sx: ${calibrator.accelerometerInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial sy: ${calibrator.accelerometerInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial sz: ${calibrator.accelerometerInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mxy: ${calibrator.accelerometerInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mxz: ${calibrator.accelerometerInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial myx: ${calibrator.accelerometerInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial myz: ${calibrator.accelerometerInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mzx: ${calibrator.accelerometerInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mzy: ${calibrator.accelerometerInitialMzy}"
        )

        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial sx: ${calibrator.gyroscopeInitialSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial sy: ${calibrator.gyroscopeInitialSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial sz: ${calibrator.gyroscopeInitialSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mxy: ${calibrator.gyroscopeInitialMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mxz: ${calibrator.gyroscopeInitialMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial myx: ${calibrator.gyroscopeInitialMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial myz: ${calibrator.gyroscopeInitialMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mzx: ${calibrator.gyroscopeInitialMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Initial mzy: ${calibrator.gyroscopeInitialMzy}"
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
            "Robust method: ${calibrator.accelerometerRobustMethod}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust confidence: ${calibrator.accelerometerRobustConfidence}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust max iterations: ${calibrator.accelerometerRobustMaxIterations}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust preliminary subset size: ${calibrator.accelerometerRobustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust threshold: ${calibrator.accelerometerRobustThreshold}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust threshold factor: ${calibrator.accelerometerRobustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust stop threshold factor: ${calibrator.accelerometerRobustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Robust method: ${calibrator.gyroscopeRobustMethod}"
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
            "Estimated sx: ${calibrator.estimatedAccelerometerSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated sy: ${calibrator.estimatedAccelerometerSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated sz: ${calibrator.estimatedAccelerometerSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedAccelerometerMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedAccelerometerMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated myx: ${calibrator.estimatedAccelerometerMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated myz: ${calibrator.estimatedAccelerometerMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedAccelerometerMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedAccelerometerMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedAccelerometerCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedAccelerometerChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mse: ${calibrator.estimatedAccelerometerMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedAccelerometerBiasStandardDeviationNorm} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated sx: ${calibrator.estimatedGyroscopeSx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated sy: ${calibrator.estimatedGyroscopeSy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated sz: ${calibrator.estimatedGyroscopeSz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedGyroscopeMxy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedGyroscopeMxz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated myx: ${calibrator.estimatedGyroscopeMyx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated myz: ${calibrator.estimatedGyroscopeMyz}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedGyroscopeMzx}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedGyroscopeMzy}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedGyroscopeCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedGyroscopeChiSq}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated mse: ${calibrator.estimatedGyroscopeMse}"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated accelerometer bias. x: ${calibrator.estimatedAccelerometerBiasX}, y: ${calibrator.estimatedAccelerometerBiasY}, z: ${calibrator.estimatedAccelerometerBiasZ} m/s^2"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated gyroscope bias. x: ${calibrator.estimatedGyroscopeBiasX}, y: ${calibrator.estimatedGyroscopeBiasY}, z: ${calibrator.estimatedGyroscopeBiasZ} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "Estimated bias standard deviation norm: ${calibrator.estimatedGyroscopeBiasStandardDeviationNorm} rad/s"
        )
        Log.i(
            "StaticIntervalAccelerometerAndGyroscopeCalibratorTest",
            "initial Gg: ${calibrator.estimatedGyroscopeGg?.buffer}"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }

}