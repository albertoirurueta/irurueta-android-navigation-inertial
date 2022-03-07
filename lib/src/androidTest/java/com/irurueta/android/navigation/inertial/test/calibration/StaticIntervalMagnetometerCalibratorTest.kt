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
import com.irurueta.android.navigation.inertial.calibration.SingleSensorStaticIntervalMagnetometerCalibrator
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.numerical.robust.RobustEstimatorMethod
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class StaticIntervalMagnetometerCalibratorTest {

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
    fun startAndStop_whenMagnetometerSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator =
            buildCalibrator(context, location, MagnetometerSensorCollector.SensorType.MAGNETOMETER)

        calibrator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = TIMEOUT)

        calibrator.stop()

        logCalibrationResult(calibrator)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenMagnetometerUncalibratedSensor_completesCalibration() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val calibrator = buildCalibrator(
            context,
            location,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED
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
        location: Location,
        sensorType: MagnetometerSensorCollector.SensorType
    ): SingleSensorStaticIntervalMagnetometerCalibrator {
        val calibrator = SingleSensorStaticIntervalMagnetometerCalibrator(
            context,
            location,
            sensorType = sensorType,
            initializationStartedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initialization started"
                )

            },
            initializationCompletedListener = {
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initialization completed."
                )
            },
            errorListener = { _, errorReason ->
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Error: $errorReason"
                )

                syncHelper.notifyAll { completed++ }
            },
            initialHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
                Log.i(
                    "StaticIntervalMagnetometerCalibratorTest",
                    "Initial bias available. x: $hardIronX, y: $hardIronY, z: $hardIronZ T"
                )
            },
            newCalibrationMeasurementAvailableListener = { _, _, measurementsFoundSoFar, requiredMeasurements ->
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
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.requiredMeasurements = 3 * calibrator.minimumRequiredMeasurements
        return calibrator
    }

    private fun logCalibrationResult(calibrator: SingleSensorStaticIntervalMagnetometerCalibrator) {
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Initial bias. x: ${calibrator.initialHardIronX}, y: ${calibrator.initialHardIronY}, z: ${calibrator.initialHardIronZ} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "is ground truth initial bias: ${calibrator.isGroundTruthInitialHardIron}"
        )
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Location: ${calibrator.location}")
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Accelerometer sensor: ${calibrator.magnetometerSensor}"
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
            "Base noise level absolute threshold: ${calibrator.baseNoiseLevelAbsoluteThreshold} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level: ${calibrator.baseNoiseLevel} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level PSD: ${calibrator.baseNoiseLevelPsd} T^2 * s"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Base noise level root PSD: ${calibrator.baseNoiseLevelRootPsd} T * s^0.5"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Threshold: ${calibrator.threshold} T"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Average time interval: ${calibrator.averageTimeInterval} s"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Time interval variance: ${calibrator.timeIntervalVariance} s^2"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Time interval standard deviation: ${calibrator.timeIntervalStandardDeviation} s"
        )
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial sx: ${calibrator.initialSx}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial sy: ${calibrator.initialSy}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial sz: ${calibrator.initialSz}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial mxy: ${calibrator.initialMxy}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial mxz: ${calibrator.initialMxz}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial myx: ${calibrator.initialMyx}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial myz: ${calibrator.initialMyz}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial mzx: ${calibrator.initialMzx}")
        Log.i("StaticIntervalMagnetometerCalibratorTest", "Initial mzy: ${calibrator.initialMzy}")
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Is common axis used: ${calibrator.isCommonAxisUsed}"
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
            "Robust method: ${calibrator.robustMethod}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust confidence: ${calibrator.robustConfidence}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust max iterations: ${calibrator.robustMaxIterations}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust preliminary subset size: ${calibrator.robustPreliminarySubsetSize}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust threshold: ${calibrator.robustThreshold}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust threshold factor: ${calibrator.robustThresholdFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Robust stop threshold factor: ${calibrator.robustStopThresholdFactor}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sx: ${calibrator.estimatedSx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sy: ${calibrator.estimatedSy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated sz: ${calibrator.estimatedSz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mxy: ${calibrator.estimatedMxy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mxz: ${calibrator.estimatedMxz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated myx: ${calibrator.estimatedMyx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated myz: ${calibrator.estimatedMyz}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mzx: ${calibrator.estimatedMzx}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mzy: ${calibrator.estimatedMzy}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated covariance: ${calibrator.estimatedCovariance?.buffer}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated chi sq: ${calibrator.estimatedChiSq}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated mse: ${calibrator.estimatedMse}"
        )
        Log.i(
            "StaticIntervalMagnetometerCalibratorTest",
            "Estimated bias. x: ${calibrator.estimatedHardIronX}, y: ${calibrator.estimatedHardIronY}, z: ${calibrator.estimatedHardIronZ} T"
        )
    }

    private companion object {
        const val TIMEOUT = 500000L
    }
}