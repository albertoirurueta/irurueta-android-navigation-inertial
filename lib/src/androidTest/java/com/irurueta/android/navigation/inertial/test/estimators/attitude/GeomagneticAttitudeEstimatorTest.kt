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
package com.irurueta.android.navigation.inertial.test.estimators.attitude

import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.estimators.attitude.GeomagneticAttitudeEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.test.LocationActivity
import io.mockk.spyk
import org.junit.Assert
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class GeomagneticAttitudeEstimatorTest {

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.ACCESS_COARSE_LOCATION,
        android.Manifest.permission.ACCESS_FINE_LOCATION,
        android.Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var currentLocation: Location? = null

    private var activity: LocationActivity? = null

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGravitySensor_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = false,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerSensorMagnetometerSensorAndLowPassAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            accelerometerAveragingFilter = LowPassAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerSensorMagnetometerSensorAndMeanPassAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            accelerometerAveragingFilter = MeanAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerSensorMagnetometerSensorAndMedianPassAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            accelerometerAveragingFilter = MedianAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorMagnetometerUncalibratedSensorAndLowPassAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            accelerometerAveragingFilter = LowPassAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedMagnetometerUncalibratedSensorAndMeanAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            accelerometerAveragingFilter = MeanAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorMagnetometerUncalibratedSensorAndMedianAveragingFilter_estimatesAttitude() {
        val location = getCurrentLocation()

        val activity = this.activity
        requireNotNull(activity)
        val estimator = GeomagneticAttitudeEstimator(
            activity,
            location,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            magnetometerSensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            accelerometerAveragingFilter = MedianAveragingFilter(),
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logLeveling(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    private fun logLeveling(roll: Double?, pitch: Double?, yaw: Double?) {
        Log.d(
            "GeomagneticAttitudeEstimatorTest",
            "onLevelingAvailable - roll: $roll rad, pitch: $pitch rad, yaw: $yaw rad"
        )
    }

    private fun getCurrentLocation(): Location {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@GeomagneticAttitudeEstimatorTest.activity = activity
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
        Assert.assertEquals(1, completed)
        completed = 0

        val currentLocation = this.currentLocation
        requireNotNull(currentLocation)
        return currentLocation
    }
}