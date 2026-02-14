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

package com.irurueta.android.navigation.inertial.test.estimators.attitude

import android.Manifest
import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.estimators.attitude.LevelingEstimator
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.android.testutils.RequiresRealDevice
import io.mockk.spyk
import org.junit.Assert
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class LevelingEstimatorTest {

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        Manifest.permission.ACCESS_COARSE_LOCATION,
        Manifest.permission.ACCESS_FINE_LOCATION,
        Manifest.permission.ACCESS_BACKGROUND_LOCATION
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

    @RequiresRealDevice
    @Test
    fun startAndStop_whenGravitySensor_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = false,
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerSensorAndLowPassAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter = LowPassAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerSensorAndMeanAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter = MeanAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerSensorAndMedianAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter = MedianAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndLowPassAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerAveragingFilter = LowPassAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndMeanAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerAveragingFilter = MeanAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndMedianAveragingFilter_estimatesLeveling() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = true,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerAveragingFilter = MedianAveragingFilter(),
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            },
            location = location
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    private fun getCurrentLocation(): Location {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
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

    private fun logLeveling(roll: Double?, pitch: Double?) {
        Log.d("LevelingEstimatorTest", "onLevelingAvailable - roll: $roll rad, pitch: $pitch rad")
    }
}