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

import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.estimators.attitude.LevelingEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.test.LocationActivity
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

@RequiresDevice
class LevelingEstimatorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var activity: LocationActivity? = null

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun startAndStop_whenGravitySensor_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LevelingEstimator(
            activity,
            useAccelerometer = false,
            estimateEulerAngles = true,
            levelingAvailableListener = { _, _, _, roll, pitch, _ ->
                logLeveling(roll, pitch)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerSensorAndLowPassAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerSensorAndMeanAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerSensorAndMedianAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndLowPassAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndMeanAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenAccelerometerUncalibratedSensorAndMedianAveragingFilter_estimatesLeveling() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LevelingEstimatorTest.activity = activity
            }
        }
        assertNotNull(scenario)
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
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    private fun logLeveling(roll: Double?, pitch: Double?) {
        Log.d("LevelingEstimatorTest", "onLevelingAvailable - roll: $roll rad, pitch: $pitch rad")
    }
}