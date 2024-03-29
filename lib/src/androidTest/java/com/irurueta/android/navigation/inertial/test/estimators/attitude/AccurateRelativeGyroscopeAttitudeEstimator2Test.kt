/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.estimators.attitude.AccurateRelativeGyroscopeAttitudeEstimator2
import com.irurueta.android.navigation.inertial.test.LocationActivity
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AccurateRelativeGyroscopeAttitudeEstimator2Test {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var activity: LocationActivity? = null

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun startAndStop_whenGyroscopeSensor_estimatesRelativeAttitude() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@AccurateRelativeGyroscopeAttitudeEstimator2Test.activity = activity
            }
        }
        assertNotNull(scenario)
        val activity = this.activity
        requireNotNull(activity)

        val estimator = AccurateRelativeGyroscopeAttitudeEstimator2(
            activity,
            GyroscopeSensorType.GYROSCOPE,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logAttitude(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    @Test
    fun startAndStop_whenGyroscopeUncalibratedSensor_estimatesRelativeAttitude() {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@AccurateRelativeGyroscopeAttitudeEstimator2Test.activity = activity
            }
        }
        assertNotNull(scenario)
        val activity = this.activity
        requireNotNull(activity)

        val estimator = AccurateRelativeGyroscopeAttitudeEstimator2(
            activity,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, roll, pitch, yaw, _ ->
                logAttitude(roll, pitch, yaw)
                syncHelper.notifyAll { completed++ }
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)
    }

    private fun logAttitude(roll: Double?, pitch: Double?, yaw: Double?) {
        Log.d(
            "AccurateRelativeGyroscopeAttitudeEstimator2Test",
            "onAttitudeAvailable - roll: $roll rad, pitch: $pitch rad, yaw: $yaw rad"
        )
    }
}