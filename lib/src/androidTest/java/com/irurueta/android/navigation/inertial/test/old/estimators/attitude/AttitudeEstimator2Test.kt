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
package com.irurueta.android.navigation.inertial.test.old.estimators.attitude

import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.old.estimators.attitude.AttitudeEstimator2
import com.irurueta.android.navigation.inertial.test.LocationActivity
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AttitudeEstimator2Test {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var activity: LocationActivity? = null

    @Before
    fun setUp() {
        completed = 0

        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@AttitudeEstimator2Test.activity = activity

                syncHelper.notifyAll { completed++ }
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)
        completed = 0
    }

    @Test
    fun startAndStop_whenAbsoluteAttitude_estimatesAttitude() {
        val activity = this.activity
        requireNotNull(activity)
        val estimator = AttitudeEstimator2(
            activity,
            attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, _, roll, pitch, yaw, _ ->
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
    fun startAndStop_whenRelativeAttitude_estimatesAttitude() {
        val activity = this.activity
        requireNotNull(activity)
        val estimator = AttitudeEstimator2(
            activity,
            attitudeSensorType = AttitudeSensorType.RELATIVE_ATTITUDE,
            estimateEulerAngles = true,
            attitudeAvailableListener = { _, _, _, _, roll, pitch, yaw, _ ->
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
        val rollDegrees = if (roll != null) Math.toDegrees(roll) else null
        val pitchDegrees = if (pitch != null) Math.toDegrees(pitch) else null
        val yawDegrees = if (yaw != null) Math.toDegrees(yaw) else null

        Log.d(
            "AttitudeEstimator2Test",
            "onLevelingAvailable - roll: $rollDegrees º, pitch: $pitchDegrees º, yaw: $yawDegrees º"
        )
    }
}