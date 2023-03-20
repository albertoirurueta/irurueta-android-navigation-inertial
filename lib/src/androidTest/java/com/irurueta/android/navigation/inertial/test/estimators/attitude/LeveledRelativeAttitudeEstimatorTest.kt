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
import com.irurueta.android.navigation.inertial.estimators.attitude.LeveledRelativeAttitudeEstimator
import com.irurueta.android.navigation.inertial.test.LocationActivity
import io.mockk.spyk
import org.junit.Assert
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

@RequiresDevice
class LeveledRelativeAttitudeEstimatorTest {

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

    @Test
    fun startAndStop_whenNonAccurateLevelingAndNonAccurateRelativeAttitude_estimatesRelativeAttitude() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LeveledRelativeAttitudeEstimator(
            activity,
            location,
            estimateEulerAngles = true,
            useAccurateLevelingEstimator = false,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
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
    fun startAndStop_whenAccurateLevelingAndNonAccurateRelativeAttitude_estimatesRelativeAttitude() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LeveledRelativeAttitudeEstimator(
            activity,
            location,
            estimateEulerAngles = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
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
    fun startAndStop_whenNonAccurateLevelingAndAccurateRelativeAttitude_estimatesRelativeAttitude() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LeveledRelativeAttitudeEstimator(
            activity,
            location,
            estimateEulerAngles = true,
            useAccurateLevelingEstimator = false,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
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
    fun startAndStop_whenAccurateLevelingAndAccurateRelativeAttitude_estimatesRelativeAttitude() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val estimator = LeveledRelativeAttitudeEstimator(
            activity,
            location,
            estimateEulerAngles = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
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

    private fun getCurrentLocation(): Location {
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                this@LeveledRelativeAttitudeEstimatorTest.activity = activity
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
        Assert.assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        Assert.assertEquals(1, completed)
        completed = 0

        val currentLocation = this.currentLocation
        requireNotNull(currentLocation)
        return currentLocation
    }

    private fun logAttitude(roll: Double?, pitch: Double?, yaw: Double?) {
        Log.d(
            "LeveledRelativeAttitudeEstimatorTest",
            "onAttitudeAvailable - roll: $roll rad, pitch: $pitch rad, yaw: $yaw rad"
        )
    }
}