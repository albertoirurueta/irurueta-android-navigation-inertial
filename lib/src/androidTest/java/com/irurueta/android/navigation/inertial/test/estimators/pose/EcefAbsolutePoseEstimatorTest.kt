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
package com.irurueta.android.navigation.inertial.test.estimators.pose

import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.pose.EcefAbsolutePoseEstimator
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.geometry.Point3D
import com.irurueta.navigation.frames.ECEFFrame
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test

@RequiresDevice
class EcefAbsolutePoseEstimatorTest {

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

    private val origin: Point3D = Point3D.create()

    private val translation: Point3D = Point3D.create()

    private var previousTimestamp = -1L

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun startAndStop_estimatesCurrentPose() {
        val location = getCurrentLocation()
        val activity = this.activity
        requireNotNull(activity)

        val refreshRate = activity.display?.mode?.refreshRate ?: 60.0f
        val refreshIntervalNanos = (1.0f / refreshRate * 1e9).toLong()

        val estimator = EcefAbsolutePoseEstimator(
            activity,
            location,
            sensorDelay = SensorDelay.FASTEST,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
            estimatePoseTransformation = true,
            poseAvailableListener = { _, currentEcefFrame, _, initialEcefFrame, timestamp, _ ->
                if (previousTimestamp < 0) {
                    previousTimestamp = timestamp
                    return@EcefAbsolutePoseEstimator
                }

                // refresh only as much as display allows even though sensors might run at higher refresh rates
                if (timestamp - previousTimestamp >= refreshIntervalNanos) {
                    logTranslation(currentEcefFrame, initialEcefFrame)
                    previousTimestamp = timestamp

                    syncHelper.notifyAll { completed++ }
                }
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
                this@EcefAbsolutePoseEstimatorTest.activity = activity
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

    private fun logTranslation(currentEcefFrame: ECEFFrame, initialEcefFrame: ECEFFrame) {
        currentEcefFrame.getPosition(translation)
        initialEcefFrame.getPosition(origin)
        val distance = translation.distanceTo(origin)
        Log.d("EcefAbsolutePoseEstimatorTest", "Translation: $distance meters")
    }
}