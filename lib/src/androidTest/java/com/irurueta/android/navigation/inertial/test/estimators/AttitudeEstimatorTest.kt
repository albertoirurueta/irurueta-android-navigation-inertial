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
package com.irurueta.android.navigation.inertial.test.estimators

import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.geometry.Quaternion
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Before
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test

@Ignore
class AttitudeEstimatorTest {

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

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGeomagnetic_estimatesAttitude() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = AttitudeEstimator(
            context,
            location = location,
            estimateImuLeveling = false
        ) { attitude, _, type ->
            logAttitudeAvailable(attitude, type)

            syncHelper.notifyAll { completed++ }
        }

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)

        completed = 0

        // obtain attitude with attitude collector for comparison
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, _, _, _ ->
                logMeasurement(rotation)

                syncHelper.notifyAll { completed++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        collector.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenImprovedLeveling_estimatesAttitude() {
        val location = getCurrentLocation()

        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = AttitudeEstimator(
            context,
            location = location,
            estimateImuLeveling = true
        ) { attitude, _, type ->
            logAttitudeAvailable(attitude, type)

            syncHelper.notifyAll { completed++ }
        }

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)

        completed = 0

        // obtain attitude with attitude collector for comparison
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, _, _, _ ->
                logMeasurement(rotation)

                syncHelper.notifyAll { completed++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        collector.stop()

        assertTrue(completed > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenLeveling_estimatesAttitude() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = AttitudeEstimator(
            context,
            estimateImuLeveling = true
        ) { attitude, _, type ->
            logAttitudeAvailable(attitude, type)

            syncHelper.notifyAll { completed++ }
        }

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 })

        estimator.stop()

        assertTrue(completed > 0)

        completed = 0

        // obtain attitude with attitude collector for comparison
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, _, _, _ ->
                logMeasurement(rotation)

                syncHelper.notifyAll { completed++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        collector.stop()

        assertTrue(completed > 0)
    }

    private fun logAttitudeAvailable(
        attitude: Quaternion,
        type: AttitudeEstimator.AttitudeEstimatorType
    ) {
        val angles = attitude.toEulerAngles()
        val roll = Math.toDegrees(angles[0])
        val pitch = Math.toDegrees(angles[1])
        val yaw = Math.toDegrees(angles[2])

        Log.d(
            "AttitudeEstimatorTest",
            "onAttitudeAvailable - roll: $roll º, pitch: $pitch º, yaw: $yaw º, type: $type"
        )
    }

    private fun logMeasurement(attitude: Quaternion) {
        val angles = attitude.toEulerAngles()
        val roll = Math.toDegrees(angles[0])
        val pitch = Math.toDegrees(angles[1])
        val yaw = Math.toDegrees(angles[2])

        Log.d(
            "AttitudeEstimatorTest",
            "onMeasurement - roll: $roll º, pitch: $pitch º, yaw: $yaw º"
        )
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
}