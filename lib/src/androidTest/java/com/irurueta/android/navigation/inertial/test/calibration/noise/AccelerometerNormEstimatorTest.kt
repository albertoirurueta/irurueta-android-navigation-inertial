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
package com.irurueta.android.navigation.inertial.test.calibration.noise

import android.location.Location
import android.util.Log
import androidx.test.core.app.ActivityScenario
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.noise.AccelerometerNormEstimator
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class AccelerometerNormEstimatorTest {

    @get:Rule
    val permissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.ACCESS_COARSE_LOCATION,
        android.Manifest.permission.ACCESS_FINE_LOCATION,
        android.Manifest.permission.ACCESS_BACKGROUND_LOCATION
    )

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var location: Location? = null

    @Before
    fun setUp() {
        completed = 0
    }

    @RequiresDevice
    @Test
    fun startAndStop_estimatesAccelerometerNoiseAndGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = AccelerometerNormEstimator(
            context,
            completedListener = { estimator ->
                assertFalse(estimator.running)

                syncHelper.notifyAll { completed++ }
            },
            unreliableListener = { estimator ->
                Log.d("AccelerometerNormEstimatorTest", "Sensor is unreliable")
                assertFalse(estimator.running)
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 2L * estimator.maxDurationMillis)

        // obtain results
        assertFalse(estimator.running)
        assertTrue(estimator.numberOfProcessedMeasurements > 0)
        assertTrue(estimator.resultAvailable)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getAverageNormAsMeasurement(averageNorm2)
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageNorm1.unit)

        val normVariance = estimator.normVariance
        requireNotNull(normVariance)
        assertTrue(normVariance > 0.0)

        val normStandardDeviation = estimator.normStandardDeviation
        requireNotNull(normStandardDeviation)
        assertTrue(normStandardDeviation > 0.0)
        val normStandardDeviation1 = estimator.normStandardDeviationAsMeasurement
        requireNotNull(normStandardDeviation1)
        val normStandardDeviation2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getNormStandardDeviationAsMeasurement(normStandardDeviation2)
        assertEquals(normStandardDeviation1, normStandardDeviation2)
        assertEquals(
            normStandardDeviation,
            normStandardDeviation1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, normStandardDeviation1.unit)

        val psd = estimator.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

        val rootPsd = estimator.rootPsd
        requireNotNull(rootPsd)
        assertTrue(rootPsd > 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.SECOND)
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval2)
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.unit)

        val timeIntervalVariance = estimator.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertTrue(timeIntervalVariance > 0.0)

        val timeIntervalStandardDeviation = estimator.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation)
        assertTrue(timeIntervalStandardDeviation > 0.0)
        val timeIntervalStandardDeviation1 = estimator.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviation1)
        val timeIntervalStandardDeviation2 = Time(0.0, TimeUnit.SECOND)
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2)
        assertEquals(timeIntervalStandardDeviation1, timeIntervalStandardDeviation2)
        assertEquals(
            timeIntervalStandardDeviation,
            timeIntervalStandardDeviation1.value.toDouble(),
            0.0
        )
        assertEquals(TimeUnit.SECOND, timeIntervalStandardDeviation1.unit)

        val elapsedTime = estimator.elapsedTimeNanos
        assertTrue(elapsedTime > 0L)
        val elapsedTime1 = estimator.elapsedTime
        val elapsedTime2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(elapsedTime2)
        assertEquals(elapsedTime1, elapsedTime2)
        assertEquals(elapsedTime.toDouble(), elapsedTime1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, elapsedTime1.unit)
    }

    @RequiresDevice
    @Test
    fun estimatedResult_whenDeviceStatic_returnsValueCloseToExpectedGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = AccelerometerNormEstimator(
            context,
            completedListener = { estimator ->
                assertFalse(estimator.running)

                syncHelper.notifyAll { completed++ }
            },
            unreliableListener = { estimator ->
                Log.d("AccelerometerNormEstimatorTest", "Sensor is unreliable")
                assertFalse(estimator.running)
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 2L * estimator.maxDurationMillis)

        // obtain results
        val measuredNorm = estimator.averageNorm
        requireNotNull(measuredNorm)

        completed = 0

        // obtain current location
        val scenario = ActivityScenario.launch(LocationActivity::class.java).use {
            it.onActivity { activity ->
                val service = LocationService(activity)

                val enabled = service.locationEnabled
                requireNotNull(enabled)
                assertTrue(enabled)

                val currentLocationListener =
                    spyk(object : LocationService.OnCurrentLocationListener {
                        override fun onCurrentLocation(location: Location) {
                            assertNotNull(location)
                            this@AccelerometerNormEstimatorTest.location = location

                            syncHelper.notifyAll { completed++ }
                        }
                    })

                service.getCurrentLocation(currentLocationListener)
            }
        }
        assertNotNull(scenario)

        syncHelper.waitOnCondition({ completed < 1 })
        assertEquals(1, completed)

        val location = this.location
        requireNotNull(location)
        val nedPosition = location.toNEDPosition()
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition, nedVelocity,
            ecefPosition, ecefVelocity
        )

        val ecefGravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
            ecefPosition.x,
            ecefPosition.y,
            ecefPosition.z
        )
        val gravity = ecefGravity.norm

        Log.d(
            "GravityNormEstimatorTest",
            "measuredGravity: $measuredNorm m/s^2 - gravity: $gravity m/s^2"
        )
    }
}