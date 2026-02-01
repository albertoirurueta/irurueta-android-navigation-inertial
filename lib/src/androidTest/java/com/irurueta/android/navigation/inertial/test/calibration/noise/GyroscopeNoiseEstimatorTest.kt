/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.irurueta.android.navigation.inertial.LocationService
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.calibration.noise.GyroscopeNoiseEstimator
import com.irurueta.android.navigation.inertial.test.LocationActivity
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.android.testutils.RequiresRealDevice
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.spyk
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Rule
import org.junit.Test

class GyroscopeNoiseEstimatorTest {

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

    @RequiresRealDevice
    @Test
    fun startAndStop_estimatesAccelerometerNoise() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GyroscopeNoiseEstimator(
            context,
            completedListener = { estimator ->
                assertFalse(estimator.running)

                syncHelper.notifyAll { completed++ }
            },
            unreliableListener = { estimator ->
                Log.d("GyroscopeNoiseEstimatorTest", "Sensor is unreliable")
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
        val averageNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        estimator.getAverageNormAsMeasurement(averageNorm2)
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageNorm1.unit)

        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        assertTrue(standardDeviationNorm > 0.0)
        val standardDeviationNorm1 = estimator.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNorm1)
        val standardDeviationNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm2)
        assertEquals(standardDeviationNorm1, standardDeviationNorm2)
        assertEquals(standardDeviationNorm, standardDeviationNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationNorm1.unit)

        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)
        assertTrue(averageStandardDeviation > 0.0)
        val averageStandardDeviation1 = estimator.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviation1)
        val averageStandardDeviation2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation2)
        assertEquals(averageStandardDeviation1, averageStandardDeviation2)
        assertEquals(
            averageStandardDeviation,
            averageStandardDeviation1.value.toDouble(),
            0.0
        )
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            averageStandardDeviation1.unit
        )

        val averageNoisePsd = estimator.averageNoisePsd
        requireNotNull(averageNoisePsd)
        assertTrue(averageNoisePsd > 0.0)

        val noiseRootPsdNorm = estimator.noiseRootPsdNorm
        requireNotNull(noiseRootPsdNorm)
        assertTrue(noiseRootPsdNorm > 0.0)

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
    }

    @Test
    fun estimatedResult_whenDeviceStatic_returnsValueCloseToExpectedGravity() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val estimator = GyroscopeNoiseEstimator(context,
            completedListener = { estimator ->
                assertFalse(estimator.running)

                syncHelper.notifyAll { completed++ }
            },
            unreliableListener = { estimator ->
                Log.d("GyroscopeNoiseEstimatorTest", "Sensor is unreliable")
                assertFalse(estimator.running)
            }
        )

        estimator.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 2L * estimator.maxDurationMillis)

        // obtain results
        val measuredNorm = estimator.averageNorm
        requireNotNull(measuredNorm)
        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)

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
                            this@GyroscopeNoiseEstimatorTest.location = location

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

        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val bodyKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            c,
            c,
            ecefVelocity,
            ecefVelocity,
            ecefPosition
        )

        Log.d(
            "GyroscopeNoiseEstimatorTest",
            "measuredAngularRate: $measuredNorm rad/s - "
                    + "angularRate: ${bodyKinematics.angularRateNorm} rad/s, "
                    + "standardDeviationNorm: $standardDeviationNorm rad/s, "
                    + "averageStandardDeviation: $averageStandardDeviation rad/s"
        )
    }

    private companion object {
        const val TIME_INTERVAL_MILLIS = 20L

        const val TIME_INTERVAL_SECONDS = TIME_INTERVAL_MILLIS.toDouble() / 1000
    }
}