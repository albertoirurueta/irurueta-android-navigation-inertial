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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import kotlin.math.abs

class NonAdaptiveZuptProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check
        assertNull(processor.location)
        assertEquals(
            ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD,
            processor.gravityThreshold,
            0.0
        )
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD,
            processor.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD,
            processor.gyroscopeVarianceThreshold,
            0.0
        )
        assertEquals(
            ZuptProcessor.DEFAULT_WINDOW_NANOSECONDS,
            processor.windowNanoseconds
        )

        assertNull(processor.accelerationAverage)
        assertNull(processor.accelerationVariance)
        assertNull(processor.gyroscopeAverage)
        assertNull(processor.gyroscopeVariance)
        assertEquals(
            ZuptProcessor.GRAVITY_EARTH,
            processor.expectedGravityNorm,
            0.0
        )
    }

    @Test
    fun constructor_whenAllValues_setsExpectedValues() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val gravityThreshold = randomizer.nextDouble()
        val accelerometerVarianceThreshold = randomizer.nextDouble()
        val gyroscopeVarianceThreshold = randomizer.nextDouble()
        val windowNanoseconds = abs(randomizer.nextLong())

        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(
            location,
            gravityThreshold,
            accelerometerVarianceThreshold,
            gyroscopeVarianceThreshold,
            windowNanoseconds
        )

        // check
        assertSame(location, processor.location)
        assertEquals(
            gravityThreshold,
            processor.gravityThreshold,
            0.0
        )
        assertEquals(
            accelerometerVarianceThreshold,
            processor.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            gyroscopeVarianceThreshold,
            processor.gyroscopeVarianceThreshold,
            0.0
        )
        assertEquals(
            windowNanoseconds,
            processor.windowNanoseconds
        )

        assertNull(processor.accelerationAverage)
        assertNull(processor.accelerationVariance)
        assertNull(processor.gyroscopeAverage)
        assertNull(processor.gyroscopeVariance)
        val gravity = NEDGravityEstimator.estimateGravityAndReturnNew(
            location.toNEDPosition()
        )
        assertEquals(gravity.norm, processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun location_setsExpectedValue() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertNull(processor.location)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
    }

    @Test
    fun gravityThreshold_whenValid_setsExpectedValue() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD,
            processor.gravityThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gravityThreshold = randomizer.nextDouble()
        processor.gravityThreshold = gravityThreshold

        // check
        assertEquals(gravityThreshold, processor.gravityThreshold, 0.0)
    }

    @Test
    fun gravityThreshold_whenInvalid_throwsException() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gravityThreshold = -1.0
        }
    }

    @Test
    fun accelerometerVarianceThreshold_whenValid_setsExpectedValue() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD,
            processor.accelerometerVarianceThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerVarianceThreshold = randomizer.nextDouble()

        processor.accelerometerVarianceThreshold = accelerometerVarianceThreshold

        // check
        assertEquals(
            accelerometerVarianceThreshold,
            processor.accelerometerVarianceThreshold,
            0.0
        )
    }

    @Test
    fun accelerometerVarianceThreshold_whenInvalid_throwsException() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerVarianceThreshold = -1.0
        }
    }

    @Test
    fun gyroscopeVarianceThreshold_whenValid_setsExpectedValue() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD,
            processor.gyroscopeVarianceThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeVarianceThreshold = randomizer.nextDouble()

        processor.gyroscopeVarianceThreshold = gyroscopeVarianceThreshold

        // check
        assertEquals(
            gyroscopeVarianceThreshold,
            processor.gyroscopeVarianceThreshold,
            0.0
        )
    }

    @Test
    fun gyroscopeVarianceThreshold_whenInvalid_throwsException() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeVarianceThreshold = -1.0
        }
    }

    @Test
    fun windowNanoseconds_whenValid_setsExpectedValue() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            ZuptProcessor.DEFAULT_WINDOW_NANOSECONDS,
            processor.windowNanoseconds
        )

        // set new value
        val randomizer = UniformRandomizer()
        val windowNanoseconds = abs(randomizer.nextLong())
        processor.windowNanoseconds = windowNanoseconds

        // check
        assertEquals(windowNanoseconds, processor.windowNanoseconds)
    }

    @Test
    fun windowNanoseconds_whenInvalid_throwsException() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowNanoseconds = -1L
        }
    }

    @Test
    fun expectedGravityNorm_whenNoLocation_returnsDefaultEarthGravity() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertEquals(
            ZuptProcessor.GRAVITY_EARTH,
            processor.expectedGravityNorm,
            0.0
        )
    }

    @Test
    fun expectedGravityNorm_whenLocation_returnsExpectedGravity() {
        val location = getLocation()
        val processor =
            NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(location)

        val gravity = NEDGravityEstimator.estimateGravityAndReturnNew(
            location.toNEDPosition()
        )
        assertEquals(gravity.norm, processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun process_whenEmptySyncedMeasurement_returnsZero() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertEquals(0.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerSyncedMeasurementNotCloseToGravity_returnsZero() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertEquals(0.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoVariance_returnsOne() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertEquals(1.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoGyroscopeOrAccelerometerVariance_returnsOne() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement
            )
        assertEquals(1.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerVarianceNotCloseToZero_returnsZero() {
        val processor = NonAdaptiveZuptProcessor< AccelerometerAndMagnetometerSyncedSensorMeasurement>()

        // 1st measurement
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val syncedMeasurement1 =
            AccelerometerAndMagnetometerSyncedSensorMeasurement(
                accelerometerMeasurement1
            )
        assertEquals(1.0, processor.process(syncedMeasurement1), 0.0)

        // 2nd measurement
        val accelerometerMeasurement2 = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat() + 2.0f * ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD.toFloat()
        )
        val syncedMeasurement2 =
            AccelerometerAndMagnetometerSyncedSensorMeasurement(
                accelerometerMeasurement2
            )
        assertEquals(0.0, processor.process(syncedMeasurement2), 0.0)
    }

    @Test
    fun process_whenAccelerometerVarianceNotCloseToZeroWithGyroscopeMeasurement_returnsZero() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // 1st measurement
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx = Float.MAX_VALUE
        )
        val syncedMeasurement1 =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement1,
                gyroscopeMeasurement
            )
        assertEquals(1.0, processor.process(syncedMeasurement1), 0.0)

        // 2nd measurement
        val accelerometerMeasurement2 = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat() + 2.0f * ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD.toFloat()
        )
        val syncedMeasurement2 =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement2,
                gyroscopeMeasurement
            )
        assertEquals(0.0, processor.process(syncedMeasurement2), 0.0)
    }

    @Test
    fun process_whenNoAccelerometerVarianceAndGyroscopeNotCloseToZero_returnsZero() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // 1st measurement
        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement()
        val syncedMeasurement1 =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement1
            )
        assertEquals(1.0, processor.process(syncedMeasurement1), 0.0)

        // 2nd measurement
        val gyroscopeMeasurement2 = GyroscopeSensorMeasurement(
            wx = Float.MAX_VALUE
        )
        val syncedMeasurement2 =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement2
            )
        assertEquals(0.0, processor.process(syncedMeasurement2), 0.0)
    }

    @Test
    fun processNonSoft_whenAccelerometerSyncedMeasurementNotCloseToGravity_returnsFalse() {
        val processor = NonAdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertFalse(processor.processNonSoft(syncedMeasurement))
    }

    @Test
    fun processNonSoft_whenAccelerometerCloseToGravityAndNoVariance_returnsTrue() {
        val processor =
            NonAdaptiveZuptProcessor<AccelerometerAndMagnetometerSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val syncedMeasurement =
            AccelerometerAndMagnetometerSyncedSensorMeasurement(accelerometerMeasurement)
        assertTrue(processor.processNonSoft(syncedMeasurement))
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(
            MIN_LATITUDE_DEGREES,
            MAX_LATITUDE_DEGREES
        )
        val longitudeDegrees = randomizer.nextDouble(
            MIN_LONGITUDE_DEGREES,
            MAX_LONGITUDE_DEGREES
        )
        val height = randomizer.nextDouble(
            MIN_HEIGHT,
            MAX_HEIGHT
        )

        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        return location
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0
    }
}