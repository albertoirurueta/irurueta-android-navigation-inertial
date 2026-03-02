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

class AdaptiveZuptProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check
        assertNull(processor.location)
        assertEquals(
            ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD,
            processor.gravityThreshold,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE,
            processor.accelerometerNoiseVariance,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE,
            processor.gyroscopeNoiseVariance,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            processor.accelerometerFactor,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            processor.gyroscopeFactor,
            0.0
        )
        assertEquals(
            ZuptProcessor.DEFAULT_WINDOW_NANOSECONDS,
            processor.windowNanoseconds
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE * AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            processor.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE * AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            processor.gyroscopeVarianceThreshold,
            0.0
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
        val accelerometerNoiseVariance = randomizer.nextDouble()
        val gyroscopeNoiseVariance = randomizer.nextDouble()
        val accelerometerFactor = randomizer.nextDouble()
        val gyroscopeFactor = randomizer.nextDouble()
        val windowNanoseconds = abs(randomizer.nextLong())

        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(
            location,
            gravityThreshold,
            accelerometerNoiseVariance,
            gyroscopeNoiseVariance,
            accelerometerFactor,
            gyroscopeFactor,
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
            accelerometerNoiseVariance,
            processor.accelerometerNoiseVariance,
            0.0
        )
        assertEquals(
            gyroscopeNoiseVariance,
            processor.gyroscopeNoiseVariance,
            0.0
        )
        assertEquals(
            accelerometerFactor,
            processor.accelerometerFactor,
            0.0
        )
        assertEquals(
            gyroscopeFactor,
            processor.gyroscopeFactor,
            0.0
        )
        assertEquals(
            windowNanoseconds,
            processor.windowNanoseconds
        )
        assertEquals(
            accelerometerNoiseVariance * accelerometerFactor,
            processor.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            gyroscopeNoiseVariance * gyroscopeFactor,
            processor.gyroscopeVarianceThreshold,
            0.0
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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertNull(processor.location)

        // set new values
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
    }

    @Test
    fun gravityThreshold_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gravityThreshold = -1.0
        }
    }

    @Test
    fun accelerometerNoiseVariance_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE,
            processor.accelerometerNoiseVariance,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerNoiseVariance = randomizer.nextDouble()

        processor.accelerometerNoiseVariance = accelerometerNoiseVariance

        // check
        assertEquals(
            accelerometerNoiseVariance,
            processor.accelerometerNoiseVariance,
            0.0
        )
    }

    @Test
    fun accelerometerNoiseVariance_whenInvalid_throwsException() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerNoiseVariance = -1.0
        }
    }

    @Test
    fun gyroscopeNoiseVariance_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE,
            processor.gyroscopeNoiseVariance,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeNoiseVariance = randomizer.nextDouble()

        processor.gyroscopeNoiseVariance = gyroscopeNoiseVariance

        // check
        assertEquals(
            gyroscopeNoiseVariance,
            processor.gyroscopeNoiseVariance,
            0.0
        )
    }

    @Test
    fun gyroscopeNoiseVariance_whenInvalid_throwsException() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeNoiseVariance = -1.0
        }
    }

    @Test
    fun accelerometerFactor_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            processor.accelerometerFactor,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerFactor = randomizer.nextDouble()

        processor.accelerometerFactor = accelerometerFactor

        // check
        assertEquals(
            accelerometerFactor,
            processor.accelerometerFactor,
            0.0
        )
    }

    @Test
    fun accelerometerFactor_whenInvalid_throwsException() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerFactor = -1.0
        }
    }

    @Test
    fun gyroscopeFactor_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            processor.gyroscopeFactor,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeFactor = randomizer.nextDouble()

        processor.gyroscopeFactor = gyroscopeFactor

        // check
        assertEquals(
            gyroscopeFactor,
            processor.gyroscopeFactor,
            0.0
        )
    }

    @Test
    fun gyroscopeFactor_whenInvalid_throwsException() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeFactor = -1.0
        }
    }

    @Test
    fun accelerometerVarianceThreshold_returnsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE * AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            processor.accelerometerVarianceThreshold,
            0.0
        )

        // set accelerometerNoiseVariance
        val randomizer = UniformRandomizer()
        val accelerometerNoiseVariance = randomizer.nextDouble()

        processor.accelerometerNoiseVariance = accelerometerNoiseVariance

        // check
        assertEquals(
            accelerometerNoiseVariance * AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            processor.accelerometerVarianceThreshold,
            0.0
        )

        // set accelerometerFactor
        val accelerometerFactor = randomizer.nextDouble()

        processor.accelerometerFactor = accelerometerFactor

        // check
        assertEquals(
            accelerometerNoiseVariance * accelerometerFactor,
            processor.accelerometerVarianceThreshold,
            0.0
        )
    }

    @Test
    fun gyroscopeVarianceThreshold_returnsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE * AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            processor.gyroscopeVarianceThreshold,
            0.0
        )

        // set gyroscopeNoiseVariance
        val randomizer = UniformRandomizer()
        val gyroscopeNoiseVariance = randomizer.nextDouble()

        processor.gyroscopeNoiseVariance = gyroscopeNoiseVariance

        // check
        assertEquals(
            gyroscopeNoiseVariance * AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            processor.gyroscopeVarianceThreshold,
            0.0
        )

        // set gyroscopeFactor
        val gyroscopeFactor = randomizer.nextDouble()

        processor.gyroscopeFactor = gyroscopeFactor

        // check
        assertEquals(
            gyroscopeNoiseVariance * gyroscopeFactor,
            processor.gyroscopeVarianceThreshold,
            0.0
        )
    }

    @Test
    fun windowNanoseconds_whenValid_setsExpectedValue() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowNanoseconds = -1L
        }
    }

    @Test
    fun expectedGravityNorm_whenNoLocation_returnsDefaultEarthGravity() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertEquals(
            ZuptProcessor.GRAVITY_EARTH,
            processor.expectedGravityNorm,
            0.0
        )
    }

    @Test
    fun expectedGravityNorm_whenLocation_returnsExpectedGravity() {
        val location = getLocation()
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(location)

        val gravity = NEDGravityEstimator.estimateGravityAndReturnNew(
            location.toNEDPosition()
        )
        assertEquals(gravity.norm, processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun process_whenEmptySyncedMeasurement_returnsZero() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertEquals(0.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerSyncedMeasurementNotCloseToGravity_returnsZero() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertEquals(0.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoVariance_returnsOne() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertEquals(1.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoGyroscopeOrAccelerometerVariance_returnsOne() {
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndMagnetometerSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = AdaptiveZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertFalse(processor.processNonSoft(syncedMeasurement))
    }

    @Test
    fun processNonSoft_whenAccelerometerCloseToGravityAndNoVariance_returnsTrue() {
        val processor =
            AdaptiveZuptProcessor<AccelerometerAndMagnetometerSyncedSensorMeasurement>()

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