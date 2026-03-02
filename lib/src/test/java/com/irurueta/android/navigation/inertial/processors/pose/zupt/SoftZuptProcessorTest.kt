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
import kotlin.math.max
import kotlin.math.min

class SoftZuptProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check
        assertNull(processor.location)
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_WEIGHT,
            processor.gravityWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_WEIGHT,
            processor.accelerometerWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_WEIGHT,
            processor.gyroscopeWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_NORMALIZATION_FACTOR,
            processor.gravityNormalizationFactor,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR,
            processor.accelerometerNormalizationFactor,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR,
            processor.gyroscopeNormalizationFactor,
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
        val gravityWeight = randomizer.nextDouble()
        val accelerometerWeight = randomizer.nextDouble()
        val gyroscopeWeight = randomizer.nextDouble()
        val gravityNormalizationFactor = randomizer.nextDouble()
        val accelerometerNormalizationFactor = randomizer.nextDouble()
        val gyroscopeNormalizationFactor = randomizer.nextDouble()
        val windowNanoseconds = abs(randomizer.nextLong())

        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(
            location,
            gravityWeight,
            accelerometerWeight,
            gyroscopeWeight,
            gravityNormalizationFactor,
            accelerometerNormalizationFactor,
            gyroscopeNormalizationFactor,
            windowNanoseconds
        )

        // check
        assertSame(location, processor.location)
        assertEquals(
            gravityWeight,
            processor.gravityWeight,
            0.0
        )
        assertEquals(
            accelerometerWeight,
            processor.accelerometerWeight,
            0.0
        )
        assertEquals(
            gyroscopeWeight,
            processor.gyroscopeWeight,
            0.0
        )
        assertEquals(
            gravityNormalizationFactor,
            processor.gravityNormalizationFactor,
            0.0
        )
        assertEquals(
            accelerometerNormalizationFactor,
            processor.accelerometerNormalizationFactor,
            0.0
        )
        assertEquals(
            gyroscopeNormalizationFactor,
            processor.gyroscopeNormalizationFactor,
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
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertNull(processor.location)

        // set new values
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
    }

    @Test
    fun gravityWeight_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_WEIGHT,
            processor.gravityWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gravityWeight = randomizer.nextDouble()

        processor.gravityWeight = gravityWeight

        // check
        assertEquals(gravityWeight, processor.gravityWeight, 0.0)
    }

    @Test
    fun gravityWeight_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gravityWeight = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.gravityWeight = 2.0
        }
    }

    @Test
    fun accelerometerWeight_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_WEIGHT,
            processor.accelerometerWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerWeight = randomizer.nextDouble()

        processor.accelerometerWeight = accelerometerWeight

        // check
        assertEquals(accelerometerWeight, processor.accelerometerWeight, 0.0)
    }

    @Test
    fun accelerometerWeight_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerWeight = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerWeight = 2.0
        }
    }

    @Test
    fun gyroscopeWeight_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_WEIGHT,
            processor.gyroscopeWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeWeight = randomizer.nextDouble()

        processor.gyroscopeWeight = gyroscopeWeight

        // check
        assertEquals(gyroscopeWeight, processor.gyroscopeWeight, 0.0)
    }

    @Test
    fun gyroscopeWeight_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeWeight = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeWeight = 2.0
        }
    }

    @Test
    fun gravityNormalizationFactor_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_NORMALIZATION_FACTOR,
            processor.gravityNormalizationFactor,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gravityNormalizationFactor = randomizer.nextDouble()

        processor.gravityNormalizationFactor = gravityNormalizationFactor

        // check
        assertEquals(
            gravityNormalizationFactor,
            processor.gravityNormalizationFactor,
            0.0
        )
    }

    @Test
    fun gravityNormalizationFactor_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gravityNormalizationFactor = -1.0
        }
    }

    @Test
    fun accelerometerNormalizationFactor_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR,
            processor.accelerometerNormalizationFactor,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerNormalizationFactor = randomizer.nextDouble()

        processor.accelerometerNormalizationFactor = accelerometerNormalizationFactor

        // check
        assertEquals(
            accelerometerNormalizationFactor,
            processor.accelerometerNormalizationFactor,
            0.0
        )
    }

    @Test
    fun accelerometerNormalizationFactor_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.accelerometerNormalizationFactor = -1.0
        }
    }

    @Test
    fun gyroscopeNormalizationFactor_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check default value
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR,
            processor.gyroscopeNormalizationFactor,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeNormalizationFactor = randomizer.nextDouble()

        processor.gyroscopeNormalizationFactor = gyroscopeNormalizationFactor

        // check
        assertEquals(
            gyroscopeNormalizationFactor,
            processor.gyroscopeNormalizationFactor,
            0.0
        )
    }

    @Test
    fun gyroscopeNormalizationFactor_whenInvalid_throwsException() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.gyroscopeNormalizationFactor = -1.0
        }
    }

    @Test
    fun windowNanoseconds_whenValid_setsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowNanoseconds = -1L
        }
    }

    @Test
    fun expectedGravityNorm_whenNoLocation_returnsDefaultEarthGravity() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
            SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>(location)

        val gravity = NEDGravityEstimator.estimateGravityAndReturnNew(
            location.toNEDPosition()
        )
        assertEquals(gravity.norm, processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun process_whenEmptySyncedMeasurement_returnsZero() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertEquals(0.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerSyncedMeasurementNotCloseToGravity_returnsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        val result = processor.process(syncedMeasurement)
        val expected = processor.accelerometerWeight /
                (processor.gravityWeight + processor.accelerometerWeight)
        assertEquals(expected, result, 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoVariance_returnsOne() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = ZuptProcessor.GRAVITY_EARTH.toFloat()
        )
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertEquals(1.0, processor.process(syncedMeasurement), 0.0)
    }

    @Test
    fun process_whenAccelerometerCloseToGravityAndNoGyroscopeOrAccelerometerVariance_returnsOne() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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
    fun process_whenAccelerometerVarianceNotCloseToZero_returnsExpectedValue() {
        val processor = SoftZuptProcessor<AccelerometerAndMagnetometerSyncedSensorMeasurement>()

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

        val result = processor.process(syncedMeasurement2)
        val gravityDiff =
            abs((processor.accelerationAverage ?: 0.0) - processor.expectedGravityNorm)
        val accelerationVariance = processor.accelerationVariance ?: 0.0
        val gravityScore =
            min(1.0, max(0.0, 1.0 - (gravityDiff / processor.gravityNormalizationFactor)))
        val accelerometerScore = min(
            1.0,
            max(0.0, 1.0 - (accelerationVariance / processor.accelerometerNormalizationFactor))
        )
        val expected = (processor.gravityWeight * gravityScore +
                processor.accelerometerWeight * accelerometerScore) /
                (processor.gravityWeight + processor.accelerometerWeight)

        assertEquals(expected, result, 0.0)
    }

    @Test
    fun process_whenAccelerometerVarianceNotCloseToZeroWithGyroscopeMeasurement_returnsZero() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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

        val result = processor.process(syncedMeasurement2)
        val gravityDiff =
            abs((processor.accelerationAverage ?: 0.0) - processor.expectedGravityNorm)
        val accelerationVariance = processor.accelerationVariance ?: 0.0
        val gyroscopeVariance = processor.gyroscopeVariance ?: 0.0
        val gravityScore =
            min(1.0, max(0.0, 1.0 - (gravityDiff / processor.gravityNormalizationFactor)))
        val accelerometerScore = min(
            1.0,
            max(0.0, 1.0 - (accelerationVariance / processor.accelerometerNormalizationFactor))
        )
        val gyroscopeScore = min(
            1.0,
            max(0.0, 1.0 - (gyroscopeVariance / processor.gyroscopeNormalizationFactor))
        )
        val expected = (processor.gravityWeight * gravityScore +
                processor.accelerometerWeight * accelerometerScore +
                processor.gyroscopeWeight * gyroscopeScore) /
                (processor.gravityWeight + processor.accelerometerWeight + processor.gyroscopeWeight)

        assertEquals(expected, result, 0.0)
    }

    @Test
    fun process_whenNoAccelerometerVarianceAndGyroscopeNotCloseToZero_returnsZero() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

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

        val result = processor.process(syncedMeasurement2)
        val gravityDiff =
            abs((processor.accelerationAverage ?: 0.0) - processor.expectedGravityNorm)
        val accelerationVariance = processor.accelerationVariance ?: 0.0
        val gyroscopeVariance = processor.gyroscopeVariance ?: 0.0
        val gravityScore =
            min(1.0, max(0.0, 1.0 - (gravityDiff / processor.gravityNormalizationFactor)))
        val accelerometerScore = min(
            1.0,
            max(0.0, 1.0 - (accelerationVariance / processor.accelerometerNormalizationFactor))
        )
        val gyroscopeScore = min(
            1.0,
            max(0.0, 1.0 - (gyroscopeVariance / processor.gyroscopeNormalizationFactor))
        )
        val expected = (processor.gravityWeight * gravityScore +
                processor.accelerometerWeight * accelerometerScore +
                processor.gyroscopeWeight * gyroscopeScore) /
                (processor.gravityWeight + processor.accelerometerWeight + processor.gyroscopeWeight)

        assertEquals(expected, result, 0.0)
    }

    @Test
    fun processNonSoft_whenAccelerometerSyncedMeasurementNotCloseToGravity_returnsTrue() {
        val processor = SoftZuptProcessor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(accelerometerMeasurement)
        assertTrue(processor.processNonSoft(syncedMeasurement))
    }

    @Test
    fun processNonSoft_whenAccelerometerCloseToGravityAndNoVariance_returnsTrue() {
        val processor =
            SoftZuptProcessor<AccelerometerAndMagnetometerSyncedSensorMeasurement>()

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