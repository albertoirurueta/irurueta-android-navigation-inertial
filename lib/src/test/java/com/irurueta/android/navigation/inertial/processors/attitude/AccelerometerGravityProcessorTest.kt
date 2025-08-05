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
package com.irurueta.android.navigation.inertial.processors.attitude

import android.hardware.SensorManager
import android.location.Location
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test

@Ignore("possible memory leak")
class AccelerometerGravityProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var listener:
            BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>

    @MockK
    private lateinit var location: Location

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AccelerometerGravityProcessor()

        assertNotNull(processor.averagingFilter)
        assertTrue(processor.averagingFilter is LowPassAveragingFilter)
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertNull(processor.processorListener)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
        assertEquals(0L, processor.timestamp)
        assertNull(processor.accuracy)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val averagingFilter = MeanAveragingFilter()
        val location = getLocation()
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            location,
            adjustGravityNorm = false,
            listener
        )

        assertSame(averagingFilter, processor.averagingFilter)
        assertSame(location, processor.location)
        assertFalse(processor.adjustGravityNorm)
        assertSame(listener, processor.processorListener)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
        assertEquals(0L, processor.timestamp)
        assertNull(processor.accuracy)
    }

    @Test
    fun location_setsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        // default value
        assertNull(processor.location)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        // check default value
        assertTrue(processor.adjustGravityNorm)

        // set new value
        processor.adjustGravityNorm = false

        // check
        @Suppress("KotlinConstantConditions")
        assertFalse(processor.adjustGravityNorm)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun getNedPosition_whenNoLocation_returnsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        assertFalse(processor.getNedPosition(NEDPosition()))
    }

    @Test
    fun getNedPosition_whenLocation_returnsExpectedValue() {
        val location = getLocation()
        val processor = AccelerometerGravityProcessor(location = location)

        val nedPosition = NEDPosition()
        assertTrue(processor.getNedPosition(nedPosition))

        assertEquals(nedPosition, location.toNEDPosition())
    }

    @Test
    fun process_whenNotEnoughMeasurements_doesNotSetsExpectedValuesOrNotifies() {
        val averagingFilter = spyk(LowPassAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = false,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        assertFalse(processor.process(measurement))

        val slot = slot<DoubleArray>()
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(slot),
                timestamp
            )
        }

        verify { listener wasNot Called }

        // check
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
        assertEquals(0L, processor.timestamp)
        assertNull(processor.accuracy)
    }

    @Test
    fun process_whenEnoughMeasurementsAndBiasAvailable_setsExpectedValuesAndNotifies() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = false,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp1, SensorAccuracy.HIGH)

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp2, SensorAccuracy.HIGH)

        assertTrue(processor.process(measurement2, timestamp2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1],
                array[0],
                -array[2],
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1], processor.gx, 0.0)
        assertEquals(array[0], processor.gy, 0.0)
        assertEquals(-array[2], processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun process_whenEnoughMeasurementsAndBiasNotAvailable_setsExpectedValuesAndNotifies() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = false,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp1,
            SensorAccuracy.HIGH
        )

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp2,
            SensorAccuracy.HIGH
        )

        assertTrue(processor.process(measurement2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1],
                array[0],
                -array[2],
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1], processor.gx, 0.0)
        assertEquals(array[0], processor.gy, 0.0)
        assertEquals(-array[2], processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun process_whenGravityNormAdjustedAndNoLocation_setsExpectedValuesAndNotifies() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = true,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp1, SensorAccuracy.HIGH)

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1, timestamp1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp2, SensorAccuracy.HIGH)

        assertTrue(processor.process(measurement2, timestamp2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        val norm = Utils.normF(array)
        val factor = SensorManager.GRAVITY_EARTH / norm

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1] * factor,
                array[0] * factor,
                -array[2] * factor,
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1] * factor, processor.gx, 0.0)
        assertEquals(array[0] * factor, processor.gy, 0.0)
        assertEquals(-array[2] * factor, processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun process_whenGravityNormAdjustedAndLocation_setsExpectedValuesAndNotifies() {
        val location = getLocation()
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            location = location,
            adjustGravityNorm = true,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp1, SensorAccuracy.HIGH)

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1, timestamp1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp2, SensorAccuracy.HIGH)

        assertTrue(processor.process(measurement2, timestamp2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        val norm = Utils.normF(array)
        val factor =
            NEDGravityEstimator.estimateGravityAndReturnNew(location.toNEDPosition()).norm / norm

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1] * factor,
                array[0] * factor,
                -array[2] * factor,
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1] * factor, processor.gx, 0.0)
        assertEquals(array[0] * factor, processor.gy, 0.0)
        assertEquals(-array[2] * factor, processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun process_whenProvidedTimestamp_setsExpectedValuesAndNotifies() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = false,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp1, SensorAccuracy.HIGH)

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1, timestamp1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp2, SensorAccuracy.HIGH)

        assertTrue(processor.process(measurement2, timestamp2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1],
                array[0],
                -array[2],
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1], processor.gx, 0.0)
        assertEquals(array[0], processor.gy, 0.0)
        assertEquals(-array[2], processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun reset_setsInitialValue() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val processor = AccelerometerGravityProcessor(
            averagingFilter,
            adjustGravityNorm = false,
            processorListener = listener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()
        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp1,
            SensorAccuracy.HIGH
        )

        // first measurement returns false because averaging filter need to initialize
        assertFalse(processor.process(measurement1))

        // seconds measurement already has an initialized averaging filter
        val timestamp2 = System.nanoTime() + TIME_INTERVAL_NANOS
        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp2,
            SensorAccuracy.HIGH
        )

        assertTrue(processor.process(measurement2, timestamp2))

        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                any(),
                timestamp1
            )
        }

        val slot = MutableList(size = 2) { DoubleArray(3) }
        verify(exactly = 1) {
            averagingFilter.filter(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                capture(slot),
                timestamp2
            )
        }

        val array = slot.last()
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                array[1],
                array[0],
                -array[2],
                timestamp2,
                SensorAccuracy.HIGH
            )
        }

        // check
        assertEquals(array[1], processor.gx, 0.0)
        assertEquals(array[0], processor.gy, 0.0)
        assertEquals(-array[2], processor.gz, 0.0)
        assertEquals(timestamp2, processor.timestamp)
        assertEquals(SensorAccuracy.HIGH, processor.accuracy)

        // reset
        processor.reset()

        // check
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        assertEquals(0L, processor.timestamp)
        assertNull(processor.accuracy)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun expectedGravityNorm_whenNoLocation_returnsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        assertNull(processor.location)
        assertEquals(SensorManager.GRAVITY_EARTH.toDouble(), processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun expectedGravityNorm_whenLocation_returnsExpectedValue() {
        val location = getLocation()
        val processor = AccelerometerGravityProcessor(location = location)

        assertSame(location, processor.location)

        val expectedNorm =
            NEDGravityEstimator.estimateGravityAndReturnNew(location.toNEDPosition()).norm
        assertEquals(expectedNorm, processor.expectedGravityNorm, 0.0)
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
        const val TIME_INTERVAL_NANOS = 20_000_000

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0
    }
}