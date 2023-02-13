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

import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class AccelerometerGravityProcessorTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AccelerometerGravityProcessor()

        assertNotNull(processor.averagingFilter)
        assertTrue(processor.averagingFilter is LowPassAveragingFilter)
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
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>()
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

        assertSame(averagingFilter, processor.averagingFilter)
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
    fun processorListener_setsExpectedValue() {
        val processor = AccelerometerGravityProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun process_whenNotEnoughMeasurements_doesNotSetsExpectedValuesOrNotifies() {
        val averagingFilter = spyk(LowPassAveragingFilter())
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>(
                relaxUnitFun = true
            )
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

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
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>(
                relaxUnitFun = true
            )
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

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
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>(
                relaxUnitFun = true
            )
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

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
    fun process_whenProvidedTimestamp_setsExpectedValuesAndNotifies() {
        val averagingFilter = spyk(MeanAveragingFilter())
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>(
                relaxUnitFun = true
            )
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

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
        val listener =
            mockk<BaseGravityProcessor.OnProcessedListener<AccelerometerSensorMeasurement>>(
                relaxUnitFun = true
            )
        val processor = AccelerometerGravityProcessor(averagingFilter, listener)

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

    private companion object {
        const val TIME_INTERVAL_NANOS = 20_000_000
    }
}