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
package com.irurueta.android.navigation.inertial.processors

import com.irurueta.android.navigation.inertial.collectors.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.clearAllMocks
import io.mockk.mockk
import io.mockk.verify
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class GravityProcessorTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = GravityProcessor()

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
    fun constructor_whenListener_returnsExpectedValues() {
        val listener = mockk<BaseGravityProcessor.OnProcessedListener<GravitySensorMeasurement>>()
        val processor = GravityProcessor(listener)

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
        val processor = GravityProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val listener = mockk<BaseGravityProcessor.OnProcessedListener<GravitySensorMeasurement>>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun process_setsExpectedValuesAndNotifies() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val listener = mockk<BaseGravityProcessor.OnProcessedListener<GravitySensorMeasurement>>(
            relaxUnitFun = true
        )
        val processor = GravityProcessor(listener)

        assertTrue(processor.process(measurement))

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                gy.toDouble(),
                gx.toDouble(),
                -gz.toDouble(),
                timestamp,
                SensorAccuracy.LOW
            )
        }

        // check
        assertEquals(gy.toDouble(), processor.gx, 0.0)
        assertEquals(gx.toDouble(), processor.gy, 0.0)
        assertEquals(-gz.toDouble(), processor.gz, 0.0)
        assertEquals(timestamp, processor.timestamp)
        assertEquals(SensorAccuracy.LOW, processor.accuracy)

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
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val listener = mockk<BaseGravityProcessor.OnProcessedListener<GravitySensorMeasurement>>(
            relaxUnitFun = true
        )
        val processor = GravityProcessor(listener)

        assertTrue(processor.process(measurement, timestamp))

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                gy.toDouble(),
                gx.toDouble(),
                -gz.toDouble(),
                timestamp,
                SensorAccuracy.LOW
            )
        }

        // check
        assertEquals(gy.toDouble(), processor.gx, 0.0)
        assertEquals(gx.toDouble(), processor.gy, 0.0)
        assertEquals(-gz.toDouble(), processor.gz, 0.0)
        assertEquals(timestamp, processor.timestamp)
        assertEquals(SensorAccuracy.LOW, processor.accuracy)

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
    fun reset_setsInitialValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val processor = GravityProcessor()

        assertTrue(processor.process(measurement))

        assertEquals(gy.toDouble(), processor.gx, 0.0)
        assertEquals(gx.toDouble(), processor.gy, 0.0)
        assertEquals(-gz.toDouble(), processor.gz, 0.0)
        assertEquals(timestamp, processor.timestamp)
        assertEquals(SensorAccuracy.LOW, processor.accuracy)

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
}