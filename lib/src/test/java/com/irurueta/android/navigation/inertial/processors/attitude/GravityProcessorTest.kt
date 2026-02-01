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
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import kotlin.math.sqrt

class GravityProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var listener:
            BaseGravityProcessor.OnProcessedListener<GravitySensorMeasurement>

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = GravityProcessor()

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
    fun constructor_whenAllProperties_returnsExpectedValues() {
        val location = getLocation()
        val processor = GravityProcessor(location, adjustGravityNorm = false, listener)

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
        val processor = GravityProcessor()

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
        val processor = GravityProcessor()

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
        val processor = GravityProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun getNedPosition_whenNoLocation_returnsExpectedValue() {
        val processor = GravityProcessor()

        assertFalse(processor.getNedPosition(NEDPosition()))
    }

    @Test
    fun getNedPosition_whenLocation_returnsExpectedValue() {
        val location = getLocation()
        val processor = GravityProcessor(location)

        val nedPosition = NEDPosition()
        assertTrue(processor.getNedPosition(nedPosition))

        assertEquals(nedPosition, location.toNEDPosition())
    }

    @Test
    fun process_whenGravityNormNotAdjusted_setsExpectedValuesAndNotifies() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val processor = GravityProcessor(adjustGravityNorm = false, processorListener = listener)

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
    fun process_whenGravityNormAdjustedAndNoLocation_setsExpectedValuesAndNotifies() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val processor = GravityProcessor(adjustGravityNorm = true, processorListener = listener)

        assertTrue(processor.process(measurement))

        val norm = sqrt(
            gx.toDouble() * gx.toDouble() + gy.toDouble() * gy.toDouble()
                    + gz.toDouble() * gz.toDouble()
        )
        val factor = SensorManager.GRAVITY_EARTH / norm
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                gy.toDouble() * factor,
                gx.toDouble() * factor,
                -gz.toDouble() * factor,
                timestamp,
                SensorAccuracy.LOW
            )
        }

        // check
        assertEquals(gy.toDouble() * factor, processor.gx, 0.0)
        assertEquals(gx.toDouble() * factor, processor.gy, 0.0)
        assertEquals(-gz.toDouble() * factor, processor.gz, 0.0)
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
    fun process_whenGravityNormAdjustedAndLocation_setsExpectedValuesAndNotifies() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)

        val processor = GravityProcessor(
            location = location,
            adjustGravityNorm = true,
            processorListener = listener
        )

        assertTrue(processor.process(measurement))

        val norm = sqrt(
            gx.toDouble() * gx.toDouble() + gy.toDouble() * gy.toDouble()
                    + gz.toDouble() * gz.toDouble()
        )

        val factor =
            NEDGravityEstimator.estimateGravityAndReturnNew(location.toNEDPosition()).norm / norm
        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                gy.toDouble() * factor,
                gx.toDouble() * factor,
                -gz.toDouble() * factor,
                timestamp,
                SensorAccuracy.LOW
            )
        }

        // check
        assertEquals(gy.toDouble() * factor, processor.gx, 0.0)
        assertEquals(gx.toDouble() * factor, processor.gy, 0.0)
        assertEquals(-gz.toDouble() * factor, processor.gz, 0.0)
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

        val processor = GravityProcessor(adjustGravityNorm = false, processorListener = listener)

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

        val processor = GravityProcessor(adjustGravityNorm = false)

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

    @Test
    fun expectedGravityNorm_whenNoLocation_returnsExpectedValue() {
        val processor = GravityProcessor()

        assertNull(processor.location)
        assertEquals(SensorManager.GRAVITY_EARTH.toDouble(), processor.expectedGravityNorm, 0.0)
    }

    @Test
    fun expectedGravityNorm_whenLocation_returnsExpectedValue() {
        val location = getLocation()
        val processor = GravityProcessor(location)

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
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0
    }
}