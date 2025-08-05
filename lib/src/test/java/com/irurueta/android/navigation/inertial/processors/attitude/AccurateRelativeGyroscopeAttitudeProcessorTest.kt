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

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test

@Ignore("possible memory leak")
class AccurateRelativeGyroscopeAttitudeProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var listener: BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_setsDefaultValues() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor()

        assertNull(processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
    }

    @Test
    fun constructor_whenListener_setsExpectedValues() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor(listener)

        assertSame(listener, processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun process_whenFirstMeasurement_setsPreviousTimestamp() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor(listener)

        val previousTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp"
        )
        requireNotNull(previousTimestamp1)
        assertEquals(-1L, previousTimestamp1)

        val quaternionStepIntegrator: QuaternionStepIntegrator? =
            processor.getPrivateProperty("quaternionStepIntegrator")
        requireNotNull(quaternionStepIntegrator)
        val quaternionStepIntegratorSpy = spyk(quaternionStepIntegrator)
        processor.setPrivateProperty("quaternionStepIntegrator", quaternionStepIntegratorSpy)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp)
        assertFalse(processor.process(measurement))

        // check
        val previousTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp"
        )
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)

        verify { listener wasNot Called }
        verify { quaternionStepIntegratorSpy wasNot Called }
    }

    @Test
    fun process_whenNotFirstMeasurement_setsExpectedAttitudeAndNotifies() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor(listener)

        val timestamp = System.nanoTime()
        val previousTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val quaternionStepIntegrator: QuaternionStepIntegrator? =
            processor.getPrivateProperty("quaternionStepIntegrator")
        requireNotNull(quaternionStepIntegrator)
        val quaternionStepIntegratorSpy = spyk(quaternionStepIntegrator)
        processor.setPrivateProperty("quaternionStepIntegrator", quaternionStepIntegratorSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(
            wy,
            wx,
            -wz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH
        )
        assertTrue(processor.process(measurement))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble()
            )
        }
        verify(exactly = 1) {
            quaternionStepIntegratorSpy.integrate(
                internalAttitude,
                0.0,
                0.0,
                0.0,
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble(),
                INTERVAL_SECONDS,
                internalAttitude
            )
        }

        assertEquals(internalAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                processor.attitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun process_whenNotFirstMeasurementWithBias_setsExpectedAttitudeAndNotifies() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor(listener)

        val timestamp = System.nanoTime()
        val previousTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val quaternionStepIntegrator: QuaternionStepIntegrator? =
            processor.getPrivateProperty("quaternionStepIntegrator")
        requireNotNull(quaternionStepIntegrator)
        val quaternionStepIntegratorSpy = spyk(quaternionStepIntegrator)
        processor.setPrivateProperty("quaternionStepIntegrator", quaternionStepIntegratorSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement =
            GyroscopeSensorMeasurement(wy, wx, -wz, by, bx, -bz, timestamp, SensorAccuracy.HIGH)
        assertTrue(processor.process(measurement))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble()
            )
        }
        verify(exactly = 1) {
            quaternionStepIntegratorSpy.integrate(
                internalAttitude,
                0.0,
                0.0,
                0.0,
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble(),
                INTERVAL_SECONDS,
                internalAttitude
            )
        }

        assertEquals(internalAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                processor.attitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun process_whenProvidedTimestamp_setsExpectedAttitudeAndNotifies() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor(listener)

        val timestamp = System.nanoTime()
        val previousTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val quaternionStepIntegrator: QuaternionStepIntegrator? =
            processor.getPrivateProperty("quaternionStepIntegrator")
        requireNotNull(quaternionStepIntegrator)
        val quaternionStepIntegratorSpy = spyk(quaternionStepIntegrator)
        processor.setPrivateProperty("quaternionStepIntegrator", quaternionStepIntegratorSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement =
            GyroscopeSensorMeasurement(wy, wx, -wz, by, bx, -bz, timestamp, SensorAccuracy.HIGH)
        assertTrue(processor.process(measurement, timestamp))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble()
            )
        }
        verify(exactly = 1) {
            quaternionStepIntegratorSpy.integrate(
                internalAttitude,
                0.0,
                0.0,
                0.0,
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble(),
                INTERVAL_SECONDS,
                internalAttitude
            )
        }

        assertEquals(internalAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                processor.attitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun reset_restoresInitialState() {
        val processor = AccurateRelativeGyroscopeAttitudeProcessor()

        val timestamp = System.nanoTime()
        val previousTimestamp1 = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp1
        )

        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        processor.attitude.fromQuaternion(Quaternion(roll1, pitch1, yaw1))

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        internalAttitude.fromQuaternion(Quaternion(roll2, pitch2, yaw2))

        processor.reset()

        // check
        val previousTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "previousTimestamp"
        )
        requireNotNull(previousTimestamp2)
        assertEquals(-1L, previousTimestamp2)

        assertEquals(Quaternion(), processor.attitude)
        assertEquals(Quaternion(), internalAttitude)
    }

    private companion object {
        const val INTERVAL_NANOS = 10_000_000 // 0,01 seconds

        const val INTERVAL_SECONDS = 0.01

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0
    }
}