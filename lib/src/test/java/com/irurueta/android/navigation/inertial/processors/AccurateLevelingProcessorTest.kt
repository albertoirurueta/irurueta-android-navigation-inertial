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

import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccurateLevelingProcessorTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredParameters_returnsExpectedValues() {
        val location = getLocation()
        val processor = AccurateLevelingProcessor(location)

        assertSame(location, processor.location)
        assertNull(processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)

        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val location = getLocation()
        val listener = mockk<BaseLevelingProcessor.OnProcessedListener>()
        val processor = AccurateLevelingProcessor(location, listener)

        assertSame(location, processor.location)
        assertSame(listener, processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)

        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val location = getLocation()
        val processor = AccurateLevelingProcessor(location)

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val listener = mockk<BaseLevelingProcessor.OnProcessedListener>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun process_setsExpectedAttitudeAndNotifies() {
        val location = getLocation()
        val listener = mockk<BaseLevelingProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = AccurateLevelingProcessor(location, listener)

        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

        // body attitude
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = 0.0

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )

        // obtain specific force neglecting north component of gravity, which contains gravity for
        // current device attitude
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        processor.process(fx, fy, fz)

        val slot = slot<Quaternion>()
        verify(exactly = 1) { listener.onProcessed(processor, capture(slot)) }

        val expectedAttitude = Quaternion()
        bodyC.asRotation(expectedAttitude)
        expectedAttitude.inverse()
        expectedAttitude.normalize()

        val capturedAttitude = slot.captured
        capturedAttitude.normalize()
        assertTrue(capturedAttitude.equals(expectedAttitude, ABSOLUTE_ERROR))
        assertTrue(processor.attitude.equals(expectedAttitude, ABSOLUTE_ERROR))

        assertEquals(fx, processor.gx, 0.0)
        assertEquals(fy, processor.gy, 0.0)
        assertEquals(fz, processor.gz, 0.0)

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
    fun reset_setsExpectedAttitudeAndNotifies() {
        val location = getLocation()
        val listener = mockk<BaseLevelingProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = AccurateLevelingProcessor(location, listener)

        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

        // body attitude
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = 0.0

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )

        // obtain specific force neglecting north component of gravity, which contains gravity for
        // current device attitude
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        processor.process(fx, fy, fz)

        val slot = slot<Quaternion>()
        verify(exactly = 1) { listener.onProcessed(processor, capture(slot)) }

        val expectedAttitude = Quaternion()
        bodyC.asRotation(expectedAttitude)
        expectedAttitude.inverse()
        expectedAttitude.normalize()

        val capturedAttitude = slot.captured
        capturedAttitude.normalize()
        assertTrue(capturedAttitude.equals(expectedAttitude, ABSOLUTE_ERROR))
        assertTrue(processor.attitude.equals(expectedAttitude, ABSOLUTE_ERROR))

        assertEquals(fx, processor.gx, 0.0)
        assertEquals(fy, processor.gy, 0.0)
        assertEquals(fz, processor.gz, 0.0)

        val gravity1 = processor.gravity
        assertEquals(processor.gx, gravity1.valueX, 0.0)
        assertEquals(processor.gy, gravity1.valueY, 0.0)
        assertEquals(processor.gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)

        // reset
        processor.reset()

        // check
        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        assertEquals(AccelerationTriad(), processor.gravity)
        val gravity = AccelerationTriad()
        processor.getGravity(gravity)
        assertEquals(AccelerationTriad(), gravity)
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val ABSOLUTE_ERROR = 1e-1

        fun getLocation(): Location {
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

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }
    }
}