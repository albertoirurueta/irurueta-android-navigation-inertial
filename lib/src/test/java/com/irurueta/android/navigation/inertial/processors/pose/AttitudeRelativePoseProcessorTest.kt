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
package com.irurueta.android.navigation.inertial.processors.pose

import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.pose.SpeedTriad
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGravityProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.estimators.NEDKinematicsEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeRelativePoseProcessorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AttitudeRelativePoseProcessor()

        // check
        assertEquals(SpeedTriad(), processor.initialSpeed)
        assertNull(processor.processorListener)
        assertNotNull(processor.poseTransformation)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertNotNull(processor.averagingFilter)
        assertTrue(processor.averagingFilter is LowPassAveragingFilter)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val initialSpeed = getSpeed()
        val averagingFilter = MeanAveragingFilter()
        val processorListener =
            mockk<BaseRelativePoseProcessor.OnProcessedListener>()
        val processor =
            AttitudeRelativePoseProcessor(initialSpeed, averagingFilter, processorListener)

        // check
        assertSame(initialSpeed, processor.initialSpeed)
        assertSame(processorListener, processor.processorListener)
        assertNotNull(processor.poseTransformation)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertSame(averagingFilter, processor.averagingFilter)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AttitudeRelativePoseProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val processorListener = mockk<BaseRelativePoseProcessor.OnProcessedListener>()
        processor.processorListener = processorListener

        // check
        assertSame(processorListener, processor.processorListener)
    }

    @Test
    fun reset_initializesInternalParameters() {
        val processor = AttitudeRelativePoseProcessor()

        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "initialized",
            true
        )
        val previousTimestamp = System.nanoTime()
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "timeIntervalSeconds",
            TIME_INTERVAL_SECONDS
        )

        // check initial values
        val initialized1: Boolean? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds1, 0.0)

        // reset
        processor.reset()

        // check
        val initialized2: Boolean? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "initialized")
        requireNotNull(initialized2)
        assertFalse(initialized2)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(-1L, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)
    }

    @Test
    fun process_whenEmpty_returnsFalse() {
        val processor = AttitudeRelativePoseProcessor()

        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAttitudeMeasurement_returnsFalse() {
        val processor = AttitudeRelativePoseProcessor()

        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement = null,
            accelerometerMeasurement = AccelerometerSensorMeasurement(),
            timestamp = System.nanoTime()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAccelerometerMeasurement_returnsFalse() {
        val processor = AttitudeRelativePoseProcessor()

        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement = AttitudeSensorMeasurement(),
            accelerometerMeasurement = null,
            timestamp = System.nanoTime()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenAttitudeProcessedAndGravityNotProcessed_returnsFalse() {
        val processor = AttitudeRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any(), any()) }.returns(false)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(exactly = 0) { gravityProcessorSpy.getGravity(any()) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)

        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val nedAttitude = Quaternion(enuAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(), gravity)
    }

    @Test
    fun process_whenAttitudeAndGravityProcessedAndTimeIntervalNotProcessed_returnsFalse() {
        val processor = AttitudeRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)

        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val nedAttitude = Quaternion(enuAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(-1L, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)
    }

    @Test
    fun process_whenAttitudeAndGravityAndTimeIntervalProcessed_returnsTrue() {
        val processor = AttitudeRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        enuAttitude.normalize()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)

        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val nedAttitude = Quaternion(enuAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.normalize()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        // define kinematics to keep device static with current frame (same position, speed and
        // attitude)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val transformation = processor.poseTransformation
        val transformationRotation = transformation.rotation.toQuaternion()
        transformationRotation.normalize()
        val transformationRotation2 = nedAttitude.multiplyAndReturnNew(conversionRotation)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)
    }

    @Test
    fun process_whenAttitudeAndGravityAndTimeIntervalProcessedWithListener_returnsTrueAndNotifies() {
        val processorListener =
            mockk<BaseRelativePoseProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = AttitudeRelativePoseProcessor(processorListener = processorListener)

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        enuAttitude.normalize()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)

        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val nedAttitude = Quaternion(enuAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.normalize()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        // define kinematics to keep device static with current frame (same position, speed and
        // attitude)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val transformation = processor.poseTransformation
        val transformationRotation = transformation.rotation.toQuaternion()
        transformationRotation.normalize()
        val transformationRotation2 = nedAttitude.multiplyAndReturnNew(conversionRotation)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)

        verify(exactly = 1) { processorListener.onProcessed(processor, timestamp, transformation) }
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

        const val MIN_SPEED = -1.0
        const val MAX_SPEED = 1.0

        const val TIME_INTERVAL_SECONDS = 0.01

        const val TIME_INTERVAL_NANOS = 10_000_000L

        const val VERY_LARGE_ABSOLUTE_ERROR = 1e-3

        fun getSpeed(): SpeedTriad {
            val randomizer = UniformRandomizer()
            val vx = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            val vy = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            val vz = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            return SpeedTriad(vx, vy, vz)
        }

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }

        fun getNEDPosition(): NEDPosition {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
            val longitudeDegrees =
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

            return NEDPosition(
                Math.toRadians(latitudeDegrees),
                Math.toRadians(longitudeDegrees),
                height
            )
        }
    }
}