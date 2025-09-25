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

import android.location.Location
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeLocalPoseProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var processorListener: BaseLocalPoseProcessor.OnProcessedListener

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenRequiredParameters_returnsExpectedValues() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        // check
        assertSame(initialLocation, processor.initialLocation)
        assertEquals(NEDVelocity(), processor.initialVelocity)
        assertFalse(processor.estimatePoseTransformation)
        assertNull(processor.processorListener)
        assertNotNull(processor.initialEcefFrame)
        assertNotNull(processor.initialNedFrame)
        assertNotNull(processor.previousEcefFrame)
        assertNotNull(processor.currentEcefFrame)
        assertNotNull(processor.previousNedFrame)
        assertNotNull(processor.currentNedFrame)
        assertNotNull(processor.poseTransformation)
        assertTrue(processor.useLeveledRelativeAttitudeRespectStart)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val initialLocation = getLocation()
        val initialVelocity = getVelocity()
        val processor = AttitudeLocalPoseProcessor(
            initialLocation,
            initialVelocity,
            estimatePoseTransformation = true,
            processorListener = processorListener
        )

        // check
        assertSame(initialLocation, processor.initialLocation)
        assertSame(initialVelocity, processor.initialVelocity)
        assertTrue(processor.estimatePoseTransformation)
        assertSame(processorListener, processor.processorListener)
        assertNotNull(processor.initialEcefFrame)
        assertNotNull(processor.initialNedFrame)
        assertNotNull(processor.previousEcefFrame)
        assertNotNull(processor.currentEcefFrame)
        assertNotNull(processor.previousNedFrame)
        assertNotNull(processor.currentNedFrame)
        assertNotNull(processor.poseTransformation)
        assertTrue(processor.useLeveledRelativeAttitudeRespectStart)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
    }

    @Test
    fun initialLocation_setsExpectedValue() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        // check
        assertSame(initialLocation, processor.initialLocation)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        // check default value
        assertNull(processor.processorListener)

        // set new value
        processor.processorListener = processorListener

        // check
        assertSame(processorListener, processor.processorListener)
    }

    @Test
    fun reset_initializesInternalParameters() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "initializedFrame",
            true
        )
        val previousTimestamp = System.nanoTime()
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "timeIntervalSeconds",
            TIME_INTERVAL_SECONDS
        )

        // check initial values
        val initializedFrame1: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame1)
        assertTrue(initializedFrame1)

        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds1, 0.0)

        // reset
        processor.reset()

        // check
        val initializedFrame2: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame2)
        assertFalse(initializedFrame2)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(-1L, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)
    }

    @Test
    fun process_whenEmpty_returnsFalse() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAttitudeMeasurement_returnsFalse() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement = null,
                accelerometerMeasurement = AccelerometerSensorMeasurement(),
                gyroscopeMeasurement = GyroscopeSensorMeasurement(),
                timestamp = System.nanoTime()
            )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAccelerometerMeasurement_returnsFalse() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement = AttitudeSensorMeasurement(),
                accelerometerMeasurement = null,
                gyroscopeMeasurement = GyroscopeSensorMeasurement(),
                timestamp = System.nanoTime()
            )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoGyroscopeMeasurement_returnsFalse() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement = AttitudeSensorMeasurement(),
                accelerometerMeasurement = AccelerometerSensorMeasurement(),
                gyroscopeMeasurement = null,
                timestamp = System.nanoTime()
            )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenAttitudeProcessedAndTimeIntervalNotProcessed_returnsFalse() {
        val initialLocation = getLocation()
        val processor = AttitudeLocalPoseProcessor(initialLocation)

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement,
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty(
                "attitudeProcessor"
            )
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty(
            "attitudeProcessor",
            attitudeProcessorSpy
        )

        // check
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(-1L, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)

        val conversionRotation = ENUtoNEDConverter.conversionRotation
        val nedAttitude = Quaternion(enuAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        nedAttitude.inverse()
        Quaternion.product(conversionRotation, nedAttitude, nedAttitude)
        assertEquals(nedAttitude, currentAttitude)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)
    }

    @Test
    fun process_whenAttitudeTimeIntervalProcessedAndPoseTransformationNotEstimated_returnsTrue() {
        val initialLocation = getLocation()
        val initialVelocity = getVelocity()
        val processor = AttitudeLocalPoseProcessor(
            initialLocation,
            initialVelocity,
            estimatePoseTransformation = false
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val specificForce = AccelerationTriad(ay.toDouble(), ax.toDouble(), -az.toDouble())

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val angularSpeed = AngularSpeedTriad(wy.toDouble(), wx.toDouble(), -wz.toDouble())

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(ax, ay, az, timestamp = timestamp)
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(wx, wy, wz, timestamp = timestamp)

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty(
                "attitudeProcessor"
            )
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty(
            "attitudeProcessor",
            attitudeProcessorSpy
        )

        // check
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        val initializedFrame1: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        // process
        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement,
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )

        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        val initialNedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)
        assertEquals(initialNedAttitude, currentAttitude)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val initializedFrame2: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)

        val initialNedPosition = initialLocation.toNEDPosition()
        val initialC = CoordinateTransformation(
            initialNedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(initialC, coordinateTransformation)

        val initialNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialNedFrame")
        requireNotNull(initialNedFrame2)
        assertEquals(initialNedFrame, initialNedFrame2)

        val initialEcefFrame2: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialEcefFrame")
        requireNotNull(initialEcefFrame2)
        assertEquals(initialEcefFrame, initialEcefFrame2)

        val initialAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(initialNedAttitude, initialAttitude)

        val specificForce2: AccelerationTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "specificForce")
        requireNotNull(specificForce2)
        assertEquals(specificForce, specificForce2)

        val angularSpeed2: AngularSpeedTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "angularSpeed")
        requireNotNull(angularSpeed2)
        assertEquals(angularSpeed, angularSpeed2)

        val bodyKinematics = BodyKinematics(specificForce, angularSpeed)

        val currentEcefFrame: ECEFFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
            TIME_INTERVAL_SECONDS,
            previousEcefFrame2,
            bodyKinematics
        )

        val currentNedFrame: NEDFrame =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame)
        currentNedFrame.coordinateTransformationRotation = currentAttitude
        val currentNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(
            currentNedFrame.coordinateTransformation,
            currentNedFrame2.coordinateTransformation
        )
        assertEquals(currentNedFrame.latitude, currentNedFrame2.latitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.longitude, currentNedFrame2.longitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.height, currentNedFrame2.height, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vn, currentNedFrame2.vn, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.ve, currentNedFrame2.ve, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vd, currentNedFrame2.vd, VERY_LARGE_ABSOLUTE_ERROR)

        val currentEcefFrame2 =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val currentEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentEcefFrame")
        requireNotNull(currentEcefFrame3)

        assertTrue(
            currentEcefFrame2.coordinateTransformation.equals(
                currentEcefFrame3.coordinateTransformation,
                ABSOLUTE_ERROR
            )
        )
        assertEquals(currentEcefFrame2.x, currentEcefFrame3.x, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.y, currentEcefFrame3.y, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.z, currentEcefFrame3.z, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vx, currentEcefFrame3.vx, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vy, currentEcefFrame3.vy, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vz, currentEcefFrame3.vz, VERY_LARGE_ABSOLUTE_ERROR)

        val previousEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousEcefFrame")
        requireNotNull(previousEcefFrame3)
        assertEquals(currentEcefFrame3, previousEcefFrame3)

        val previousNedFrame: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(currentNedFrame2, previousNedFrame)
    }

    @Test
    fun process_whenAttitudeTimeIntervalProcessedAndPoseTransformationNotEstimatedWithListener_returnsTrueAndNotifies() {
        val initialLocation = getLocation()
        val initialVelocity = getVelocity()
        val processor = AttitudeLocalPoseProcessor(
            initialLocation,
            initialVelocity,
            estimatePoseTransformation = false,
            processorListener = processorListener
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val specificForce = AccelerationTriad(ay.toDouble(), ax.toDouble(), -az.toDouble())

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val angularSpeed = AngularSpeedTriad(wy.toDouble(), wx.toDouble(), -wz.toDouble())

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(ax, ay, az, timestamp = timestamp)
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(wx, wy, wz, timestamp = timestamp)

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty(
                "attitudeProcessor"
            )
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty(
            "attitudeProcessor",
            attitudeProcessorSpy
        )

        // check
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        val initializedFrame1: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        // process
        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement,
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )

        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        val initialNedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)
        assertEquals(initialNedAttitude, currentAttitude)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val initializedFrame2: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)

        val initialNedPosition = initialLocation.toNEDPosition()
        val initialC = CoordinateTransformation(
            initialNedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(initialC, coordinateTransformation)

        val initialNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialNedFrame")
        requireNotNull(initialNedFrame2)
        assertEquals(initialNedFrame, initialNedFrame2)

        val initialEcefFrame2: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialEcefFrame")
        requireNotNull(initialEcefFrame2)
        assertEquals(initialEcefFrame, initialEcefFrame2)

        val initialAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(initialNedAttitude, initialAttitude)

        val specificForce2: AccelerationTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "specificForce")
        requireNotNull(specificForce2)
        assertEquals(specificForce, specificForce2)

        val angularSpeed2: AngularSpeedTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "angularSpeed")
        requireNotNull(angularSpeed2)
        assertEquals(angularSpeed, angularSpeed2)

        val bodyKinematics = BodyKinematics(specificForce, angularSpeed)

        val currentEcefFrame: ECEFFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
            TIME_INTERVAL_SECONDS,
            previousEcefFrame2,
            bodyKinematics
        )

        val currentNedFrame: NEDFrame =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame)
        currentNedFrame.coordinateTransformationRotation = currentAttitude
        val currentNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(
            currentNedFrame.coordinateTransformation,
            currentNedFrame2.coordinateTransformation
        )
        assertEquals(currentNedFrame.latitude, currentNedFrame2.latitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.longitude, currentNedFrame2.longitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.height, currentNedFrame2.height, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vn, currentNedFrame2.vn, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.ve, currentNedFrame2.ve, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vd, currentNedFrame2.vd, VERY_LARGE_ABSOLUTE_ERROR)

        val currentEcefFrame2 =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val currentEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentEcefFrame")
        requireNotNull(currentEcefFrame3)

        assertTrue(
            currentEcefFrame2.coordinateTransformation.equals(
                currentEcefFrame3.coordinateTransformation,
                ABSOLUTE_ERROR
            )
        )
        assertEquals(currentEcefFrame2.x, currentEcefFrame3.x, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.y, currentEcefFrame3.y, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.z, currentEcefFrame3.z, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vx, currentEcefFrame3.vx, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vy, currentEcefFrame3.vy, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vz, currentEcefFrame3.vz, VERY_LARGE_ABSOLUTE_ERROR)

        val previousEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousEcefFrame")
        requireNotNull(previousEcefFrame3)
        assertEquals(currentEcefFrame3, previousEcefFrame3)

        val previousNedFrame: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(currentNedFrame2, previousNedFrame)

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                currentEcefFrame3,
                previousEcefFrame3,
                initialEcefFrame2,
                timestamp,
                null
            )
        }
    }

    @Test
    fun process_whenAttitudeTimeIntervalProcessedAndPoseTransformationEstimatedNotUseLeveledRelativeAttitudeRespectStart_returnsTrueAndNotifies() {
        val initialLocation = getLocation()
        val initialVelocity = getVelocity()
        val processor = AttitudeLocalPoseProcessor(
            initialLocation,
            initialVelocity,
            estimatePoseTransformation = true,
            processorListener = processorListener
        )
        processor.useLeveledRelativeAttitudeRespectStart = false

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val specificForce = AccelerationTriad(ay.toDouble(), ax.toDouble(), -az.toDouble())

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val angularSpeed = AngularSpeedTriad(wy.toDouble(), wx.toDouble(), -wz.toDouble())

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(ax, ay, az, timestamp = timestamp)
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(wx, wy, wz, timestamp = timestamp)

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty(
                "attitudeProcessor"
            )
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty(
            "attitudeProcessor",
            attitudeProcessorSpy
        )

        // check
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        val initializedFrame1: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        // process
        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement,
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )

        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        val initialNedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)
        assertEquals(initialNedAttitude, currentAttitude)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val initializedFrame2: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)

        val initialNedPosition = initialLocation.toNEDPosition()
        val initialC = CoordinateTransformation(
            initialNedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(initialC, coordinateTransformation)

        val initialNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialNedFrame")
        requireNotNull(initialNedFrame2)
        assertEquals(initialNedFrame, initialNedFrame2)

        val initialEcefFrame2: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialEcefFrame")
        requireNotNull(initialEcefFrame2)
        assertEquals(initialEcefFrame, initialEcefFrame2)

        val initialAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(initialNedAttitude, initialAttitude)

        val specificForce2: AccelerationTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "specificForce")
        requireNotNull(specificForce2)
        assertEquals(specificForce, specificForce2)

        val angularSpeed2: AngularSpeedTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "angularSpeed")
        requireNotNull(angularSpeed2)
        assertEquals(angularSpeed, angularSpeed2)

        val bodyKinematics = BodyKinematics(specificForce, angularSpeed)

        val currentEcefFrame: ECEFFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
            TIME_INTERVAL_SECONDS,
            previousEcefFrame2,
            bodyKinematics
        )

        val currentNedFrame: NEDFrame =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame)
        currentNedFrame.coordinateTransformationRotation = currentAttitude
        val currentNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(
            currentNedFrame.coordinateTransformation,
            currentNedFrame2.coordinateTransformation
        )
        assertEquals(currentNedFrame.latitude, currentNedFrame2.latitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.longitude, currentNedFrame2.longitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.height, currentNedFrame2.height, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vn, currentNedFrame2.vn, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.ve, currentNedFrame2.ve, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vd, currentNedFrame2.vd, VERY_LARGE_ABSOLUTE_ERROR)

        val currentEcefFrame2 =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val currentEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentEcefFrame")
        requireNotNull(currentEcefFrame3)

        assertTrue(
            currentEcefFrame2.coordinateTransformation.equals(
                currentEcefFrame3.coordinateTransformation,
                ABSOLUTE_ERROR
            )
        )
        assertEquals(currentEcefFrame2.x, currentEcefFrame3.x, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.y, currentEcefFrame3.y, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.z, currentEcefFrame3.z, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vx, currentEcefFrame3.vx, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vy, currentEcefFrame3.vy, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vz, currentEcefFrame3.vz, VERY_LARGE_ABSOLUTE_ERROR)

        val previousEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousEcefFrame")
        requireNotNull(previousEcefFrame3)
        assertEquals(currentEcefFrame3, previousEcefFrame3)

        val previousNedFrame: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(currentNedFrame2, previousNedFrame)

        val transformationRotation = ENUtoNEDConverter.convertAndReturnNew(currentAttitude)
        val transformationRotation2: Quaternion? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "transformationRotation"
        )
        requireNotNull(transformationRotation2)
        assertEquals(transformationRotation, transformationRotation2)

        val ecefDiffPosition = InhomogeneousPoint3D(
            currentEcefFrame3.x - initialEcefFrame2.x,
            currentEcefFrame3.y - initialEcefFrame2.y,
            currentEcefFrame3.z - initialEcefFrame2.z
        )
        val ecefDiffPosition2: InhomogeneousPoint3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "ecefDiffPosition")
        requireNotNull(ecefDiffPosition2)
        assertEquals(ecefDiffPosition, ecefDiffPosition2)

        val startEcefRotation = initialEcefFrame.coordinateTransformationRotation
        val startEcefRotation2: Rotation3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "startEcefRotation")
        requireNotNull(startEcefRotation2)
        assertEquals(startEcefRotation, startEcefRotation2)

        val inverseEcefRotation = startEcefRotation.inverseRotationAndReturnNew()
        val inverseEcefRotation2: Rotation3D? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "inverseEcefRotation"
        )
        requireNotNull(inverseEcefRotation2)
        assertTrue(inverseEcefRotation.equals(inverseEcefRotation2, ABSOLUTE_ERROR))

        val localDiffPosition = InhomogeneousPoint3D()
        inverseEcefRotation.rotate(ecefDiffPosition, localDiffPosition)
        localDiffPosition.setCoordinates(
            localDiffPosition.inhomY,
            localDiffPosition.inhomX,
            -localDiffPosition.inhomZ
        )
        val localDiffPosition2: InhomogeneousPoint3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "localDiffPosition")
        requireNotNull(localDiffPosition2)
        assertEquals(localDiffPosition, localDiffPosition2)

        val poseTransformation =
            EuclideanTransformation3D(transformationRotation, localDiffPosition.asArray())
        val poseTransformation2: EuclideanTransformation3D? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "poseTransformation"
        )
        requireNotNull(poseTransformation2)
        assertTrue(
            poseTransformation.asMatrix().equals(poseTransformation2.asMatrix(), ABSOLUTE_ERROR)
        )

        val distance = ecefDiffPosition.distanceTo(InhomogeneousPoint3D())
        val distance2 = localDiffPosition.distanceTo(InhomogeneousPoint3D())
        assertEquals(distance, distance2, ABSOLUTE_ERROR)

        val distance3 = Utils.normF(poseTransformation.translation)
        assertEquals(distance, distance3, ABSOLUTE_ERROR)

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                currentEcefFrame3,
                previousEcefFrame3,
                initialEcefFrame2,
                timestamp,
                poseTransformation2
            )
        }
    }

    @Test
    fun process_whenAttitudeTimeIntervalProcessedAndPoseTransformationEstimatedUseLeveledRelativeAttitudeRespectStart_returnsTrueAndNotifies() {
        val initialLocation = getLocation()
        val initialVelocity = getVelocity()
        val processor = AttitudeLocalPoseProcessor(
            initialLocation,
            initialVelocity,
            estimatePoseTransformation = true,
            processorListener = processorListener
        )
        processor.useLeveledRelativeAttitudeRespectStart = true

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val specificForce = AccelerationTriad(ay.toDouble(), ax.toDouble(), -az.toDouble())

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val angularSpeed = AngularSpeedTriad(wy.toDouble(), wx.toDouble(), -wz.toDouble())

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(ax, ay, az, timestamp = timestamp)
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(wx, wy, wz, timestamp = timestamp)

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: AttitudeProcessor? =
            processor.getPrivateProperty(
                "attitudeProcessor"
            )
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty(
            "attitudeProcessor",
            attitudeProcessorSpy
        )

        // check
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        val initializedFrame1: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        // process
        val syncedMeasurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
                attitudeMeasurement,
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )

        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        val initialNedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)
        assertEquals(initialNedAttitude, currentAttitude)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val initializedFrame2: Boolean? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)

        val initialNedPosition = initialLocation.toNEDPosition()
        val initialC = CoordinateTransformation(
            initialNedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseLocalPoseProcessor::class,
                processor,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(initialC, coordinateTransformation)

        val initialNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialNedFrame")
        requireNotNull(initialNedFrame2)
        assertEquals(initialNedFrame, initialNedFrame2)

        val initialEcefFrame2: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialEcefFrame")
        requireNotNull(initialEcefFrame2)
        assertEquals(initialEcefFrame, initialEcefFrame2)

        val initialAttitude: Quaternion? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(initialNedAttitude, initialAttitude)

        val specificForce2: AccelerationTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "specificForce")
        requireNotNull(specificForce2)
        assertEquals(specificForce, specificForce2)

        val angularSpeed2: AngularSpeedTriad? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "angularSpeed")
        requireNotNull(angularSpeed2)
        assertEquals(angularSpeed, angularSpeed2)

        val bodyKinematics = BodyKinematics(specificForce, angularSpeed)

        val currentEcefFrame: ECEFFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
            TIME_INTERVAL_SECONDS,
            previousEcefFrame2,
            bodyKinematics
        )

        val currentNedFrame: NEDFrame =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame)
        currentNedFrame.coordinateTransformationRotation = currentAttitude
        val currentNedFrame2: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(
            currentNedFrame.coordinateTransformation,
            currentNedFrame2.coordinateTransformation
        )
        assertEquals(currentNedFrame.latitude, currentNedFrame2.latitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.longitude, currentNedFrame2.longitude, ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.height, currentNedFrame2.height, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vn, currentNedFrame2.vn, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.ve, currentNedFrame2.ve, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentNedFrame.vd, currentNedFrame2.vd, VERY_LARGE_ABSOLUTE_ERROR)

        val currentEcefFrame2 =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val currentEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "currentEcefFrame")
        requireNotNull(currentEcefFrame3)

        assertTrue(
            currentEcefFrame2.coordinateTransformation.equals(
                currentEcefFrame3.coordinateTransformation,
                ABSOLUTE_ERROR
            )
        )
        assertEquals(currentEcefFrame2.x, currentEcefFrame3.x, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.y, currentEcefFrame3.y, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.z, currentEcefFrame3.z, LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vx, currentEcefFrame3.vx, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vy, currentEcefFrame3.vy, VERY_LARGE_ABSOLUTE_ERROR)
        assertEquals(currentEcefFrame2.vz, currentEcefFrame3.vz, VERY_LARGE_ABSOLUTE_ERROR)

        val previousEcefFrame3: ECEFFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousEcefFrame")
        requireNotNull(previousEcefFrame3)
        assertEquals(currentEcefFrame3, previousEcefFrame3)

        val previousNedFrame: NEDFrame? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(currentNedFrame2, previousNedFrame)

        val initYaw = initialAttitude.toEulerAngles()[2]
        val eulerAngles = currentAttitude.toEulerAngles()
        val transformationRotation =
            Quaternion(eulerAngles[0], eulerAngles[1], eulerAngles[2] - initYaw)
        ENUtoNEDConverter.convert(transformationRotation, transformationRotation)
        val transformationRotation2: Quaternion? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "transformationRotation"
        )
        requireNotNull(transformationRotation2)
        assertEquals(transformationRotation, transformationRotation2)

        val ecefDiffPosition = InhomogeneousPoint3D(
            currentEcefFrame3.x - initialEcefFrame2.x,
            currentEcefFrame3.y - initialEcefFrame2.y,
            currentEcefFrame3.z - initialEcefFrame2.z
        )
        val ecefDiffPosition2: InhomogeneousPoint3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "ecefDiffPosition")
        requireNotNull(ecefDiffPosition2)
        assertEquals(ecefDiffPosition, ecefDiffPosition2)

        val startEcefRotation = initialEcefFrame.coordinateTransformationRotation
        val startEcefRotation2: Rotation3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "startEcefRotation")
        requireNotNull(startEcefRotation2)
        assertEquals(startEcefRotation, startEcefRotation2)

        val inverseEcefRotation = startEcefRotation.inverseRotationAndReturnNew()
        val inverseEcefRotation2: Rotation3D? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "inverseEcefRotation"
        )
        requireNotNull(inverseEcefRotation2)
        assertTrue(inverseEcefRotation.equals(inverseEcefRotation2, ABSOLUTE_ERROR))

        val localDiffPosition = InhomogeneousPoint3D()
        inverseEcefRotation.rotate(ecefDiffPosition, localDiffPosition)
        localDiffPosition.setCoordinates(
            localDiffPosition.inhomY,
            localDiffPosition.inhomX,
            -localDiffPosition.inhomZ
        )
        val localDiffPosition2: InhomogeneousPoint3D? =
            getPrivateProperty(BaseLocalPoseProcessor::class, processor, "localDiffPosition")
        requireNotNull(localDiffPosition2)
        assertEquals(localDiffPosition, localDiffPosition2)

        val poseTransformation =
            EuclideanTransformation3D(transformationRotation, localDiffPosition.asArray())
        val poseTransformation2: EuclideanTransformation3D? = getPrivateProperty(
            BaseLocalPoseProcessor::class,
            processor,
            "poseTransformation"
        )
        requireNotNull(poseTransformation2)
        assertTrue(
            poseTransformation.asMatrix().equals(poseTransformation2.asMatrix(), ABSOLUTE_ERROR)
        )

        val distance = ecefDiffPosition.distanceTo(InhomogeneousPoint3D())
        val distance2 = localDiffPosition.distanceTo(InhomogeneousPoint3D())
        assertEquals(distance, distance2, ABSOLUTE_ERROR)

        val distance3 = Utils.normF(poseTransformation.translation)
        assertEquals(distance, distance3, ABSOLUTE_ERROR)

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                currentEcefFrame3,
                previousEcefFrame3,
                initialEcefFrame2,
                timestamp,
                poseTransformation2
            )
        }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
        val longitudeDegrees =
            randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

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

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val MIN_VELOCITY = -1.0
        const val MAX_VELOCITY = 1.0

        const val TIME_INTERVAL_SECONDS = 0.01

        const val TIME_INTERVAL_NANOS = 10_000_000L

        const val ABSOLUTE_ERROR = 1e-8

        const val LARGE_ABSOLUTE_ERROR = 1e-4

        const val VERY_LARGE_ABSOLUTE_ERROR = 1e-3

        fun getVelocity(): NEDVelocity {
            val randomizer = UniformRandomizer()
            val vn = randomizer.nextDouble(MIN_VELOCITY, MAX_VELOCITY)
            val ve = randomizer.nextDouble(MIN_VELOCITY, MAX_VELOCITY)
            val vd = randomizer.nextDouble(MIN_VELOCITY, MAX_VELOCITY)
            return NEDVelocity(vn, ve, vd)
        }

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}