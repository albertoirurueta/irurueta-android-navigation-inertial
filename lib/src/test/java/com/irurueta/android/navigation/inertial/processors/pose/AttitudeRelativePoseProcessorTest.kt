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
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.pose.SpeedTriad
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGravityProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.estimators.NEDKinematicsEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class AttitudeRelativePoseProcessorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var processorListener: BaseRelativePoseProcessor.OnProcessedListener

//    @MockK
    @Mock
    private lateinit var location: Location

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

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
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val initialSpeed = getSpeed()
        val averagingFilter = MeanAveragingFilter()
        val processor =
            AttitudeRelativePoseProcessor(initialSpeed, averagingFilter, processorListener)

        // check
        assertSame(initialSpeed, processor.initialSpeed)
        assertSame(processorListener, processor.processorListener)
        assertNotNull(processor.poseTransformation)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertSame(averagingFilter, processor.averagingFilter)
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AttitudeRelativePoseProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
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
    fun location_setsExpectedValue() {
        val processor = AttitudeRelativePoseProcessor()

        // check default value
        assertNull(processor.location)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertNull(gravityProcessor.location)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
        assertSame(location, gravityProcessor.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val processor = AttitudeRelativePoseProcessor()

        // check default value
        assertTrue(processor.adjustGravityNorm)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertTrue(gravityProcessor.adjustGravityNorm)

        // set new value
        processor.adjustGravityNorm = false

        // check
        assertFalse(gravityProcessor.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
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
        val attitudeProcessorSpy = spy(attitudeProcessor)
//        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(false).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(false)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(attitudeProcessorSpy, only()).process(attitudeMeasurement)
//        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(gravityProcessorSpy, only()).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, never()).getGravity(any())
//        verify(exactly = 0) { gravityProcessorSpy.getGravity(any()) }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)

        val conversionRotation = ENUtoNEDConverter.conversionRotation
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

        val conversionRotation = ENUtoNEDConverter.conversionRotation
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
        ENUtoNEDConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDConverter.convert(
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
        val attitudeProcessorSpy = spy(attitudeProcessor)
//        val attitudeProcessorSpy = spyk(attitudeProcessor)
        doReturn(nedAttitude).whenever(attitudeProcessorSpy).process(any())
//        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        doAnswer { invocation ->
            val result = invocation.getArgument<AccelerationTriad>(0)
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }.whenever(gravityProcessorSpy).getGravity(any())
/*        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }*/
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
        verify(attitudeProcessorSpy, only()).process(attitudeMeasurement)
//        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, times(1)).getGravity(any())
//        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

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

        val nedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)

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
        ENUtoNEDConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDConverter.convert(
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
        val attitudeProcessorSpy = spy(attitudeProcessor)
//        val attitudeProcessorSpy = spyk(attitudeProcessor)
        doReturn(nedAttitude).whenever(attitudeProcessorSpy).process(any())
//        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        doAnswer { invocation ->
            val result = invocation.getArgument<AccelerationTriad>(0)
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }.whenever(gravityProcessorSpy).getGravity(any())
/*        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }*/
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
        verify(attitudeProcessorSpy, only()).process(attitudeMeasurement)
//        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, times(1)).getGravity(any())
//        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

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
        val transformationRotation2 = ENUtoNEDConverter.convertAndReturnNew(nedAttitude)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)
    }

    @Test
    fun process_whenAttitudeAndGravityAndTimeIntervalProcessedWithListener_returnsTrueAndNotifies() {
        val processor = AttitudeRelativePoseProcessor(processorListener = processorListener)

        val timestamp = System.nanoTime()
        val enuAttitude = getAttitude()
        enuAttitude.normalize()
        val attitudeMeasurement =
            AttitudeSensorMeasurement(attitude = enuAttitude, timestamp = timestamp)

        val nedAttitude = ENUtoNEDConverter.convertAndReturnNew(enuAttitude)

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
        ENUtoNEDConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDConverter.convert(
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
        val attitudeProcessorSpy = spy(attitudeProcessor)
//        val attitudeProcessorSpy = spyk(attitudeProcessor)
        doReturn(nedAttitude).whenever(attitudeProcessorSpy).process(any())
//        every { attitudeProcessorSpy.process(any()) }.returns(nedAttitude)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        doAnswer { invocation ->
            val result = invocation.getArgument<AccelerationTriad>(0)
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }.whenever(gravityProcessorSpy).getGravity(any())
/*        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }*/
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
        verify(attitudeProcessorSpy, only()).process(attitudeMeasurement)
//        verify(exactly = 1) { attitudeProcessorSpy.process(attitudeMeasurement) }
        verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, times(1)).getGravity(any())
//        verify(exactly = 1) { gravityProcessorSpy.getGravity(any()) }

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
        val transformationRotation2 = ENUtoNEDConverter.convertAndReturnNew(nedAttitude)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)

        verify(processorListener, only()).onProcessed(processor, timestamp, transformation)
//        verify(exactly = 1) { processorListener.onProcessed(processor, timestamp, transformation) }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(
            MIN_LATITUDE_DEGREES,
            MAX_LATITUDE_DEGREES
        )
        val longitudeDegrees =
            randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES,
                MAX_LONGITUDE_DEGREES
            )
        val height = randomizer.nextDouble(
            MIN_HEIGHT,
            MAX_HEIGHT
        )

        whenever(location.latitude).thenReturn(latitudeDegrees)
//        every { location.latitude }.returns(latitudeDegrees)
        whenever(location.longitude).thenReturn(longitudeDegrees)
//        every { location.longitude }.returns(longitudeDegrees)
        whenever(location.altitude).thenReturn(height)
//        every { location.altitude }.returns(height)

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