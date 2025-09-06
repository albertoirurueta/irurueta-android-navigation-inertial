/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.os.SystemClock
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.statistics.UniformRandomizer
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
import org.mockito.Spy
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class RelativeGyroscopeAttitudeEstimatorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var attitudeAvailableListener:
            RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener

//    @MockK
    @Mock
    private lateinit var gyroscopeMeasurementListener:
            GyroscopeSensorCollector.OnMeasurementListener

//    @MockK
    @Mock
    private lateinit var display: Display

    @Spy
    private val context = ApplicationProvider.getApplicationContext<Context>()

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener,
            gyroscopeMeasurementListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        estimator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
    }

    @Test
    fun averageTimeInterval_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        doReturn(averageTimeInterval).whenever(timeIntervalEstimatorSpy).averageTimeInterval
//        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(averageTimeInterval)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertEquals(averageTimeInterval, estimator.averageTimeInterval, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "running",
            true
        )
        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunning_resetsAndStarts() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        doReturn(true).whenever(gyroscopeSensorCollectorSpy).start()
//        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start())

        verify(timeIntervalEstimatorSpy, only()).reset()
//        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(attitudeSpy, times(1)).a = 1.0
//        verify(exactly = 1) { attitudeSpy.a = 1.0 }
        verify(attitudeSpy, times(1)).b = 0.0
//        verify(exactly = 1) { attitudeSpy.b = 0.0 }
        verify(attitudeSpy, times(1)).c = 0.0
//        verify(exactly = 1) { attitudeSpy.c = 0.0 }
        verify(attitudeSpy, times(1)).d = 0.0
//        verify(exactly = 1) { attitudeSpy.d = 0.0 }
        verify(attitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { attitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).a = 1.0
//        verify(exactly = 1) { internalAttitudeSpy.a = 1.0 }
        verify(internalAttitudeSpy, times(1)).b = 0.0
//        verify(exactly = 1) { internalAttitudeSpy.b = 0.0 }
        verify(internalAttitudeSpy, times(1)).c = 0.0
//        verify(exactly = 1) { internalAttitudeSpy.c = 0.0 }
        verify(internalAttitudeSpy, times(1)).d = 0.0
//        verify(exactly = 1) { internalAttitudeSpy.d = 0.0 }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(gyroscopeSensorCollectorSpy, times(1)).start()
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start() }

        assertTrue(estimator.running)
    }

    @Test
    fun stop_stopsGyroscopeSensor() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "running",
            true
        )
        assertTrue(estimator.running)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        estimator.stop()

        verify(gyroscopeSensorCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun onGyroscopeMeasurement_whenNoProcessedSample_setsInitialTimestamp() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(0).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verifyNoInteractions(coordinateTransformationSpy)
//        verify { coordinateTransformationSpy wasNot Called }
        verify(attitudeSpy, times(1)).toEulerAngles(eulerAngles)
//        verify(exactly = 1) { attitudeSpy.toEulerAngles(eulerAngles) }
    }

    @Test
    fun onGyroscopeMeasurement_whenProcessedSample_keepsInitialTimestamp() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)

        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp",
            1L
        )
        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(1L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(1).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(1)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector"
            )
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(initialTimestamp1, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verifyNoInteractions(coordinateTransformationSpy)
//        verify { coordinateTransformationSpy wasNot Called }
        verify(attitudeSpy, times(1)).toEulerAngles(eulerAngles)
//        verify(exactly = 1) { attitudeSpy.toEulerAngles(eulerAngles) }
    }

    @Test
    fun onGyroscopeMeasurement_whenListenerEstimateCoordinateTransformationAndEstimateDisplayEulerAnglesDisabled_notifiesWithoutSuchParameters() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertFalse(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(0).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verifyNoInteractions(coordinateTransformationSpy)
//        verify { coordinateTransformationSpy wasNot Called }
        verify(attitudeSpy, never()).toEulerAngles(eulerAngles)
//        verify(exactly = 0) { attitudeSpy.toEulerAngles(eulerAngles) }

        verify(attitudeAvailableListener, only()).onAttitudeAvailable(
            estimator,
            attitudeSpy,
            timestamp,
            null,
            null,
            null,
            null
        )
/*        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                null,
                null,
                null,
                null
            )
        }*/
    }

    @Test
    fun onGyroscopeMeasurement_whenListenerEstimateCoordinateTransformationAndEstimateDisplayEulerAnglesEnabled_notifiesWithSuchParameters() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertTrue(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(0).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(coordinateTransformationSpy, only()).fromRotation(attitudeSpy)
//        verify(exactly = 1) { coordinateTransformationSpy.fromRotation(attitudeSpy) }
        verify(attitudeSpy, times(1)).toEulerAngles(eulerAngles)
//        verify(exactly = 1) { attitudeSpy.toEulerAngles(eulerAngles) }

        verify(attitudeAvailableListener, only()).onAttitudeAvailable(
            estimator,
            attitudeSpy,
            timestamp,
            eulerAngles[0],
            eulerAngles[1],
            eulerAngles[2],
            coordinateTransformationSpy
        )
/*        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformationSpy
            )
        }*/
    }

    @Test
    fun onGyroscopeMeasurement_whenBiases_notifiesWithSuchParameters() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertTrue(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(0).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector"
            )
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(coordinateTransformationSpy, only()).fromRotation(attitudeSpy)
//        verify(exactly = 1) { coordinateTransformationSpy.fromRotation(attitudeSpy) }
        verify(attitudeSpy, times(1)).toEulerAngles(eulerAngles)
//        verify(exactly = 1) { attitudeSpy.toEulerAngles(eulerAngles) }

        verify(attitudeAvailableListener, only()).onAttitudeAvailable(
            estimator,
            attitudeSpy,
            timestamp,
            eulerAngles[0],
            eulerAngles[1],
            eulerAngles[2],
            coordinateTransformationSpy
        )
/*        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformationSpy
            )
        }*/
    }

    @Test
    fun onGyroscopeMeasurement_whenListenerIgnoreDisplayOrientation_notifiesWithoutSuchParameters() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
//        val context = spyk(ApplicationProvider.getApplicationContext())
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertFalse(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        doReturn(0).whenever(timeIntervalEstimatorSpy).numberOfProcessedSamples
//        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val deltaAttitude: Quaternion? = estimator.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spy(deltaAttitude)
//        val deltaAttitudeSpy = spyk(deltaAttitude)
        estimator.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spy(internalAttitude)
//        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spy(attitude)
//        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "coordinateTransformation"
            )
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spy(coordinateTransformation)
//        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        verify(timeIntervalEstimatorSpy, times(1)).numberOfProcessedSamples
//        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(timeIntervalEstimatorSpy, times(1)).addTimestamp(any<Double>())
//        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(timeIntervalEstimatorSpy, times(1)).averageTimeInterval
//        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(deltaAttitudeSpy, times(1)).setFromEulerAngles(any(), any(), any())
//        verify(exactly = 1) { deltaAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(internalAttitudeSpy, times(1)).combine(deltaAttitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(internalAttitudeSpy, times(1)).normalize()
//        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(internalAttitudeSpy, times(1)).copyTo(attitudeSpy)
//        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verifyNoInteractions(coordinateTransformationSpy)
//        verify { coordinateTransformationSpy wasNot Called }
        verify(attitudeSpy, never()).toEulerAngles(eulerAngles)
//        verify(exactly = 0) { attitudeSpy.toEulerAngles(eulerAngles) }

        verify(attitudeAvailableListener, only()).onAttitudeAvailable(
            estimator,
            attitudeSpy,
            timestamp,
            null,
            null,
            null,
            null
        )
/*        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                null,
                null,
                null,
                null
            )
        }*/
    }
}