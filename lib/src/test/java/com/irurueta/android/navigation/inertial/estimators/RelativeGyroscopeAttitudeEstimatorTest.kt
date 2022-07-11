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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.os.SystemClock
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.InvalidRotationMatrixException
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class RelativeGyroscopeAttitudeEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, estimator.sensorType)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateDisplayEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>()
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start())

        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { attitudeSpy.a = 1.0 }
        verify(exactly = 1) { attitudeSpy.b = 0.0 }
        verify(exactly = 1) { attitudeSpy.c = 0.0 }
        verify(exactly = 1) { attitudeSpy.d = 0.0 }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.a = 1.0 }
        verify(exactly = 1) { internalAttitudeSpy.b = 0.0 }
        verify(exactly = 1) { internalAttitudeSpy.c = 0.0 }
        verify(exactly = 1) { internalAttitudeSpy.d = 0.0 }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { deltaAttitudeSpy.a = 1.0 }
        verify(exactly = 1) { deltaAttitudeSpy.b = 0.0 }
        verify(exactly = 1) { deltaAttitudeSpy.c = 0.0 }
        verify(exactly = 1) { deltaAttitudeSpy.d = 0.0 }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start() }

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
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        estimator.stop()

        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun onGyroscopeMeasurement_whenNoProcessedSample_setsInitialTimestamp() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify { coordinateTransformationSpy wasNot Called }
        verify { rotationMatrixSpy wasNot Called }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(displayEulerAngles) }
    }

    @Test
    fun onGyroscopeMeasurement_whenProcessedSample_keepsInitialTimestamp() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(1)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify { coordinateTransformationSpy wasNot Called }
        verify { rotationMatrixSpy wasNot Called }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(displayEulerAngles) }
    }

    @Test
    fun onGyroscopeMeasurement_whenListenerEstimateCoordinateTransformationAndEstimateDisplayEulerAnglesDisabled_notifiesWithoutSuchParameters() {
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertFalse(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify { coordinateTransformationSpy wasNot Called }
        verify { rotationMatrixSpy wasNot Called }
        verify(exactly = 0) { attitudeSpy.toEulerAngles(displayEulerAngles) }

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun onGyroscopeMeasurement_whenListenerEstimateCoordinateTransformationAndEstimateDisplayEulerAnglesEnabled_notifiesWithSuchParameters() {
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertTrue(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify(exactly = 1) { attitudeSpy.asInhomogeneousMatrix(rotationMatrixSpy) }
        verify(exactly = 1) { coordinateTransformationSpy.matrix = rotationMatrixSpy }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(displayEulerAngles) }

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                displayEulerAngles[0],
                displayEulerAngles[1],
                displayEulerAngles[2],
                coordinateTransformationSpy
            )
        }
    }

    @Test
    fun onGyroscopeMeasurement_whenBiases_notifiesWithSuchParameters() {
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertTrue(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify(exactly = 1) { attitudeSpy.asInhomogeneousMatrix(rotationMatrixSpy) }
        verify(exactly = 1) { coordinateTransformationSpy.matrix = rotationMatrixSpy }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(displayEulerAngles) }

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                displayEulerAngles[0],
                displayEulerAngles[1],
                displayEulerAngles[2],
                coordinateTransformationSpy
            )
        }
    }

    @Test
    fun onGyroscopeMeasurement_whenInvalidCoordinateTransformationMatrix_notifiesWithNullCoordinateTransformation() {
        val attitudeAvailableListener =
            mockk<RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        assertTrue(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
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
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(0)
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

        val displayOrientation: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation"
        )
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayOrientation",
            displayOrientationSpy
        )

        val deltaAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude"
        )
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "deltaAttitude",
            deltaAttitudeSpy
        )

        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        val internalAttitudeSpy = spyk(internalAttitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "internalAttitude",
            internalAttitudeSpy
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseRelativeGyroscopeAttitudeEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        every {
            coordinateTransformationSpy.matrix = any()
        }.throws(InvalidRotationMatrixException())
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val rotationMatrix: Matrix? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix"
        )
        requireNotNull(rotationMatrix)
        val rotationMatrixSpy = spyk(rotationMatrix)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "rotationMatrix",
            rotationMatrixSpy
        )

        val displayEulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "displayEulerAngles"
        )
        requireNotNull(displayEulerAngles)

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

        verify(exactly = 1) { timeIntervalEstimatorSpy.numberOfProcessedSamples }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, -0.0) }
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromAxisAndRotation(
                any<Double>(),
                any(),
                any(),
                any()
            )
        }
        verify(exactly = 1) { deltaAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.combine(deltaAttitudeSpy) }
        verify(exactly = 1) { internalAttitudeSpy.normalize() }
        verify(exactly = 1) { internalAttitudeSpy.copyTo(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 1) { attitudeSpy.inverse() }
        verify(exactly = 1) { attitudeSpy.normalize() }
        verify(exactly = 1) { attitudeSpy.asInhomogeneousMatrix(rotationMatrixSpy) }
        verify(exactly = 1) { coordinateTransformationSpy.matrix = rotationMatrixSpy }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(displayEulerAngles) }

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                displayEulerAngles[0],
                displayEulerAngles[1],
                displayEulerAngles[2],
                null
            )
        }
    }
}