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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeEstimator2Test {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, estimator.attitudeSensorType)
        assertTrue(estimator.startOffsetEnabled)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val attitudeAvailableListener = mockk<AttitudeEstimator2.OnAttitudeAvailableListener>()
        val accuracyChangedListener = mockk<AttitudeEstimator2.OnAccuracyChangedListener>()
        val estimator = AttitudeEstimator2(
            context,
            SensorDelay.FASTEST,
            AttitudeSensorType.RELATIVE_ATTITUDE,
            startOffsetEnabled = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener,
            accuracyChangedListener = accuracyChangedListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, estimator.attitudeSensorType)
        assertFalse(estimator.startOffsetEnabled)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertFalse(estimator.running)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        val attitudeAvailableListener = mockk<AttitudeEstimator2.OnAttitudeAvailableListener>()
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<AttitudeEstimator2.OnAccuracyChangedListener>()
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        // start
        estimator.start()
    }

    @Test
    fun start_whenNotRunning_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // setup spy
        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.start(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)


        val startTimestamp = System.nanoTime()
        assertTrue(estimator.start(startTimestamp))

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(startTimestamp) }
    }

    @Test
    fun stop_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        // setup spy
        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        justRun { attitudeSensorCollectorSpy.stop() }
        estimator.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        // set as running
        estimator.setPrivateProperty("running", true)


        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { attitudeSensorCollectorSpy.stop() }
    }

    @Test
    fun attitudeSensorCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(context)

        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(attitudeSensorCollector, null)
    }

    @Test
    fun attitudeSensorCollector_whenAccuracyChangedAndListener_notifies() {
        val accuracyChangedListener =
            mockk<AttitudeEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AttitudeEstimator2(context, accuracyChangedListener = accuracyChangedListener)

        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(attitudeSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementAndNoListener_processesMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false
        )

        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.measurementListener
        requireNotNull(listener)

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)

        // setup spy
        val attitudeProcessor: AttitudeProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check default value
        assertEquals(Quaternion(), attitude)

        // call listener
        val measurementAttitude = getAttitude()
        val randomizer = UniformRandomizer()
        val headingAccuracy =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_ANGLE_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeSensorMeasurement(
                measurementAttitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH
            )
        listener.onMeasurement(attitudeSensorCollector, measurement)

        // check
        assertEquals(
            ENUtoNEDConverter.conversionRotation.multiplyAndReturnNew(
                ENUtoNEDConverter.conversionRotation.multiplyAndReturnNew(
                    measurementAttitude.inverseAndReturnNew()
                ).inverseAndReturnNew()
            ), attitude
        )

        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        assertEquals(
            CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        assertArrayEquals(DoubleArray(Quaternion.N_ANGLES), eulerAngles, 0.0)
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementListenerCoordinateTransformationAndEulerAnglesDisabled_notifies() {
        val attitudeAvailableListener =
            mockk<AttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.measurementListener
        requireNotNull(listener)

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)

        // setup spy
        val attitudeProcessor: AttitudeProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check default value
        assertEquals(Quaternion(), attitude)

        // call listener
        val measurementAttitude = getAttitude()
        val randomizer = UniformRandomizer()
        val headingAccuracy =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_ANGLE_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeSensorMeasurement(
                measurementAttitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH
            )
        listener.onMeasurement(attitudeSensorCollector, measurement)

        // check
        val expectedAttitude =
            ENUtoNEDConverter.conversionRotation.multiplyAndReturnNew(
                ENUtoNEDConverter.conversionRotation.multiplyAndReturnNew(
                    measurementAttitude.inverseAndReturnNew()
                ).inverseAndReturnNew()
            )
        assertEquals(expectedAttitude, attitude)

        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        assertEquals(
            CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        assertArrayEquals(DoubleArray(Quaternion.N_ANGLES), eulerAngles, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                expectedAttitude,
                timestamp,
                headingAccuracy,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementListenerCoordinateTransformationAndEulerAnglesEnabled_notifies() {
        val attitudeAvailableListener =
            mockk<AttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val attitudeSensorCollector: AttitudeSensorCollector2? =
            estimator.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.measurementListener
        requireNotNull(listener)

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)

        // setup spy
        val attitudeProcessor: AttitudeProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check default value
        assertEquals(Quaternion(), attitude)

        // call listener
        val measurementAttitude = getAttitude()
        val randomizer = UniformRandomizer()
        val headingAccuracy =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_ANGLE_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeSensorMeasurement(
                measurementAttitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH
            )
        listener.onMeasurement(attitudeSensorCollector, measurement)

        // check
        val expectedAttitude = ENUtoNEDConverter.convertAndReturnNew(measurementAttitude)
        assertEquals(expectedAttitude, attitude)

        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }

        val expectedCoordinateTransformation = CoordinateTransformation(
            expectedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        assertEquals(expectedCoordinateTransformation, coordinateTransformation)

        val expectedEulerAngles = expectedAttitude.toEulerAngles()
        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        assertArrayEquals(expectedEulerAngles, eulerAngles, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                expectedAttitude,
                timestamp,
                headingAccuracy,
                expectedEulerAngles[0],
                expectedEulerAngles[1],
                expectedEulerAngles[2],
                expectedCoordinateTransformation
            )
        }
    }

    private companion object {
        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}