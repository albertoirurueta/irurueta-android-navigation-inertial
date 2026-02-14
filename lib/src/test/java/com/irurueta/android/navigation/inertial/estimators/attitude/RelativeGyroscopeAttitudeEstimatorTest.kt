/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.processors.attitude.RelativeGyroscopeAttitudeProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.slot
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class RelativeGyroscopeAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeAvailableListener:
            BaseRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener<RelativeGyroscopeAttitudeEstimator>

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            BaseRelativeGyroscopeAttitudeEstimator.OnAccuracyChangedListener<RelativeGyroscopeAttitudeEstimator>

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, estimator.sensorType)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener,
            accuracyChangedListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(GyroscopeSensorType.GYROSCOPE, estimator.sensorType)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertFalse(estimator.running)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun running_returnsExpectedValue() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.running }.returns(true)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector",
            gyroscopeSensorCollectorSpy
        )

        assertTrue(estimator.running)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.running }
    }

    @Test
    fun averageTimeInterval_returnsExpectedValue() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val processor: RelativeGyroscopeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        every { processorSpy.timeIntervalSeconds }.returns(INTERVAL_SECONDS)
        estimator.setPrivateProperty("processor", processorSpy)

        assertEquals(INTERVAL_SECONDS, estimator.timeIntervalSeconds, 0.0)
        verify(exactly = 1) { processorSpy.timeIntervalSeconds }
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsGyroscopeCollector() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
                .returns(gyroscopeSensor)
            every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)

            val estimator = RelativeGyroscopeAttitudeEstimator(context)

            val processor: RelativeGyroscopeAttitudeProcessor? =
                estimator.getPrivateProperty("processor")
            requireNotNull(processor)
            val processorSpy = spyk(processor)
            estimator.setPrivateProperty("processor", processorSpy)

            val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector"
            )
            requireNotNull(gyroscopeSensorCollector)
            val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
            every { gyroscopeSensorCollectorSpy.start() }.returns(true)
            setPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector",
                gyroscopeSensorCollectorSpy
            )

            assertTrue(estimator.start(startTimestamp))
            verify(exactly = 1) { processorSpy.reset() }
            verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(startTimestamp) }
        }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = RelativeGyroscopeAttitudeEstimator(context)

            val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector"
            )
            requireNotNull(gyroscopeSensorCollector)
            val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
            every { gyroscopeSensorCollectorSpy.running }.returns(true)
            setPrivateProperty(
                BaseRelativeGyroscopeAttitudeEstimator::class,
                estimator,
                "gyroscopeSensorCollector",
                gyroscopeSensorCollectorSpy
            )

            assertTrue(estimator.running)
            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }
    }

    @Test
    fun stop_stopsCollector() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector",
            gyroscopeSensorCollectorSpy
        )

        estimator.stop()

        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_makesNoAction() {
        val estimator = RelativeGyroscopeAttitudeEstimator(context)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gyroscopeSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gyroscopeSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onMeasurement_whenNotProcessed_makesNoAction() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val processor: RelativeGyroscopeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        every { processorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("processor", processorSpy)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement =
            GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)
        listener.onMeasurement(gyroscopeSensorCollector, measurement)

        verify(exactly = 1) { processorSpy.process(measurement) }
        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun onMeasurement_whenNoListener_updateAttitude() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false
        )

        val processor: RelativeGyroscopeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        every { processorSpy.process(any(), any()) }.returns(true)

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        every { processorSpy.attitude }.returns(attitude)
        estimator.setPrivateProperty("processor", processorSpy)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement =
            GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)
        listener.onMeasurement(gyroscopeSensorCollector, measurement)

        val attitude2: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude"
        )
        requireNotNull(attitude2)
        assertEquals(attitude, attitude2)

        verify(exactly = 1) { processorSpy.process(measurement) }
    }

    @Test
    fun onMeasurement_whenProcessedNotEstimateCoordinateTransformationAndEulerAngles_updateAttitudeAndNotifies() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val processor: RelativeGyroscopeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        every { processorSpy.process(any(), any()) }.returns(true)

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        every { processorSpy.attitude }.returns(attitude)
        estimator.setPrivateProperty("processor", processorSpy)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement =
            GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)
        listener.onMeasurement(gyroscopeSensorCollector, measurement)

        val slot = slot<Quaternion>()
        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                capture(slot),
                timestamp,
                null,
                null,
                null,
                null
            )
        }

        val capturedAttitude = slot.captured
        assertEquals(attitude, capturedAttitude)

        verify(exactly = 1) { processorSpy.process(measurement) }
    }

    @Test
    fun onMeasurement_whenProcessedEstimateCoordinateTransformationAndEulerAngles_updateAttitudeAndNotifies() {
        val estimator = RelativeGyroscopeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val processor: RelativeGyroscopeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        every { processorSpy.process(any(), any()) }.returns(true)

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val processorAttitude = Quaternion(roll, pitch, yaw)
        every { processorSpy.attitude }.returns(processorAttitude)
        estimator.setPrivateProperty("processor", processorSpy)

        val gyroscopeSensorCollector: GyroscopeSensorCollector? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "gyroscopeSensorCollector"
        )
        requireNotNull(gyroscopeSensorCollector)

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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val attitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude"
        )
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "attitude",
            attitudeSpy
        )

        val eulerAngles: DoubleArray? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeEstimator::class,
            estimator,
            "eulerAngles"
        )
        requireNotNull(eulerAngles)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement =
            GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp, SensorAccuracy.HIGH)
        listener.onMeasurement(gyroscopeSensorCollector, measurement)

        verify(exactly = 1) { attitudeSpy.fromQuaternion(processorAttitude) }
        verify(exactly = 1) { coordinateTransformationSpy.fromRotation(attitudeSpy) }
        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformationSpy
            )
        }
        verify(exactly = 1) { processorSpy.process(measurement) }
    }

    private companion object {
        const val INTERVAL_SECONDS = 0.01

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0
    }
}