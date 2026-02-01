/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.slot
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test

class AccelerometerSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            sensor
        )

        val collector = AccelerometerSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, collector.sensorType)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.measurementListener)
        assertSame(sensor, collector.sensor)
        assertTrue(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            sensor
        )

        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            accuracyChangedListener,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, collector.sensorType)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(sensor, collector.sensor)
        assertTrue(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val collector = AccelerometerSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val collector = AccelerometerSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun sensor_whenSensorTypeAccelerometer_returnsExpectedValue() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }.returns(sensor)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector =
            AccelerometerSensorCollector(context, AccelerometerSensorType.ACCELEROMETER)

        assertSame(sensor, collector.sensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Test
    fun sensor_whenSensorTypeAccelerometerUncalibrated_returnsExpectedValue() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)

        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(sensor, collector.sensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun sensorAvailable_whenSensorNotAvailable_returnsFalse() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(null)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector =
            AccelerometerSensorCollector(context, AccelerometerSensorType.ACCELEROMETER)

        assertFalse(collector.sensorAvailable)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeAccelerometerAvailable_returnsTrue() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensor)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector =
            AccelerometerSensorCollector(context, AccelerometerSensorType.ACCELEROMETER)

        assertTrue(collector.sensorAvailable)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeAccelerometerUncalibrated_returnsTrue() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun start_whenRunning_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val collector = AccelerometerSensorCollector(context)

            collector.setPrivateProperty("running", true)
            assertTrue(collector.running)

            assertFalse(collector.start())

            verify { context wasNot Called }
        }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

            val collector = AccelerometerSensorCollector(context)
            assertEquals(0L, collector.startTimestamp)
            assertFalse(collector.start())

            verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
                .returns(null)
            every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

            val collector = AccelerometerSensorCollector(context)
            assertEquals(0L, collector.startTimestamp)
            assertFalse(collector.start())

            verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
            verify(exactly = 0) {
                sensorManager.registerListener(
                    any(),
                    any<Sensor>(),
                    collector.sensorDelay.value
                )
            }

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
                .returns(sensor)
            every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(false)
            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

            val collector = AccelerometerSensorCollector(context)
            assertEquals(0L, collector.startTimestamp)
            assertFalse(collector.start())

            verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
            val slot = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot),
                    sensor,
                    collector.sensorDelay.value
                )
            }

            val eventListener = slot.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertFalse(collector.running)
            assertNotEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenRegisterListenerSucceeds_registersListener() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
                .returns(sensor)
            every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

            val collector = AccelerometerSensorCollector(context)
            assertEquals(0L, collector.startTimestamp)
            assertTrue(collector.start())

            verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
            val slot = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot),
                    sensor,
                    collector.sensorDelay.value
                )
            }

            val eventListener = slot.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertTrue(collector.running)
            assertNotEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenStartTimestampProvided_registersListenerAndSetsStartTimestamp() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector = AccelerometerSensorCollector(context)
        assertEquals(0L, collector.startTimestamp)
        val startTimestamp = System.nanoTime()
        assertTrue(collector.start(startTimestamp))

        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManager.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)

        assertTrue(collector.running)
        assertEquals(startTimestamp, collector.startTimestamp)
    }

    @Test
    fun stop_whenNoSensorManager_resetsParameters() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerSensorCollector(context)

        // set initial values
        val randomizer = UniformRandomizer()

        val numberOfProcessedMeasurements = randomizer.nextLong()
        setPrivateProperty(
            SensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements
        )

        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorCollector::class,
            collector,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        setPrivateProperty(SensorCollector::class, collector, "running", true)

        assertEquals(numberOfProcessedMeasurements, collector.numberOfProcessedMeasurements)
        assertEquals(mostRecentTimestamp, collector.mostRecentTimestamp)
        assertTrue(collector.running)

        collector.stop()

        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertFalse(collector.running)
    }

    @Test
    fun stop_whenSensorManager_unregistersListenerAndResetsParameters() {
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector = AccelerometerSensorCollector(context)

        // set initial values
        val randomizer = UniformRandomizer()

        val numberOfProcessedMeasurements = randomizer.nextLong()
        setPrivateProperty(
            SensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements
        )

        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorCollector::class,
            collector,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        setPrivateProperty(SensorCollector::class, collector, "running", true)

        assertEquals(numberOfProcessedMeasurements, collector.numberOfProcessedMeasurements)
        assertEquals(mostRecentTimestamp, collector.mostRecentTimestamp)
        assertTrue(collector.running)

        collector.stop()

        // check that values have been reset
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertFalse(collector.running)

        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManager.unregisterListener(capture(slot), sensor) }

        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val collector =
            AccelerometerSensorCollector(context, measurementListener = measurementListener)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(null)

        // check
        verify { measurementListener wasNot Called }

        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector = AccelerometerSensorCollector(
            context,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        event.sensor = sensor
        sensorEventListener.onSensorChanged(event)

        // check
        verify { measurementListener wasNot Called }

        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun onSensorChanged_whenValidSensor_notifiesMeasurement() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)

        val collector = AccelerometerSensorCollector(
            context,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // set start time
        val startTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorCollector::class,
            collector,
            "startTimestamp",
            startTimestamp
        )

        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        val slot = slot<AccelerometerSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val measurement = slot.captured
        assertEquals(1.0f, measurement.ax, 0.0f)
        assertEquals(2.0f, measurement.ay, 0.0f)
        assertEquals(3.0f, measurement.az, 0.0f)
        assertEquals(4.0f, measurement.bx)
        assertEquals(5.0f, measurement.by)
        assertEquals(6.0f, measurement.bz)
        assertEquals(event.timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)

        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertEquals(measurement.timestamp, collector.mostRecentTimestamp)
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val collector = AccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenUnsupportedSensorType_makesNoAction() {
        val collector = AccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_executes() {
        val collector = AccelerometerSensorCollector(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val collector = AccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
    }
}