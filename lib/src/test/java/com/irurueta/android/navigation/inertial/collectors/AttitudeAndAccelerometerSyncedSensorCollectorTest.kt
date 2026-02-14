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

package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeAndAccelerometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.testutils.callPrivateFunc
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockk
import io.mockk.mockkStatic
import io.mockk.slot
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import java.util.LinkedList
import java.util.Queue

@Suppress("UNCHECKED_CAST")
class AttitudeAndAccelerometerSyncedSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SyncedSensorCollector.OnMeasurementListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSyncedSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SyncedSensorCollector.OnAccuracyChangedListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSyncedSensorCollector>

    @MockK
    private lateinit var attitudeSensor: Sensor

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SyncedSensorCollector.DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            collector.attitudeSensorType
        )
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            collector.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, collector.attitudeSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.accelerometerSensorDelay)
        assertEquals(
            AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            collector.primarySensor
        )
        assertTrue(collector.interpolationEnabled)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(attitudeSensor, collector.attitudeSensor)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertTrue(collector.attitudeSensorAvailable)
        assertTrue(collector.accelerometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(attitudeSensor))
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(collector.sensorsAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.RELATIVE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            DEFAULT_WINDOW_NANOSECONDS,
            AttitudeSensorType.RELATIVE_ATTITUDE,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            false,
            measurementListener,
            accuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            AttitudeSensorType.RELATIVE_ATTITUDE,
            collector.attitudeSensorType
        )
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            collector.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.attitudeSensorDelay)
        assertEquals(SensorDelay.GAME, collector.accelerometerSensorDelay)
        assertEquals(
            AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            collector.primarySensor
        )
        assertFalse(collector.interpolationEnabled)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(attitudeSensor, collector.attitudeSensor)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertTrue(collector.attitudeSensorAvailable)
        assertTrue(collector.accelerometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(attitudeSensor))
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(collector.sensorsAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(
            accuracyChangedListener,
            collector.accuracyChangedListener
        )
    }

    @Test
    fun attitudeSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertNull(collector.attitudeSensor)
    }

    @Test
    fun attitudeSensor_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE
        )

        assertSame(attitudeSensor, collector.attitudeSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
    }

    @Test
    fun attitudeSensor_whenSensorTypeRelativeAttitude_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.RELATIVE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            attitudeSensorType = AttitudeSensorType.RELATIVE_ATTITUDE
        )

        assertSame(attitudeSensor, collector.attitudeSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun accelerometerSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertNull(collector.accelerometerSensor)
    }

    @Test
    fun accelerometerSensor_whenSensorTypeAccelerometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Test
    fun accelerometerSensor_whenSensorTypeAccelerometerUncalibrated_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun attitudeSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertFalse(collector.attitudeSensorAvailable)
    }

    @Test
    fun attitudeSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertTrue(collector.attitudeSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertFalse(collector.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @Test
    fun primarySensorType_whenAbsoluteAttitude_returnsExpectedResult() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE.value, result)
    }

    @Test
    fun primarySensorType_whenRelativeAttitude_returnsExpectedResult() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            attitudeSensorType = AttitudeSensorType.RELATIVE_ATTITUDE,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE.value, result)
    }

    @Test
    fun primarySensorType_whenAccelerometerUncalibrated_returnsExpectedResult() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenAccelerometer_returnsExpectedResult() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER.value, result)
    }

    @Test
    fun sensors_whenAvailable_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(attitudeSensor))
        assertTrue(sensors.contains(accelerometerSensor))
    }

    @Test
    fun sensors_whenNotAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoAttitudeAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoAccelerometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun getSensorDelayFor_whenAttitudeSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            attitudeSensorDelay = SensorDelay.GAME
        )

        assertSame(attitudeSensor, collector.attitudeSensor)
        assertSame(accelerometerSensor, collector.accelerometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", attitudeSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenAccelerometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accelerometerSensorDelay = SensorDelay.GAME
        )

        assertSame(attitudeSensor, collector.attitudeSensor)
        assertSame(accelerometerSensor, collector.accelerometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", accelerometerSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun sensorsAvailable_whenAllSensorsAvailable_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertTrue(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoAttitudeSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoAccelerometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun createSensorMeasurement_whenNoEvent_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", null)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenNoEventSensor_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        event.sensor = null

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenUnsupportedSensorEventType_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenAttitudeSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        setUpAttitudeEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        val accelerometerResult = result as AttitudeSensorMeasurement
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            accelerometerResult.sensorType
        )
    }

    @Test
    fun createSensorMeasurement_whenAccelerometerSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        setUpAccelerometerEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        val accelerometerResult = result as AccelerometerSensorMeasurement
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerResult.sensorType
        )
    }

    @Test
    fun processSyncedSample_whenNoAttitudeQueue_returnsFalse() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val timestamp = System.nanoTime()
        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)
    }

    @Test
    fun processSyncedSample_whenNoAccelerometerQueue_returnsFalse() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue

        val timestamp = System.nanoTime()
        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val attitudeMeasurement = createAttitudeMeasurement()
        val accelerometerMeasurement = createAccelerometerMeasurement()
        attitudeQueue.add(attitudeMeasurement)
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        val interpolatedAttitudeMeasurement = attitudeMeasurement.copy()
        interpolatedAttitudeMeasurement.timestamp = timestamp
        val interpolatedAccelerometerMeasurement = accelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = timestamp

        assertEquals(interpolatedAttitudeMeasurement, measurement.attitudeMeasurement)
        assertNotSame(interpolatedAttitudeMeasurement, measurement.attitudeMeasurement)
        assertEquals(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoAttitudeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val attitudeMeasurement = createAttitudeMeasurement()
        attitudeQueue.add(attitudeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val attitudeMeasurement = createAttitudeMeasurement()
        val accelerometerMeasurement = createAccelerometerMeasurement()
        attitudeQueue.add(attitudeMeasurement)
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(attitudeMeasurement, measurement.attitudeMeasurement)
        assertNotSame(attitudeMeasurement, measurement.attitudeMeasurement)
        assertEquals(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoAttitudeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        val attitudeMeasurement = createAttitudeMeasurement()
        attitudeQueue.add(attitudeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AttitudeAndAccelerometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processAccuracyChanges_whenNoSensor_desNotCallAnyListener() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            null,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedAccuracy_desNotCallAnyListener() {
        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            accelerometerSensor,
            -1
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedSensor_desNotCallAnyListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val unknownSensor = mockk<Sensor>()
        every { unknownSensor.type }.returns(-1)
        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            unknownSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenAttitudeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { attitudeSensor.type }.returns(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            attitudeSensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenAttitudeSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { attitudeSensor.type }.returns(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            attitudeSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            accelerometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenAccelerometerSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            accelerometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(null)

        // check
        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenUnsupportedEvent_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNotPrimarySensorEvent_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            measurementListener = measurementListener
        )

        setUpAccelerometerEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        assertEquals(1, buffer.size)
        assertTrue(buffer.containsKey(collector.accelerometerSensorType.value))
        val accelerometerQueue = buffer[collector.accelerometerSensorType.value]
        requireNotNull(accelerometerQueue)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenBufferIsTrimmed_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        // save initial measurements in buffer that will be trimmed
        val attitudeMeasurement = createAttitudeMeasurement()
        attitudeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        attitudeQueue.add(attitudeMeasurement)
        accelerometerQueue.add(accelerometerMeasurement)

        setUpAccelerometerEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertTrue(attitudeQueue.isEmpty())
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertNotEquals(accelerometerMeasurement, accelerometerQueue.first())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndBufferNotEmpty_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAttitudeMeasurement = createAttitudeMeasurement()
        prevAttitudeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        attitudeQueue.add(prevAttitudeMeasurement)
        accelerometerQueue.add(prevAccelerometerMeasurement)

        setUpAttitudeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, attitudeQueue.size)
        assertTrue(attitudeQueue.first() is AttitudeSensorMeasurement)
        // new primary measurement is stored in buffer
        val primaryAttitudeMeasurement = attitudeQueue.first() as AttitudeSensorMeasurement
        // and does not match previous measurement
        assertNotEquals(prevAttitudeMeasurement, attitudeQueue.first())
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())

        val slot = slot<AttitudeAndAccelerometerSyncedSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val syncedMeasurement = slot.captured

        val interpolatedAccelerometerMeasurement = prevAccelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = primaryAttitudeMeasurement.timestamp
        assertEquals(
            primaryAttitudeMeasurement,
            syncedMeasurement.attitudeMeasurement
        )
        assertEquals(
            interpolatedAccelerometerMeasurement,
            syncedMeasurement.accelerometerMeasurement
        )
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventBufferNotEmptyAndNoListener_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAttitudeMeasurement = createAttitudeMeasurement()
        prevAttitudeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        attitudeQueue.add(prevAttitudeMeasurement)
        accelerometerQueue.add(prevAccelerometerMeasurement)

        setUpAttitudeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, attitudeQueue.size)
        assertTrue(attitudeQueue.first() is AttitudeSensorMeasurement)
        // new primary measurement is stored in buffer
        // and does not match previous measurement
        assertNotEquals(prevAttitudeMeasurement, attitudeQueue.first())
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndEmptyBuffer_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            primarySensor = AttitudeAndAccelerometerSyncedSensorCollector.PrimarySensor.ATTITUDE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val attitudeQueue = LinkedList<AttitudeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.attitudeSensorType.value] = attitudeQueue
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue

        assertTrue(attitudeQueue.isEmpty())
        assertTrue(accelerometerQueue.isEmpty())

        setUpAttitudeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, attitudeQueue.size)
        assertTrue(attitudeQueue.first() is AttitudeSensorMeasurement)
        assertTrue(accelerometerQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenAttitudeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { attitudeSensor.type }.returns(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(attitudeSensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(accelerometerSensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun start_whenRunning_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            setPrivateProperty(SyncedSensorCollector::class, collector, "running", true)
            assertTrue(collector.running)

            assertFalse(collector.start())
        }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNull(collector.sensors)

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenNoSensors_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
                null
            )
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                null
            )

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNull(collector.sensors)

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenAttitudeRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
                attitudeSensor
            )
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )

            every { sensorManager.registerListener(any(), attitudeSensor, any()) }.returns(
                false
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    attitudeSensor,
                    collector.attitudeSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            val eventListener = slot1.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertFalse(collector.running)
            assertNotEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenAccelerometerRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
                attitudeSensor
            )
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )

            every { sensorManager.registerListener(any(), attitudeSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                false
            )

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    attitudeSensor,
                    collector.attitudeSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            val eventListener = slot1.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertFalse(collector.running)
            assertNotEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenRegisterListenerSucceedsForAllSensors_returnsTrue() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
                attitudeSensor
            )
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )

            every { sensorManager.registerListener(any(), attitudeSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertTrue(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    attitudeSensor,
                    collector.attitudeSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            val eventListener = slot1.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertTrue(collector.running)
            assertNotEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenStartTimestampProvided_setsProvidedTimestamp() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
                attitudeSensor
            )
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )

            every { sensorManager.registerListener(any(), attitudeSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )

            val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

            assertTrue(collector.start(startTimestamp))
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    attitudeSensor,
                    collector.attitudeSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            val eventListener = slot1.captured

            val sensorEventListener: SensorEventListener? =
                getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
            requireNotNull(sensorEventListener)
            assertSame(sensorEventListener, eventListener)

            assertTrue(collector.running)
            assertEquals(startTimestamp, collector.startTimestamp)
        }
    }

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            1L
        )
        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "mostRecentTimestamp",
            2L
        )
        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "running",
            true
        )

        assertNull(collector.sensors)

        collector.stop()

        assertEquals(1L, collector.numberOfProcessedMeasurements)
        assertEquals(2L, collector.mostRecentTimestamp)
        assertTrue(collector.running)
    }

    @Test
    fun stop_whenSensorManager_unregisterListenersAndResetsParameters() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }.returns(
            attitudeSensor
        )
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val collector = AttitudeAndAccelerometerSyncedSensorCollector(context)

        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            1L
        )
        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "mostRecentTimestamp",
            2L
        )
        setPrivateProperty(
            SyncedSensorCollector::class,
            collector,
            "running",
            true
        )

        assertNotNull(collector.sensors)
        assertEquals(1L, collector.numberOfProcessedMeasurements)
        assertEquals(2L, collector.mostRecentTimestamp)
        assertTrue(collector.running)

        collector.stop()

        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertFalse(collector.running)

        val slot1 = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManager.unregisterListener(
                capture(slot1), attitudeSensor
            )
        }
        val slot2 = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManager.unregisterListener(
                capture(slot2),
                accelerometerSensor
            )
        }

        assertSame(slot1.captured, slot2.captured)

        val eventListener = slot1.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        assertSame(sensorEventListener, eventListener)
    }

    private fun setUpAttitudeEvent() {
        every { attitudeSensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        event.sensor = attitudeSensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat(),
            headingAccuracy
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
    }

    private fun setUpAccelerometerEvent() {
        every { accelerometerSensor.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        event.sensor = accelerometerSensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(ax, ay, az, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
    }

    private companion object {
        const val DEFAULT_WINDOW_NANOSECONDS = 1000000L

        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0

        private fun createAttitudeMeasurement(): AttitudeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val attitude = Quaternion(roll, pitch, yaw)
            val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
            val timestamp = System.nanoTime()

            return AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )
        }

        private fun createAccelerometerMeasurement(): AccelerometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val ax = randomizer.nextFloat()
            val ay = randomizer.nextFloat()
            val az = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }
    }
}