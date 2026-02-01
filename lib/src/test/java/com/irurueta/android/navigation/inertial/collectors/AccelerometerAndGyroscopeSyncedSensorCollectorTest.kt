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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.testutils.callPrivateFunc
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
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
class AccelerometerAndGyroscopeSyncedSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SyncedSensorCollector.OnMeasurementListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSyncedSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var accelerometerAccuracyChangedListener:
            AccelerometerAndGyroscopeSyncedSensorCollector.OnAccelerometerAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var gyroscopeAccuracyChangedListener:
            AccelerometerAndGyroscopeSyncedSensorCollector.OnGyroscopeAccuracyChangedListener

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

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
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SyncedSensorCollector.DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            collector.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, collector.gyroscopeSensorType)
        assertEquals(SensorDelay.FASTEST, collector.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.gyroscopeSensorDelay)
        assertEquals(
            AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            collector.primarySensor
        )
        assertTrue(collector.interpolationEnabled)
        assertNull(collector.measurementListener)
        assertNull(collector.accelerometerAccuracyChangedListener)
        assertNull(collector.gyroscopeAccuracyChangedListener)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(collector.sensorsAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            DEFAULT_WINDOW_NANOSECONDS,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            false,
            measurementListener,
            accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, collector.accelerometerSensorType)
        assertEquals(GyroscopeSensorType.GYROSCOPE, collector.gyroscopeSensorType)
        assertEquals(SensorDelay.NORMAL, collector.accelerometerSensorDelay)
        assertEquals(SensorDelay.GAME, collector.gyroscopeSensorDelay)
        assertEquals(
            AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            collector.primarySensor
        )
        assertFalse(collector.interpolationEnabled)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(
            accelerometerAccuracyChangedListener,
            collector.accelerometerAccuracyChangedListener
        )
        assertSame(gyroscopeAccuracyChangedListener, collector.gyroscopeAccuracyChangedListener)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(collector.sensorsAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accelerometerAccuracyChangedListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        // check default value
        assertNull(collector.accelerometerAccuracyChangedListener)

        // set new value
        collector.accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener

        // check
        assertSame(
            accelerometerAccuracyChangedListener,
            collector.accelerometerAccuracyChangedListener
        )
    }

    @Test
    fun gyroscopeAccuracyChangedListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        // check default value
        assertNull(collector.gyroscopeAccuracyChangedListener)

        // set new value
        collector.gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
    }

    @Test
    fun accelerometerSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertNull(collector.accelerometerSensor)
    }

    @Test
    fun accelerometerSensor_whenSensorTypeAccelerometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun gyroscopeSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertNull(collector.gyroscopeSensor)
    }

    @Test
    fun gyroscopeSensor_whenSensorTypeGyroscope_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
    }

    @Test
    fun gyroscopeSensor_whenSensorTypeGyroscopeUncalibrated_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
    }

    @Test
    fun accelerometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertFalse(collector.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertFalse(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertTrue(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun primarySensorType_whenAccelerometerUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenAccelerometer_returnsExpectedResult() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER.value, result)
    }

    @Test
    fun primarySensorType_whenGyroscopeUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenGyroscopeCalibrated_returnsExpectedResult() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE.value, result)
    }

    @Test
    fun sensors_whenAvailable_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
    }

    @Test
    fun sensors_whenNotAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoAccelerometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoGyroscopeAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun getSensorDelayFor_whenAccelerometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", accelerometerSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenGyroscopeSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorDelay = SensorDelay.NORMAL
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)

        val result: Int? = collector.callPrivateFuncWithResult("getSensorDelayFor", gyroscopeSensor)
        assertEquals(SensorDelay.NORMAL.value, result)
    }

    @Test
    fun sensorsAvailable_whenAllSensorsAvailable_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertTrue(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoAccelerometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoGyroscopeSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun createSensorMeasurement_whenNoEvent_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", null)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenNoEventSensor_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        event.sensor = null

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenUnsupportedSensorEventType_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenAccelerometerSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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
    fun createSensorMeasurement_whenGyroscopeSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        setUpGyroscopeEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        val gyroscopeResult = result as GyroscopeSensorMeasurement
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, gyroscopeResult.sensorType)
    }

    @Test
    fun processSyncedSample_whenNoAccelerometerQueue_returnsFalse() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val timestamp = System.nanoTime()
        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)
    }

    @Test
    fun processSyncedSample_whenNoGyroscopeQueue_returnsFalse() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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
    fun processSyncedSample_whenInterpolationEnabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        val interpolatedAccelerometerMeasurement = accelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = timestamp
        val interpolatedGyroscopeMeasurement = gyroscopeMeasurement.copy()
        interpolatedGyroscopeMeasurement.timestamp = timestamp

        assertEquals(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processAccuracyChanges_whenNoSensor_desNotCallAnyListener() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        collector.callPrivateFunc("processAccuracyChanged", null, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedAccuracy_desNotCallAnyListener() {
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        collector.callPrivateFunc("processAccuracyChanged", accelerometerSensor, -1)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedSensor_desNotCallAnyListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        val unknownSensor = mockk<Sensor>()
        collector.callPrivateFunc(
            "processAccuracyChanged",
            unknownSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            accelerometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accelerometerAccuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenAccelerometerSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            accelerometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenGyroscopeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            gyroscopeSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify(exactly = 1) {
            gyroscopeAccuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenGyroscopeSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            gyroscopeSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        // save initial measurements in buffer that will be trimmed
        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)

        setUpAccelerometerEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertNotEquals(accelerometerMeasurement, accelerometerQueue.first())
        assertTrue(gyroscopeQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndBufferNotEmpty_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        prevGyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        // new primary measurement is stored in buffer
        val primaryGyroscopeMeasurement = gyroscopeQueue.first() as GyroscopeSensorMeasurement
        // and does not match previous measurement
        assertNotEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())

        val slot = slot<AccelerometerAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val syncedMeasurement = slot.captured

        val interpolatedAccelerometerMeasurement = prevAccelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = primaryGyroscopeMeasurement.timestamp
        assertEquals(
            interpolatedAccelerometerMeasurement,
            syncedMeasurement.accelerometerMeasurement
        )
        assertEquals(primaryGyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventBufferNotEmptyAndNoListener_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        prevGyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        // new primary measurement is stored in buffer
        // and does not match previous measurement
        assertNotEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndEmptyBuffer_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            primarySensor = AccelerometerAndGyroscopeSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue

        assertTrue(accelerometerQueue.isEmpty())
        assertTrue(gyroscopeQueue.isEmpty())

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(2, buffer.size)
        assertTrue(accelerometerQueue.isEmpty())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(accelerometerSensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accelerometerAccuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenGyroscopeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(gyroscopeSensor, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify(exactly = 1) {
            gyroscopeAccuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun start_whenRunning_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                null
            )
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                null
            )

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNull(collector.sensors)

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenAccelerometerRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                false
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    gyroscopeSensor,
                    collector.gyroscopeSensorDelay.value
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
    fun start_whenGyroscopeRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(false)

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    gyroscopeSensor,
                    collector.gyroscopeSensorDelay.value
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
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)

            val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

            assertTrue(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    accelerometerSensor,
                    collector.accelerometerSensorDelay.value
                )
            }
            val slot2 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot2),
                    gyroscopeSensor,
                    collector.gyroscopeSensorDelay.value
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
    fun start_whenStartTimestampProvided_setsProvidedTimestamp() {
        val startTimestamp = System.nanoTime()

        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
            true
        )
        every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

        assertTrue(collector.start(startTimestamp))
        assertNotNull(collector.sensors)

        val slot1 = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManager.registerListener(
                capture(slot1),
                accelerometerSensor,
                collector.accelerometerSensorDelay.value
            )
        }
        val slot2 = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManager.registerListener(
                capture(slot2),
                gyroscopeSensor,
                collector.gyroscopeSensorDelay.value
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

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(context)

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
                capture(slot1),
                accelerometerSensor
            )
        }
        val slot2 = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManager.unregisterListener(capture(slot2), gyroscopeSensor) }

        assertSame(slot1.captured, slot2.captured)

        val eventListener = slot1.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        assertSame(sensorEventListener, eventListener)
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

    private fun setUpGyroscopeEvent() {
        every { gyroscopeSensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        event.sensor = gyroscopeSensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(wx, wy, wz, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
    }

    private companion object {
        const val DEFAULT_WINDOW_NANOSECONDS = 1000000L

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

        private fun createGyroscopeMeasurement(): GyroscopeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val wx = randomizer.nextFloat()
            val wy = randomizer.nextFloat()
            val wz = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH,
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }
    }
}