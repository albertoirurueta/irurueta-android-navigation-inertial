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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
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
class AccelerometerGyroscopeAndMagnetometerSyncedSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SyncedSensorCollector.OnMeasurementListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var accelerometerAccuracyChangedListener:
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.OnAccelerometerAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var gyroscopeAccuracyChangedListener:
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.OnGyroscopeAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var magnetometerAccuracyChangedListener:
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.OnMagnetometerAccuracyChangedListener

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SyncedSensorCollector.DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            collector.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            collector.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, collector.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            collector.primarySensor
        )
        assertTrue(collector.interpolationEnabled)
        assertNull(collector.measurementListener)
        assertNull(collector.accelerometerAccuracyChangedListener)
        assertNull(collector.gyroscopeAccuracyChangedListener)
        assertNull(collector.magnetometerAccuracyChangedListener)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(sensors.contains(magnetometerSensor))
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            DEFAULT_WINDOW_NANOSECONDS,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.FASTEST,
            SensorDelay.GAME,
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            false,
            measurementListener,
            accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            collector.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.gyroscopeSensorDelay)
        assertEquals(SensorDelay.GAME, collector.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            collector.primarySensor
        )
        assertFalse(collector.interpolationEnabled)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(
            accelerometerAccuracyChangedListener,
            collector.accelerometerAccuracyChangedListener
        )
        assertSame(gyroscopeAccuracyChangedListener, collector.gyroscopeAccuracyChangedListener)
        assertSame(
            magnetometerAccuracyChangedListener,
            collector.magnetometerAccuracyChangedListener
        )
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(sensors.contains(magnetometerSensor))
        assertTrue(collector.sensorsAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.running)
        assertEquals(0L, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        // check default value
        assertNull(collector.gyroscopeAccuracyChangedListener)

        // set new value
        collector.gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener

        // check
        assertSame(
            gyroscopeAccuracyChangedListener,
            collector.gyroscopeAccuracyChangedListener
        )
    }

    @Test
    fun magnetometerAccuracyChangedListener_setsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        // check default value
        assertNull(collector.magnetometerAccuracyChangedListener)

        // set new value
        collector.magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener

        // check
        assertSame(
            magnetometerAccuracyChangedListener,
            collector.magnetometerAccuracyChangedListener
        )
    }

    @Test
    fun accelerometerSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
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

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
    }

    @Test
    fun magnetometerSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.magnetometerSensor)
    }

    @Test
    fun magnetometerSensor_whenSensorTypeMagnetometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        assertSame(magnetometerSensor, collector.magnetometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }
    }

    @Test
    fun magnetometerSensor_whenSensorTypeMagnetometerUncalibrated_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertSame(magnetometerSensor, collector.magnetometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
    }

    @Test
    fun accelerometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.magnetometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.magnetometerSensorAvailable)
    }

    @Test
    fun primarySensorType_whenAccelerometerUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenAccelerometer_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER.value, result)
    }

    @Test
    fun primarySensorType_whenGyroscopeUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenGyroscope_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE.value, result)
    }

    @Test
    fun primarySensorType_whenMagnetometerUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenMagnetometer_returnsExpectedResult() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(MagnetometerSensorType.MAGNETOMETER.value, result)
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(sensors.contains(magnetometerSensor))
    }

    @Test
    fun sensors_whenNotAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoAccelerometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoGyroscopeAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoMagnetometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun getSensorDelayFor_whenAccelerometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", accelerometerSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenGyroscopeSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", gyroscopeSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenMagnetometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", magnetometerSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun sensorsAvailable_whenAllSensorsAvailable_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoAccelerometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoGyroscopeSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoMagnetometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = null

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        setUpGyroscopeEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        val gyroscopeResult = result as GyroscopeSensorMeasurement
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            gyroscopeResult.sensorType
        )
    }

    @Test
    fun createSensorMeasurement_whenMagnetometerSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        setUpMagnetometerEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        val magnetometerResult = result as MagnetometerSensorMeasurement
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            magnetometerResult.sensorType
        )
    }

    @Test
    fun processSyncedSample_whenNoAccelerometerQueue_returnsFalse() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

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
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val timestamp = System.nanoTime()
        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)
    }

    @Test
    fun processSyncedSample_whenNoMagnetometerQueue_returnsFalse() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
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
    fun processSyncedSample_whenInterpolationEnabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        val interpolatedAccelerometerMeasurement = accelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = timestamp
        val interpolatedGyroscopeMeasurement = gyroscopeMeasurement.copy()
        interpolatedGyroscopeMeasurement.timestamp = timestamp
        val interpolatedMagnetometerMeasurement = magnetometerMeasurement.copy()
        interpolatedMagnetometerMeasurement.timestamp = timestamp

        assertEquals(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

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
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

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
        assertFalse(result)

        val measurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processAccuracyChanges_whenNoSensor_desNotCallAnyListener() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        collector.callPrivateFunc("processAccuracyChanged", null, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedAccuracy_desNotCallAnyListener() {
        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        collector.callPrivateFunc("processAccuracyChanged", accelerometerSensor, -1)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        val unknownSensor = mockk<Sensor>()
        collector.callPrivateFunc(
            "processAccuracyChanged",
            unknownSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            accelerometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            gyroscopeSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            magnetometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify(exactly = 1) {
            magnetometerAccuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenMagnetometerSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
        )

        collector.callPrivateFunc(
            "processAccuracyChanged",
            magnetometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val magnetometerMeasurement = createMagnetometerMeasurement()
        magnetometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        accelerometerQueue.add(accelerometerMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)

        setUpAccelerometerEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertNotEquals(accelerometerMeasurement, accelerometerQueue.first())
        assertTrue(gyroscopeQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        prevGyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        // new primary measurement is stored in buffer
        val primaryGyroscopeMeasurement = gyroscopeQueue.first() as GyroscopeSensorMeasurement
        // and does not match previous measurement
        assertNotEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())
        assertEquals(1, magnetometerQueue.size)
        assertTrue(magnetometerQueue.first() is MagnetometerSensorMeasurement)
        assertEquals(prevMagnetometerMeasurement, magnetometerQueue.first())

        val slot = slot<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>()
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        prevGyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        // new primary measurement is stored in buffer
        // and does not match previous measurement
        assertNotEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())
        assertEquals(1, magnetometerQueue.size)
        assertTrue(magnetometerQueue.first() is MagnetometerSensorMeasurement)
        assertEquals(prevMagnetometerMeasurement, magnetometerQueue.first())

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        assertTrue(accelerometerQueue.isEmpty())
        assertTrue(gyroscopeQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertTrue(accelerometerQueue.isEmpty())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        assertTrue(magnetometerQueue.isEmpty())

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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
        verify { magnetometerAccuracyChangedListener wasNot Called }
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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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
        verify { magnetometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(magnetometerSensor, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
        verify { gyroscopeAccuracyChangedListener wasNot Called }
        verify(exactly = 1) {
            magnetometerAccuracyChangedListener.onAccuracyChanged(
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

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                null
            )

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                false
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            val slot3 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot3),
                    magnetometerSensor,
                    collector.magnetometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            assertSame(slot2.captured, slot3.captured)
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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(false)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            val slot3 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot3),
                    magnetometerSensor,
                    collector.magnetometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            assertSame(slot2.captured, slot3.captured)
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
    fun start_whenMagnetometerRegisterListenerFails_returnsFalse() {
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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(false)

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            val slot3 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot3),
                    magnetometerSensor,
                    collector.magnetometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            assertSame(slot2.captured, slot3.captured)
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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            val slot3 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot3),
                    magnetometerSensor,
                    collector.magnetometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            assertSame(slot2.captured, slot3.captured)
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
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            val slot3 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot3),
                    magnetometerSensor,
                    collector.magnetometerSensorDelay.value
                )
            }

            assertSame(slot1.captured, slot2.captured)
            assertSame(slot2.captured, slot3.captured)
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

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val collector = AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        val slot3 = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManager.unregisterListener(capture(slot3), magnetometerSensor) }

        assertSame(slot1.captured, slot2.captured)
        assertSame(slot2.captured, slot3.captured)

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

    private fun setUpMagnetometerEvent() {
        every { magnetometerSensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        event.sensor = magnetometerSensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, hardIronX, hardIronY, hardIronZ)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW
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

        private fun createMagnetometerMeasurement(): MagnetometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val hardIronX = randomizer.nextFloat()
            val hardIronY = randomizer.nextFloat()
            val hardIronZ = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.HIGH,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }
    }
}