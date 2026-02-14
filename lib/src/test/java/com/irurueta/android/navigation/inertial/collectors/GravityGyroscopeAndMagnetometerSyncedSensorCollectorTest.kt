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
import com.irurueta.android.navigation.inertial.collectors.measurements.GravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
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
class GravityGyroscopeAndMagnetometerSyncedSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SyncedSensorCollector.OnMeasurementListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SyncedSensorCollector.OnAccuracyChangedListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorCollector>

    @MockK
    private lateinit var gravitySensor: Sensor

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SyncedSensorCollector.DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            collector.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            collector.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, collector.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.magnetometerSensorDelay)
        assertEquals(
            GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE,
            collector.primarySensor
        )
        assertTrue(collector.interpolationEnabled)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.gravitySensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(gravitySensor))
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            DEFAULT_WINDOW_NANOSECONDS,
            GyroscopeSensorType.GYROSCOPE,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            SensorDelay.FASTEST,
            GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            false,
            measurementListener,
            accuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(DEFAULT_WINDOW_NANOSECONDS, collector.windowNanoseconds)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            collector.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.gravitySensorDelay)
        assertEquals(SensorDelay.GAME, collector.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.magnetometerSensorDelay)
        assertEquals(
            GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            collector.primarySensor
        )
        assertFalse(collector.interpolationEnabled)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.gravitySensorAvailable)
        assertTrue(collector.gyroscopeSensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(gravitySensor))
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
    fun gravitySensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.gravitySensor)
    }

    @Test
    fun gravitySensor_whenSensorManager_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertSame(gravitySensor, collector.gravitySensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun gyroscopeSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.gyroscopeSensor)
    }

    @Test
    fun gyroscopeSensor_whenSensorTypeGyroscope_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.magnetometerSensor)
    }

    @Test
    fun magnetometerSensor_whenSensorTypeMagnetometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertSame(magnetometerSensor, collector.magnetometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
    }

    @Test
    fun gravitySensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.gravitySensorAvailable)
    }

    @Test
    fun gravitySensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(
            gravitySensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.gravitySensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.gyroscopeSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.magnetometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.magnetometerSensorAvailable)
    }

    @Test
    fun primarySensorType_whenGravity_returnsExpectedResult() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(Sensor.TYPE_GRAVITY, result)
    }

    @Test
    fun primarySensorType_whenGyroscopeUncalibrated_returnsExpectedResult() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenGyroscope_returnsExpectedResult() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GYROSCOPE
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(GyroscopeSensorType.GYROSCOPE.value, result)
    }

    @Test
    fun primarySensorType_whenMagnetometerUncalibrated_returnsExpectedResult() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenMagnetometer_returnsExpectedResult() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(MagnetometerSensorType.MAGNETOMETER.value, result)
    }

    @Test
    fun sensors_whenAvailable_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(gravitySensor))
        assertTrue(sensors.contains(gyroscopeSensor))
        assertTrue(sensors.contains(magnetometerSensor))
    }

    @Test
    fun sensors_whenNotAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoGravityAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoGyroscopeAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoMagnetometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun getSensorDelayFor_whenGravitySensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gravitySensorDelay = SensorDelay.GAME
        )

        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(gyroscopeSensor, collector.gyroscopeSensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", gravitySensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenGyroscopeSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            gyroscopeSensorDelay = SensorDelay.GAME
        )

        assertSame(gravitySensor, collector.gravitySensor)
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorDelay = SensorDelay.GAME
        )

        assertSame(gravitySensor, collector.gravitySensor)
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoGravitySensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoGyroscopeSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoMagnetometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun createSensorMeasurement_whenNoEvent_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", null)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenNoEventSensor_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = null

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenUnsupportedSensorEventType_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = gravitySensor
        every { gravitySensor.type }.returns(-1)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNull(result)
    }

    @Test
    fun createSensorMeasurement_whenGravitySensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        setUpGravityEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        assertTrue(result is GravitySensorMeasurement)
    }

    @Test
    fun createSensorMeasurement_whenGyroscopeSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
    fun processSyncedSample_whenNoGravityQueue_returnsFalse() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(gravityMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        val interpolatedGravityMeasurement = gravityMeasurement.copy()
        interpolatedGravityMeasurement.timestamp = timestamp
        val interpolatedGyroscopeMeasurement = gyroscopeMeasurement.copy()
        interpolatedGyroscopeMeasurement.timestamp = timestamp
        val interpolatedMagnetometerMeasurement = magnetometerMeasurement.copy()
        interpolatedMagnetometerMeasurement.timestamp = timestamp

        assertEquals(interpolatedGravityMeasurement, measurement.gravityMeasurement)
        assertNotSame(interpolatedGravityMeasurement, measurement.gravityMeasurement)
        assertEquals(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(interpolatedGyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoGravityMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(gravityMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gravityQueue.add(gravityMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(gravityMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(gravityMeasurement, measurement.gravityMeasurement)
        assertNotSame(gravityMeasurement, measurement.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, measurement.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoGravityMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoGyroscopeMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(gravityMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue =
            LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val gravityMeasurement = createGravityMeasurement()
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gravityQueue.add(gravityMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: GravityGyroscopeAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processAccuracyChanges_whenNoSensor_desNotCallAnyListener() {
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            gravitySensor,
            -1
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenUnsupportedSensor_desNotCallAnyListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
    fun processAccuracyChanges_whenGravitySensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            gravitySensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.GRAVITY,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenGravitySensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            gravitySensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenGyroscopeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { gyroscopeSensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            gyroscopeSensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.GYROSCOPE_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenGyroscopeSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { gyroscopeSensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            gyroscopeSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun processAccuracyChanges_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            magnetometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun processAccuracyChanges_whenMagnetometerSensorAndNoListener_noCallIsMade() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

        callPrivateFunc(
            SyncedSensorCollector::class,
            collector,
            "processAccuracyChanged",
            magnetometerSensor,
            SensorAccuracy.HIGH.value
        )

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = gravitySensor
        every { gravitySensor.type }.returns(-1)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        assertEquals(1, buffer.size)
        assertTrue(buffer.containsKey(collector.gyroscopeSensorType.value))
        val gyroscopeQueue = buffer[collector.gyroscopeSensorType.value]
        requireNotNull(gyroscopeQueue)
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenBufferIsTrimmed_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val gravityMeasurement = createGravityMeasurement()
        gravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val gyroscopeMeasurement = createGyroscopeMeasurement()
        gyroscopeMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val magnetometerMeasurement = createMagnetometerMeasurement()
        magnetometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        gravityQueue.add(gravityMeasurement)
        gyroscopeQueue.add(gyroscopeMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)

        setUpGyroscopeEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertTrue(gravityQueue.isEmpty())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        assertNotEquals(gyroscopeMeasurement, gyroscopeQueue.first())
        assertTrue(magnetometerQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndBufferNotEmpty_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevGravityMeasurement = createGravityMeasurement()
        prevGravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(prevGravityMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        // new primary measurement is stored in buffer
        val primaryGravityMeasurement = gravityQueue.first() as GravitySensorMeasurement
        // and does not match previous measurement
        assertNotEquals(prevGravityMeasurement, gravityQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        assertEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())
        assertEquals(1, magnetometerQueue.size)
        assertTrue(magnetometerQueue.first() is MagnetometerSensorMeasurement)
        assertEquals(prevMagnetometerMeasurement, magnetometerQueue.first())

        val slot = slot<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val syncedMeasurement = slot.captured

        assertEquals(primaryGravityMeasurement, syncedMeasurement.gravityMeasurement)
        val interpolatedGyroscopeMeasurement = prevGyroscopeMeasurement.copy()
        interpolatedGyroscopeMeasurement.timestamp = primaryGravityMeasurement.timestamp
        assertEquals(
            interpolatedGyroscopeMeasurement,
            syncedMeasurement.gyroscopeMeasurement
        )
        val interpolatedMagnetometerMeasurement = prevMagnetometerMeasurement.copy()
        interpolatedMagnetometerMeasurement.timestamp = primaryGravityMeasurement.timestamp
        assertEquals(
            interpolatedMagnetometerMeasurement,
            syncedMeasurement.magnetometerMeasurement
        )
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventBufferNotEmptyAndNoListener_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevGravityMeasurement = createGravityMeasurement()
        prevGravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevGyroscopeMeasurement = createGyroscopeMeasurement()
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        gravityQueue.add(prevGravityMeasurement)
        gyroscopeQueue.add(prevGyroscopeMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        // new primary measurement is stored in buffer
        // and does not match previous measurement
        assertNotEquals(prevGravityMeasurement, gravityQueue.first())
        assertEquals(1, gyroscopeQueue.size)
        assertTrue(gyroscopeQueue.first() is GyroscopeSensorMeasurement)
        assertEquals(prevGyroscopeMeasurement, gyroscopeQueue.first())
        assertEquals(1, magnetometerQueue.size)
        assertTrue(magnetometerQueue.first() is MagnetometerSensorMeasurement)
        assertEquals(prevMagnetometerMeasurement, magnetometerQueue.first())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndEmptyBuffer_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = GravityGyroscopeAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gyroscopeQueue = LinkedList<GyroscopeSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.gyroscopeSensorType.value] = gyroscopeQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        assertTrue(gravityQueue.isEmpty())
        assertTrue(gyroscopeQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        assertTrue(gyroscopeQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenGravitySensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(gravitySensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.GRAVITY,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenGyroscopeSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { gyroscopeSensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(gyroscopeSensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.GYROSCOPE_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(magnetometerSensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun start_whenRunning_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                null
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                null
            )

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNull(collector.sensors)

            assertFalse(collector.running)
            assertEquals(0L, collector.startTimestamp)
        }
    }

    @Test
    fun start_whenGravityRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(false)
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(false)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(false)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertFalse(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertTrue(collector.start())
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
                gyroscopeSensor
            )
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), gyroscopeSensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

            assertTrue(collector.start(startTimestamp))
            assertNotNull(collector.sensors)

            val slot1 = slot<SensorEventListener>()
            verify(exactly = 1) {
                sensorManager.registerListener(
                    capture(slot1),
                    gravitySensor,
                    collector.gravitySensorDelay.value
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

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val collector = GravityGyroscopeAndMagnetometerSyncedSensorCollector(context)

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
        verify(exactly = 1) { sensorManager.unregisterListener(capture(slot1), gravitySensor) }
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

    private fun setUpGravityEvent() {
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        event.sensor = gravitySensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val values = floatArrayOf(gx, gy, gz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM
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

        private fun createGravityMeasurement(): GravitySensorMeasurement {
            val randomizer = UniformRandomizer()
            val gx = randomizer.nextFloat()
            val gy = randomizer.nextFloat()
            val gz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
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