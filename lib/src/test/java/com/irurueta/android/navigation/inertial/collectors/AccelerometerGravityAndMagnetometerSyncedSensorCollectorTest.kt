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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
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
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import java.util.LinkedList
import java.util.Queue

@Suppress("UNCHECKED_CAST")
class AccelerometerGravityAndMagnetometerSyncedSensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener:
            SyncedSensorCollector.OnMeasurementListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSyncedSensorCollector>

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SyncedSensorCollector.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSyncedSensorCollector>

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gravitySensor: Sensor

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        assertEquals(SensorDelay.FASTEST, collector.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            collector.primarySensor
        )
        assertTrue(collector.interpolationEnabled)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gravitySensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gravitySensor))
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            DEFAULT_WINDOW_NANOSECONDS,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            SensorDelay.FASTEST,
            AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            false,
            measurementListener,
            accuracyChangedListener
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
        assertEquals(SensorDelay.GAME, collector.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, collector.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER,
            collector.primarySensor
        )
        assertFalse(collector.interpolationEnabled)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)
        assertTrue(collector.accelerometerSensorAvailable)
        assertTrue(collector.gravitySensorAvailable)
        assertTrue(collector.magnetometerSensorAvailable)
        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gravitySensor))
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
    fun accelerometerSensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.accelerometerSensor)
    }

    @Test
    fun accelerometerSensor_whenSensorTypeAccelerometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun gravitySensor_whenNoSensorManager_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.gravitySensor)
    }

    @Test
    fun gravitySensor_whenSensorManager_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertSame(gravitySensor, collector.gravitySensor)
        verify(exactly = 1) { context.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }

    }

    @Test
    fun magnetometerSensor_whenSensorTypeMagnetometer_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @Test
    fun gravitySensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.gravitySensorAvailable)
    }

    @Test
    fun gravitySensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(
            gravitySensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.gravitySensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenNoSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.magnetometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenSensor_returnsTrue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.magnetometerSensorAvailable)
    }

    @Test
    fun primarySensorType_whenAccelerometerUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenAccelerometer_returnsExpectedResult() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.ACCELEROMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(AccelerometerSensorType.ACCELEROMETER.value, result)
    }

    @Test
    fun primarySensorType_whenGravity_returnsExpectedResult() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(Sensor.TYPE_GRAVITY, result)
    }

    @Test
    fun primarySensorType_whenMagnetometerUncalibrated_returnsExpectedResult() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
        )

        val result: Int? = collector.getPrivateProperty("primarySensorType")

        requireNotNull(result)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value, result)
    }

    @Test
    fun primarySensorType_whenMagnetometer_returnsExpectedResult() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.MAGNETOMETER
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        val sensors = collector.sensors
        requireNotNull(sensors)
        assertTrue(sensors.contains(accelerometerSensor))
        assertTrue(sensors.contains(gravitySensor))
        assertTrue(sensors.contains(magnetometerSensor))
    }

    @Test
    fun sensors_whenNotAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoAccelerometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoGravityAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun sensors_whenNoMagnetometerAvailable_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertNull(collector.sensors)
    }

    @Test
    fun getSensorDelayFor_whenAccelerometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            accelerometerSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", accelerometerSensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenGravitySensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            gravitySensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gravitySensor, collector.gravitySensor)
        assertSame(magnetometerSensor, collector.magnetometerSensor)

        val result: Int? =
            collector.callPrivateFuncWithResult("getSensorDelayFor", gravitySensor)
        assertEquals(SensorDelay.GAME.value, result)
    }

    @Test
    fun getSensorDelayFor_whenMagnetometerSensor_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorDelay = SensorDelay.GAME
        )

        assertSame(accelerometerSensor, collector.accelerometerSensor)
        assertSame(gravitySensor, collector.gravitySensor)
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoAccelerometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            null
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoGravitySensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun sensorsAvailable_whenNoMagnetometerSensor_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            null
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        assertFalse(collector.sensorsAvailable)
    }

    @Test
    fun createSensorMeasurement_whenNoEvent_returnsNull() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = null

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
    fun createSensorMeasurement_whenGravitySensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

        setUpGravityEvent()

        val result: SensorMeasurement<*>? =
            collector.callPrivateFuncWithResult("createSensorMeasurement", event)
        assertNotNull(result)
        assertTrue(result is GravitySensorMeasurement)
    }

    @Test
    fun createSensorMeasurement_whenMagnetometerSensorType_returnsExpectedResult() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

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
    fun processSyncedSample_whenNoGravityQueue_returnsFalse() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

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
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue

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
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gravityMeasurement = createGravityMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gravityQueue.add(gravityMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        val interpolatedAccelerometerMeasurement = accelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = timestamp
        val interpolatedGravityMeasurement = gravityMeasurement.copy()
        interpolatedGravityMeasurement.timestamp = timestamp
        val interpolatedMagnetometerMeasurement = magnetometerMeasurement.copy()
        interpolatedMagnetometerMeasurement.timestamp = timestamp

        assertEquals(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(interpolatedAccelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(interpolatedGravityMeasurement, measurement.gravityMeasurement)
        assertNotSame(interpolatedGravityMeasurement, measurement.gravityMeasurement)
        assertEquals(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(interpolatedMagnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoGravityMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationEnabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = true
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gravityMeasurement = createGravityMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gravityQueue.add(gravityMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndMeasurementsAvailable_returnsTrueAndSetsExpectedMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gravityMeasurement = createGravityMeasurement()
        val magnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gravityQueue.add(gravityMeasurement)
        magnetometerQueue.add(magnetometerMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertTrue(result)

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, measurement.accelerometerMeasurement)
        assertEquals(gravityMeasurement, measurement.gravityMeasurement)
        assertNotSame(gravityMeasurement, measurement.gravityMeasurement)
        assertEquals(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, measurement.magnetometerMeasurement)
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoAccelerometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoGravityMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
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

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processSyncedSample_whenInterpolationDisabledAndNoMagnetometerMeasurementAvailable_returnsFalseAndDoesNotUpdateMeasurement() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            interpolationEnabled = false
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        val accelerometerMeasurement = createAccelerometerMeasurement()
        val gravityMeasurement = createGravityMeasurement()
        accelerometerQueue.add(accelerometerMeasurement)
        gravityQueue.add(gravityMeasurement)
        val timestamp = System.nanoTime()

        val result: Boolean? = collector.callPrivateFuncWithResult(
            "processSyncedSample",
            timestamp
        )

        requireNotNull(result)
        assertFalse(result)

        val measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement? =
            collector.getPrivateProperty("measurement")

        requireNotNull(measurement)
        assertEquals(0L, measurement.timestamp)
    }

    @Test
    fun processAccuracyChanges_whenNoSensor_desNotCallAnyListener() {
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
    fun processAccuracyChanges_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
    fun processAccuracyChanges_whenGravitySensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
    fun processAccuracyChanges_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        event.sensor = accelerometerSensor
        every { accelerometerSensor.type }.returns(-1)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val accelerometerMeasurement = createAccelerometerMeasurement()
        accelerometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val gravityMeasurement = createGravityMeasurement()
        gravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val magnetometerMeasurement = createMagnetometerMeasurement()
        magnetometerMeasurement.timestamp -= 2 * collector.windowNanoseconds
        accelerometerQueue.add(accelerometerMeasurement)
        gravityQueue.add(gravityMeasurement)
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
        assertTrue(gravityQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventAndBufferNotEmpty_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGravityMeasurement = createGravityMeasurement()
        prevGravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gravityQueue.add(prevGravityMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        // new primary measurement is stored in buffer
        val primaryGravityMeasurement = gravityQueue.first() as GravitySensorMeasurement
        // and does not match previous measurement
        assertNotEquals(prevGravityMeasurement, gravityQueue.first())
        assertEquals(1, magnetometerQueue.size)
        assertTrue(magnetometerQueue.first() is MagnetometerSensorMeasurement)
        assertEquals(prevMagnetometerMeasurement, magnetometerQueue.first())

        val slot = slot<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val syncedMeasurement = slot.captured

        val interpolatedAccelerometerMeasurement = prevAccelerometerMeasurement.copy()
        interpolatedAccelerometerMeasurement.timestamp = primaryGravityMeasurement.timestamp
        assertEquals(
            interpolatedAccelerometerMeasurement,
            syncedMeasurement.accelerometerMeasurement
        )
        assertEquals(primaryGravityMeasurement, syncedMeasurement.gravityMeasurement)
    }

    @Test
    fun onSensorChanged_whenPrimarySensorEventBufferNotEmptyAndNoListener_updatesBuffer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        // save initial measurements in buffer that will be trimmed
        val prevAccelerometerMeasurement = createAccelerometerMeasurement()
        val prevGravityMeasurement = createGravityMeasurement()
        prevGravityMeasurement.timestamp -= 2 * collector.windowNanoseconds
        val prevMagnetometerMeasurement = createMagnetometerMeasurement()
        accelerometerQueue.add(prevAccelerometerMeasurement)
        gravityQueue.add(prevGravityMeasurement)
        magnetometerQueue.add(prevMagnetometerMeasurement)

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertEquals(1, accelerometerQueue.size)
        assertTrue(accelerometerQueue.first() is AccelerometerSensorMeasurement)
        assertEquals(prevAccelerometerMeasurement, accelerometerQueue.first())
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        // new primary measurement is stored in buffer
        // and does not match previous measurement
        assertNotEquals(prevGravityMeasurement, gravityQueue.first())
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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
            context,
            primarySensor = AccelerometerGravityAndMagnetometerSyncedSensorCollector.PrimarySensor.GRAVITY,
            measurementListener = measurementListener
        )

        val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>>? =
            collector.getPrivateProperty("buffer")
        requireNotNull(buffer)

        val accelerometerQueue =
            LinkedList<AccelerometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        val gravityQueue = LinkedList<GravitySensorMeasurement>() as Queue<SensorMeasurement<*>>
        val magnetometerQueue =
            LinkedList<MagnetometerSensorMeasurement>() as Queue<SensorMeasurement<*>>
        buffer[collector.accelerometerSensorType.value] = accelerometerQueue
        buffer[Sensor.TYPE_GRAVITY] = gravityQueue
        buffer[collector.magnetometerSensorType.value] = magnetometerQueue

        assertTrue(accelerometerQueue.isEmpty())
        assertTrue(gravityQueue.isEmpty())
        assertTrue(magnetometerQueue.isEmpty())

        setUpGravityEvent()

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SyncedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(3, buffer.size)
        assertTrue(accelerometerQueue.isEmpty())
        assertEquals(1, gravityQueue.size)
        assertTrue(gravityQueue.first() is GravitySensorMeasurement)
        assertTrue(magnetometerQueue.isEmpty())

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenAccelerometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { accelerometerSensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
    fun onAccuracyChanged_whenGravitySensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { gravitySensor.type }.returns(Sensor.TYPE_GRAVITY)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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
    fun onAccuracyChanged_whenMagnetometerSensor_callsExpectedListener() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        every { magnetometerSensor.type }.returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(
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

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                null
            )

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                false
            )
            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
    fun start_whenGravityRegisterListenerFails_returnsFalse() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
            every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
                accelerometerSensor
            )
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(false)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every {
                sensorManager.registerListener(
                    any(),
                    magnetometerSensor,
                    any()
                )
            }.returns(false)

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
                    gravitySensor,
                    collector.gravitySensorDelay.value
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
            every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
            every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
                magnetometerSensor
            )

            every { sensorManager.registerListener(any(), accelerometerSensor, any()) }.returns(
                true
            )
            every { sensorManager.registerListener(any(), gravitySensor, any()) }.returns(true)
            every { sensorManager.registerListener(any(), magnetometerSensor, any()) }.returns(true)

            val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
                    gravitySensor,
                    collector.gravitySensorDelay.value
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

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val collector = AccelerometerGravityAndMagnetometerSyncedSensorCollector(context)

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
        verify(exactly = 1) { sensorManager.unregisterListener(capture(slot2), gravitySensor) }
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