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

package com.irurueta.android.navigation.inertial.test.collectors

import android.hardware.Sensor
import android.hardware.SensorDirectChannel
import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.testutils.RequiresRealDevice
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class AccelerometerAndGyroscopeSyncedSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @RequiresRealDevice
    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        val sensor = collector.accelerometerSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun accelerometerSensor_whenAccelerometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = collector.accelerometerSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun gyroscopeSensor_whenGyroscopeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        val sensor = collector.gyroscopeSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun gyroscopeSensor_whenGyroscopeUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        val sensor = collector.gyroscopeSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun accelerometerSensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(collector.accelerometerSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertTrue(collector.gyroscopeSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(collector.gyroscopeSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerAndGyroscopeSensors_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE,
            accelerometerSensorDelay = SensorDelay.FASTEST,
            gyroscopeSensorDelay = SensorDelay.FASTEST,
            accelerometerAccuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    "onAccuracyChanged - accelerometer accuracy: $accuracy"
                )
            },
            gyroscopeAccuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    "onAccuracyChanged - gyroscope accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val timestamp = measurement.timestamp
                val accelerometerMeasurement = measurement.accelerometerMeasurement
                requireNotNull(accelerometerMeasurement)
                val gyroscopeMeasurement = measurement.gyroscopeMeasurement
                requireNotNull(gyroscopeMeasurement)

                val ax = accelerometerMeasurement.ax
                val ay = accelerometerMeasurement.ay
                val az = accelerometerMeasurement.az
                val abx = accelerometerMeasurement.bx
                val aby = accelerometerMeasurement.by
                val abz = accelerometerMeasurement.bz
                val accelerometerTimestamp = accelerometerMeasurement.timestamp
                val accelerometerAccuracy = accelerometerMeasurement.accuracy
                val accelerometerSensorType = accelerometerMeasurement.sensorType
                val accelerometerCoordinateSystem = accelerometerMeasurement.sensorCoordinateSystem

                val wx = gyroscopeMeasurement.wx
                val wy = gyroscopeMeasurement.wy
                val wz = gyroscopeMeasurement.wz
                val wbx = gyroscopeMeasurement.bx
                val wby = gyroscopeMeasurement.by
                val wbz = gyroscopeMeasurement.bz
                val gyroscopeTimestamp = gyroscopeMeasurement.timestamp
                val gyroscopeAccuracy = gyroscopeMeasurement.accuracy
                val gyroscopeSensorType = gyroscopeMeasurement.sensorType
                val gyroscopeCoordinateSystem = gyroscopeMeasurement.sensorCoordinateSystem

                assertNull(abx)
                assertNull(aby)
                assertNull(abz)
                assertEquals(AccelerometerSensorType.ACCELEROMETER, accelerometerSensorType)
                assertEquals(SensorCoordinateSystem.ENU, accelerometerCoordinateSystem)

                assertNull(wbx)
                assertNull(wby)
                assertNull(wbz)
                assertEquals(GyroscopeSensorType.GYROSCOPE, gyroscopeSensorType)
                assertEquals(SensorCoordinateSystem.ENU, gyroscopeCoordinateSystem)

                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    """onMeasurement - timestamp: $timestamp, 
                        |ax: $ax m/s^2, ay: $ay m/s^2, az: $az m/s^2, 
                        |abx: $abx m/s^2, aby: $aby m/s^2, abz: $abz m/s^2, 
                        |accelerometerTimestamp: $accelerometerTimestamp, 
                        |accelerometerAccuracy: $accelerometerAccuracy, 
                        |accelerometerSensorType: $accelerometerSensorType,
                        |accelerometerCoordinateSystem: $accelerometerCoordinateSystem,
                        |wx: $wx rad/s, wy: $wy rad/s, wz: $wz rad/s, 
                        |wbx: $wbx rad/s, wby: $wby rad/s, wbz: $wbz rad/s, 
                        |gyroscopeTimestamp: $gyroscopeTimestamp, 
                        |gyroscopeAccuracy: $gyroscopeAccuracy, 
                        |gyroscopeSensorType: $gyroscopeSensorType
                        |gyroscopeCoordinateSystem: $gyroscopeCoordinateSystem.
                    """.trimMargin()
                )

                syncHelper.notifyAll { measured++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ measured < 1 })

        collector.stop()

        assertTrue(measured > 0)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerAndGyroscopeUncalibratedSensors_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            accelerometerSensorDelay = SensorDelay.FASTEST,
            gyroscopeSensorDelay = SensorDelay.FASTEST,
            accelerometerAccuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    "onAccuracyChanged - accelerometer accuracy: $accuracy"
                )
            },
            gyroscopeAccuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    "onAccuracyChanged - gyroscope accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val timestamp = measurement.timestamp
                val accelerometerMeasurement = measurement.accelerometerMeasurement
                requireNotNull(accelerometerMeasurement)
                val gyroscopeMeasurement = measurement.gyroscopeMeasurement
                requireNotNull(gyroscopeMeasurement)

                val ax = accelerometerMeasurement.ax
                val ay = accelerometerMeasurement.ay
                val az = accelerometerMeasurement.az
                val abx = accelerometerMeasurement.bx
                val aby = accelerometerMeasurement.by
                val abz = accelerometerMeasurement.bz
                val accelerometerTimestamp = accelerometerMeasurement.timestamp
                val accelerometerAccuracy = accelerometerMeasurement.accuracy
                val accelerometerSensorType = accelerometerMeasurement.sensorType
                val accelerometerCoordinateSystem = accelerometerMeasurement.sensorCoordinateSystem

                val wx = gyroscopeMeasurement.wx
                val wy = gyroscopeMeasurement.wy
                val wz = gyroscopeMeasurement.wz
                val wbx = gyroscopeMeasurement.bx
                val wby = gyroscopeMeasurement.by
                val wbz = gyroscopeMeasurement.bz
                val gyroscopeTimestamp = gyroscopeMeasurement.timestamp
                val gyroscopeAccuracy = gyroscopeMeasurement.accuracy
                val gyroscopeSensorType = gyroscopeMeasurement.sensorType
                val gyroscopeCoordinateSystem = gyroscopeMeasurement.sensorCoordinateSystem

                assertNotNull(abx)
                assertNotNull(aby)
                assertNotNull(abz)
                assertEquals(
                    AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
                    accelerometerSensorType
                )
                assertEquals(SensorCoordinateSystem.ENU, accelerometerCoordinateSystem)

                assertNotNull(wbx)
                assertNotNull(wby)
                assertNotNull(wbz)
                assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, gyroscopeSensorType)
                assertEquals(SensorCoordinateSystem.ENU, gyroscopeCoordinateSystem)

                Log.d(
                    "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
                    """onMeasurement - timestamp: $timestamp, 
                        |ax: $ax m/s^2, ay: $ay m/s^2, az: $az m/s^2, 
                        |abx: $abx m/s^2, aby: $aby m/s^2, abz: $abz m/s^2, 
                        |accelerometerTimestamp: $accelerometerTimestamp, 
                        |accelerometerAccuracy: $accelerometerAccuracy, 
                        |accelerometerSensorType: $accelerometerSensorType,
                        |accelerometerCoordinateSystem: $accelerometerCoordinateSystem, 
                        |wx: $wx rad/s, wy: $wy rad/s, wz: $wz rad/s, 
                        |wbx: $wbx rad/s, wby: $wby rad/s, wbz: $wbz rad/s, 
                        |gyroscopeTimestamp: $gyroscopeTimestamp, 
                        |gyroscopeAccuracy: $gyroscopeAccuracy, 
                        |gyroscopeSensorType: $gyroscopeSensorType
                        |gyroscopeCoordinateSystem: $gyroscopeCoordinateSystem,
                    """.trimMargin()
                )

                syncHelper.notifyAll { measured++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ measured < 1 })

        collector.stop()

        assertTrue(measured > 0)
    }

    private fun logSensor(sensor: Sensor) {
        val fifoMaxEventCount = sensor.fifoMaxEventCount
        val fifoReservedEventCount = sensor.fifoReservedEventCount
        val highestDirectReportRateLevel = sensor.highestDirectReportRateLevel
        val highestDirectReportRateLevelName = when (highestDirectReportRateLevel) {
            SensorDirectChannel.RATE_STOP -> "RATE_STOP"
            SensorDirectChannel.RATE_NORMAL -> "RATE_NORMAL"
            SensorDirectChannel.RATE_FAST -> "RATE_FAST"
            SensorDirectChannel.RATE_VERY_FAST -> "RATE_VERY_FAST"
            else -> ""
        }
        val id = sensor.id
        val maxDelay = sensor.maxDelay // microseconds (µs)
        val maximumRange = sensor.maximumRange // m/s^2
        val minDelay = sensor.minDelay // microseconds (µs)
        val name = sensor.name
        val power = sensor.power // milli-amperes (mA)
        val reportingMode = sensor.reportingMode
        val reportingModeName = when (reportingMode) {
            Sensor.REPORTING_MODE_CONTINUOUS -> "REPORTING_MODE_CONTINUOUS"
            Sensor.REPORTING_MODE_ON_CHANGE -> "REPORTING_MODE_ON_CHANGE"
            Sensor.REPORTING_MODE_ONE_SHOT -> "REPORTING_MODE_ONE_SHOT"
            Sensor.REPORTING_MODE_SPECIAL_TRIGGER -> "REPORTING_MODE_SPECIAL_TRIGGER"
            else -> ""
        }
        val resolution = sensor.resolution // m/s^2
        val stringType = sensor.stringType
        val type = sensor.type
        val vendor = sensor.vendor
        val version = sensor.version
        val additionInfoSupported = sensor.isAdditionalInfoSupported
        val dynamicSensor = sensor.isDynamicSensor
        val wakeUpSensor = sensor.isWakeUpSensor

        Log.d(
            "AccelerometerAndGyroscopeSyncedSensorCollectorTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReservedEventCount: $fifoReservedEventCount, "
                    + "highestDirectReportRateLevel: $highestDirectReportRateLevel, "
                    + "highestDirectReportRateLevelName: $highestDirectReportRateLevelName, "
                    + "id: $id, "
                    + "maxDelay: $maxDelay µs, "
                    + "maximumRange: $maximumRange m/s^2, "
                    + "minDelay: $minDelay µs, "
                    + "name: $name, "
                    + "power: $power mA, "
                    + "reportingMode: $reportingMode, "
                    + "reportingModeName: $reportingModeName, "
                    + "resolution: $resolution m/s^2, "
                    + "stringType: $stringType, "
                    + "type: $type, "
                    + "vendor: $vendor, "
                    + "version: $version, "
                    + "additionInfoSupported: $additionInfoSupported, "
                    + "dynamicSensor: $dynamicSensor, "
                    + "wakeUpSensor: $wakeUpSensor"
        )
    }
}