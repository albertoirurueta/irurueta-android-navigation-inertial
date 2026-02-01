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

package com.irurueta.android.navigation.inertial.test.collectors

import android.hardware.Sensor
import android.hardware.SensorDirectChannel
import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.testutils.RequiresRealDevice
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class AccelerometerSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @RequiresRealDevice
    @Test
    fun sensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER)

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun sensor_whenAccelerometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun sensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(collector.sensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun sensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenAccelerometerSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.FASTEST,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val ax = measurement.ax
                val ay = measurement.ay
                val az = measurement.az
                val bx = measurement.bx
                val by = measurement.by
                val bz = measurement.bz
                val timestamp = measurement.timestamp
                val accuracy = measurement.accuracy
                val sensorType = measurement.sensorType
                val coordinateSystem = measurement.sensorCoordinateSystem

                assertNull(bx)
                assertNull(by)
                assertNull(bz)
                assertEquals(AccelerometerSensorType.ACCELEROMETER, sensorType)
                assertEquals(SensorCoordinateSystem.ENU, coordinateSystem)

                Log.d(
                    "AccelerometerSensorCollectorTest",
                    """onMeasurement - ax: $ax m/s^2, ay: $ay m/s^2, az: $az m/s^2, 
                        |bx: $bx m/s^2, by: $by m/s^2, bz: $bz m/s^2,
                        |timestamp: $timestamp, 
                        |accuracy: $accuracy, 
                        |sensorType: $sensorType,
                        |coordinateSystem: $coordinateSystem
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
    fun startAndStop_whenAccelerometerUncalibratedSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.FASTEST,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "AccelerometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val ax = measurement.ax
                val ay = measurement.ay
                val az = measurement.az
                val bx = measurement.bx
                val by = measurement.by
                val bz = measurement.bz
                val timestamp = measurement.timestamp
                val accuracy = measurement.accuracy
                val sensorType = measurement.sensorType
                val coordinateSystem = measurement.sensorCoordinateSystem

                assertNotNull(bx)
                assertNotNull(by)
                assertNotNull(bz)
                assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, sensorType)
                assertEquals(SensorCoordinateSystem.ENU, coordinateSystem)

                Log.d(
                    "AccelerometerSensorCollectorTest",
                    """onMeasurement - ax: $ax m/s^2, ay: $ay m/s^2, az: $az m/s^2, 
                        |bx: $bx m/s^2, by: $by m/s^2, bz: $bz m/s^2,
                        |timestamp: $timestamp, 
                        |accuracy: $accuracy, 
                        |sensorType: $sensorType,
                        |coordinateSystem: $coordinateSystem
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
            "AccelerometerSensorCollectorTest", "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
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
