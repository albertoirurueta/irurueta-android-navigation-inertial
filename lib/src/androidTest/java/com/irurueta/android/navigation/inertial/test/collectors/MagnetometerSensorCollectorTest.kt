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
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.testutils.RequiresRealDevice
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class MagnetometerSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @RequiresRealDevice
    @Test
    fun sensor_whenMagnetometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER
        )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun sensor_whenMagnetometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun sensorAvailable_whenMagnetometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER
        )

        assertTrue(collector.sensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun sensorAvailable_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenMagnetometerSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.FASTEST,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "MagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val bx = measurement.bx
                val by = measurement.by
                val bz = measurement.bz
                val hardIronX = measurement.hardIronX
                val hardIronY = measurement.hardIronY
                val hardIronZ = measurement.hardIronZ
                val timestamp = measurement.timestamp
                val accuracy = measurement.accuracy
                val sensorType = measurement.sensorType
                val coordinateSystem = measurement.sensorCoordinateSystem

                assertNull(hardIronX)
                assertNull(hardIronY)
                assertNull(hardIronZ)
                assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
                assertEquals(SensorCoordinateSystem.ENU, coordinateSystem)

                Log.d(
                    "MagnetometerSensorCollectorTest",
                    """onMeasurement - bx: $bx µT, by: $by µT, bz: $bz µT, 
                        |hardIronX: $hardIronX µT, hardIronY: $hardIronY µT, hardIronZ: $hardIronZ µT, 
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
    fun startAndStop_whenMagnetometerUncalibratedSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.FASTEST,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "MagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val bx = measurement.bx
                val by = measurement.by
                val bz = measurement.bz
                val hardIronX = measurement.hardIronX
                val hardIronY = measurement.hardIronY
                val hardIronZ = measurement.hardIronZ
                val timestamp = measurement.timestamp
                val accuracy = measurement.accuracy
                val sensorType = measurement.sensorType
                val coordinateSystem = measurement.sensorCoordinateSystem

                assertNotNull(hardIronX)
                assertNotNull(hardIronY)
                assertNotNull(hardIronZ)
                assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)
                assertEquals(SensorCoordinateSystem.ENU, coordinateSystem)

                Log.d(
                    "MagnetometerSensorCollectorTest",
                    """onMeasurement - bx: $bx µT, by: $by µT, bz: $bz µT, 
                        |hardIronX: $hardIronX µT, hardIronY: $hardIronY µT, hardIronZ: $hardIronZ µT, 
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
        val maximumRange = sensor.maximumRange // µT
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
        val resolution = sensor.resolution // µT
        val stringType = sensor.stringType
        val type = sensor.type
        val vendor = sensor.vendor
        val version = sensor.version
        val additionInfoSupported = sensor.isAdditionalInfoSupported
        val dynamicSensor = sensor.isDynamicSensor
        val wakeUpSensor = sensor.isWakeUpSensor

        Log.d(
            "MagnetometerSensorCollector2Test", "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReservedEventCount: $fifoReservedEventCount, "
                    + "highestDirectReportRateLevel: $highestDirectReportRateLevel, "
                    + "highestDirectReportRateLevelName: $highestDirectReportRateLevelName, "
                    + "id: $id, "
                    + "maxDelay: $maxDelay µs, "
                    + "maximumRange: $maximumRange µT, "
                    + "minDelay: $minDelay µs, "
                    + "name: $name, "
                    + "power: $power mA, "
                    + "reportingMode: $reportingMode, "
                    + "reportingModeName: $reportingModeName, "
                    + "resolution: $resolution µT, "
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