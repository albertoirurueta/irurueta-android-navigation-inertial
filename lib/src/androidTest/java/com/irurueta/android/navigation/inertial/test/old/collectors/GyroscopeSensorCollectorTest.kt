/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.test.old.collectors

import android.hardware.Sensor
import android.hardware.SensorDirectChannel
import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.old.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class GyroscopeSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @Test
    fun sensor_whenGyroscopeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            GyroscopeSensorCollector(context, GyroscopeSensorType.GYROSCOPE)

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresDevice
    @Test
    fun sensor_whenGyroscopeUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GyroscopeSensorCollector(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @Test
    fun sensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            GyroscopeSensorCollector(context, GyroscopeSensorType.GYROSCOPE)

        assertTrue(collector.sensorAvailable)
    }

    @RequiresDevice
    @Test
    fun sensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GyroscopeSensorCollector(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
    }

    @Test
    fun startAndStop_whenGyroscopeSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GyroscopeSensorCollector(
            context,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.FASTEST,
            measurementListener = { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
                assertNull(bx)
                assertNull(by)
                assertNull(bz)

                Log.d(
                    "GyroscopeSensorCollectorTest",
                    "onMeasurement - wx: $wx, wy: $wy, wz: $wz, bx: $bx, by: $by, bz: $bz, "
                            + "timestamp: $timestamp, accuracy: $accuracy"
                )

                syncHelper.notifyAll { measured++ }
            }
        ) { accuracy ->
            Log.d(
                "GyroscopeSensorCollectorTest",
                "onAccuracyChanged - accuracy: $accuracy"
            )
        }

        collector.start()

        syncHelper.waitOnCondition({ measured < 1 })

        collector.stop()

        assertTrue(measured > 0)
    }

    @RequiresDevice
    @Test
    fun startAndStop_whenGyroscopeUncalibratedSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GyroscopeSensorCollector(
            context,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.FASTEST,
            measurementListener = { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
                assertNotNull(bx)
                assertNotNull(by)
                assertNotNull(bz)

                Log.d(
                    "GyroscopeSensorCollectorTest",
                    "onMeasurement - wx: $wx, wy: $wy, wz: $wz, bx: $bx, by: $by, bz: $bz, "
                            + "timestamp: $timestamp, accuracy: $accuracy"
                )

                syncHelper.notifyAll { measured++ }
            }
        ) { accuracy ->
            Log.d(
                "GyroscopeSensorCollectorTest",
                "onAccuracyChanged - accuracy: $accuracy"
            )
        }

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
        val maximumRange = sensor.maximumRange // rad/s
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
        val resolution = sensor.resolution // rad/s
        val stringType = sensor.stringType
        val type = sensor.type
        val vendor = sensor.vendor
        val version = sensor.version
        val additionInfoSupported = sensor.isAdditionalInfoSupported
        val dynamicSensor = sensor.isDynamicSensor
        val wakeUpSensor = sensor.isWakeUpSensor

        Log.d(
            "GyroscopeSensorCollectorTest", "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReservedEventCount: $fifoReservedEventCount, "
                    + "highestDirectReportRateLevel: $highestDirectReportRateLevel, "
                    + "highestDirectReportRateLevelName: $highestDirectReportRateLevelName, "
                    + "id: $id, "
                    + "maxDelay: $maxDelay µs, "
                    + "maximumRange: $maximumRange rad/s, "
                    + "minDelay: $minDelay µs, "
                    + "name: $name, "
                    + "power: $power mA, "
                    + "reportingMode: $reportingMode, "
                    + "reportingModeName: $reportingModeName, "
                    + "resolution: $resolution rad/s, "
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