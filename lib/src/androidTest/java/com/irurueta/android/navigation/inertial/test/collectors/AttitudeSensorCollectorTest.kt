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
package com.irurueta.android.navigation.inertial.test.collectors

import android.hardware.Sensor
import android.hardware.SensorDirectChannel
import android.util.Log
import androidx.test.filters.RequiresDevice
import androidx.test.platform.app.InstrumentationRegistry
import com.irurueta.android.navigation.inertial.ThreadSyncHelper
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AttitudeSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @Test
    fun sensor_whenAbsoluteAttitudeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(
                context,
                AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE
            )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @Test
    fun sensor_whenGeomagneticAbsoluteAttitudeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(
                context,
                AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE
            )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @Test
    fun sensor_whenRelativeAttitudeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(context, AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE)

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @Test
    fun sensorAvailable_whenAbsoluteAttitudeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(context, AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE)

        assertTrue(collector.sensorAvailable)
    }

    @Test
    fun sensorAvailable_whenGeomagneticAbsoluteAttitudeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(context, AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE)

        assertTrue(collector.sensorAvailable)
    }

    @Test
    fun sensorAvailable_whenRelativeAttitudeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            AttitudeSensorCollector(context, AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE)

        assertTrue(collector.sensorAvailable)
    }

    @Test
    fun startAndStop_whenAbsoluteAttitudeSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, headingAccuracyRadians, timestamp, accuracy ->
                val quaternion = rotation.toQuaternion()
                val angles = quaternion.toEulerAngles()
                val roll = angles[0]
                val pitch = angles[1]
                val yaw = angles[2]

                Log.d(
                    "AttitudeSensorCollectorTest", "onMeasurement - roll: $roll rad, "
                            + "pitch: $pitch rad, "
                            + "yaw: $yaw rad, "
                            + "headingAccuracy: $headingAccuracyRadians rad, "
                            + "timestamp: $timestamp, accuracy: $accuracy"
                )

                syncHelper.notifyAll { measured++ }
            }
        ) { accuracy ->
            Log.d(
                "AttitudeSensorCollectorTest",
                "onAccuracyChanged - accuracy: $accuracy"
            )
        }

        collector.start()

        syncHelper.waitOnCondition({ measured < 1 })

        collector.stop()

        assertTrue(measured > 0)
    }

    @Test
    fun startAndStop_whenGeomagneticAbsoluteAttitudeSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, headingAccuracyRadians, timestamp, accuracy ->
                val quaternion = rotation.toQuaternion()
                val angles = quaternion.toEulerAngles()
                val roll = angles[0]
                val pitch = angles[1]
                val yaw = angles[2]

                Log.d(
                    "AttitudeSensorCollectorTest", "onMeasurement - roll: $roll rad, "
                            + "pitch: $pitch rad, "
                            + "yaw: $yaw rad, "
                            + "headingAccuracy: $headingAccuracyRadians rad, "
                            + "timestamp: $timestamp, accuracy: $accuracy"
                )

                syncHelper.notifyAll { measured++ }
            }
        ) { accuracy ->
            Log.d(
                "AttitudeSensorCollectorTest",
                "onAccuracyChanged - accuracy: $accuracy"
            )
        }

        collector.start()

        syncHelper.waitOnCondition({ measured < 1 })

        collector.stop()

        assertTrue(measured > 0)
    }

    @Test
    fun startAndStop_whenRelativeOrientationSensorType_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE,
            SensorDelay.FASTEST,
            measurementListener = { rotation, _, headingAccuracyRadians, timestamp, accuracy ->
                val quaternion = rotation.toQuaternion()
                val angles = quaternion.toEulerAngles()
                val roll = angles[0]
                val pitch = angles[1]
                val yaw = angles[2]

                Log.d(
                    "AttitudeSensorCollectorTest", "onMeasurement - roll: $roll rad, "
                            + "pitch: $pitch rad, "
                            + "yaw: $yaw rad, "
                            + "headingAccuracy: $headingAccuracyRadians rad, "
                            + "timestamp: $timestamp, accuracy: $accuracy"
                )

                syncHelper.notifyAll { measured++ }
            }
        ) { accuracy ->
            Log.d(
                "AttitudeSensorCollectorTest",
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
        val fifoReversedEventCount = sensor.fifoReservedEventCount
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
        val maximumRange = sensor.maximumRange // unit-less
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
        val resolution = sensor.resolution // unit-less
        val stringType = sensor.stringType
        val type = sensor.type
        val vendor = sensor.vendor
        val version = sensor.version
        val additionInfoSupported = sensor.isAdditionalInfoSupported
        val dynamicSensor = sensor.isDynamicSensor
        val wakeUpSensor = sensor.isWakeUpSensor

        Log.d(
            "AttitudeSensorCollectorTest", "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReversedEventCount: $fifoReversedEventCount, "
                    + "highestDirectReportRateLevel: $highestDirectReportRateLevel, "
                    + "highestDirectReportRateLevelName: $highestDirectReportRateLevelName, "
                    + "id: $id, "
                    + "maxDelay: $maxDelay µs, "
                    + "maximumRange: $maximumRange unknown unit, "
                    + "minDelay: $minDelay µa, "
                    + "name: $name, "
                    + "power: $power mA, "
                    + "reportingMode: $reportingMode, "
                    + "reportingModeName: $reportingModeName, "
                    + "resolution: $resolution unknown unit, "
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