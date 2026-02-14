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
import com.irurueta.android.navigation.inertial.collectors.GravityAndMagnetometerSyncedSensorCollector
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

class GravityAndMagnetometerSyncedSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var measured = 0

    @Before
    fun setUp() {
        measured = 0
    }

    @RequiresRealDevice
    @Test
    fun gravitySensor_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(context)

        val sensor = collector.gravitySensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun magnetometerSensor_whenMagnetometerSensorType_returnSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        val sensor = collector.magnetometerSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun magnetometerSensor_whenMagnetometerUncalibratedSensorType_returnSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        val sensor = collector.magnetometerSensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresRealDevice
    @Test
    fun gravitySensorAvailable_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(context)

        assertTrue(collector.gravitySensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun magnetometerSensorAvailable_whenMagnetometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        assertTrue(collector.magnetometerSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun magnetometerSensorAvailable_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(collector.magnetometerSensorAvailable)
    }

    @RequiresRealDevice
    @Test
    fun startAndStop_whenGravityAndMagnetometerSensors_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER,
            gravitySensorDelay = SensorDelay.FASTEST,
            magnetometerSensorDelay = SensorDelay.FASTEST,
            accuracyChangedListener = { _, sensorType, accuracy ->
                Log.d(
                    "GravityAndMagnetometerSyncedSensorCollectorTest",
                    "onAccuracyChanged - sensorType: $sensorType, accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val timestamp = measurement.timestamp
                val gravityMeasurement = measurement.gravityMeasurement
                requireNotNull(gravityMeasurement)
                val magnetometerMeasurement = measurement.magnetometerMeasurement
                requireNotNull(magnetometerMeasurement)

                val gx = gravityMeasurement.gx
                val gy = gravityMeasurement.gy
                val gz = gravityMeasurement.gz
                val gravityTimestamp = gravityMeasurement.timestamp
                val gravityAccuracy = gravityMeasurement.accuracy
                val gravityCoordinateSystem = gravityMeasurement.sensorCoordinateSystem

                val bx = magnetometerMeasurement.bx
                val by = magnetometerMeasurement.by
                val bz = magnetometerMeasurement.bz
                val hardIronX = magnetometerMeasurement.hardIronX
                val hardIronY = magnetometerMeasurement.hardIronY
                val hardIronZ = magnetometerMeasurement.hardIronZ
                val magnetometerTimestamp = magnetometerMeasurement.timestamp
                val magnetometerAccuracy = magnetometerMeasurement.accuracy
                val magnetometerSensorType = magnetometerMeasurement.sensorType
                val magnetometerCoordinateSystem = magnetometerMeasurement.sensorCoordinateSystem

                assertEquals(SensorCoordinateSystem.ENU, gravityCoordinateSystem)

                assertNull(hardIronX)
                assertNull(hardIronY)
                assertNull(hardIronZ)
                assertEquals(MagnetometerSensorType.MAGNETOMETER, magnetometerSensorType)
                assertEquals(SensorCoordinateSystem.ENU, magnetometerCoordinateSystem)

                Log.d(
                    "GravityAndMagnetometerSyncedSensorCollectorTest",
                    """onMeasurement - timestamp: $timestamp, 
                        |gx: $gx m/s^2, gy: $gy m/s^2, gz: $gz m/s^2, 
                        |gravityTimestamp: $gravityTimestamp, 
                        |gravityAccuracy: $gravityAccuracy, 
                        |gravityCoordinateSystem: $gravityCoordinateSystem,
                        |bx: $bx µT, by: $by µT, bz: $bz µT, 
                        |hardIronX: $hardIronX µT, hardIronY: $hardIronY µT, hardIronZ: $hardIronZ µT, 
                        |magnetometerTimestamp: $magnetometerTimestamp, 
                        |magnetometerAccuracy: $magnetometerAccuracy, 
                        |magnetometerSensorType: $magnetometerSensorType,
                        |magnetometerCoordinateSystem: $magnetometerCoordinateSystem.
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
    fun startAndStop_whenGravityAndMagnetometerUncalibratedSensors_collectsMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = GravityAndMagnetometerSyncedSensorCollector(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            gravitySensorDelay = SensorDelay.FASTEST,
            magnetometerSensorDelay = SensorDelay.FASTEST,
            accuracyChangedListener = { _, sensorType, accuracy ->
                Log.d(
                    "GravityAndMagnetometerSyncedSensorCollectorTest",
                    "onAccuracyChanged - sensorType: $sensorType, accuracy: $accuracy"
                )
            },
            measurementListener = { _, measurement ->
                val timestamp = measurement.timestamp
                val gravityMeasurement = measurement.gravityMeasurement
                requireNotNull(gravityMeasurement)
                val magnetometerMeasurement = measurement.magnetometerMeasurement
                requireNotNull(magnetometerMeasurement)

                val gx = gravityMeasurement.gx
                val gy = gravityMeasurement.gy
                val gz = gravityMeasurement.gz
                val gravityTimestamp = gravityMeasurement.timestamp
                val gravityAccuracy = gravityMeasurement.accuracy
                val gravityCoordinateSystem = gravityMeasurement.sensorCoordinateSystem

                val bx = magnetometerMeasurement.bx
                val by = magnetometerMeasurement.by
                val bz = magnetometerMeasurement.bz
                val hardIronX = magnetometerMeasurement.hardIronX
                val hardIronY = magnetometerMeasurement.hardIronY
                val hardIronZ = magnetometerMeasurement.hardIronZ
                val magnetometerTimestamp = magnetometerMeasurement.timestamp
                val magnetometerAccuracy = magnetometerMeasurement.accuracy
                val magnetometerSensorType = magnetometerMeasurement.sensorType
                val magnetometerCoordinateSystem = magnetometerMeasurement.sensorCoordinateSystem

                assertEquals(SensorCoordinateSystem.ENU, gravityCoordinateSystem)

                assertNotNull(hardIronX)
                assertNotNull(hardIronY)
                assertNotNull(hardIronZ)
                assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, magnetometerSensorType)
                assertEquals(SensorCoordinateSystem.ENU, magnetometerCoordinateSystem)

                Log.d(
                    "GravityAndMagnetometerSyncedSensorCollectorTest",
                    """onMeasurement - timestamp: $timestamp, 
                        |gx: $gx m/s^2, gy: $gy m/s^2, gz: $gz m/s^2, 
                        |gravityTimestamp: $gravityTimestamp, 
                        |gravityAccuracy: $gravityAccuracy, 
                        |gravityCoordinateSystem: $gravityCoordinateSystem,
                        |bx: $bx µT, by: $by µT, bz: $bz µT, 
                        |hardIronX: $hardIronX µT, hardIronY: $hardIronY µT, hardIronZ: $hardIronZ µT, 
                        |magnetometerTimestamp: $magnetometerTimestamp, 
                        |magnetometerAccuracy: $magnetometerAccuracy, 
                        |magnetometerSensorType: $magnetometerSensorType,
                        |magnetometerCoordinateSystem: $magnetometerCoordinateSystem.
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
            "AccelerometerGravityAndMagnetometerSyncedSensorCollectorTest",
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