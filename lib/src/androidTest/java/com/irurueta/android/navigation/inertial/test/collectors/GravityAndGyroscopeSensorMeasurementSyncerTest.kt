/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.android.navigation.inertial.collectors.*
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class GravityAndGyroscopeSensorMeasurementSyncerTest {
    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var gravityTimestamp = 0L
    private var gyroscopeTimestamp = 0L
    private var maxGravityUsage = -1.0f
    private var maxGyroscopeUsage = -1.0f
    private var maxGravityCollectorUsage = -1.0f
    private var maxGyroscopeCollectorUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        gravityTimestamp = 0L
        gyroscopeTimestamp = 0L
        maxGravityUsage = -1.0f
        maxGyroscopeUsage = -1.0f
        maxGravityCollectorUsage = -1.0f
        maxGyroscopeCollectorUsage = -1.0f
    }

    @Test
    fun gravitySensor_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(context)

        val sensor = syncer.gravitySensor
        requireNotNull(sensor)

        logGravitySensor(sensor)
    }

    @Test
    fun gyroscopeSensor_whenGyroscopeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        val sensor = syncer.gyroscopeSensor
        requireNotNull(sensor)

        logGyroscopeSensor(sensor)
    }

    @Test
    fun gyroscopeSensor_whenGyroscopeUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        val sensor = syncer.gyroscopeSensor
        requireNotNull(sensor)

        logGyroscopeSensor(sensor)
    }

    @Test
    fun gravitySensorAvailable_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(context)

        assertTrue(syncer.gravitySensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetsEnabled_notifiesMeasurements() {
        runTest(
            gravityStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            stopWhenFilledBuffer = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            gravityStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            gravityStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = false
        )
    }

    private fun runTest(
        gravityStartOffsetEnabled: Boolean,
        gyroscopeStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = GravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gravityStartOffsetEnabled = gravityStartOffsetEnabled,
            gyroscopeStartOffsetEnabled = gyroscopeStartOffsetEnabled,
            stopWhenFilledBuffer = stopWhenFilledBuffer,
            staleDetectionEnabled = true,
            accuracyChangedListener = { syncer, sensorType, accuracy ->
                logAccuracyChanged(
                    syncer,
                    sensorType,
                    accuracy
                )
            },
            bufferFilledListener = { syncer, sensorType -> logBufferFilled(syncer, sensorType) },
            syncedMeasurementListener = { syncer, measurement ->
                assertTrue(measurement.timestamp > timestamp)

                requireNotNull(measurement.gyroscopeMeasurement)
                requireNotNull(measurement.gravityMeasurement)

                val currentGyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp
                requireNotNull(currentGyroscopeTimestamp)
                assertTrue(currentGyroscopeTimestamp >= gyroscopeTimestamp)

                val currentGravityTimestamp = measurement.gravityMeasurement?.timestamp
                requireNotNull(currentGravityTimestamp)
                assertTrue(currentGravityTimestamp >= gravityTimestamp)

                timestamp = measurement.timestamp
                gyroscopeTimestamp = currentGyroscopeTimestamp
                gravityTimestamp = currentGravityTimestamp

                val gravityCollectorUsage = syncer.gravityCollectorUsage
                if (gravityCollectorUsage > maxGravityCollectorUsage) {
                    maxGravityCollectorUsage = gravityCollectorUsage
                }
                val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
                if (gyroscopeCollectorUsage > maxGyroscopeCollectorUsage) {
                    maxGyroscopeCollectorUsage = gyroscopeCollectorUsage
                }
                val gravityUsage = syncer.gravityUsage
                if (gravityUsage > maxGravityUsage) {
                    maxGravityUsage = gravityUsage
                }
                val gyroscopeUsage = syncer.gyroscopeUsage
                if (gyroscopeUsage > maxGyroscopeUsage) {
                    maxGyroscopeUsage = gyroscopeUsage
                }

                logSyncedMeasurement(
                    syncer,
                    measurement
                )

                if (syncer.numberOfProcessedMeasurements >= 10 * syncer.gravityCapacity) {
                    syncer.stop()

                    syncHelper.notifyAll { completed++ }
                }
            },
            staleDetectedMeasurementsListener = { syncer, sensorType, measurements ->
                logStaleMeasurements(syncer, sensorType, measurements)
            }
        )

        syncer.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 100000L)

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(syncer.running)

        Log.d(
            GravityAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
            "Max gravity usage: $maxGravityUsage, " +
                    "max gyroscope usage: $maxGyroscopeUsage, " +
                    "max gravity collector usage: $maxGravityCollectorUsage, " +
                    "max gyroscope collector usage: $maxGyroscopeCollectorUsage"
        )
    }

    private fun logGravitySensor(sensor: Sensor) {
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
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReversedEventCount: $fifoReversedEventCount, "
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

    private fun logGyroscopeSensor(sensor: Sensor) {
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
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReversedEventCount: $fifoReversedEventCount, "
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

    private fun logAccuracyChanged(
        syncer: GravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        accuracy: SensorAccuracy?
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "gravityCapacity: $gravityCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "gravityCollectorUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logBufferFilled(
        syncer: GravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "gravityCapacity: $gravityCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logStaleMeasurements(
        syncer: GravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        measurements: Collection<SensorMeasurement<*>>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val numberOfStaleMeasurements = measurements.size

        Log.d(
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "staleMeasurements - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "gravityCapacity: $gravityCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "numberOfStaleMeasurements: $numberOfStaleMeasurements"
        )
    }

    private fun logSyncedMeasurement(
        syncer: GravityAndGyroscopeSensorMeasurementSyncer,
        measurement: GravityAndGyroscopeSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val gravityStartOffset = syncer.gravityStartOffset
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val timestamp = measurement.timestamp
        val gx = measurement.gravityMeasurement?.gx
        val gy = measurement.gravityMeasurement?.gy
        val gz = measurement.gravityMeasurement?.gz
        val gravityTimestamp = measurement.gravityMeasurement?.timestamp
        val wx = measurement.gyroscopeMeasurement?.wx
        val wy = measurement.gyroscopeMeasurement?.wy
        val wz = measurement.gyroscopeMeasurement?.wz
        val wbx = measurement.gyroscopeMeasurement?.bx
        val wby = measurement.gyroscopeMeasurement?.by
        val wbz = measurement.gyroscopeMeasurement?.bz
        val gyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp

        Log.d(
            "GravityAndGyroscopeSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "gravityCapacity: $gravityCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "timestamp: $timestamp, gx: $gx, gy: $gy, gz: $gz, " +
                    "gravityTimestamp: $gravityTimestamp, wx: $wx, wy: $wy, wz: $wz, " +
                    "wbx: $wbx, wby: $wby, wbz: $wbz, gyroscopeTimestamp: $gyroscopeTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}