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
import com.irurueta.android.navigation.inertial.collectors.*
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AccelerometerAndMagnetometerSensorMeasurementSyncerTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var accelerometerTimestamp = 0L
    private var magnetometerTimestamp = 0L
    private var maxAccelerometerUsage = -1.0f
    private var maxMagnetometerUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        accelerometerTimestamp = 0L
        magnetometerTimestamp = 0L
        maxAccelerometerUsage = -1.0f
        maxMagnetometerUsage = -1.0f
    }

    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
    }

    @Test
    fun accelerometerSensor_whenAccelerometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
    }

    @Test
    fun magnetometerSensor_whenMagnetometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        val sensor = syncer.magnetometerSensor
        requireNotNull(sensor)

        logMagnetometerSensor(sensor)
    }

    @Test
    fun magnetometerSensor_whenMagnetometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        val sensor = syncer.magnetometerSensor
        requireNotNull(sensor)

        logMagnetometerSensor(sensor)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenMagnetometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        assertTrue(syncer.magnetometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(syncer.magnetometerSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetsEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = true,
            magnetometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = true,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    @Test
    fun startAndStop_whenOutOfOrderEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    private fun runTest(
        accelerometerStartOffsetEnabled: Boolean,
        magnetometerStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean,
        outOfOrderDetectionEnabled: Boolean,
        stopWhenOutOfOrder: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context,
            accelerometerStartOffsetEnabled = accelerometerStartOffsetEnabled,
            magnetometerStartOffsetEnabled = magnetometerStartOffsetEnabled,
            stopWhenFilledBuffer = stopWhenFilledBuffer,
            outOfOrderDetectionEnabled = outOfOrderDetectionEnabled,
            stopWhenOutOfOrder = stopWhenOutOfOrder,
            accuracyChangedListener = { syncer, sensorType, accuracy ->
                logAccuracyChanged(
                    syncer,
                    sensorType,
                    accuracy
                )
            },
            bufferFilledListener = { syncer, sensorType -> logBufferFilled(syncer, sensorType) },
            outOfOrderMeasurementListener = { syncer, sensorType, measurement ->
                logOutOfOrderMeasurement(
                    syncer,
                    sensorType,
                    measurement
                )
            },
            syncedMeasurementListener = { syncer, measurement ->
                assertTrue(measurement.timestamp > timestamp)

                requireNotNull(measurement.magnetometerMeasurement)
                requireNotNull(measurement.accelerometerMeasurement)

                val currentMagnetometerTimestamp = measurement.magnetometerMeasurement?.timestamp
                requireNotNull(currentMagnetometerTimestamp)
                assertTrue(currentMagnetometerTimestamp > magnetometerTimestamp)

                val currentAccelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
                requireNotNull(currentAccelerometerTimestamp)
                assertTrue(currentAccelerometerTimestamp >= accelerometerTimestamp)

                timestamp = measurement.timestamp
                magnetometerTimestamp = currentMagnetometerTimestamp
                accelerometerTimestamp = currentAccelerometerTimestamp

                val accelerometerUsage = syncer.accelerometerUsage
                if (accelerometerUsage > maxAccelerometerUsage) {
                    maxAccelerometerUsage = accelerometerUsage
                }
                val magnetometerUsage = syncer.magnetometerUsage
                if (magnetometerUsage > maxMagnetometerUsage) {
                    maxMagnetometerUsage = magnetometerUsage
                }

                logSyncedMeasurement(
                    syncer,
                    measurement
                )

                if (syncer.numberOfProcessedMeasurements >= 2 * syncer.accelerometerCapacity) {
                    syncer.stop()

                    syncHelper.notifyAll { completed++ }
                }
            }
        )

        syncer.start()

        syncHelper.waitOnCondition({ completed < 1 }, timeout = 100000L)

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(syncer.running)

        Log.d(
            AccelerometerAndMagnetometerSensorMeasurementSyncerTest::class.simpleName,
            "Max accelerometer usage: $maxAccelerometerUsage, " +
                    "max magnetometer usage: $maxMagnetometerUsage"
        )
    }

    private fun logAccelerometerSensor(sensor: Sensor) {
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
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
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

    private fun logMagnetometerSensor(sensor: Sensor) {
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
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReversedEventCount: $fifoReversedEventCount, "
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

    private fun logAccuracyChanged(
        syncer: AccelerometerAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType,
        accuracy: SensorAccuracy?
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val magnetometerUsage = syncer.magnetometerUsage

        Log.d(
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $magnetometerStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $magnetometerUsage"
        )
    }

    private fun logBufferFilled(
        syncer: AccelerometerAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val magnetometerUsage = syncer.magnetometerUsage

        Log.d(
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $magnetometerStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $magnetometerUsage"
        )
    }

    private fun logOutOfOrderMeasurement(
        syncer: AccelerometerAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType,
        measurement: SensorMeasurement<*>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val magnetometerUsage = syncer.magnetometerUsage

        val measurementTimestamp = measurement.timestamp

        Log.d(
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
            "outOfOrderMeasurement - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $magnetometerStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $magnetometerUsage, " +
                    "measurementTimestamp: $measurementTimestamp"
        )
    }

    private fun logSyncedMeasurement(
        syncer: AccelerometerAndMagnetometerSensorMeasurementSyncer,
        measurement: AccelerometerAndMagnetometerSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val magnetometerUsage = syncer.magnetometerUsage

        val timestamp = measurement.timestamp
        val ax = measurement.accelerometerMeasurement?.ax
        val ay = measurement.accelerometerMeasurement?.ay
        val az = measurement.accelerometerMeasurement?.az
        val abx = measurement.accelerometerMeasurement?.bx
        val aby = measurement.accelerometerMeasurement?.by
        val abz = measurement.accelerometerMeasurement?.bz
        val accelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
        val bx = measurement.magnetometerMeasurement?.bx
        val by = measurement.magnetometerMeasurement?.by
        val bz = measurement.magnetometerMeasurement?.bz
        val hardIronX = measurement.magnetometerMeasurement?.hardIronX
        val hardIronY = measurement.magnetometerMeasurement?.hardIronY
        val hardIronZ = measurement.magnetometerMeasurement?.hardIronZ
        val magnetometerTimestamp = measurement.magnetometerMeasurement?.timestamp

        Log.d(
            "AccelerometerAndMagnetometerSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $magnetometerStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $magnetometerUsage, " +
                    "timestamp: $timestamp, ax: $ax, ay: $ay, az: $az, " +
                    "abx: $abx, aby: $aby, abz: $abz, " +
                    "accelerometerTimestamp: $accelerometerTimestamp, bx: $bx, by: $by, bz: $bz, " +
                    "hardIronX: $hardIronX, hardIronY: $hardIronY, hardIronZ: $hardIronZ, " +
                    "magnetometerTimestamp: $magnetometerTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}