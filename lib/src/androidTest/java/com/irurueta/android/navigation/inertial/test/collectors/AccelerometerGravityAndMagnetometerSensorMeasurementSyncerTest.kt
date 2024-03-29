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
class AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var accelerometerTimestamp = 0L
    private var gravityTimestamp = 0L
    private var magnetometerTimestamp = 0L
    private var maxAccelerometerUsage = -1.0f
    private var maxGravityUsage = -1.0f
    private var maxMagnetometerUsage = -1.0f
    private var maxAccelerometerCollectorUsage = -1.0f
    private var maxGravityCollectorUsage = -1.0f
    private var maxMagnetometerCollectorUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        accelerometerTimestamp = 0L
        gravityTimestamp = 0L
        magnetometerTimestamp = 0L
        maxAccelerometerUsage = -1.0f
        maxGravityUsage = -1.0f
        maxMagnetometerUsage = -1.0f
        maxAccelerometerCollectorUsage = -1.0f
        maxGravityCollectorUsage = -1.0f
        maxMagnetometerCollectorUsage = -1.0f
    }

    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
    }

    @Test
    fun gravitySensor_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        val sensor = syncer.gravitySensor
        requireNotNull(sensor)

        logGravitySensor(sensor)
    }

    @Test
    fun magnetometerSensor_whenMagnetometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logMagnetometerSensor(sensor)
    }

    @Test
    fun magnetometerSensor_whenMagnetometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logMagnetometerSensor(sensor)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun gravitySensorAvailable_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        assertTrue(syncer.gravitySensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenMagnetometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER
        )

        assertTrue(syncer.magnetometerSensorAvailable)
    }

    @Test
    fun magnetometerSensorAvailable_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(syncer.magnetometerSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = true,
            gravityStartOffsetEnabled = true,
            magnetometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gravityStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gravityStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = false
        )
    }

    private fun runTest(
        accelerometerStartOffsetEnabled: Boolean,
        gravityStartOffsetEnabled: Boolean,
        magnetometerStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerStartOffsetEnabled = accelerometerStartOffsetEnabled,
            gravityStartOffsetEnabled = gravityStartOffsetEnabled,
            magnetometerStartOffsetEnabled = magnetometerStartOffsetEnabled,
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

                requireNotNull(measurement.magnetometerMeasurement)
                requireNotNull(measurement.gravityMeasurement)
                requireNotNull(measurement.accelerometerMeasurement)

                val currentMagnetometerTimestamp = measurement.magnetometerMeasurement?.timestamp
                requireNotNull(currentMagnetometerTimestamp)
                assertTrue(currentMagnetometerTimestamp >= magnetometerTimestamp)

                val currentGravityTimestamp = measurement.gravityMeasurement?.timestamp
                requireNotNull(currentGravityTimestamp)
                assertTrue(currentGravityTimestamp >= gravityTimestamp)

                val currentAccelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
                requireNotNull(currentAccelerometerTimestamp)
                if (currentAccelerometerTimestamp < accelerometerTimestamp) {
                    Log.d(
                        AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest::class.simpleName,
                        "Max accelerometer usage: $maxAccelerometerUsage, " +
                                "max gravity usage: $maxGravityUsage, " +
                                "max magnetometer usage: $maxMagnetometerUsage, " +
                                "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                                "max gravity collector usage: $maxGravityCollectorUsage, " +
                                "max magnetometer collector usage: $maxMagnetometerCollectorUsage"
                    )
                }
                assertTrue(currentAccelerometerTimestamp >= accelerometerTimestamp)

                timestamp = measurement.timestamp
                magnetometerTimestamp = currentMagnetometerTimestamp
                gravityTimestamp = currentGravityTimestamp
                accelerometerTimestamp = currentAccelerometerTimestamp

                val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
                if (accelerometerCollectorUsage > maxAccelerometerCollectorUsage) {
                    maxAccelerometerCollectorUsage = accelerometerCollectorUsage
                }
                val gravityCollectorUsage = syncer.gravityCollectorUsage
                if (gravityCollectorUsage > maxGravityCollectorUsage) {
                    maxGravityCollectorUsage = gravityCollectorUsage
                }
                val magnetometerCollectorUsage = syncer.magnetometerCollectorUsage
                if (magnetometerCollectorUsage > maxMagnetometerCollectorUsage) {
                    maxMagnetometerCollectorUsage = magnetometerCollectorUsage
                }
                val accelerometerUsage = syncer.accelerometerUsage
                if (accelerometerUsage > maxAccelerometerUsage) {
                    maxAccelerometerUsage = accelerometerUsage
                }
                val gravityUsage = syncer.gravityUsage
                if (gravityUsage > maxGravityUsage) {
                    maxGravityUsage = gravityUsage
                }
                val magnetometerUsage = syncer.magnetometerUsage
                if (magnetometerUsage > maxMagnetometerUsage) {
                    maxMagnetometerUsage = magnetometerUsage
                }

                logSyncedMeasurement(
                    syncer,
                    measurement
                )

                if (syncer.numberOfProcessedMeasurements >= 10 * syncer.accelerometerCapacity) {
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
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest::class.simpleName,
            "Max accelerometer usage: $maxAccelerometerUsage, " +
                    "max gravity usage: $maxGravityUsage, " +
                    "max magnetometer usage: $maxMagnetometerUsage, " +
                    "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                    "max gravity collector usage: $maxGravityCollectorUsage, " +
                    "max magnetometer collector usage: $maxMagnetometerCollectorUsage"
        )
    }

    private fun logAccelerometerSensor(sensor: Sensor) {
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
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
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

    private fun logGravitySensor(sensor: Sensor) {
        logAccelerometerSensor(sensor)
    }

    private fun logMagnetometerSensor(sensor: Sensor) {
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
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
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

    private fun logAccuracyChanged(
        syncer: AccelerometerGravityAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorType,
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
        val gravitySensorDelay = syncer.gravitySensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val magnetometerCollectorUsage = syncer.magnetometerCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val magnetometerUsage = syncer.magnetometerUsage

        Log.d(
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "magnetometerSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "magnetometerSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gravityCapacity: $gravityCapacity, " +
                    "magnetometerCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "magnetometerStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "magnetometerSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "magnetometerStartOffset: $magnetometerStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "magnetometerCollectorUsage: $magnetometerCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "magnetometerUsage: $magnetometerUsage"
        )
    }

    private fun logBufferFilled(
        syncer: AccelerometerGravityAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val magnetometerCollectorUsage = syncer.magnetometerCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val magnetometerUsage = syncer.magnetometerUsage

        Log.d(
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "magnetometerSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "magnetometerSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gravityCapacity: $gravityCapacity, " +
                    "magnetometerCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "magnetometerStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "magnetometerSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "magnetometerStartOffset: $magnetometerStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "magnetometerCollectorUsage: $magnetometerCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "magnetometerUsage: $magnetometerUsage"
        )
    }

    private fun logStaleMeasurements(
        syncer: AccelerometerGravityAndMagnetometerSensorMeasurementSyncer,
        sensorType: SensorType,
        measurements: Collection<SensorMeasurement<*>>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val magnetometerCollectorUsage = syncer.magnetometerCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val magnetometerUsage = syncer.magnetometerUsage

        val numberOfStaleMeasurements = measurements.size

        Log.d(
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
            "staleMeasurements - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "magnetometerSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "magnetometerSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity, " +
                    "gravityCapacity: $gravityCapacity, " +
                    "magnetometerCapacity: $magnetometerCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "magnetometerStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "magnetometerSensorAvailable: $magnetometerSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "magnetometerStartOffset: $magnetometerStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "magnetometerCollectorUsage: $magnetometerCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "magnetometerUsage: $magnetometerUsage, " +
                    "numberOfStaleMeasurements: $numberOfStaleMeasurements"
        )
    }

    private fun logSyncedMeasurement(
        syncer: AccelerometerGravityAndMagnetometerSensorMeasurementSyncer,
        measurement: AccelerometerGravityAndMagnetometerSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val magnetometerSensorType = syncer.magnetometerSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val magnetometerSensorDelay = syncer.magnetometerSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val magnetometerCapacity = syncer.magnetometerCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val magnetometerStartOffsetEnabled = syncer.magnetometerStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val magnetometerSensorAvailable = syncer.magnetometerSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val magnetometerCollectorUsage = syncer.magnetometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val magnetometerStartOffset = syncer.magnetometerStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val magnetometerUsage = syncer.magnetometerUsage

        val timestamp = measurement.timestamp
        val ax = measurement.accelerometerMeasurement?.ax
        val ay = measurement.accelerometerMeasurement?.ay
        val az = measurement.accelerometerMeasurement?.az
        val abx = measurement.accelerometerMeasurement?.bx
        val aby = measurement.accelerometerMeasurement?.by
        val abz = measurement.accelerometerMeasurement?.bz
        val accelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
        val gx = measurement.gravityMeasurement?.gx
        val gy = measurement.gravityMeasurement?.gy
        val gz = measurement.gravityMeasurement?.gz
        val gravityTimestamp = measurement.gravityMeasurement?.timestamp
        val bx = measurement.magnetometerMeasurement?.bx
        val by = measurement.magnetometerMeasurement?.by
        val bz = measurement.magnetometerMeasurement?.bz
        val hardIronX = measurement.magnetometerMeasurement?.hardIronX
        val hardIronY = measurement.magnetometerMeasurement?.hardIronY
        val hardIronZ = measurement.magnetometerMeasurement?.hardIronZ
        val magnetometerTimestamp = measurement.magnetometerMeasurement?.timestamp

        Log.d(
            "AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "magnetometerSensorType: $magnetometerSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "magnetometerSensorDelay: $magnetometerSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "magnetometerCapacity: $magnetometerCapacity, " +
                    "gravityCapacity: $gravityCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "magnetometerStartOffsetEnabled: $magnetometerStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "magnetometerSensorAvailable: $magnetometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "magnetometerStartOffset: $magnetometerStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "magnetometerCollectorUsage: $magnetometerCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "magnetometerUsage: $magnetometerUsage, " +
                    "timestamp: $timestamp, ax: $ax, ay: $ay, az: $az, " +
                    "abx: $abx, aby: $aby, abz: $abz, " +
                    "accelerometerTimestamp: $accelerometerTimestamp, " +
                    "gx: $gx, gy: $gy, gz: $gz, " +
                    "gravityTimestamp: $gravityTimestamp, " +
                    "bx: $bx, by: $by, bz: $bz, " +
                    "hardIronX: $hardIronX, hardIronY: $hardIronY, hardIronZ: $hardIronZ, " +
                    "magnetometerTimestamp: $magnetometerTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}