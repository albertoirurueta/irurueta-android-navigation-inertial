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
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerAndGyroscopeSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.SensorType
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var accelerometerTimestamp = 0L
    private var gravityTimestamp = 0L
    private var gyroscopeTimestamp = 0L
    private var maxAccelerometerUsage = -1.0f
    private var maxGravityUsage = -1.0f
    private var maxGyroscopeUsage = -1.0f
    private var maxAccelerometerCollectorUsage = -1.0f
    private var maxGravityCollectorUsage = -1.0f
    private var maxGyroscopeCollectorUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        accelerometerTimestamp = 0L
        gravityTimestamp = 0L
        gyroscopeTimestamp = 0L
        maxAccelerometerUsage = -1.0f
        maxGravityUsage = -1.0f
        maxGyroscopeUsage = -1.0f
        maxAccelerometerCollectorUsage = -1.0f
        maxGravityCollectorUsage = -1.0f
        maxGyroscopeCollectorUsage = -1.0f
    }

    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        val sensor = syncer.gravitySensor
        requireNotNull(sensor)

        logGravitySensor(sensor)
    }

    @Test
    fun gyroscopeSensor_whenGyroscopeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        val sensor = syncer.gyroscopeSensor
        requireNotNull(sensor)

        logGyroscopeSensor(sensor)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun gravitySensorAvailable_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        assertTrue(syncer.gravitySensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetsEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = true,
            gravityStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            stopWhenFilledBuffer = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gravityStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gravityStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = false
        )
    }

    private fun runTest(
        accelerometerStartOffsetEnabled: Boolean,
        gravityStartOffsetEnabled: Boolean,
        gyroscopeStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerStartOffsetEnabled = accelerometerStartOffsetEnabled,
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
                requireNotNull(measurement.accelerometerMeasurement)

                val currentGyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp
                requireNotNull(currentGyroscopeTimestamp)
                assertTrue(currentGyroscopeTimestamp >= gyroscopeTimestamp)

                val currentGravityTimestamp = measurement.gravityMeasurement?.timestamp
                requireNotNull(currentGravityTimestamp)
                assertTrue(currentGravityTimestamp >= gravityTimestamp)

                val currentAccelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
                requireNotNull(currentAccelerometerTimestamp)
                if (currentAccelerometerTimestamp < accelerometerTimestamp) {
                    Log.d(
                        AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
                        "Max accelerometer usage: $maxAccelerometerUsage, " +
                                "max gravity usage: $maxGravityUsage, " +
                                "max gyroscope usage: $maxGyroscopeUsage, " +
                                "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                                "max gravity collector usage: $maxGravityCollectorUsage, " +
                                "max gyroscope collector usage: $maxGyroscopeCollectorUsage"
                    )
                }
                assertTrue(currentAccelerometerTimestamp >= accelerometerTimestamp)

                timestamp = measurement.timestamp
                gyroscopeTimestamp = currentGyroscopeTimestamp
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
                val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
                if (gyroscopeCollectorUsage > maxGyroscopeCollectorUsage) {
                    maxGyroscopeCollectorUsage = gyroscopeCollectorUsage
                }
                val accelerometerUsage = syncer.accelerometerUsage
                if (accelerometerUsage > maxAccelerometerUsage) {
                    maxAccelerometerUsage = accelerometerUsage
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
            AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
            "Max accelerometer usage: $maxAccelerometerUsage, " +
                    "max gravity usage: $maxGravityUsage, " +
                    "max gyroscope usage: $maxGyroscopeUsage, " +
                    "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                    "max gravity collector usage: $maxGravityCollectorUsage, " +
                    "max gyroscope collector usage: $maxGyroscopeCollectorUsage"
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
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
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

    private fun logGyroscopeSensor(sensor: Sensor) {
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
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
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

    private fun logAccuracyChanged(
        syncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        accuracy: SensorAccuracy?
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gravityCapacity: $gravityCapacity, " +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logBufferFilled(
        syncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gravityCapacity: $gravityCapacity, " +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logStaleMeasurements(
        syncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        measurements: Collection<SensorMeasurement<*>>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val numberOfStaleMeasurements = measurements.size

        Log.d(
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
            "staleMeasurements - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity, " +
                    "gravityCapacity: $gravityCapacity, " +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "numberOfStaleMeasurements: $numberOfStaleMeasurements"
        )
    }

    private fun logSyncedMeasurement(
        syncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
        measurement: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gravitySensorDelay = syncer.gravitySensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gravityCapacity = syncer.gravityCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gravityStartOffsetEnabled = syncer.gravityStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gravitySensorAvailable = syncer.gravitySensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gravityStartOffset = syncer.gravityStartOffset
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val gravityCollectorUsage = syncer.gravityCollectorUsage
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gravityUsage = syncer.gravityUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

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
        val wx = measurement.gyroscopeMeasurement?.wx
        val wy = measurement.gyroscopeMeasurement?.wy
        val wz = measurement.gyroscopeMeasurement?.wz
        val wbx = measurement.gyroscopeMeasurement?.bx
        val wby = measurement.gyroscopeMeasurement?.by
        val wbz = measurement.gyroscopeMeasurement?.bz
        val gyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp

        Log.d(
            "AccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gravitySensorDelay: $gravitySensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "gravityCapacity: $gravityCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gravityStartOffsetEnabled: $gravityStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "gravitySensorAvailable: $gravitySensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gravityStartOffset: $gravityStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gravityCollectorUsage: $gravityCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gravityUsage: $gravityUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "timestamp: $timestamp, ax: $ax, ay: $ay, az: $az, " +
                    "abx: $abx, aby: $aby, abz: $abz, " +
                    "accelerometerTimestamp: $accelerometerTimestamp, " +
                    "gx: $gx, gy: $gy, gz: $gz, " +
                    "gravityTimestamp: $gravityTimestamp, " +
                    "wx: $wx, wy: $wy, wz: $wz, " +
                    "wbx: $wbx, wby: $wby, wbz: $wbz, " +
                    "gyroscopeTimestamp: $gyroscopeTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}