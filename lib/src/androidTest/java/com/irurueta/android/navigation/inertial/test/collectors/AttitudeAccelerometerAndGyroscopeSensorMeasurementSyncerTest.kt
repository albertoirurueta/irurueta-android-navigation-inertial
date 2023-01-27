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
import org.junit.Assert
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

@RequiresDevice
class AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var attitudeTimestamp = 0L
    private var accelerometerTimestamp = 0L
    private var gyroscopeTimestamp = 0L
    private var maxAttitudeUsage = -1.0f
    private var maxAccelerometerUsage = -1.0f
    private var maxGyroscopeUsage = -1.0f
    private var maxAttitudeCollectorUsage = -1.0f
    private var maxAccelerometerCollectorUsage = -1.0f
    private var maxGyroscopeCollectorUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        attitudeTimestamp = 0L
        accelerometerTimestamp = 0L
        gyroscopeTimestamp = 0L
        maxAttitudeUsage = -1.0f
        maxAccelerometerUsage = -1.0f
        maxGyroscopeUsage = -1.0f
        maxAttitudeCollectorUsage = -1.0f
        maxAccelerometerCollectorUsage = -1.0f
        maxGyroscopeCollectorUsage = -1.0f
    }

    @Test
    fun attitudeSensor_whenAbsoluteSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE
        )

        val sensor = syncer.attitudeSensor
        requireNotNull(sensor)

        logAttitudeSensor(sensor)
    }

    @Test
    fun attitudeSensor_whenRelativeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeSensorType = AttitudeSensorType.RELATIVE_ATTITUDE
        )

        val sensor = syncer.attitudeSensor
        requireNotNull(sensor)

        logAttitudeSensor(sensor)
    }

    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        val sensor = syncer.attitudeSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
    }

    @Test
    fun accelerometerSensor_whenAccelerometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = syncer.attitudeSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
    }

    @Test
    fun gyroscopeSensor_whenGyroscopeSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        val sensor = syncer.gyroscopeSensor
        requireNotNull(sensor)

        logGyroscopeSensor(sensor)
    }

    @Test
    fun attitudeSensorAvailable_whenAbsoluteSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE
        )

        assertTrue(syncer.attitudeSensorAvailable)
    }

    @Test
    fun attitudeSensorAvailable_whenRelativeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeSensorType = AttitudeSensorType.RELATIVE_ATTITUDE
        )

        assertTrue(syncer.attitudeSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetsEnabled_notifiesMeasurements() {
        runTest(
            attitudeStartOffsetEnabled = true,
            accelerometerStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            stopWhenFilledBuffer = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            attitudeStartOffsetEnabled = false,
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            attitudeStartOffsetEnabled = false,
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = false
        )
    }

    private fun runTest(
        attitudeStartOffsetEnabled: Boolean,
        accelerometerStartOffsetEnabled: Boolean,
        gyroscopeStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeStartOffsetEnabled = attitudeStartOffsetEnabled,
            accelerometerStartOffsetEnabled = accelerometerStartOffsetEnabled,
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
                requireNotNull(measurement.accelerometerMeasurement)
                requireNotNull(measurement.attitudeMeasurement)

                val currentGyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp
                requireNotNull(currentGyroscopeTimestamp)
                assertTrue(currentGyroscopeTimestamp >= gyroscopeTimestamp)

                val currentAccelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
                requireNotNull(currentAccelerometerTimestamp)
                assertTrue(currentAccelerometerTimestamp >= accelerometerTimestamp)

                val currentAttitudeTimestamp = measurement.attitudeMeasurement?.timestamp
                requireNotNull(currentAttitudeTimestamp)
                if (currentAttitudeTimestamp < attitudeTimestamp) {
                    Log.d(
                        AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
                        "Max attitude usage: $maxAttitudeUsage, " +
                                "max accelerometer usage: $maxAccelerometerUsage, " +
                                "max gyroscope usage: $maxGyroscopeUsage, " +
                                "max attitude collector usage: $maxAttitudeCollectorUsage, " +
                                "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                                "max gyroscope collector usage: $maxGyroscopeCollectorUsage"
                    )
                }
                assertTrue(currentAttitudeTimestamp >= attitudeTimestamp)

                timestamp = measurement.timestamp
                gyroscopeTimestamp = currentGyroscopeTimestamp
                accelerometerTimestamp = currentAccelerometerTimestamp
                attitudeTimestamp = currentAttitudeTimestamp

                val attitudeCollectorUsage = syncer.attitudeCollectorUsage
                if (attitudeCollectorUsage > maxAttitudeCollectorUsage) {
                    maxAttitudeCollectorUsage = attitudeCollectorUsage
                }
                val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
                if (accelerometerCollectorUsage > maxAccelerometerCollectorUsage) {
                    maxAccelerometerCollectorUsage = accelerometerCollectorUsage
                }
                val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
                if (gyroscopeCollectorUsage > maxGyroscopeCollectorUsage) {
                    maxGyroscopeCollectorUsage = gyroscopeCollectorUsage
                }
                val attitudeUsage = syncer.attitudeUsage
                if (attitudeUsage > maxAttitudeUsage) {
                    maxAttitudeUsage = attitudeUsage
                }
                val accelerometerUsage = syncer.accelerometerUsage
                if (accelerometerUsage > maxAccelerometerUsage) {
                    maxAccelerometerUsage = accelerometerUsage
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

        Assert.assertFalse(syncer.running)

        Log.d(
            AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
            "Max attitude usage: $maxAttitudeUsage, " +
                    "max accelerometer usage: $maxAccelerometerUsage, " +
                    "max gyroscope usage: $maxGyroscopeUsage, " +
                    "max attitude collector usage: $maxAttitudeCollectorUsage, " +
                    "max accelerometer collector usage: $maxAccelerometerCollectorUsage, " +
                    "max gyroscope collector usage: $maxGyroscopeCollectorUsage"
        )
    }

    private fun logAttitudeSensor(sensor: Sensor) {
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
        val maximumRange = sensor.maximumRange // unitless
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
        val resolution = sensor.resolution // unitless
        val stringType = sensor.stringType
        val type = sensor.type
        val vendor = sensor.vendor
        val version = sensor.version
        val additionInfoSupported = sensor.isAdditionalInfoSupported
        val dynamicSensor = sensor.isDynamicSensor
        val wakeUpSensor = sensor.isWakeUpSensor

        Log.d(
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "Sensor - fifoMaxEventCount: $fifoMaxEventCount, "
                    + "fifoReservedEventCount: $fifoReservedEventCount, "
                    + "highestDirectReportRateLevel: $highestDirectReportRateLevel, "
                    + "highestDirectReportRateLevelName: $highestDirectReportRateLevelName, "
                    + "id: $id, "
                    + "maxDelay: $maxDelay µs, "
                    + "maximumRange: $maximumRange unitless, "
                    + "minDelay: $minDelay µs, "
                    + "name: $name, "
                    + "power: $power mA, "
                    + "reportingMode: $reportingMode, "
                    + "reportingModeName: $reportingModeName, "
                    + "resolution: $resolution unitless, "
                    + "stringType: $stringType, "
                    + "type: $type, "
                    + "vendor: $vendor, "
                    + "version: $version, "
                    + "additionInfoSupported: $additionInfoSupported, "
                    + "dynamicSensor: $dynamicSensor, "
                    + "wakeUpSensor: $wakeUpSensor"
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
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
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
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
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
        syncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        accuracy: SensorAccuracy?
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val attitudeSensorType = syncer.attitudeSensorType
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val attitudeSensorDelay = syncer.attitudeSensorDelay
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val attitudeCapacity = syncer.attitudeCapacity
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val attitudeStartOffsetEnabled = syncer.attitudeStartOffsetEnabled
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val attitudeSensorAvailable = syncer.attitudeSensorAvailable
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val attitudeStartOffset = syncer.attitudeStartOffset
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val attitudeCollectorUsage = syncer.attitudeCollectorUsage
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val attitudeUsage = syncer.attitudeUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "attitudeSensorType: $attitudeSensorType, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "attitudeSensorDelay: $attitudeSensorDelay, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "attitudeCapacity: $attitudeCapacity, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "attitudeStartOffsetEnabled: $attitudeStartOffsetEnabled, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "attitudeSensorAvailable: $attitudeSensorAvailable, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "attitudeStartOffset: $attitudeStartOffset, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "attitudeCollectorUsage: $attitudeCollectorUsage, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "attitudeUsage: $attitudeUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logBufferFilled(
        syncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val attitudeSensorType = syncer.attitudeSensorType
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val attitudeSensorDelay = syncer.attitudeSensorDelay
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val attitudeCapacity = syncer.attitudeCapacity
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val attitudeStartOffsetEnabled = syncer.attitudeStartOffsetEnabled
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val attitudeSensorAvailable = syncer.attitudeSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val attitudeStartOffset = syncer.attitudeStartOffset
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val attitudeCollectorUsage = syncer.attitudeCollectorUsage
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val attitudeUsage = syncer.attitudeUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "attitudeSensorType: $attitudeSensorType, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "attitudeSensorDelay: $attitudeSensorDelay, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "attitudeCapacity: $attitudeCapacity, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "attitudeStartOffsetEnabled: $attitudeStartOffsetEnabled, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "attitudeSensorAvailable: $attitudeSensorAvailable, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "attitudeStartOffset: $attitudeStartOffset, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "attitudeCollectorUsage: $attitudeCollectorUsage, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "attitudeUsage: $attitudeUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logStaleMeasurements(
        syncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorType,
        measurements: Collection<SensorMeasurement<*>>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val attitudeSensorType = syncer.attitudeSensorType
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val attitudeSensorDelay = syncer.attitudeSensorDelay
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val attitudeCapacity = syncer.attitudeCapacity
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val attitudeStartOffsetEnabled = syncer.attitudeStartOffsetEnabled
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val attitudeSensorAvailable = syncer.attitudeSensorAvailable
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val attitudeStartOffset = syncer.attitudeStartOffset
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val attitudeCollectorUsage = syncer.attitudeCollectorUsage
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val attitudeUsage = syncer.attitudeUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val numberOfStaleMeasurements = measurements.size

        Log.d(
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "staleMeasurements - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "attitudeSensorType: $attitudeSensorType, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "attitudeSensorDelay: $attitudeSensorDelay, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "attitudeCapacity: $attitudeCapacity, " +
                    "accelerometerCapacity: $accelerometerCapacity, " +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "attitudeStartOffsetEnabled: $attitudeStartOffsetEnabled, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "attitudeSensorAvailable: $attitudeSensorAvailable, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "attitudeStartOffset: $attitudeStartOffset, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "attitudeCollectorUsage: $attitudeCollectorUsage, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "attitudeUsage: $attitudeUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "numberOfStaleMeasurements: $numberOfStaleMeasurements"
        )
    }

    private fun logSyncedMeasurement(
        syncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer,
        measurement: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val attitudeSensorType = syncer.attitudeSensorType
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val attitudeSensorDelay = syncer.attitudeSensorDelay
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val attitudeCapacity = syncer.attitudeCapacity
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val attitudeStartOffsetEnabled = syncer.attitudeStartOffsetEnabled
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val attitudeSensorAvailable = syncer.attitudeSensorAvailable
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val attitudeStartOffset = syncer.attitudeStartOffset
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val attitudeCollectorUsage = syncer.attitudeCollectorUsage
        val accelerometerCollectorUsage = syncer.accelerometerCollectorUsage
        val gyroscopeCollectorUsage = syncer.gyroscopeCollectorUsage
        val attitudeUsage = syncer.attitudeUsage
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val timestamp = measurement.timestamp
        val a = measurement.attitudeMeasurement?.attitude?.a
        val b = measurement.attitudeMeasurement?.attitude?.b
        val c = measurement.attitudeMeasurement?.attitude?.c
        val d = measurement.attitudeMeasurement?.attitude?.d
        val headingAccuracy = measurement.attitudeMeasurement?.headingAccuracy
        val attitudeTimestamp = measurement.attitudeMeasurement?.timestamp
        val ax = measurement.accelerometerMeasurement?.ax
        val ay = measurement.accelerometerMeasurement?.ay
        val az = measurement.accelerometerMeasurement?.az
        val abx = measurement.accelerometerMeasurement?.bx
        val aby = measurement.accelerometerMeasurement?.by
        val abz = measurement.accelerometerMeasurement?.bz
        val accelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
        val wx = measurement.gyroscopeMeasurement?.wx
        val wy = measurement.gyroscopeMeasurement?.wy
        val wz = measurement.gyroscopeMeasurement?.wz
        val wbx = measurement.gyroscopeMeasurement?.bx
        val wby = measurement.gyroscopeMeasurement?.by
        val wbz = measurement.gyroscopeMeasurement?.bz
        val gyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp

        Log.d(
            "AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "attitudeSensorType: $attitudeSensorType, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "attitudeSensorDelay: $attitudeSensorDelay, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "attitudeCapacity: $attitudeCapacity, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "attitudeStartOffsetEnabled: $attitudeStartOffsetEnabled, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "attitudeSensorAvailable: $attitudeSensorAvailable, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "attitudeStartOffset: $attitudeStartOffset, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "attitudeCollectorUsage: $attitudeCollectorUsage, " +
                    "accelerometerCollectorUsage: $accelerometerCollectorUsage, " +
                    "gyroscopeCollectorUsage: $gyroscopeCollectorUsage, " +
                    "attitudeUsage: $attitudeUsage, " +
                    "accelerometerUsage: $accelerometerUsage, " +
                    "gyroscopeUsage: $gyroscopeUsage, " +
                    "timestamp: $timestamp, a: $a, b: $b, c: $c, d: $d, " +
                    "headingAccuracy: $headingAccuracy, attitudeTimestamp: $attitudeTimestamp" +
                    "ax: $ax, ay: $ay, az: $az, " +
                    "abx: $abx, aby: $aby, abz: $abz, " +
                    "accelerometerTimestamp: $accelerometerTimestamp, " +
                    "wx: $wx, wy: $wy, wz: $wz, " +
                    "wbx: $wbx, wby: $wby, wbz: $wbz, " +
                    "gyroscopeTimestamp: $gyroscopeTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}