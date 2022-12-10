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
class AccelerometerAndGyroscopeSensorMeasurementSyncerTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    private var timestamp = 0L
    private var accelerometerTimestamp = 0L
    private var gyroscopeTimestamp = 0L
    private var maxAccelerometerUsage = -1.0f
    private var maxGyroscopeUsage = -1.0f

    @Before
    fun setUp() {
        completed = 0

        timestamp = 0L
        accelerometerTimestamp = 0L
        gyroscopeTimestamp = 0L
        maxAccelerometerUsage = -1.0f
        maxGyroscopeUsage = -1.0f
    }

    @Test
    fun accelerometerSensor_whenAccelerometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val sensor = syncer.accelerometerSensor
        requireNotNull(sensor)

        logAccelerometerSensor(sensor)
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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun accelerometerSensorAvailable_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(syncer.accelerometerSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun gyroscopeSensorAvailable_whenGyroscopeUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertTrue(syncer.gyroscopeSensorAvailable)
    }

    @Test
    fun startAndStop_whenStartOffsetsEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    @Test
    fun startAndStop_whenFilledBufferEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = true,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    @Test
    fun startAndStop_whenOutOfOrderEnabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = true
        )
    }

    @Test
    fun startAndStop_whenAllFlagsDisabled_notifiesMeasurements() {
        runTest(
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false
        )
    }

    private fun runTest(
        accelerometerStartOffsetEnabled: Boolean,
        gyroscopeStartOffsetEnabled: Boolean,
        stopWhenFilledBuffer: Boolean,
        outOfOrderDetectionEnabled: Boolean,
        stopWhenOutOfOrder: Boolean
    ) {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context,
            accelerometerStartOffsetEnabled = accelerometerStartOffsetEnabled,
            gyroscopeStartOffsetEnabled = gyroscopeStartOffsetEnabled,
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

                requireNotNull(measurement.gyroscopeMeasurement)
                requireNotNull(measurement.accelerometerMeasurement)

                val currentGyroscopeTimestamp = measurement.gyroscopeMeasurement?.timestamp
                requireNotNull(currentGyroscopeTimestamp)
                assertTrue(currentGyroscopeTimestamp > gyroscopeTimestamp)

                val currentAccelerometerTimestamp = measurement.accelerometerMeasurement?.timestamp
                requireNotNull(currentAccelerometerTimestamp)
                assertTrue(currentAccelerometerTimestamp >= accelerometerTimestamp)

                timestamp = measurement.timestamp
                gyroscopeTimestamp = currentGyroscopeTimestamp
                accelerometerTimestamp = currentAccelerometerTimestamp

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
            AccelerometerAndGyroscopeSensorMeasurementSyncerTest::class.simpleName,
            "Max accelerometer usage: $maxAccelerometerUsage, " +
                    "max gyroscope usage: $maxGyroscopeUsage"
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
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
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
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
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
        syncer: AccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType,
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
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "accuracyChanged - sensorType: $sensorType, accuracy: $accuracy, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logBufferFilled(
        syncer: AccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        Log.d(
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "bufferFilled - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $gyroscopeUsage"
        )
    }

    private fun logOutOfOrderMeasurement(
        syncer: AccelerometerAndGyroscopeSensorMeasurementSyncer,
        sensorType: SensorMeasurementSyncer.SensorType,
        measurement: SensorMeasurement<*>
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val measurementTimestamp = measurement.timestamp

        Log.d(
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "outOfOrderMeasurement - sensorType: $sensorType, " +
                    "startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $gyroscopeUsage, " +
                    "measurementTimestamp: $measurementTimestamp"
        )
    }

    private fun logSyncedMeasurement(
        syncer: AccelerometerAndGyroscopeSensorMeasurementSyncer,
        measurement: AccelerometerAndGyroscopeSyncedSensorMeasurement
    ) {
        val startTime = syncer.startTimestamp
        val running = syncer.running
        val numberOfProcessedMeasurements = syncer.numberOfProcessedMeasurements
        val mostRecentTimestamp = syncer.mostRecentTimestamp
        val oldestTimestamp = syncer.oldestTimestamp
        val accelerometerSensorType = syncer.accelerometerSensorType
        val gyroscopeSensorType = syncer.gyroscopeSensorType
        val accelerometerSensorDelay = syncer.accelerometerSensorDelay
        val gyroscopeSensorDelay = syncer.gyroscopeSensorDelay
        val accelerometerCapacity = syncer.accelerometerCapacity
        val gyroscopeCapacity = syncer.gyroscopeCapacity
        val accelerometerStartOffsetEnabled = syncer.accelerometerStartOffsetEnabled
        val gyroscopeStartOffsetEnabled = syncer.gyroscopeStartOffsetEnabled
        val stopWhenFilledBuffer = syncer.stopWhenFilledBuffer
        val stopWhenOutOfOrder = syncer.stopWhenOutOfOrder
        val accelerometerSensorAvailable = syncer.accelerometerSensorAvailable
        val gyroscopeSensorAvailable = syncer.gyroscopeSensorAvailable
        val accelerometerStartOffset = syncer.accelerometerStartOffset
        val gyroscopeStartOffset = syncer.gyroscopeStartOffset
        val accelerometerUsage = syncer.accelerometerUsage
        val gyroscopeUsage = syncer.gyroscopeUsage

        val timestamp = measurement.timestamp
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
            "AccelerometerAndGyroscopeSensorMeasurementSyncerTest",
            "syncedMeasurement - startTime: $startTime, running: $running, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "oldestTimestamp: $oldestTimestamp, " +
                    "accelerometerSensorType: $accelerometerSensorType, " +
                    "gyroscopeSensorType: $gyroscopeSensorType, " +
                    "accelerometerSensorDelay: $accelerometerSensorDelay, " +
                    "gyroscopeSensorDelay: $gyroscopeSensorDelay, " +
                    "accelerometerCapacity: $accelerometerCapacity," +
                    "gyroscopeCapacity: $gyroscopeCapacity, " +
                    "accelerometerStartOffsetEnabled: $accelerometerStartOffsetEnabled, " +
                    "gyroscopeStartOffsetEnabled: $gyroscopeStartOffsetEnabled, " +
                    "stopWhenFilledBuffer: $stopWhenFilledBuffer, " +
                    "stopWhenOutOfOrder: $stopWhenOutOfOrder, " +
                    "accelerometerSensorAvailable: $accelerometerSensorAvailable, " +
                    "gyroscopeSensorAvailable: $gyroscopeSensorAvailable, " +
                    "accelerometerStartOffset: $accelerometerStartOffset, " +
                    "gyroscopeStartOffset: $gyroscopeStartOffset, " +
                    "accelerometerUsage: $accelerometerUsage, gyroscopeUsage: $gyroscopeUsage, " +
                    "timestamp: $timestamp, ax: $ax, ay: $ay, az: $az, " +
                    "abx: $abx, aby: $aby, abz: $abz, " +
                    "accelerometerTimestamp: $accelerometerTimestamp, wx: $wx, wy: $wy, wz: $wz, " +
                    "wbx: $wbx, wby: $wby, wbz: $wbz, gyroscopeTimestamp: $gyroscopeTimestamp"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}