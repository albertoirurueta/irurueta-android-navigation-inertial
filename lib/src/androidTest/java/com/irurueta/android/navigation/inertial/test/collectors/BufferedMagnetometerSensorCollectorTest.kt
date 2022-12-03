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
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test

class BufferedMagnetometerSensorCollectorTest {

    private val syncHelper = ThreadSyncHelper()

    @Volatile
    private var completed = 0

    @Before
    fun setUp() {
        completed = 0
    }

    @Test
    fun sensor_whenMagnetometerSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            BufferedMagnetometerSensorCollector(context, MagnetometerSensorType.MAGNETOMETER)

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @RequiresDevice
    @Test
    fun sensor_whenMagnetometerUncalibratedSensorType_returnsSensor() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        val sensor = collector.sensor
        requireNotNull(sensor)

        logSensor(sensor)
    }

    @Test
    fun sensorAvailable_whenMagnetometerSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector =
            BufferedMagnetometerSensorCollector(context, MagnetometerSensorType.MAGNETOMETER)

        assertTrue(collector.sensorAvailable)
    }

    @RequiresDevice
    @Test
    fun sensorAvailable_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
    }

    @Test
    fun startAndStop_whenMagnetometerSensorTypeStartOffsetEnabledAndStopWhenFilledBuffer_stopsWhenBuferFills() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.FASTEST,
            startOffsetEnabled = true,
            stopWhenFilledBuffer = true,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "BufferedMagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            bufferFilledListener = { collector ->
                assertTrue(collector.running)

                Log.d("BufferedMagnetometerSensorCollectorTest", "onBufferFilled")

                syncHelper.notifyAll { completed++ }
            },
            measurementListener = { collector, measurement, bufferPosition ->
                assertTrue(collector.running)
                logMeasurement(collector, measurement, bufferPosition)
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(collector.running)
    }

    @Test
    fun startAndStop_whenMagnetometerUncalibratedSensorTypeStartOffsetEnabledAndStopWhenFilledBuffer_stopsWhenBuferFills() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.FASTEST,
            startOffsetEnabled = true,
            stopWhenFilledBuffer = true,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "BufferedMagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            bufferFilledListener = { collector ->
                assertTrue(collector.running)

                Log.d("BufferedMagnetometerSensorCollectorTest", "onBufferFilled")

                syncHelper.notifyAll { completed++ }
            },
            measurementListener = { collector, measurement, bufferPosition ->
                assertTrue(collector.running)
                logMeasurement(collector, measurement, bufferPosition)
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(collector.running)
    }

    @Test
    fun startAndStop_whenMagnetometerSensorTypeStartOffsetDisabledAndStopWhenFilledBuffer_stopsWhenBuferFills() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.FASTEST,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = true,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "BufferedMagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            bufferFilledListener = { collector ->
                assertTrue(collector.running)

                Log.d("BufferedMagnetometerSensorCollectorTest", "onBufferFilled")

                syncHelper.notifyAll { completed++ }
            },
            measurementListener = { collector, measurement, bufferPosition ->
                assertTrue(collector.running)
                logMeasurement(collector, measurement, bufferPosition)
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(collector.running)
    }

    @Test
    fun startAndStop_whenMagnetometerUncalibratedSensorTypeStartOffsetDisabledAndStopWhenFilledBuffer_stopsWhenBuferFills() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.FASTEST,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = true,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "BufferedMagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            bufferFilledListener = { collector ->
                assertTrue(collector.running)

                Log.d("BufferedMagnetometerSensorCollectorTest", "onBufferFilled")

                syncHelper.notifyAll { completed++ }
            },
            measurementListener = { collector, measurement, bufferPosition ->
                assertTrue(collector.running)
                logMeasurement(collector, measurement, bufferPosition)
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(collector.running)
    }

    @Test
    fun startAndStop_whenStopWhenFilledBufferDisabled_collectsMeasurementsUntilStopped() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.FASTEST,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            accuracyChangedListener = { _, accuracy ->
                Log.d(
                    "BufferedMagnetometerSensorCollectorTest",
                    "onAccuracyChanged - accuracy: $accuracy"
                )
            },
            bufferFilledListener = { collector ->
                assertTrue(collector.running)

                Log.d("BufferedMagnetometerSensorCollectorTest", "onBufferFilled")
            },
            measurementListener = { collector, measurement, bufferPosition ->
                assertTrue(collector.running)
                logMeasurement(collector, measurement, bufferPosition)

                if (collector.numberOfProcessedMeasurements >= 2 * collector.capacity) {
                    collector.stop()

                    syncHelper.notifyAll { completed++ }
                }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)

        assertFalse(collector.running)
    }

    @Test
    fun getMeasurementsBefore_obtainsExpectedMeasurements() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val collector = BufferedMagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.FASTEST,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = true,
            bufferFilledListener = { collector ->
                assertTrue(collector.running)
                val beforeStart = collector.getMeasurementsBeforeTimestamp(collector.startTimestamp)
                assertTrue(beforeStart.isEmpty())

                val bufferCopy = collector.bufferedMeasurements
                val measurementsBefore = collector.getMeasurementsBeforeTimestamp(collector.mostRecentTimestamp)
                assertEqualAndValidMeasurements(measurementsBefore, bufferCopy)

                syncHelper.notifyAll { completed++ }
            }
        )

        collector.start()

        syncHelper.waitOnCondition({ completed < 1 })

        assertTrue(completed > 0)
        Thread.sleep(SLEEP)

        assertFalse(collector.running)
    }

    private fun assertEqualAndValidMeasurements(
        list1: List<MagnetometerSensorMeasurement>,
        list2: List<MagnetometerSensorMeasurement>
    ) {
        var previousTimestamp = -1L
        assertEquals(list1.size, list2.size)
        for (i in list1.indices) {
            assertEqualMeasurement(list1[i], list2[i])

            // ensure timestamps are monotonically increasing
            assertTrue(list1[i].timestamp > previousTimestamp)
            previousTimestamp = list1[i].timestamp
        }
    }

    private fun assertEqualMeasurement(
        measurement1: MagnetometerSensorMeasurement,
        measurement2: MagnetometerSensorMeasurement
    ) {
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
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
            "BufferedMagnetometerSensorCollectorTest",
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

    private fun logMeasurement(
        collector: BufferedMagnetometerSensorCollector,
        measurement: MagnetometerSensorMeasurement,
        bufferPosition: Int
    ) {
        val startTimestamp = collector.startTimestamp
        val startOffset = collector.startOffset
        val usage = collector.usage
        val numberOfProcessedMeasurements = collector.numberOfProcessedMeasurements
        val mostRecentTimestamp = collector.mostRecentTimestamp
        val bx = measurement.bx
        val by = measurement.by
        val bz = measurement.bz
        val hardIronX = measurement.hardIronX
        val hardIronY = measurement.hardIronY
        val hardIronZ = measurement.hardIronZ
        val timestamp = measurement.timestamp
        val accuracy = measurement.accuracy

        Log.d(
            "BufferedMagnetometerSensorCollectorTest",
            "onMeasurement - startTimestamp: $startTimestamp, startOffset: $startOffset, " +
                    "buffer position: $bufferPosition, " +
                    "usage: $usage, " +
                    "numberOfProcessedMeasurements: $numberOfProcessedMeasurements, " +
                    "mostRecentTimestamp: $mostRecentTimestamp, " +
                    "bx: $bx µT, by: $by µT, bz: $bz µT, " +
                    "hardIronX: $hardIronX µT, hardIronY: $hardIronY µT, hardIronZ: $hardIronZ µT, " +
                    "timestamp: $timestamp, accuracy: $accuracy"
        )
    }

    private companion object {
        const val SLEEP = 1000L
    }
}