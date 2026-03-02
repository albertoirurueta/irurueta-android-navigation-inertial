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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt

class AverageAndVarianceSensorMeasurementProcessorTest {

    @Test
    fun constructor_whenDefaultValues_setsExpectedProperties() {
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        // check
        assertEquals(
            AverageAndVarianceSensorMeasurementProcessor.DEFAULT_WINDOW_NANOSECONDS,
            processor.windowNanoseconds
        )
        assertEquals(0, processor.count)
        assertNull(processor.average)
        assertNull(processor.variance)
    }

    @Test
    fun constructor_whenAllValues_setsExpectedProperties() {
        val randomizer = UniformRandomizer()
        val windowNanoSeconds = abs(randomizer.nextLong())

        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>(
                windowNanoSeconds
            )

        // check
        assertEquals(
            windowNanoSeconds,
            processor.windowNanoseconds
        )
        assertEquals(0, processor.count)
        assertNull(processor.average)
        assertNull(processor.variance)
    }

    @Test
    fun constructor_whenNegativeWindowNanoseconds_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>(
                -1L
            )
        }
    }

    @Test
    fun windowNanoseconds_whenValid_setsExpectedValue() {
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        // check default value
        assertEquals(
            AverageAndVarianceSensorMeasurementProcessor.DEFAULT_WINDOW_NANOSECONDS,
            processor.windowNanoseconds
        )

        // set new value
        val randomizer = UniformRandomizer()
        val windowNanoSeconds = abs(randomizer.nextLong())

        processor.windowNanoseconds = windowNanoSeconds

        // check
        assertEquals(
            windowNanoSeconds,
            processor.windowNanoseconds
        )
    }

    @Test
    fun windowNanoseconds_whenNegative_throwsIllegalArgumentException() {
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowNanoseconds = -1L
        }
    }

    @Test
    fun process_whenFirstMeasurement_setsAverageAndVariance() {
        val timestamp = System.nanoTime()
        val measurement = createMeasurement(timestamp)

        val triad = measurement.toTriad()
        val norm = triad.norm

        // process
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        processor.process(measurement)

        // check
        assertEquals(1, processor.count)
        assertEquals(norm, processor.average)
        assertEquals(0.0, processor.variance)
    }

    @Test
    fun process_whenSecondMeasurement_setsAverageAndVariance() {
        val timestamp = System.nanoTime()
        val measurement1 = createMeasurement(timestamp)
        val measurement2 = createMeasurement(timestamp)

        val triad1 = measurement1.toTriad()
        val triad2 = measurement2.toTriad()

        val avgValueX = (triad1.valueX + triad2.valueX) / 2.0
        val avgValueY = (triad1.valueY + triad2.valueY) / 2.0
        val avgValueZ = (triad1.valueZ + triad2.valueZ) / 2.0

        val avg = sqrt(avgValueX * avgValueX + avgValueY * avgValueY + avgValueZ * avgValueZ)

        val diffX1 = triad1.valueX - avgValueX
        val diffY1 = triad1.valueY - avgValueY
        val diffZ1 = triad1.valueZ - avgValueZ

        val diffX2 = triad2.valueX - avgValueX
        val diffY2 = triad2.valueY - avgValueY
        val diffZ2 = triad2.valueZ - avgValueZ

        val sqrDiffX1 = diffX1 * diffX1
        val sqrDiffY1 = diffY1 * diffY1
        val sqrDiffZ1 = diffZ1 * diffZ1

        val sqrDiffX2 = diffX2 * diffX2
        val sqrDiffY2 = diffY2 * diffY2
        val sqrDiffZ2 = diffZ2 * diffZ2

        val varianceX = (sqrDiffX1 + sqrDiffX2) / 2.0
        val varianceY = (sqrDiffY1 + sqrDiffY2) / 2.0
        val varianceZ = (sqrDiffZ1 + sqrDiffZ2) / 2.0

        val variance = varianceX + varianceY + varianceZ

        // process
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        processor.process(measurement1)
        processor.process(measurement2)

        // check
        assertEquals(2, processor.count)
        assertEquals(avg, processor.average)
        assertEquals(variance, processor.variance)
    }

    @Test
    fun process_whenOldMeasurementIsRemoved_setsAverageAndVariance() {
        val timestamp2 = System.nanoTime()
        val timestamp1 = timestamp2 - 2 * AverageAndVarianceSensorMeasurementProcessor.DEFAULT_WINDOW_NANOSECONDS
        val measurement1 = createMeasurement(timestamp1)
        val measurement2 = createMeasurement(timestamp2)

        val triad2 = measurement2.toTriad()

        // process
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        processor.process(measurement1)
        processor.process(measurement2)

        // check
        val norm = triad2.norm
        assertEquals(1, processor.count)
        assertEquals(norm, processor.average)
        assertEquals(0.0, processor.variance)
    }

    @Test
    fun reset_whenAlreadyProcessedMeasurements_resetsMeanAndVariance() {
        val timestamp = System.nanoTime()
        val measurement1 = createMeasurement(timestamp)
        val measurement2 = createMeasurement(timestamp)

        // process
        val processor =
            AverageAndVarianceSensorMeasurementProcessor<AccelerometerSensorMeasurement, AccelerationTriad>()

        processor.process(measurement1)
        processor.process(measurement2)

        // reset
        processor.reset()

        // check
        assertEquals(0, processor.count)
        assertNull(processor.average)
        assertNull(processor.variance)
    }

    companion object {
        private fun createMeasurement(timestamp: Long): AccelerometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val ax = randomizer.nextFloat()
            val ay = randomizer.nextFloat()
            val az = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()

            return AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
                SensorCoordinateSystem.NED
            )
        }
    }
}