/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.calibration.noise

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationTriadNoiseEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedTriadNoiseEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.confirmVerified
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.pow
import kotlin.math.sqrt

class AccelerometerNoiseProcessorTest {

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = AccelerometerNoiseProcessor()

        // check default values
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            processor.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            processor.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, processor.stopMode)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
        assertFalse(processor.resultUnreliable)
        assertNull(processor.averageX)
        assertNull(processor.averageXAsMeasurement)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAverageXAsMeasurement(acceleration))
        assertNull(processor.averageY)
        assertNull(processor.averageYAsMeasurement)
        assertFalse(processor.getAverageYAsMeasurement(acceleration))
        assertNull(processor.averageZ)
        assertNull(processor.averageZAsMeasurement)
        assertFalse(processor.getAverageZAsMeasurement(acceleration))
        assertNull(processor.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(processor.getAverageTriad(triad))
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        assertFalse(processor.getAverageNormAsMeasurement(acceleration))
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.standardDeviationX)
        assertNull(processor.standardDeviationXAsMeasurement)
        assertFalse(processor.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(processor.standardDeviationY)
        assertNull(processor.standardDeviationYAsMeasurement)
        assertFalse(processor.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(processor.standardDeviationZ)
        assertNull(processor.standardDeviationZAsMeasurement)
        assertFalse(processor.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(processor.standardDeviationTriad)
        assertFalse(processor.getStandardDeviationTriad(triad))
        assertNull(processor.standardDeviationNorm)
        assertNull(processor.standardDeviationNormAsMeasurement)
        assertFalse(processor.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(processor.averageStandardDeviation)
        assertNull(processor.averageStandardDeviationAsMeasurement)
        assertFalse(processor.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)
        assertNull(processor.rootPsdX)
        assertNull(processor.rootPsdY)
        assertNull(processor.rootPsdZ)
        assertNull(processor.averageNoisePsd)
        assertNull(processor.noiseRootPsdNorm)
    }

    @Test
    fun constructor_whenNegativeMaxSamples_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerNoiseProcessor(-1)
        }
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val processor = AccelerometerNoiseProcessor(MAX_SAMPLES)

        // check default values
        assertEquals(
            MAX_SAMPLES,
            processor.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            processor.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, processor.stopMode)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
        assertFalse(processor.resultUnreliable)
        assertNull(processor.averageX)
        assertNull(processor.averageXAsMeasurement)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAverageXAsMeasurement(acceleration))
        assertNull(processor.averageY)
        assertNull(processor.averageYAsMeasurement)
        assertFalse(processor.getAverageYAsMeasurement(acceleration))
        assertNull(processor.averageZ)
        assertNull(processor.averageZAsMeasurement)
        assertFalse(processor.getAverageZAsMeasurement(acceleration))
        assertNull(processor.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(processor.getAverageTriad(triad))
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        assertFalse(processor.getAverageNormAsMeasurement(acceleration))
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.standardDeviationX)
        assertNull(processor.standardDeviationXAsMeasurement)
        assertFalse(processor.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(processor.standardDeviationY)
        assertNull(processor.standardDeviationYAsMeasurement)
        assertFalse(processor.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(processor.standardDeviationZ)
        assertNull(processor.standardDeviationZAsMeasurement)
        assertFalse(processor.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(processor.standardDeviationTriad)
        assertFalse(processor.getStandardDeviationTriad(triad))
        assertNull(processor.standardDeviationNorm)
        assertNull(processor.standardDeviationNormAsMeasurement)
        assertFalse(processor.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(processor.averageStandardDeviation)
        assertNull(processor.averageStandardDeviationAsMeasurement)
        assertFalse(processor.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)
        assertNull(processor.rootPsdX)
        assertNull(processor.rootPsdY)
        assertNull(processor.rootPsdZ)
        assertNull(processor.averageNoisePsd)
        assertNull(processor.noiseRootPsdNorm)
    }

    @Test
    fun constructor_whenNegativeMaxDurationMillis_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerNoiseProcessor(maxDurationMillis = -1L)
        }
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val processor = AccelerometerNoiseProcessor(
            maxDurationMillis = MAX_DURATION_MILLIS
        )

        // check default values
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            processor.maxSamples
        )
        assertEquals(
            MAX_DURATION_MILLIS,
            processor.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, processor.stopMode)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
        assertFalse(processor.resultUnreliable)
        assertNull(processor.averageX)
        assertNull(processor.averageXAsMeasurement)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAverageXAsMeasurement(acceleration))
        assertNull(processor.averageY)
        assertNull(processor.averageYAsMeasurement)
        assertFalse(processor.getAverageYAsMeasurement(acceleration))
        assertNull(processor.averageZ)
        assertNull(processor.averageZAsMeasurement)
        assertFalse(processor.getAverageZAsMeasurement(acceleration))
        assertNull(processor.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(processor.getAverageTriad(triad))
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        assertFalse(processor.getAverageNormAsMeasurement(acceleration))
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.standardDeviationX)
        assertNull(processor.standardDeviationXAsMeasurement)
        assertFalse(processor.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(processor.standardDeviationY)
        assertNull(processor.standardDeviationYAsMeasurement)
        assertFalse(processor.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(processor.standardDeviationZ)
        assertNull(processor.standardDeviationZAsMeasurement)
        assertFalse(processor.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(processor.standardDeviationTriad)
        assertFalse(processor.getStandardDeviationTriad(triad))
        assertNull(processor.standardDeviationNorm)
        assertNull(processor.standardDeviationNormAsMeasurement)
        assertFalse(processor.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(processor.averageStandardDeviation)
        assertNull(processor.averageStandardDeviationAsMeasurement)
        assertFalse(processor.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)
        assertNull(processor.rootPsdX)
        assertNull(processor.rootPsdY)
        assertNull(processor.rootPsdZ)
        assertNull(processor.averageNoisePsd)
        assertNull(processor.noiseRootPsdNorm)
    }

    @Test
    fun constructor_whenStopMode_setsExpectedValues() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_DURATION_ONLY
        )

        // check default values
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            processor.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            processor.maxDurationMillis
        )
        assertEquals(StopMode.MAX_DURATION_ONLY, processor.stopMode)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
        assertFalse(processor.resultUnreliable)
        assertNull(processor.averageX)
        assertNull(processor.averageXAsMeasurement)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAverageXAsMeasurement(acceleration))
        assertNull(processor.averageY)
        assertNull(processor.averageYAsMeasurement)
        assertFalse(processor.getAverageYAsMeasurement(acceleration))
        assertNull(processor.averageZ)
        assertNull(processor.averageZAsMeasurement)
        assertFalse(processor.getAverageZAsMeasurement(acceleration))
        assertNull(processor.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(processor.getAverageTriad(triad))
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        assertFalse(processor.getAverageNormAsMeasurement(acceleration))
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.standardDeviationX)
        assertNull(processor.standardDeviationXAsMeasurement)
        assertFalse(processor.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(processor.standardDeviationY)
        assertNull(processor.standardDeviationYAsMeasurement)
        assertFalse(processor.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(processor.standardDeviationZ)
        assertNull(processor.standardDeviationZAsMeasurement)
        assertFalse(processor.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(processor.standardDeviationTriad)
        assertFalse(processor.getStandardDeviationTriad(triad))
        assertNull(processor.standardDeviationNorm)
        assertNull(processor.standardDeviationNormAsMeasurement)
        assertFalse(processor.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(processor.averageStandardDeviation)
        assertNull(processor.averageStandardDeviationAsMeasurement)
        assertFalse(processor.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)
        assertNull(processor.rootPsdX)
        assertNull(processor.rootPsdY)
        assertNull(processor.rootPsdZ)
        assertNull(processor.averageNoisePsd)
        assertNull(processor.noiseRootPsdNorm)
    }

    @Test
    fun numberOfProcessedMeasurements_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("numberOfProcessedMeasurements", 5L)
        assertEquals(5L, processor.numberOfProcessedMeasurements)
    }

    @Test
    fun maxSamples_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("maxSamples", 5)
        assertEquals(5, processor.maxSamples)
    }

    @Test
    fun maxDurationMillis_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("maxDurationMillis", 5L)
        assertEquals(5L, processor.maxDurationMillis)
    }

    @Test
    fun resultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        assertTrue(processor.resultAvailable)
    }

    @Test
    fun averageTimeInterval_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
    }

    @Test
    fun averageTimeInterval_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty("averageTimeInterval", averageTimeInterval)

        assertTrue(processor.resultAvailable)
        assertEquals(averageTimeInterval, processor.averageTimeInterval)
    }

    @Test
    fun averageTimeIntervalAsTime_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeIntervalAsTime)
    }

    @Test
    fun averageTimeIntervalAsTime_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty("averageTimeInterval", averageTimeInterval)

        assertTrue(processor.resultAvailable)
        val averageTimeIntervalAsTime = processor.averageTimeIntervalAsTime
        requireNotNull(averageTimeIntervalAsTime)
        assertEquals(
            averageTimeInterval,
            averageTimeIntervalAsTime.value.toDouble(), 0.0
        )
        assertEquals(TimeUnit.SECOND, averageTimeIntervalAsTime.unit)
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty("averageTimeInterval", averageTimeInterval)

        assertTrue(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertTrue(processor.getAverageTimeIntervalAsTime(result))
        assertEquals(
            averageTimeInterval,
            result.value.toDouble(), 0.0
        )
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun timeIntervalVariance_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalVariance)
    }

    @Test
    fun timeIntervalVariance_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty("timeIntervalVariance", timeIntervalVariance)

        assertTrue(processor.resultAvailable)
        assertEquals(timeIntervalVariance, processor.timeIntervalVariance)
    }

    @Test
    fun timeIntervalStandardDeviation_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalStandardDeviation)
    }

    @Test
    fun timeIntervalStandardDeviation_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty(
            "timeIntervalVariance",
            timeIntervalStandardDeviation * timeIntervalStandardDeviation
        )

        assertTrue(processor.resultAvailable)
        val result = processor.timeIntervalStandardDeviation
        requireNotNull(result)
        assertEquals(timeIntervalStandardDeviation, result, ABSOLUTE_ERROR)
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty(
            "timeIntervalVariance",
            timeIntervalStandardDeviation * timeIntervalStandardDeviation
        )

        assertTrue(processor.resultAvailable)
        val timeIntervalStandardDeviationAsTime =
            processor.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviationAsTime)
        assertEquals(
            timeIntervalStandardDeviation,
            timeIntervalStandardDeviationAsTime.value.toDouble(),
            ABSOLUTE_ERROR
        )
        assertEquals(
            TimeUnit.SECOND,
            timeIntervalStandardDeviationAsTime.unit
        )
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val timeIntervalEstimator: TimeIntervalEstimator? = processor.getPrivateProperty(
            "timeIntervalEstimator"
        )
        assertNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        timeIntervalEstimator?.setPrivateProperty(
            "timeIntervalVariance",
            timeIntervalStandardDeviation * timeIntervalStandardDeviation
        )

        assertTrue(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertTrue(processor.getTimeIntervalStandardDeviationAsTime(result))
        assertEquals(
            timeIntervalStandardDeviation,
            result.value.toDouble(),
            ABSOLUTE_ERROR
        )
        assertEquals(
            TimeUnit.SECOND,
            result.unit
        )
    }

    @Test
    fun elapsedTimeNanos_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        val randomizer = UniformRandomizer()
        val initialTimestampNanos = System.nanoTime()
        val elapsedTimeNanos = randomizer.nextLong(MIN_NANOS, MAX_NANOS)
        val endTimestampNanos = initialTimestampNanos + elapsedTimeNanos
        processor.setPrivateProperty("initialTimestampNanos", initialTimestampNanos)
        setPrivateProperty(
            BaseAccumulatedProcessor::class, processor,
            "endTimestampNanos", endTimestampNanos
        )

        assertEquals(elapsedTimeNanos, processor.elapsedTimeNanos)
    }

    @Test
    fun elapsedTime_whenExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        val randomizer = UniformRandomizer()
        val initialTimestampNanos = System.nanoTime()
        val elapsedTimeNanos = randomizer.nextLong(MIN_NANOS, MAX_NANOS)
        val endTimestampNanos = initialTimestampNanos + elapsedTimeNanos
        processor.setPrivateProperty("initialTimestampNanos", initialTimestampNanos)
        setPrivateProperty(
            BaseAccumulatedProcessor::class, processor,
            "endTimestampNanos", endTimestampNanos
        )

        val elapsedTime = processor.elapsedTime
        assertEquals(elapsedTimeNanos.toDouble(), elapsedTime.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, elapsedTime.unit)
    }

    @Test
    fun getElapsedTime_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        val randomizer = UniformRandomizer()
        val initialTimestampNanos = System.nanoTime()
        val elapsedTimeNanos = randomizer.nextLong(MIN_NANOS, MAX_NANOS)
        val endTimestampNanos = initialTimestampNanos + elapsedTimeNanos
        processor.setPrivateProperty("initialTimestampNanos", initialTimestampNanos)
        setPrivateProperty(
            BaseAccumulatedProcessor::class, processor,
            "endTimestampNanos", endTimestampNanos
        )

        val result = Time(0.0, TimeUnit.SECOND)
        processor.getElapsedTime(result)
        assertEquals(elapsedTimeNanos.toDouble(), result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, result.unit)
    }

    @Test
    fun resultUnreliable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultUnreliable", true)
        assertTrue(processor.resultUnreliable)
    }

    @Test
    fun averageX_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageX)
    }

    @Test
    fun averageX_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )

        assertTrue(processor.resultAvailable)
        assertEquals(averageX, processor.averageX)
    }

    @Test
    fun averageXAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageXAsMeasurement)
    }

    @Test
    fun averageXAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )

        assertTrue(processor.resultAvailable)
        val averageXAsMeasurement = processor.averageXAsMeasurement
        requireNotNull(averageXAsMeasurement)
        assertEquals(averageX, averageXAsMeasurement.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageXAsMeasurement.unit)
    }

    @Test
    fun getAverageXAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAverageXAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageXAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getAverageXAsMeasurement(result))
        assertEquals(averageX, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun averageY_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageY)
    }

    @Test
    fun averageY_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgY", averageY
        )

        assertTrue(processor.resultAvailable)
        assertEquals(averageY, processor.averageY)
    }

    @Test
    fun averageYAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageYAsMeasurement)
    }

    @Test
    fun averageYAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgY", averageY
        )

        assertTrue(processor.resultAvailable)
        val averageYAsMeasurement = processor.averageYAsMeasurement
        requireNotNull(averageYAsMeasurement)
        assertEquals(averageY, averageYAsMeasurement.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageYAsMeasurement.unit)
    }

    @Test
    fun getAverageYAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAverageYAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageYAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgY", averageY
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getAverageYAsMeasurement(result))
        assertEquals(averageY, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun averageZ_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageZ)
    }

    @Test
    fun averageZ_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgZ", averageZ
        )

        assertTrue(processor.resultAvailable)
        assertEquals(averageZ, processor.averageZ)
    }

    @Test
    fun averageZAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageZAsMeasurement)
    }

    @Test
    fun averageZAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgZ", averageZ
        )

        assertTrue(processor.resultAvailable)
        val averageZAsMeasurement = processor.averageZAsMeasurement
        requireNotNull(averageZAsMeasurement)
        assertEquals(averageZ, averageZAsMeasurement.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageZAsMeasurement.unit)
    }

    @Test
    fun getAverageZAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAverageZAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageZAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgZ", averageZ
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getAverageZAsMeasurement(result))
        assertEquals(averageZ, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun averageTriad_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTriad)
    }

    @Test
    fun averageTriad_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        val averageY = randomizer.nextDouble()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgY", averageY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgZ", averageZ
        )

        assertTrue(processor.resultAvailable)
        val averageTriad = processor.averageTriad
        requireNotNull(averageTriad)
        assertEquals(averageX, averageTriad.valueX, 0.0)
        assertEquals(averageY, averageTriad.valueY, 0.0)
        assertEquals(averageZ, averageTriad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageTriad.unit)
    }

    @Test
    fun getAverageTriad_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = AccelerationTriad()
        assertFalse(processor.getAverageTriad(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.valueX, 0.0)
        assertEquals(0.0, result.valueY, 0.0)
        assertEquals(0.0, result.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageTriad_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        val averageY = randomizer.nextDouble()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgY", averageY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgZ", averageZ
        )

        assertTrue(processor.resultAvailable)
        val result = AccelerationTriad()
        assertTrue(processor.getAverageTriad(result))
        assertEquals(averageX, result.valueX, 0.0)
        assertEquals(averageY, result.valueY, 0.0)
        assertEquals(averageZ, result.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun averageNorm_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNorm)
    }

    @Test
    fun averageNorm_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        val averageY = randomizer.nextDouble()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgY", averageY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgZ", averageZ
        )
        val averageNorm = sqrt(averageX * averageX + averageY * averageY + averageZ * averageZ)

        assertTrue(processor.resultAvailable)
        assertEquals(averageNorm, processor.averageNorm)
    }

    @Test
    fun averageNormAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNormAsMeasurement)
    }

    @Test
    fun averageNormAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        val averageY = randomizer.nextDouble()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgY", averageY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgZ", averageZ
        )
        val averageNorm = sqrt(averageX * averageX + averageY * averageY + averageZ * averageZ)

        assertTrue(processor.resultAvailable)
        val averageNormAsMeasurement = processor.averageNormAsMeasurement
        requireNotNull(averageNormAsMeasurement)
        assertEquals(averageNorm, averageNormAsMeasurement.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageNormAsMeasurement.unit)
    }

    @Test
    fun getAverageNormAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAverageNormAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageNormAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        val averageY = randomizer.nextDouble()
        val averageZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "avgX", averageX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgY", averageY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "avgZ", averageZ
        )
        val averageNorm = sqrt(averageX * averageX + averageY * averageY + averageZ * averageZ)

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getAverageNormAsMeasurement(result))
        assertEquals(averageNorm, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun varianceX_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.varianceX)
    }

    @Test
    fun varianceX_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val varianceX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", varianceX
        )

        assertTrue(processor.resultAvailable)
        assertEquals(varianceX, processor.varianceX)
    }

    @Test
    fun varianceY_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.varianceY)
    }

    @Test
    fun varianceY_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val varianceY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", varianceY
        )

        assertTrue(processor.resultAvailable)
        assertEquals(varianceY, processor.varianceY)
    }

    @Test
    fun varianceZ_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.varianceZ)
    }

    @Test
    fun varianceZ_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val varianceZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", varianceZ
        )

        assertTrue(processor.resultAvailable)
        assertEquals(varianceZ, processor.varianceZ)
    }

    @Test
    fun standardDeviationX_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationX)
    }

    @Test
    fun standardDeviationX_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )

        assertTrue(processor.resultAvailable)
        val result = processor.standardDeviationX
        requireNotNull(result)
        assertEquals(standardDeviationX, result, ABSOLUTE_ERROR)
    }

    @Test
    fun standardDeviationXAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationXAsMeasurement)
    }

    @Test
    fun standardDeviationXAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )

        assertTrue(processor.resultAvailable)
        val standardDeviationXAsMeasurement = processor.standardDeviationXAsMeasurement
        requireNotNull(standardDeviationXAsMeasurement)
        assertEquals(standardDeviationX, standardDeviationXAsMeasurement.value.toDouble(), 0.0)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            standardDeviationXAsMeasurement.unit
        )
    }

    @Test
    fun getStandardDeviationXAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getStandardDeviationXAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationXAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getStandardDeviationXAsMeasurement(result))
        assertEquals(standardDeviationX, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun standardDeviationY_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationY)
    }

    @Test
    fun standardDeviationY_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", standardDeviationY * standardDeviationY
        )

        assertTrue(processor.resultAvailable)
        val result = processor.standardDeviationY
        requireNotNull(result)
        assertEquals(standardDeviationY, result, ABSOLUTE_ERROR)
    }

    @Test
    fun standardDeviationYAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationYAsMeasurement)
    }

    @Test
    fun standardDeviationYAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", standardDeviationY * standardDeviationY
        )

        assertTrue(processor.resultAvailable)
        val standardDeviationYAsMeasurement = processor.standardDeviationYAsMeasurement
        requireNotNull(standardDeviationYAsMeasurement)
        assertEquals(standardDeviationY, standardDeviationYAsMeasurement.value.toDouble(), 0.0)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            standardDeviationYAsMeasurement.unit
        )
    }

    @Test
    fun getStandardDeviationYAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getStandardDeviationYAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationYAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", standardDeviationY * standardDeviationY
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getStandardDeviationYAsMeasurement(result))
        assertEquals(standardDeviationY, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun standardDeviationZ_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationZ)
    }

    @Test
    fun standardDeviationZ_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = processor.standardDeviationZ
        requireNotNull(result)
        assertEquals(standardDeviationZ, result, ABSOLUTE_ERROR)
    }

    @Test
    fun standardDeviationZAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationZAsMeasurement)
    }

    @Test
    fun standardDeviationZAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val standardDeviationZAsMeasurement = processor.standardDeviationZAsMeasurement
        requireNotNull(standardDeviationZAsMeasurement)
        assertEquals(standardDeviationZ, standardDeviationZAsMeasurement.value.toDouble(), 0.0)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            standardDeviationZAsMeasurement.unit
        )
    }

    @Test
    fun getStandardDeviationZAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getStandardDeviationZAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationZAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getStandardDeviationZAsMeasurement(result))
        assertEquals(standardDeviationZ, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun standardDeviationTriad_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationTriad)
    }

    @Test
    fun standardDeviationTriad_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = processor.standardDeviationTriad
        requireNotNull(result)
        assertEquals(standardDeviationX, result.valueX, ABSOLUTE_ERROR)
        assertEquals(standardDeviationY, result.valueY, ABSOLUTE_ERROR)
        assertEquals(standardDeviationZ, result.valueZ, ABSOLUTE_ERROR)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationTriad_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = AccelerationTriad()
        assertFalse(processor.getStandardDeviationTriad(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.valueX, 0.0)
        assertEquals(0.0, result.valueY, 0.0)
        assertEquals(0.0, result.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationTriad_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = AccelerationTriad()
        assertTrue(processor.getStandardDeviationTriad(result))
        assertEquals(standardDeviationX, result.valueX, ABSOLUTE_ERROR)
        assertEquals(standardDeviationY, result.valueY, ABSOLUTE_ERROR)
        assertEquals(standardDeviationZ, result.valueZ, ABSOLUTE_ERROR)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun standardDeviationNorm_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationNorm)
    }

    @Test
    fun standardDeviationNorm_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val standardDeviationNorm = sqrt(
            standardDeviationX * standardDeviationX +
                    standardDeviationY * standardDeviationY +
                    standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = processor.standardDeviationNorm
        requireNotNull(result)
        assertEquals(standardDeviationNorm, result, LARGE_ABSOLUTE_ERROR)
    }

    @Test
    fun standardDeviationNormAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.standardDeviationNormAsMeasurement)
    }

    @Test
    fun standardDeviationNormAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val standardDeviationNorm = sqrt(
            standardDeviationX * standardDeviationX +
                    standardDeviationY * standardDeviationY +
                    standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val standardDeviationNormAsMeasurement = processor.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNormAsMeasurement)
        assertEquals(
            standardDeviationNorm,
            standardDeviationNormAsMeasurement.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            standardDeviationNormAsMeasurement.unit
        )
    }

    @Test
    fun getStandardDeviationNormAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getStandardDeviationNormAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getStandardDeviationNormAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val standardDeviationNorm = sqrt(
            standardDeviationX * standardDeviationX +
                    standardDeviationY * standardDeviationY +
                    standardDeviationZ * standardDeviationZ
        )

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getStandardDeviationNormAsMeasurement(result))
        assertEquals(standardDeviationNorm, result.value.toDouble(), LARGE_ABSOLUTE_ERROR)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun averageStandardDeviation_whenNoResultAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageStandardDeviation)
    }

    @Test
    fun averageStandardDeviation_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val averageStandardDeviation =
            (standardDeviationX + standardDeviationY + standardDeviationZ) / 3.0

        assertTrue(processor.resultAvailable)
        val result = processor.averageStandardDeviation
        requireNotNull(result)
        assertEquals(averageStandardDeviation, result, ABSOLUTE_ERROR)
    }

    @Test
    fun averageStandardDeviationAsMeasurement_whenNoResultAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageStandardDeviationAsMeasurement)
    }

    @Test
    fun averageStandardDeviationAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val averageStandardDeviation =
            (standardDeviationX + standardDeviationY + standardDeviationZ) / 3.0

        assertTrue(processor.resultAvailable)
        val averageStandardDeviationAsMeasurement =
            processor.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviationAsMeasurement)
        assertEquals(
            averageStandardDeviation,
            averageStandardDeviationAsMeasurement.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            averageStandardDeviationAsMeasurement.unit
        )
    }

    @Test
    fun getAverageStandardDeviationAsMeasurement_whenNoResultAvailable_returnsFalse() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAverageStandardDeviationAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun getAverageStandardDeviationAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val standardDeviationY = randomizer.nextDouble()
        val standardDeviationZ = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", standardDeviationX * standardDeviationX
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", standardDeviationY * standardDeviationY
        )
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", standardDeviationZ * standardDeviationZ
        )
        val averageStandardDeviation =
            (standardDeviationX + standardDeviationY + standardDeviationZ) / 3.0

        assertTrue(processor.resultAvailable)
        val result = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(processor.getAverageStandardDeviationAsMeasurement(result))
        assertEquals(averageStandardDeviation, result.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.unit)
    }

    @Test
    fun psdX_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.psdX)
    }

    @Test
    fun psdX_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val varianceX = standardDeviationX * standardDeviationX
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", varianceX
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdX = varianceX * timeInterval

        assertTrue(processor.resultAvailable)
        assertEquals(psdX, processor.psdX)
    }

    @Test
    fun psdY_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.psdY)
    }

    @Test
    fun psdY_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        val varianceY = standardDeviationY * standardDeviationY
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", varianceY
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdY = varianceY * timeInterval

        assertTrue(processor.resultAvailable)
        assertEquals(psdY, processor.psdY)
    }

    @Test
    fun psdZ_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.psdZ)
    }

    @Test
    fun psdZ_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        val varianceZ = standardDeviationZ * standardDeviationZ
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", varianceZ
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdZ = varianceZ * timeInterval

        assertTrue(processor.resultAvailable)
        assertEquals(psdZ, processor.psdZ)
    }

    @Test
    fun rootPsdX_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.rootPsdX)
    }

    @Test
    fun rootPsdX_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val varianceX = standardDeviationX * standardDeviationX
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", varianceX
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdX = varianceX * timeInterval
        val rootPsdX = sqrt(psdX)

        assertTrue(processor.resultAvailable)
        assertEquals(rootPsdX, processor.rootPsdX)
    }

    @Test
    fun rootPsdY_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.rootPsdY)
    }

    @Test
    fun rootPsdY_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        val varianceY = standardDeviationY * standardDeviationY
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceY", varianceY
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdY = varianceY * timeInterval
        val rootPsdY = sqrt(psdY)

        assertTrue(processor.resultAvailable)
        assertEquals(rootPsdY, processor.rootPsdY)
    }

    @Test
    fun rootPsdZ_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.rootPsdZ)
    }

    @Test
    fun rootPsdZ_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        val varianceZ = standardDeviationZ * standardDeviationZ
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceZ", varianceZ
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdZ = varianceZ * timeInterval
        val rootPsdZ = sqrt(psdZ)

        assertTrue(processor.resultAvailable)
        assertEquals(rootPsdZ, processor.rootPsdZ)
    }

    @Test
    fun averageNoisePsd_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNoisePsd)
    }

    @Test
    fun averageNoisePsd_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val varianceX = standardDeviationX * standardDeviationX
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", varianceX
        )
        val standardDeviationY = randomizer.nextDouble()
        val varianceY = standardDeviationY * standardDeviationY
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", varianceY
        )
        val standardDeviationZ = randomizer.nextDouble()
        val varianceZ = standardDeviationZ * standardDeviationZ
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", varianceZ
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdX = varianceX * timeInterval
        val psdY = varianceY * timeInterval
        val psdZ = varianceZ * timeInterval
        val averagePsd = (psdX + psdY + psdZ) / 3.0

        assertTrue(processor.resultAvailable)
        val result = processor.averageNoisePsd
        requireNotNull(result)
        assertEquals(averagePsd, result, ABSOLUTE_ERROR)
    }

    @Test
    fun noiseRootPsdNorm_whenResultNotAvailable_returnsNull() {
        val processor = AccelerometerNoiseProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.noiseRootPsdNorm)
    }

    @Test
    fun noiseRootPsdNorm_whenResultAvailable_returnsExpectedValue() {
        val processor = AccelerometerNoiseProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        val varianceX = standardDeviationX * standardDeviationX
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "varianceX", varianceX
        )
        val standardDeviationY = randomizer.nextDouble()
        val varianceY = standardDeviationY * standardDeviationY
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceY", varianceY
        )
        val standardDeviationZ = randomizer.nextDouble()
        val varianceZ = standardDeviationZ * standardDeviationZ
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator,
            "varianceZ", varianceZ
        )

        val timeInterval = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedTriadNoiseEstimator::class,
            noiseEstimator as AccumulatedTriadNoiseEstimator<*, *, *, *, *>,
            "timeInterval", timeInterval
        )
        val psdX = varianceX * timeInterval
        val psdY = varianceY * timeInterval
        val psdZ = varianceZ * timeInterval
        val rootPsdNorm = sqrt(psdX + psdY + psdZ)

        assertTrue(processor.resultAvailable)
        val result = processor.noiseRootPsdNorm
        requireNotNull(result)
        assertEquals(rootPsdNorm, result, ABSOLUTE_ERROR)
    }

    @Test
    fun process_whenFirstMeasurement_setsInitialTimestamp() {
        val processor = AccelerometerNoiseProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        processor.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(ax, ay, az, timestamp = timestamp)

        assertFalse(processor.process(measurement))

        // check
        assertEquals(timestamp, processor.initialTimestampNanos)
        assertEquals(timestamp, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)
    }

    @Test
    fun process_whenSecondMeasurement_updatesValues() {
        val processor = AccelerometerNoiseProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(ax1, ay1, az1, timestamp = timestamp1)
        val nedMeasurement1 = measurement1.toNed()

        assertFalse(processor.process(measurement1))

        // check
        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp1, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(nedMeasurement1.ax.toDouble(), noiseEstimator.avgX, 0.0)
        assertEquals(nedMeasurement1.ay.toDouble(), noiseEstimator.avgY, 0.0)
        assertEquals(nedMeasurement1.az.toDouble(), noiseEstimator.avgZ, 0.0)
        assertEquals(0.0, noiseEstimator.varianceX, 0.0)
        assertEquals(0.0, noiseEstimator.varianceY, 0.0)
        assertEquals(0.0, noiseEstimator.varianceZ, 0.0)

        // add second measurement
        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement2 = AccelerometerSensorMeasurement(ax2, ay2, az2, timestamp = timestamp2)
        val nedMeasurement2 = measurement2.toNed()

        assertFalse(processor.process(measurement2))

        val diff = timestamp2 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp2, processor.endTimestampNanos)
        assertEquals(diff, processor.elapsedTimeNanos)
        assertEquals(2L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        val avgX2 = 0.5 * (nedMeasurement1.ax.toDouble() + nedMeasurement2.ax.toDouble())
        assertEquals(avgX2, noiseEstimator.avgX, ABSOLUTE_ERROR)
        val avgY2 = 0.5 * (nedMeasurement1.ay.toDouble() + nedMeasurement2.ay.toDouble())
        assertEquals(avgY2, noiseEstimator.avgY, ABSOLUTE_ERROR)
        val avgZ2 = 0.5 * (nedMeasurement1.az.toDouble() + nedMeasurement2.az.toDouble())
        assertEquals(avgZ2, noiseEstimator.avgZ, ABSOLUTE_ERROR)
        assertEquals(
            (nedMeasurement2.ax.toDouble() - avgX2).pow(2.0) / 2.0,
            noiseEstimator.varianceX,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (nedMeasurement2.ay.toDouble() - avgY2).pow(2.0) / 2.0,
            noiseEstimator.varianceY,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (nedMeasurement2.az.toDouble() - avgZ2).pow(2.0) / 2.0,
            noiseEstimator.varianceZ,
            ABSOLUTE_ERROR
        )
    }

    @Test
    fun process_whenThirdMeasurement_updatesValues() {
        val processor = AccelerometerNoiseProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(ax1, ay1, az1, timestamp = timestamp1)
        val nedMeasurement1 = measurement1.toNed()

        assertFalse(processor.process(measurement1))

        // check
        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp1, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(nedMeasurement1.ax.toDouble(), noiseEstimator.avgX, 0.0)
        assertEquals(nedMeasurement1.ay.toDouble(), noiseEstimator.avgY, 0.0)
        assertEquals(nedMeasurement1.az.toDouble(), noiseEstimator.avgZ, 0.0)
        assertEquals(0.0, noiseEstimator.varianceX, 0.0)
        assertEquals(0.0, noiseEstimator.varianceY, 0.0)
        assertEquals(0.0, noiseEstimator.varianceZ, 0.0)

        // add second measurement
        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement2 = AccelerometerSensorMeasurement(ax2, ay2, az2, timestamp = timestamp2)
        val nedMeasurement2 = measurement2.toNed()

        assertFalse(processor.process(measurement2))

        val diff = timestamp2 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp2, processor.endTimestampNanos)
        assertEquals(diff, processor.elapsedTimeNanos)
        assertEquals(2L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        val avgX2 = 0.5 * (nedMeasurement1.ax.toDouble() + nedMeasurement2.ax.toDouble())
        assertEquals(avgX2, noiseEstimator.avgX, ABSOLUTE_ERROR)
        val avgY2 = 0.5 * (nedMeasurement1.ay.toDouble() + nedMeasurement2.ay.toDouble())
        assertEquals(avgY2, noiseEstimator.avgY, ABSOLUTE_ERROR)
        val avgZ2 = 0.5 * (nedMeasurement1.az.toDouble() + nedMeasurement2.az.toDouble())
        assertEquals(avgZ2, noiseEstimator.avgZ, ABSOLUTE_ERROR)

        // add third measurement
        val ax3 = randomizer.nextFloat()
        val ay3 = randomizer.nextFloat()
        val az3 = randomizer.nextFloat()
        val timestamp3 = timestamp2 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement3 = AccelerometerSensorMeasurement(ax3, ay3, az3, timestamp = timestamp3)
        val nedMeasurement3 = measurement3.toNed()

        assertFalse(processor.process(measurement3))

        val diff2 = timestamp3 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp3, processor.endTimestampNanos)
        assertEquals(diff2, processor.elapsedTimeNanos)
        assertEquals(3L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        // time interval slowly converges to actual time interval
        assertEquals(
            0.5 * TIME_INTERVAL_SECONDS,
            timeIntervalEstimator.averageTimeInterval, ABSOLUTE_ERROR
        )
        val avgX3 =
            (nedMeasurement1.ax.toDouble() + nedMeasurement2.ax.toDouble() + nedMeasurement3.ax.toDouble()) / 3.0
        assertEquals(avgX3, noiseEstimator.avgX, ABSOLUTE_ERROR)
        val avgY3 =
            (nedMeasurement1.ay.toDouble() + nedMeasurement2.ay.toDouble() + nedMeasurement3.ay.toDouble()) / 3.0
        assertEquals(avgY3, noiseEstimator.avgY, ABSOLUTE_ERROR)
        val avgZ3 =
            (nedMeasurement1.az.toDouble() + nedMeasurement2.az.toDouble() + nedMeasurement3.az.toDouble()) / 3.0
        assertEquals(avgZ3, noiseEstimator.avgZ, ABSOLUTE_ERROR)
    }

    @Test
    fun process_whenIsCompleteMaxSamplesOnly_returnsTrue() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_SAMPLES_ONLY
        )

        val maxSamples = processor.maxSamples

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        for (i in 1 until maxSamples) {
            val measurementCopy = measurement.copy()
            measurementCopy.timestamp = timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS

            assertFalse(processor.process(measurementCopy))
            assertFalse(processor.resultAvailable)
        }

        // process last one
        val lastMeasurement = measurement.copy()
        lastMeasurement.timestamp = timestamp + maxSamples * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        assertTrue(processor.process(lastMeasurement))
        assertTrue(processor.resultAvailable)

        val avgX = processor.averageX
        requireNotNull(avgX)
        assertEquals(ax.toDouble(), avgX, ABSOLUTE_ERROR)
        val avgY = processor.averageY
        requireNotNull(avgY)
        assertEquals(ay.toDouble(), avgY, ABSOLUTE_ERROR)
        val avgZ = processor.averageZ
        requireNotNull(avgZ)
        assertEquals(az.toDouble(), avgZ, ABSOLUTE_ERROR)

        val avgNorm = processor.averageNorm
        requireNotNull(avgNorm)
        assertEquals(sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ), avgNorm,
            ABSOLUTE_ERROR)

        val varianceX = processor.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)
        val varianceY = processor.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)
        val varianceZ = processor.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val psdX = processor.psdX
        requireNotNull(psdX)
        assertTrue(psdX > 0.0)
        val psdY = processor.psdY
        requireNotNull(psdY)
        assertTrue(psdY > 0.0)
        val psdZ = processor.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val averageTimeInterval = processor.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        assertEquals(
            TIME_INTERVAL_SECONDS, averageTimeInterval,
            LARGE_ABSOLUTE_ERROR
        )

        val timeIntervalVariance = processor.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance,
            LARGE_ABSOLUTE_ERROR
        )

        val elapsedTimeNanos = processor.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        assertEquals(lastMeasurement.timestamp - measurement.timestamp
                - TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS, elapsedTimeNanos)
    }

    @Test
    fun process_whenIsCompleteMaxDurationOnly_returnsTrue() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_DURATION_ONLY
        )

        val maxDurationMillis = processor.maxDurationMillis
        val maxSamples = maxDurationMillis / TIME_INTERVAL_MILLIS + 1

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        for (i in 1 until maxSamples) {
            val measurementCopy = measurement.copy()
            measurementCopy.timestamp = timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS

            assertFalse(processor.process(measurementCopy))
            assertFalse(processor.resultAvailable)
        }

        // process last one
        val lastMeasurement = measurement.copy()
        lastMeasurement.timestamp = timestamp + maxSamples * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        assertTrue(processor.process(lastMeasurement))
        assertTrue(processor.resultAvailable)

        val avgX = processor.averageX
        requireNotNull(avgX)
        assertEquals(ax.toDouble(), avgX, ABSOLUTE_ERROR)
        val avgY = processor.averageY
        requireNotNull(avgY)
        assertEquals(ay.toDouble(), avgY, ABSOLUTE_ERROR)
        val avgZ = processor.averageZ
        requireNotNull(avgZ)
        assertEquals(az.toDouble(), avgZ, ABSOLUTE_ERROR)

        val avgNorm = processor.averageNorm
        requireNotNull(avgNorm)
        assertEquals(sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ), avgNorm,
            ABSOLUTE_ERROR)

        val varianceX = processor.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)
        val varianceY = processor.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)
        val varianceZ = processor.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val psdX = processor.psdX
        requireNotNull(psdX)
        assertTrue(psdX > 0.0)
        val psdY = processor.psdY
        requireNotNull(psdY)
        assertTrue(psdY > 0.0)
        val psdZ = processor.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val averageTimeInterval = processor.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        assertEquals(
            TIME_INTERVAL_SECONDS, averageTimeInterval,
            LARGE_ABSOLUTE_ERROR
        )

        val timeIntervalVariance = processor.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance,
            LARGE_ABSOLUTE_ERROR
        )

        val elapsedTimeNanos = processor.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        assertEquals(lastMeasurement.timestamp - measurement.timestamp
                - TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS, elapsedTimeNanos)
    }

    @Test
    fun process_whenIsCompleteMaxSamplesOrDurationAndMaxDurationExceeded_returnsTrue() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            maxSamples = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES * 2,
            maxDurationMillis = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS / 2
        )

        val maxDurationMillis = processor.maxDurationMillis
        val maxSamples = maxDurationMillis / TIME_INTERVAL_MILLIS + 1

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        for (i in 1 until maxSamples) {
            val measurementCopy = measurement.copy()
            measurementCopy.timestamp = timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS

            assertFalse(processor.process(measurementCopy))
            assertFalse(processor.resultAvailable)
        }

        // process last one
        val lastMeasurement = measurement.copy()
        lastMeasurement.timestamp = timestamp + maxSamples * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        assertTrue(processor.process(lastMeasurement))
        assertTrue(processor.resultAvailable)

        val avgX = processor.averageX
        requireNotNull(avgX)
        assertEquals(ax.toDouble(), avgX, ABSOLUTE_ERROR)
        val avgY = processor.averageY
        requireNotNull(avgY)
        assertEquals(ay.toDouble(), avgY, ABSOLUTE_ERROR)
        val avgZ = processor.averageZ
        requireNotNull(avgZ)
        assertEquals(az.toDouble(), avgZ, ABSOLUTE_ERROR)

        val avgNorm = processor.averageNorm
        requireNotNull(avgNorm)
        assertEquals(sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ), avgNorm,
            ABSOLUTE_ERROR)

        val varianceX = processor.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)
        val varianceY = processor.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)
        val varianceZ = processor.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val psdX = processor.psdX
        requireNotNull(psdX)
        assertTrue(psdX > 0.0)
        val psdY = processor.psdY
        requireNotNull(psdY)
        assertTrue(psdY > 0.0)
        val psdZ = processor.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val averageTimeInterval = processor.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        assertEquals(
            TIME_INTERVAL_SECONDS, averageTimeInterval,
            LARGE_ABSOLUTE_ERROR
        )

        val timeIntervalVariance = processor.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance,
            LARGE_ABSOLUTE_ERROR
        )

        val elapsedTimeNanos = processor.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        assertEquals(lastMeasurement.timestamp - measurement.timestamp
                - TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS, elapsedTimeNanos)
    }

    @Test
    fun process_whenIsCompleteMaxSamplesOrDurationAndMaxSamplesExceeded_returnsTrue() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            maxSamples = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES / 2,
            maxDurationMillis = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS * 2
        )

        val maxSamples = processor.maxSamples

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        for (i in 1 until maxSamples) {
            val measurementCopy = measurement.copy()
            measurementCopy.timestamp = timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS

            assertFalse(processor.process(measurementCopy))
            assertFalse(processor.resultAvailable)
        }

        // process last one
        val lastMeasurement = measurement.copy()
        lastMeasurement.timestamp = timestamp + maxSamples * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        assertTrue(processor.process(lastMeasurement))
        assertTrue(processor.resultAvailable)

        val avgX = processor.averageX
        requireNotNull(avgX)
        assertEquals(ax.toDouble(), avgX, ABSOLUTE_ERROR)
        val avgY = processor.averageY
        requireNotNull(avgY)
        assertEquals(ay.toDouble(), avgY, ABSOLUTE_ERROR)
        val avgZ = processor.averageZ
        requireNotNull(avgZ)
        assertEquals(az.toDouble(), avgZ, ABSOLUTE_ERROR)

        val avgNorm = processor.averageNorm
        requireNotNull(avgNorm)
        assertEquals(sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ), avgNorm,
            ABSOLUTE_ERROR)

        val varianceX = processor.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)
        val varianceY = processor.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)
        val varianceZ = processor.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val psdX = processor.psdX
        requireNotNull(psdX)
        assertTrue(psdX > 0.0)
        val psdY = processor.psdY
        requireNotNull(psdY)
        assertTrue(psdY > 0.0)
        val psdZ = processor.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val averageTimeInterval = processor.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        assertEquals(
            TIME_INTERVAL_SECONDS, averageTimeInterval,
            LARGE_ABSOLUTE_ERROR
        )

        val timeIntervalVariance = processor.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance,
            LARGE_ABSOLUTE_ERROR
        )

        val elapsedTimeNanos = processor.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        assertEquals(lastMeasurement.timestamp - measurement.timestamp
                - TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS, elapsedTimeNanos)
    }

    @Test
    fun reset_whenMaxDurationOnlyStopMode_resetsValues() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_DURATION_ONLY
        )

        processor.setPrivateProperty("initialTimestampNanos", 1L)
        processor.setPrivateProperty("endTimestampNanos", 2L)
        processor.setPrivateProperty("numberOfProcessedMeasurements", 3L)
        processor.setPrivateProperty("resultAvailable", true)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        processor.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseAccumulatedProcessor::class,
                processor,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        processor.reset()

        // check
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        verify(exactly = 1) { noiseEstimatorSpy.timeInterval = 0.0 }
        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { timeIntervalEstimatorSpy.totalSamples = Integer.MAX_VALUE }
        confirmVerified(noiseEstimatorSpy, timeIntervalEstimatorSpy)
    }

    @Test
    fun reset_whenOtherStopMode_resetsValues() {
        val processor = AccelerometerNoiseProcessor(
            stopMode = StopMode.MAX_SAMPLES_ONLY
        )
        val maxSamples = processor.maxSamples

        processor.setPrivateProperty("initialTimestampNanos", 1L)
        processor.setPrivateProperty("endTimestampNanos", 2L)
        processor.setPrivateProperty("numberOfProcessedMeasurements", 3L)
        processor.setPrivateProperty("resultAvailable", true)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        processor.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseAccumulatedProcessor::class,
                processor,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )
        requireNotNull(timeIntervalEstimator)

        processor.reset()

        // check
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageX)
        assertNull(processor.averageY)
        assertNull(processor.averageZ)
        assertNull(processor.averageNorm)
        assertNull(processor.varianceX)
        assertNull(processor.varianceY)
        assertNull(processor.varianceZ)
        assertNull(processor.psdX)
        assertNull(processor.psdY)
        assertNull(processor.psdZ)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        verify(exactly = 1) { noiseEstimatorSpy.timeInterval = 0.0 }
        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { timeIntervalEstimatorSpy.totalSamples = maxSamples }
        confirmVerified(noiseEstimatorSpy, timeIntervalEstimatorSpy)
    }

    private companion object {
        const val MAX_SAMPLES = 100

        const val MAX_DURATION_MILLIS = 1000L

        const val MIN_NANOS = 1000000L

        const val MAX_NANOS = 5000000L

        const val TIME_INTERVAL_MILLIS = 20L

        const val MILLIS_TO_NANOS = 1000000

        const val TIME_INTERVAL_SECONDS = 0.02

        const val ABSOLUTE_ERROR = 1e-12

        const val LARGE_ABSOLUTE_ERROR = 6e-4
    }
}