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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityMeasurementNoiseEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMeasurementNoiseEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.Called
import io.mockk.confirmVerified
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class MagnetometerNormProcessorTest {

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = MagnetometerNormProcessor()

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
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0,
            MagneticFluxDensityUnit.MICROTESLA
        )
        assertFalse(processor.getAverageNormAsMeasurement(magneticFluxDensity))
        assertNull(processor.normVariance)
        assertNull(processor.normStandardDeviation)
        assertNull(processor.normStandardDeviationAsMeasurement)
        assertFalse(processor.getNormStandardDeviationAsMeasurement(magneticFluxDensity))
        assertNull(processor.psd)
        assertNull(processor.rootPsd)
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
    }

    @Test
    fun constructor_whenNegativeMaxSamples_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerNormProcessor(-1)
        }
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val processor = MagnetometerNormProcessor(MAX_SAMPLES)

        // check default values
        assertEquals(MAX_SAMPLES, processor.maxSamples)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            processor.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, processor.stopMode)
        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0,
            MagneticFluxDensityUnit.MICROTESLA
        )
        assertFalse(processor.getAverageNormAsMeasurement(magneticFluxDensity))
        assertNull(processor.normVariance)
        assertNull(processor.normStandardDeviation)
        assertNull(processor.normStandardDeviationAsMeasurement)
        assertFalse(processor.getNormStandardDeviationAsMeasurement(magneticFluxDensity))
        assertNull(processor.psd)
        assertNull(processor.rootPsd)
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
    }

    @Test
    fun constructor_whenNegativeMaxDurationMillis_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerNormProcessor(maxDurationMillis = -1L)
        }
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val processor = MagnetometerNormProcessor(
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
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0,
            MagneticFluxDensityUnit.MICROTESLA
        )
        assertFalse(processor.getAverageNormAsMeasurement(magneticFluxDensity))
        assertNull(processor.normVariance)
        assertNull(processor.normStandardDeviation)
        assertNull(processor.normStandardDeviationAsMeasurement)
        assertFalse(processor.getNormStandardDeviationAsMeasurement(magneticFluxDensity))
        assertNull(processor.psd)
        assertNull(processor.rootPsd)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
    }

    @Test
    fun constructor_whenStopMode_setsExpectedValues() {
        val processor = MagnetometerNormProcessor(
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
        assertNull(processor.averageNorm)
        assertNull(processor.averageNormAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0,
            MagneticFluxDensityUnit.MICROTESLA
        )
        assertFalse(processor.getAverageNormAsMeasurement(magneticFluxDensity))
        assertNull(processor.normVariance)
        assertNull(processor.normStandardDeviation)
        assertNull(processor.normStandardDeviationAsMeasurement)
        assertFalse(processor.getNormStandardDeviationAsMeasurement(magneticFluxDensity))
        assertNull(processor.psd)
        assertNull(processor.rootPsd)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(time1))
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.timeIntervalStandardDeviation)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)
        assertEquals(
            Time(0.0, TimeUnit.NANOSECOND),
            processor.elapsedTime
        )
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        processor.getElapsedTime(time2)
        assertEquals(processor.elapsedTime, time2)
    }

    @Test
    fun numberOfProcessedMeasurements_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("numberOfProcessedMeasurements", 5L)
        assertEquals(5L, processor.numberOfProcessedMeasurements)
    }

    @Test
    fun maxSamples_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("maxSamples", 5)
        assertEquals(5, processor.maxSamples)
    }

    @Test
    fun maxDurationMillis_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("maxDurationMillis", 5L)
        assertEquals(5L, processor.maxDurationMillis)
    }

    @Test
    fun resultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        assertTrue(processor.resultAvailable)
    }

    @Test
    fun averageTimeInterval_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
    }

    @Test
    fun averageTimeInterval_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeIntervalAsTime)
    }

    @Test
    fun averageTimeIntervalAsTime_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAverageTimeIntervalAsTime(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenResultAvailable_returnsTrue() {
        val processor = MagnetometerNormProcessor()

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
        assertEquals(averageTimeInterval, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun timeIntervalVariance_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalVariance)
    }

    @Test
    fun timeIntervalVariance_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalStandardDeviation)
    }

    @Test
    fun timeIntervalStandardDeviation_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        assertEquals(
            timeIntervalStandardDeviation, result,
            ABSOLUTE_ERROR
        )
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.timeIntervalStandardDeviationAsTime)
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        assertEquals(TimeUnit.SECOND, timeIntervalStandardDeviationAsTime.unit)
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenResultNotAvailable_returnsFalse() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        val result = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getTimeIntervalStandardDeviationAsTime(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenResultAvailable_returnsTrue() {
        val processor = MagnetometerNormProcessor()

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
            timeIntervalStandardDeviation, result.value.toDouble(),
            ABSOLUTE_ERROR
        )
        assertEquals(TimeUnit.SECOND, result.unit)
    }

    @Test
    fun elapsedTimeNanos_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
    fun elapsedTime_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

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
        val processor = MagnetometerNormProcessor()

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
    fun averageNorm_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNorm)
    }

    @Test
    fun averageNorm_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageNorm = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "avg",
            averageNorm
        )

        assertTrue(processor.resultAvailable)
        assertEquals(averageNorm, processor.averageNorm)
    }

    @Test
    fun averageNormAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.averageNormAsMeasurement)
    }

    @Test
    fun averageNormAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageNorm = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "avg",
            averageNorm
        )

        assertTrue(processor.resultAvailable)
        val averageNormAsMeasurement = processor.averageNormAsMeasurement
        requireNotNull(averageNormAsMeasurement)
        assertEquals(averageNorm, averageNormAsMeasurement.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, averageNormAsMeasurement.unit)
    }

    @Test
    fun getAverageNormAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.MICROTESLA)
        assertFalse(processor.getAverageNormAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, result.unit)
    }

    @Test
    fun getAverageNormAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val averageNorm = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "avg",
            averageNorm
        )

        assertTrue(processor.resultAvailable)
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.MICROTESLA)
        assertTrue(processor.getAverageNormAsMeasurement(result))
        assertEquals(averageNorm, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun normVariance_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.normVariance)
    }

    @Test
    fun normVariance_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val normVariance = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            normVariance
        )

        assertTrue(processor.resultAvailable)
        assertEquals(normVariance, processor.normVariance)
    }

    @Test
    fun normStandardDeviation_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.normStandardDeviation)
    }

    @Test
    fun normStandardDeviation_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val normStandardDeviation = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            normStandardDeviation * normStandardDeviation
        )

        assertTrue(processor.resultAvailable)
        val result = processor.normStandardDeviation
        requireNotNull(result)
        assertEquals(normStandardDeviation, result, ABSOLUTE_ERROR)
    }

    @Test
    fun normStandardDeviationAsMeasurement_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.normStandardDeviationAsMeasurement)
    }

    @Test
    fun normStandardDeviationAsMeasurement_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val normStandardDeviation = randomizer.nextDouble()
        setPrivateProperty(
            AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            normStandardDeviation * normStandardDeviation
        )

        assertTrue(processor.resultAvailable)
        val normStandardDeviationAsMeasurement = processor.normStandardDeviationAsMeasurement
        requireNotNull(normStandardDeviationAsMeasurement)
        assertEquals(
            normStandardDeviation,
            normStandardDeviationAsMeasurement.value.toDouble(),
            ABSOLUTE_ERROR
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, normStandardDeviationAsMeasurement.unit)
    }

    @Test
    fun getNormStandardDeviationAsMeasurement_whenResultNotAvailable_returnsFalse() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.MICROTESLA)
        assertFalse(processor.getNormStandardDeviationAsMeasurement(result))

        // ensure result remains unchanged
        assertEquals(0.0, result.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, result.unit)
    }

    @Test
    fun getNormStandardDeviationAsMeasurement_whenResultAvailable_returnsTrue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val normStandardDeviation = randomizer.nextDouble()
        setPrivateProperty(AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            normStandardDeviation * normStandardDeviation)

        assertTrue(processor.resultAvailable)
        val result = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.MICROTESLA)
        assertTrue(processor.getNormStandardDeviationAsMeasurement(result))
        assertEquals(normStandardDeviation, result.value.toDouble(),
            ABSOLUTE_ERROR
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, result.unit)
    }

    @Test
    fun psd_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.psd)
    }

    @Test
    fun psd_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val variance = randomizer.nextDouble()
        val timeInterval = randomizer.nextDouble()
        val psd = variance * timeInterval
        setPrivateProperty(AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            variance)
        setPrivateProperty(AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "timeInterval",
            timeInterval)

        assertTrue(processor.resultAvailable)
        assertEquals(psd, processor.psd)
    }

    @Test
    fun rootPsd_whenResultNotAvailable_returnsNull() {
        val processor = MagnetometerNormProcessor()

        assertFalse(processor.resultAvailable)
        assertNull(processor.rootPsd)
    }

    @Test
    fun rootPsd_whenResultAvailable_returnsExpectedValue() {
        val processor = MagnetometerNormProcessor()

        processor.setPrivateProperty("resultAvailable", true)
        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        assertNotNull(noiseEstimator)
        val randomizer = UniformRandomizer()
        val variance = randomizer.nextDouble()
        val timeInterval = randomizer.nextDouble()
        val psd = variance * timeInterval
        val rootPsd = sqrt(psd)
        setPrivateProperty(AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "variance",
            variance)
        setPrivateProperty(AccumulatedMeasurementNoiseEstimator::class,
            noiseEstimator as AccumulatedMeasurementNoiseEstimator<*, *, *, *>, "timeInterval",
            timeInterval)

        assertTrue(processor.resultAvailable)
        assertEquals(rootPsd, processor.rootPsd)
    }

    @Test
    fun process_whenFirstMeasurement_setsInitialTimestamp() {
        val processor = MagnetometerNormProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
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

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = MagnetometerSensorMeasurement(bx, by, bz, timestamp = timestamp)
        assertFalse(processor.process(measurement))

        // check
        assertEquals(timestamp, processor.initialTimestampNanos)
        assertEquals(timestamp, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageNorm)
        assertNull(processor.normVariance)
        assertNull(processor.psd)

        val norm = measurement.toNorm()
        val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
        verify(exactly = 1) { noiseEstimatorSpy.addMeasurement(normT) }
        verify { timeIntervalEstimatorSpy wasNot Called }
        confirmVerified(noiseEstimatorSpy)
    }

    @Test
    fun process_whenSecondMeasurement_updatesValues() {
        val processor = MagnetometerNormProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseAccumulatedProcessor::class,
                processor,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        // add first measurement
        val measurement1 = MagnetometerSensorMeasurement(bx1, by1, bz1, timestamp = timestamp1)
        assertFalse(processor.process(measurement1))

        // check
        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp1, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageNorm)
        assertNull(processor.normVariance)
        assertNull(processor.psd)

        val norm1 = measurement1.toNorm()
        val normT1 = MagneticFluxDensityConverter.microTeslaToTesla(norm1)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(normT1, noiseEstimator.avg, 0.0)

        // add second measurement
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement2 = MagnetometerSensorMeasurement(bx2, by2, bz2, timestamp = timestamp2)

        assertFalse(processor.process(measurement2))

        val norm2 = measurement2.toNorm()
        val normT2 = MagneticFluxDensityConverter.microTeslaToTesla(norm2)
        val diff = timestamp2 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp2, processor.endTimestampNanos)
        assertEquals(diff, processor.elapsedTimeNanos)
        assertEquals(2L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(0.5 * (normT1 + normT2), noiseEstimator.avg, 0.0)
    }

    @Test
    fun process_whenThirdMeasurement_updatesValues() {
        val processor = MagnetometerNormProcessor()

        assertEquals(0L, processor.numberOfProcessedMeasurements)
        assertEquals(0L, processor.initialTimestampNanos)
        assertEquals(0L, processor.endTimestampNanos)

        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
            processor.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseAccumulatedProcessor::class,
                processor,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        // add first measurement
        val measurement1 = MagnetometerSensorMeasurement(bx1, by1, bz1, timestamp = timestamp1)
        assertFalse(processor.process(measurement1))

        // check
        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp1, processor.endTimestampNanos)
        assertEquals(0L, processor.elapsedTimeNanos)
        assertEquals(1L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)
        assertNull(processor.averageNorm)
        assertNull(processor.normVariance)
        assertNull(processor.psd)

        val norm1 = measurement1.toNorm()
        val normT1 = MagneticFluxDensityConverter.microTeslaToTesla(norm1)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(normT1, noiseEstimator.avg, 0.0)

        // add second measurement
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement2 = MagnetometerSensorMeasurement(bx2, by2, bz2, timestamp = timestamp2)

        assertFalse(processor.process(measurement2))

        val norm2 = measurement2.toNorm()
        val normT2 = MagneticFluxDensityConverter.microTeslaToTesla(norm2)
        val diff = timestamp2 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp2, processor.endTimestampNanos)
        assertEquals(diff, processor.elapsedTimeNanos)
        assertEquals(2L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)

        assertEquals(0.0, timeIntervalEstimator.averageTimeInterval, 0.0)
        assertEquals(0.5 * (normT1 + normT2), noiseEstimator.avg, 0.0)

        // add third measurement
        val bx3 = randomizer.nextFloat()
        val by3 = randomizer.nextFloat()
        val bz3 = randomizer.nextFloat()
        val timestamp3 = timestamp2 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val measurement3 = MagnetometerSensorMeasurement(bx3, by3, bz3, timestamp = timestamp3)

        assertFalse(processor.process(measurement3))

        val norm3 = measurement3.toNorm()
        val normT3 = MagneticFluxDensityConverter.microTeslaToTesla(norm3)
        val diff2 = timestamp3 - timestamp1

        assertEquals(timestamp1, processor.initialTimestampNanos)
        assertEquals(timestamp3, processor.endTimestampNanos)
        assertEquals(diff2, processor.elapsedTimeNanos)
        assertEquals(3L, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)
        assertNull(processor.averageTimeInterval)
        assertNull(processor.timeIntervalVariance)

        // time interval slowly converges to actual time interval
        assertEquals(0.5 * TIME_INTERVAL_SECONDS,
            timeIntervalEstimator.averageTimeInterval,
            ABSOLUTE_ERROR
        )
        assertEquals((normT1 + normT2 + normT3) / 3.0, noiseEstimator.avg,
            ABSOLUTE_ERROR
        )
    }

    @Test
    fun process_whenIsCompleteMaxSamplesOnly_returnsTrue() {
        val processor = MagnetometerNormProcessor(
            stopMode = StopMode.MAX_SAMPLES_ONLY,
        )

        val maxSamples = processor.maxSamples

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = MagnetometerSensorMeasurement(bx, by, bz, timestamp = timestamp)

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

        val averageNorm = processor.averageNorm
        requireNotNull(averageNorm)
        val norm = measurement.toNorm()
        val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
        assertEquals(normT, averageNorm, ABSOLUTE_ERROR)

        val normVariance = processor.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val psd = processor.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

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
        val processor = MagnetometerNormProcessor(
            stopMode = StopMode.MAX_DURATION_ONLY,
        )

        val maxDurationMillis = processor.maxDurationMillis
        val maxSamples = maxDurationMillis / TIME_INTERVAL_MILLIS + 1

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = MagnetometerSensorMeasurement(bx, by, bz, timestamp = timestamp)

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

        val averageNorm = processor.averageNorm
        requireNotNull(averageNorm)
        val norm = measurement.toNorm()
        val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
        assertEquals(normT, averageNorm, ABSOLUTE_ERROR)

        val normVariance = processor.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val psd = processor.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

        val averageTimeInterval = processor.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        assertEquals(
            TIME_INTERVAL_SECONDS, averageTimeInterval,
            LARGE_ABSOLUTE_ERROR
        )

        val timeIntervalVariance = processor.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance, LARGE_ABSOLUTE_ERROR)

        val elapsedTimeNanos = processor.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        assertEquals(lastMeasurement.timestamp - measurement.timestamp
                - TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS, elapsedTimeNanos)
    }

    @Test
    fun process_whenIsCompleteMaxSamplesOrDurationAndMaxDurationExceeded_returnsTrue() {
        val processor = MagnetometerNormProcessor(
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            maxSamples = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES * 2,
            maxDurationMillis = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS / 2
        )

        val maxDurationMillis = processor.maxDurationMillis
        val maxSamples = maxDurationMillis / TIME_INTERVAL_MILLIS + 1

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = MagnetometerSensorMeasurement(bx, by, bz, timestamp = timestamp)

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

        val averageNorm = processor.averageNorm
        requireNotNull(averageNorm)
        val norm = measurement.toNorm()
        val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
        assertEquals(normT, averageNorm, ABSOLUTE_ERROR)

        val normVariance = processor.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val psd = processor.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

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
        val processor = MagnetometerNormProcessor(
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            maxSamples = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES / 2,
            maxDurationMillis = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS * 2
        )

        val maxSamples = processor.maxSamples

        assertEquals(0, processor.numberOfProcessedMeasurements)
        assertFalse(processor.resultAvailable)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = MagnetometerSensorMeasurement(bx, by, bz, timestamp = timestamp)

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

        val averageNorm = processor.averageNorm
        requireNotNull(averageNorm)
        val norm = measurement.toNorm()
        val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
        assertEquals(normT, averageNorm, ABSOLUTE_ERROR)

        val normVariance = processor.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val psd = processor.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

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
        val processor = MagnetometerNormProcessor(stopMode = StopMode.MAX_DURATION_ONLY)

        processor.setPrivateProperty("initialTimestampNanos", 1L)
        processor.setPrivateProperty("endTimestampNanos", 2L)
        processor.setPrivateProperty("numberOfProcessedMeasurements", 3L)
        processor.setPrivateProperty("resultAvailable", true)

        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
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
        assertNull(processor.averageNorm)
        assertNull(processor.normVariance)
        assertNull(processor.psd)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        verify(exactly = 1) { noiseEstimatorSpy.timeInterval = 0.0 }
        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { timeIntervalEstimatorSpy.totalSamples = Integer.MAX_VALUE }
        confirmVerified(noiseEstimatorSpy, timeIntervalEstimatorSpy)
    }

    @Test
    fun reset_whenOtherStopMode_resetsValues() {
        val processor = MagnetometerNormProcessor(stopMode = StopMode.MAX_SAMPLES_ONLY)
        val maxSamples = processor.maxSamples

        processor.setPrivateProperty("initialTimestampNanos", 1L)
        processor.setPrivateProperty("endTimestampNanos", 2L)
        processor.setPrivateProperty("numberOfProcessedMeasurements", 3L)
        processor.setPrivateProperty("resultAvailable", true)

        val noiseEstimator: AccumulatedMagneticFluxDensityMeasurementNoiseEstimator? =
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
        assertNull(processor.averageNorm)
        assertNull(processor.normVariance)
        assertNull(processor.psd)

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