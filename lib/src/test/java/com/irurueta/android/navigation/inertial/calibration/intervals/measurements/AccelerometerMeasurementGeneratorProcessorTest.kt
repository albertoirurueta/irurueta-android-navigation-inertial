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

package com.irurueta.android.navigation.inertial.calibration.intervals.measurements

import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter
import com.irurueta.units.TimeUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.slot
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test

class AccelerometerMeasurementGeneratorProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var initializationStartedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnInitializationStartedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var initializationCompletedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnInitializationCompletedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var errorListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnErrorListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalDetectedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnStaticIntervalDetectedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalDetectedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnDynamicIntervalDetectedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalSkippedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnStaticIntervalSkippedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalSkippedListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnDynamicIntervalSkippedListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK(relaxUnitFun = true)
    private lateinit var generatedMeasurementListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnGeneratedMeasurementListener<
                    AccelerometerMeasurementGeneratorProcessor, StandardDeviationBodyKinematics>

    @MockK(relaxUnitFun = true)
    private lateinit var resetListener:
            SingleSensorCalibrationMeasurementGeneratorProcessor.OnResetListener<
                    AccelerometerMeasurementGeneratorProcessor>

    @MockK
    private lateinit var internalGenerator: AccelerometerMeasurementsGenerator

    @Test
    fun constructor_whenDefaultValues_setsDefaultValues() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default values
        assertNull(processor.initializationStartedListener)
        assertNull(processor.initializationCompletedListener)
        assertNull(processor.errorListener)
        assertNull(processor.staticIntervalDetectedListener)
        assertNull(processor.dynamicIntervalDetectedListener)
        assertNull(processor.staticIntervalSkippedListener)
        assertNull(processor.dynamicIntervalSkippedListener)
        assertNull(processor.generatedMeasurementListener)
        assertNull(processor.resetListener)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            processor.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            processor.maxDynamicSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            processor.windowSize
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            processor.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            processor.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            processor.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            processor.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            processor.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(processor.accelerometerBaseNoiseLevel)
        assertNull(processor.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(processor.accelerometerBaseNoiseLevelPsd)
        assertNull(processor.accelerometerBaseNoiseLevelRootPsd)
        assertNull(processor.threshold)
        assertNull(processor.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getThresholdAsMeasurement(threshold))
        assertEquals(0, processor.processedStaticSamples)
        assertEquals(0, processor.processedDynamicSamples)
        assertFalse(processor.isStaticIntervalSkipped)
        assertFalse(processor.isDynamicIntervalSkipped)
        assertNull(processor.accelerometerAverageTimeInterval)
        assertNull(processor.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(processor.accelerometerTimeIntervalVariance)
        assertNull(processor.accelerometerTimeIntervalStandardDeviation)
        assertNull(processor.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)
        assertEquals(Status.IDLE, processor.status)
    }

    @Test
    fun constructor_whenAllListeners_setsDefaultValues() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedMeasurementListener,
            resetListener
        )

        // check default values
        assertSame(initializationStartedListener, processor.initializationStartedListener)
        assertSame(initializationCompletedListener, processor.initializationCompletedListener)
        assertSame(errorListener, processor.errorListener)
        assertSame(staticIntervalDetectedListener, processor.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, processor.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, processor.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, processor.dynamicIntervalSkippedListener)
        assertSame(generatedMeasurementListener, processor.generatedMeasurementListener)
        assertSame(resetListener, processor.resetListener)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            processor.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            processor.maxDynamicSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            processor.windowSize
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            processor.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            processor.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            processor.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            processor.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            processor.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(processor.accelerometerBaseNoiseLevel)
        assertNull(processor.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(processor.accelerometerBaseNoiseLevelPsd)
        assertNull(processor.accelerometerBaseNoiseLevelRootPsd)
        assertNull(processor.threshold)
        assertNull(processor.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getThresholdAsMeasurement(threshold))
        assertEquals(0, processor.processedStaticSamples)
        assertEquals(0, processor.processedDynamicSamples)
        assertFalse(processor.isStaticIntervalSkipped)
        assertFalse(processor.isDynamicIntervalSkipped)
        assertNull(processor.accelerometerAverageTimeInterval)
        assertNull(processor.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(processor.accelerometerTimeIntervalVariance)
        assertNull(processor.accelerometerTimeIntervalStandardDeviation)
        assertNull(processor.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)
        assertEquals(Status.IDLE, processor.status)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.initializationStartedListener)

        // set new value
        processor.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, processor.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.initializationCompletedListener)

        // set new value
        processor.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, processor.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.errorListener)

        // set new value
        processor.errorListener = errorListener

        // check
        assertSame(errorListener, processor.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.staticIntervalDetectedListener)

        // set new value
        processor.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, processor.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.dynamicIntervalDetectedListener)

        // set new value
        processor.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, processor.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.staticIntervalSkippedListener)

        // set new value
        processor.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, processor.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.dynamicIntervalSkippedListener)

        // set new value
        processor.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, processor.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedMeasurementListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.generatedMeasurementListener)

        // set new value
        processor.generatedMeasurementListener = generatedMeasurementListener

        // check
        assertSame(generatedMeasurementListener, processor.generatedMeasurementListener)
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.resetListener)

        // set new value
        processor.resetListener = resetListener

        // check
        assertSame(resetListener, processor.resetListener)
    }

    @Test
    fun minStaticSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, processor.minStaticSamples)

        // set new value
        processor.minStaticSamples = MIN_STATIC_SAMPLES

        // check
        assertEquals(MIN_STATIC_SAMPLES, processor.minStaticSamples)
    }

    @Test
    fun minStaticSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, processor.minStaticSamples)

        // set new value
        assertThrows(IllegalArgumentException::class.java) {
            processor.minStaticSamples = 2
        }
    }

    @Test
    fun maxDynamicSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, processor.maxDynamicSamples)

        // set new value
        processor.maxDynamicSamples = MAX_DYNAMIC_SAMPLES

        // check
        assertEquals(MAX_DYNAMIC_SAMPLES, processor.maxDynamicSamples)
    }

    @Test
    fun maxDynamicSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, processor.maxDynamicSamples)

        // set new value
        assertThrows(IllegalArgumentException::class.java) {
            processor.maxDynamicSamples = 2
        }
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, processor.windowSize)

        // set new value
        processor.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, processor.windowSize)
    }

    @Test
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, processor.windowSize)

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowSize = 0
        }
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            processor.initialStaticSamples
        )

        // set new value
        processor.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, processor.initialStaticSamples)
    }

    @Test
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            processor.initialStaticSamples
        )

        assertThrows(IllegalArgumentException::class.java) {
            processor.initialStaticSamples = 0
        }
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            processor.thresholdFactor,
            0.0
        )

        // check default value
        processor.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, processor.thresholdFactor, 0.0)
    }

    @Test
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            processor.thresholdFactor,
            0.0
        )

        // set new value
        assertThrows(IllegalArgumentException::class.java) {
            processor.thresholdFactor = 0.0
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            processor.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        processor.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(
            INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            processor.instantaneousNoiseLevelFactor, 0.0
        )
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            processor.instantaneousNoiseLevelFactor,
            0.0
        )

        assertThrows(IllegalArgumentException::class.java) {
            processor.instantaneousNoiseLevelFactor = 0.0
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            processor.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        processor.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            processor.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        assertThrows(IllegalArgumentException::class.java) {
            processor.baseNoiseLevelAbsoluteThreshold = 0.0
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        val baseNoiseLevelAbsoluteThreshold1 =
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold1,
            baseNoiseLevelAbsoluteThreshold2
        )

        // set new value
        val baseNoiseLevelAbsoluteThreshold3 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        processor.baseNoiseLevelAbsoluteThresholdAsMeasurement = baseNoiseLevelAbsoluteThreshold3

        // check
        val baseNoiseLevelAbsoluteThreshold4 =
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold4.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold4.unit
        )
        val baseNoiseLevelAbsoluteThreshold5 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold5
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold4,
            baseNoiseLevelAbsoluteThreshold5
        )
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        assertThrows(IllegalArgumentException::class.java) {
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement =
                Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerBaseNoiseLevel)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevel }
            .returns(baseNoiseLevel1)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertNull(processor.accelerometerBaseNoiseLevel)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val baseNoiseLevel2 = processor.accelerometerBaseNoiseLevel
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerBaseNoiseLevelAsMeasurement)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val baseNoiseLevel1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelAsMeasurement }.returns(
            baseNoiseLevel1
        )
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(processor.accelerometerBaseNoiseLevelAsMeasurement)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val baseNoiseLevel2 = processor.accelerometerBaseNoiseLevelAsMeasurement
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsFalse() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        val baseNoiseLevel1 = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel1))
        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(any()) }
            .answers { answer ->
                val result = answer.invocation.args[0] as Acceleration
                result.value = value
                result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            }
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val baseNoiseLevel = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertFalse(processor.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(0.0, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevel.unit
        )

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        assertTrue(processor.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(value, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevel.unit
        )
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerBaseNoiseLevelPsd)
        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelPsd }
            .returns(baseNoiseLevelPsd1)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertNull(processor.accelerometerBaseNoiseLevelPsd)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val baseNoiseLevelPsd2 = processor.accelerometerBaseNoiseLevelPsd
        requireNotNull(baseNoiseLevelPsd2)
        assertEquals(baseNoiseLevelPsd1, baseNoiseLevelPsd2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerBaseNoiseLevelRootPsd)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelRootPsd }.returns(
            baseNoiseLevelRootPsd1
        )
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertNull(processor.accelerometerBaseNoiseLevelRootPsd)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val baseNoiseLevelRootPsd2 = processor.accelerometerBaseNoiseLevelRootPsd
        requireNotNull(baseNoiseLevelRootPsd2)
        assertEquals(baseNoiseLevelRootPsd1, baseNoiseLevelRootPsd2, 0.0)
    }

    @Test
    fun threshold_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.threshold)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun threshold_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val threshold1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.threshold }.returns(threshold1)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertNull(processor.threshold)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val threshold2 = processor.threshold
        requireNotNull(threshold2)
        assertEquals(threshold1, threshold2, 0.0)
    }

    @Test
    fun thresholdAsMeasurement_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.thresholdAsMeasurement)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun thresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val threshold1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.thresholdAsMeasurement }.returns(threshold1)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertNull(processor.thresholdAsMeasurement)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val threshold2 = processor.thresholdAsMeasurement
        requireNotNull(threshold2)
        assertSame(threshold1, threshold2)
    }

    @Test
    fun getThresholdAsMeasurement_whenNotInitialized_returnsFalse() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getThresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.getThresholdAsMeasurement(any()) }
            .answers { answer ->
                val result = answer.invocation.args[0] as Acceleration
                result.value = value
                result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            }
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(processor.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        assertTrue(processor.getThresholdAsMeasurement(threshold))
        assertEquals(value, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
    }

    @Test
    fun processedStaticSamples_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(0, processor.processedStaticSamples)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.processedStaticSamples }.returns(value)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        // check
        assertEquals(value, processor.processedStaticSamples)
        verify(exactly = 1) { measurementsGeneratorSpy.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(0, processor.processedDynamicSamples)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.processedDynamicSamples }.returns(value)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        // check
        assertEquals(value, processor.processedDynamicSamples)
        verify(exactly = 1) { measurementsGeneratorSpy.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertFalse(processor.isStaticIntervalSkipped)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.isStaticIntervalSkipped }.returns(true)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        // check
        assertTrue(processor.isStaticIntervalSkipped)
        verify(exactly = 1) { measurementsGeneratorSpy.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check default value
        assertFalse(processor.isDynamicIntervalSkipped)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.isDynamicIntervalSkipped }.returns(true)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        // check
        assertTrue(processor.isDynamicIntervalSkipped)
        verify(exactly = 1) { measurementsGeneratorSpy.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerAverageTimeInterval)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeInterval_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(processor.accelerometerAverageTimeInterval)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val value2 = processor.accelerometerAverageTimeInterval
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerAverageTimeIntervalAsTime)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeIntervalAsTime }.returns(time1)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(processor.accelerometerAverageTimeIntervalAsTime)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val time2 = processor.accelerometerAverageTimeIntervalAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsFalse() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerAverageTimeIntervalAsTime(time))
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        assertTrue(processor.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerTimeIntervalVariance)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalVariance }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(processor.accelerometerTimeIntervalVariance)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val value2 = processor.accelerometerTimeIntervalVariance
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerTimeIntervalStandardDeviation)
        val initialized: Boolean? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized"
        )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviation }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(processor.accelerometerTimeIntervalStandardDeviation)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val value2 = processor.accelerometerTimeIntervalStandardDeviation
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsNull() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        assertNull(processor.accelerometerTimeIntervalStandardDeviationAsTime)
        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviationAsTime }.returns(time1)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(processor.accelerometerTimeIntervalStandardDeviationAsTime)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        val time2 = processor.accelerometerTimeIntervalStandardDeviationAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsFalse() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        // check
        assertTrue(processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun status_whenUnreliable_returnsFailed() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        processor.unreliable = true

        assertEquals(Status.FAILED, processor.status)
    }

    @Test
    fun status_whenReliableAndIdle_returnsIdle() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.IDLE)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertEquals(Status.IDLE, processor.status)
    }

    @Test
    fun status_whenReliableAndInitializing_returnsInitializing() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertEquals(Status.INITIALIZING, processor.status)
    }

    @Test
    fun status_whenReliableAndInitializationCompleted_returnsInitializationCompleted() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
    }

    @Test
    fun status_whenReliableAndStaticInterval_returnsStaticInterval() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.STATIC_INTERVAL, processor.status)
    }

    @Test
    fun status_whenReliableAndDynamicInterval_returnsDynamicInterval() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class,
                processor, "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.DYNAMIC_INTERVAL, processor.status)
    }

    @Test
    fun status_whenReliableAndFailed_returnsFailed() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.FAILED)
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.FAILED, processor.status)
    }

    @Test
    fun processAccelerometerMeasurement_whenInitializingAndNoMeasurements_setsInitialTimestamp() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy =
            spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        assertEquals(Status.INITIALIZING, processor.status)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(timestamp, initialAccelerometerTimestamp)
        assertEquals(1, processor.numberOfProcessedAccelerometerMeasurements)
        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0
        )
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
    }

    @Test
    fun processAccelerometerMeasurement_whenInitializingAndAvailableMeasurements_addsTimestamp() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy =
            spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)

        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "numberOfProcessedAccelerometerMeasurements",
            1
        )

        assertEquals(Status.INITIALIZING, processor.status)
        assertEquals(1, processor.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val seconds = TimeConverter.nanosecondToSecond(timestamp.toDouble())
        verify(exactly = 1) {
            accelerometerTimeIntervalEstimatorSpy.addTimestamp(seconds)
        }

        assertEquals(2, processor.numberOfProcessedAccelerometerMeasurements)

        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0)
        assertEquals(ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0)
        assertEquals(-az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)
    }

    @Test
    fun processAccelerometerMeasurement_whenInitializationCompleted_setsInitialized() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
            .returns(timeInterval)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0)
        assertEquals(ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0)
        assertEquals(-az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized")
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenStaticInterval_setsInitialized() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
            .returns(timeInterval)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.STATIC_INTERVAL
        )
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)

        assertEquals(Status.STATIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0)
        assertEquals(ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0)
        assertEquals(-az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized")
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenDynamicInterval_setsInitialized() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
            .returns(timeInterval)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL
        )
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)

        assertEquals(Status.DYNAMIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0)
        assertEquals(ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0)
        assertEquals(-az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized")
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenAlreadyInitialized_increasesMeasurements() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
            .returns(timeInterval)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)
        processor.setPrivateProperty("initialized", true)

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = AccelerometerSensorMeasurement(
            ax, ay, az, bx, by, bz, timestamp,
            accuracy, AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )
        processor.processAccelerometerMeasurement(measurement)

        val slot = slot<BodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val bodyKinematics = slot.captured
        assertEquals(ay.toDouble() + by.toDouble(), bodyKinematics.fx,
            0.0)
        assertEquals(ax.toDouble() + bx.toDouble(), bodyKinematics.fy,
            0.0)
        assertEquals(-az.toDouble() - bz.toDouble(), bodyKinematics.fz,
            0.0)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        verify(exactly = 0) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 0) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized")
        requireNotNull(initialized)
        assertTrue(initialized)

        assertEquals(1, processor.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun reset_setsValuesToInitialState() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy =
            spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        processor.setPrivateProperty("measurementsGenerator",
            measurementsGeneratorSpy)

        setPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
            "unreliable", true)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "initialAccelerometerTimestamp",
            1L
        )
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "numberOfProcessedAccelerometerMeasurements",
            1
        )
        setPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true)

        assertEquals(
            TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES,
            accelerometerTimeIntervalEstimatorSpy.totalSamples
        )

        // reset
        processor.reset()

        assertEquals(Integer.MAX_VALUE,
            accelerometerTimeIntervalEstimatorSpy.totalSamples)
        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }
        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)
        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(0L, initialAccelerometerTimestamp)
        val numberOfProcessedAccelerometerMeasurements: Int? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "numberOfProcessedAccelerometerMeasurements"
        )
        requireNotNull(numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, numberOfProcessedAccelerometerMeasurements)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun mapErrorReason_whenNotUnreliable_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        var result: ErrorReason? = callPrivateFuncWithResult(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(
            ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            result)

        result = callPrivateFuncWithResult(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(
            ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            result)
    }

    @Test
    fun mapErrorReason_whenUnreliable_returnsExpectedValue() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        setPrivateProperty(CalibrationMeasurementGeneratorProcessor::class, processor,
            "unreliable", true)

        var result: ErrorReason? = callPrivateFuncWithResult(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)

        result = callPrivateFuncWithResult(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)
    }

    @Test
    fun onInitializationStarted_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)
    }

    @Test
    fun onInitializationStarted_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            initializationStartedListener = initializationStartedListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(processor) }
    }

    @Test
    fun onInitializationCompleted_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)
    }

    @Test
    fun onInitializationCompleted_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
                initializationCompletedListener = initializationCompletedListener
            )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)

        verify(exactly = 1) {
            initializationCompletedListener.onInitializationCompleted(
                processor,
                baseNoiseLevel
            )
        }
    }

    @Test
    fun onError_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
    }

    @Test
    fun onError_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            errorListener = errorListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) {
            errorListener.onError(
                processor,
                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }
    }

    @Test
    fun onStaticIntervalDetected_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(processor) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(processor) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(processor) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(processor) }
    }

    @Test
    fun onGeneratedMeasurement_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedMeasurement(internalGenerator, measurement)
    }

    @Test
    fun onGeneratedMeasurement_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            generatedMeasurementListener = generatedMeasurementListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedMeasurement(internalGenerator, measurement)

        verify(exactly = 1) {
            generatedMeasurementListener.onGeneratedMeasurement(
                processor,
                measurement
            )
        }
    }

    @Test
    fun onReset_whenNoListener_makesNoAction() {
        val processor = AccelerometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)
    }

    @Test
    fun onReset_whenListener_notifies() {
        val processor = AccelerometerMeasurementGeneratorProcessor(
            resetListener = resetListener
        )

        val measurementsGeneratorListener: AccelerometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)

        verify(exactly = 1) { resetListener.onReset(processor) }
    }

    private companion object {
        const val MIN_STATIC_SAMPLES = 501

        const val MAX_DYNAMIC_SAMPLES = 10001

        const val WINDOW_SIZE = 303

        const val INITIAL_STATIC_SAMPLES = 10000

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 100.0
    }
}