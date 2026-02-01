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
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematicsAndMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
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
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var initializationStartedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnInitializationStartedListener

    @MockK(relaxUnitFun = true)
    private lateinit var initializationCompletedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnInitializationCompletedListener

    @MockK(relaxUnitFun = true)
    private lateinit var errorListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnErrorListener

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnStaticIntervalDetectedListener

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnDynamicIntervalDetectedListener

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnStaticIntervalSkippedListener

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnDynamicIntervalSkippedListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedAccelerometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnGeneratedAccelerometerMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedGyroscopeMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnGeneratedGyroscopeMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedMagnetometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnGeneratedMagnetometerMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var resetListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor.OnResetListener

    @MockK
    private lateinit var internalGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator

    @Test
    fun constructor_whenDefaultValues_setsDefaultValues() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default values
        assertNull(processor.initializationStartedListener)
        assertNull(processor.initializationCompletedListener)
        assertNull(processor.errorListener)
        assertNull(processor.staticIntervalDetectedListener)
        assertNull(processor.dynamicIntervalDetectedListener)
        assertNull(processor.staticIntervalSkippedListener)
        assertNull(processor.dynamicIntervalSkippedListener)
        assertNull(processor.generatedAccelerometerMeasurementListener)
        assertNull(processor.generatedGyroscopeMeasurementListener)
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
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenAllListeners_setsDefaultValues() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener,
            generatedMagnetometerMeasurementListener,
            resetListener
        )

        // check default values
        assertSame(
            initializationStartedListener,
            processor.initializationStartedListener
        )
        assertSame(
            initializationCompletedListener,
            processor.initializationCompletedListener
        )
        assertSame(errorListener, processor.errorListener)
        assertSame(staticIntervalDetectedListener, processor.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, processor.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, processor.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, processor.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            processor.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            processor.generatedGyroscopeMeasurementListener
        )
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
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.initializationStartedListener)

        // set new value
        processor.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, processor.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.initializationCompletedListener)

        // set new value
        processor.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(
            initializationCompletedListener,
            processor.initializationCompletedListener
        )
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.errorListener)

        // set new value
        processor.errorListener = errorListener

        // check
        assertSame(errorListener, processor.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.staticIntervalDetectedListener)

        // set new value
        processor.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, processor.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.dynamicIntervalDetectedListener)

        // set new value
        processor.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, processor.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.staticIntervalSkippedListener)

        // set new value
        processor.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, processor.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.dynamicIntervalSkippedListener)

        // set new value
        processor.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, processor.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.generatedAccelerometerMeasurementListener)

        // set new value
        processor.generatedAccelerometerMeasurementListener =
            generatedAccelerometerMeasurementListener

        // check
        assertSame(
            generatedAccelerometerMeasurementListener,
            processor.generatedAccelerometerMeasurementListener
        )
    }

    @Test
    fun generatedGyroscopeMeasurementListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.generatedGyroscopeMeasurementListener)

        // set new value
        processor.generatedGyroscopeMeasurementListener =
            generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            processor.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.generatedMagnetometerMeasurementListener)

        // set new value
        processor.generatedMagnetometerMeasurementListener =
            generatedMagnetometerMeasurementListener

        // check
        assertSame(
            generatedMagnetometerMeasurementListener,
            processor.generatedMagnetometerMeasurementListener
        )
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertNull(processor.resetListener)

        // set new value
        processor.resetListener = resetListener

        // check
        assertSame(resetListener, processor.resetListener)
    }

    @Test
    fun minStaticSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            processor.minStaticSamples
        )

        // set new value
        processor.minStaticSamples = MIN_STATIC_SAMPLES

        // check
        assertEquals(MIN_STATIC_SAMPLES, processor.minStaticSamples)
    }

    @Test
    fun minStaticSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            processor.minStaticSamples)

        // set new value
        assertThrows(IllegalArgumentException::class.java) {
            processor.minStaticSamples = 2
        }
    }

    @Test
    fun maxDynamicSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, processor.maxDynamicSamples)

        // set new value
        processor.maxDynamicSamples = MAX_DYNAMIC_SAMPLES

        // check
        assertEquals(MAX_DYNAMIC_SAMPLES, processor.maxDynamicSamples)
    }

    @Test
    fun maxDynamicSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            processor.maxDynamicSamples
        )

        // set new value
        assertThrows(IllegalArgumentException::class.java) {
            processor.maxDynamicSamples = 2
        }
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            processor.windowSize
        )

        // set new value
        processor.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, processor.windowSize)
    }

    @Test
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            processor.windowSize
        )

        assertThrows(IllegalArgumentException::class.java) {
            processor.windowSize = 0
        }
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertThrows(IllegalArgumentException::class.java) {
            processor.baseNoiseLevelAbsoluteThreshold = 0.0
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertThrows(IllegalArgumentException::class.java) {
            processor.baseNoiseLevelAbsoluteThresholdAsMeasurement =
                Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenNotInitialized_returnsNull() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(0, processor.processedStaticSamples)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertEquals(0, processor.processedDynamicSamples)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertFalse(processor.isStaticIntervalSkipped)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        // check default value
        assertFalse(processor.isDynamicIntervalSkipped)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        processor.unreliable = true

        assertEquals(Status.FAILED, processor.status)
    }

    @Test
    fun status_whenReliableAndIdle_returnsIdle() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable = processor.unreliable
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class,
                processor, "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        processor.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.DYNAMIC_INTERVAL, processor.status)
    }

    @Test
    fun status_whenReliableAndFailed_returnsFailed() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
    }

    @Test
    fun processAccelerometerMeasurement_whenInitializingAndAvailableMeasurements_addsTimestamp() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

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

        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz,
            0.0
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )
    }

    @Test
    fun processAccelerometerMeasurement_whenInitializationCompleted_setsInitialized() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

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

        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz,
            0.0
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenStaticInterval_setsInitialized() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.STATIC_INTERVAL
        )
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

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

        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz,
            0.0
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenDynamicInterval_setsInitialized() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL
        )
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

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

        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz,
            0.0
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun processAccelerometerMeasurement_whenAlreadyInitialized_increasesMeasurements() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )
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

        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(
            ay.toDouble() + by.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx,
            0.0
        )
        assertEquals(
            ax.toDouble() + bx.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy,
            0.0
        )
        assertEquals(
            -az.toDouble() - bz.toDouble(),
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz,
            0.0
        )
        assertEquals(0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX,
            0.0
        )
        assertEquals(0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY,
            0.0
        )
        assertEquals(
            0.0,
            timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ,
            0.0
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify(exactly = 0) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 0) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertTrue(initialized)

        assertEquals(1, processor.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun reset_setsValuesToInitialState() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

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

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "unreliable", true
        )
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
        processor.setPrivateProperty("numberOfProcessedGyroscopeMeasurements", 1)
        processor.setPrivateProperty("numberOfProcessedMagnetometerMeasurements", 1)
        processor.setPrivateProperty("gyroscopeBaseNoiseLevel", 2.0)
        processor.setPrivateProperty("magnetometerBaseNoiseLevel", 3.0)
        processor.setPrivateProperty("initialMagneticFluxDensityNorm", 4.0)
        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "initialized", true
        )

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            processor.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        processor.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        assertEquals(
            TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES,
            accelerometerTimeIntervalEstimatorSpy.totalSamples
        )

        // reset
        processor.reset()

        assertEquals(
            Integer.MAX_VALUE,
            accelerometerTimeIntervalEstimatorSpy.totalSamples
        )
        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }
        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
        requireNotNull(unreliable)
        assertFalse(unreliable)
        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(0L, initialAccelerometerTimestamp)
        assertEquals(0, processor.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        val initialized: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "initialized"
            )
        requireNotNull(initialized)
        assertFalse(initialized)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.initialMagneticFluxDensityNorm)
        verify(exactly = 1) { gyroscopeAccumulatedNoiseEstimatorSpy.reset() }
    }

    @Test
    fun mapErrorReason_whenNotUnreliable_returnsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val unreliable: Boolean? =
            getPrivateProperty(
                CalibrationMeasurementGeneratorProcessor::class, processor,
                "unreliable"
            )
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
            result
        )

        result = callPrivateFuncWithResult(
            CalibrationMeasurementGeneratorProcessor::class,
            processor,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(
            ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            result
        )
    }

    @Test
    fun mapErrorReason_whenUnreliable_returnsExpectedValue() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        setPrivateProperty(
            CalibrationMeasurementGeneratorProcessor::class, processor,
            "unreliable", true
        )

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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)
    }

    @Test
    fun onInitializationStarted_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            initializationStartedListener = initializationStartedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(processor) }
    }

    @Test
    fun onInitializationCompleted_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)
    }

    @Test
    fun onInitializationCompleted_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            initializationCompletedListener = initializationCompletedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
    }

    @Test
    fun onError_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            errorListener = errorListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
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
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(processor) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(processor) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(processor) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(processor) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )

        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                processor,
                measurement
            )
        }
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedAGyroscopeMeasurement_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )

        verify(exactly = 1) {
            generatedGyroscopeMeasurementListener.onGeneratedGyroscopeMeasurement(
                processor,
                measurement
            )
        }
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        measurementsGeneratorListener.onGeneratedMagnetometerMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedAMagnetometerMeasurement_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        measurementsGeneratorListener.onGeneratedMagnetometerMeasurement(
            internalGenerator,
            measurement
        )

        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                processor,
                measurement
            )
        }
    }

    @Test
    fun onReset_whenNoListener_makesNoAction() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)
    }

    @Test
    fun onReset_whenListener_notifies() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor(
            resetListener = resetListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            processor.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)

        verify(exactly = 1) { resetListener.onReset(processor) }
    }

    @Test
    fun processGyroscopeMeasurement_whenInitializing_accumulatedAngularSpeedAndIncreatesMeasurements() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)

        val kinematics: BodyKinematics? = processor.getPrivateProperty("kinematics")
        requireNotNull(kinematics)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = GyroscopeSensorMeasurement(
            wx, wy, wz, bx, by, bz, timestamp,
            accuracy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )
        processor.processGyroscopeMeasurement(measurement)

        // check
        assertEquals(wy.toDouble() + by.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, processor.numberOfProcessedGyroscopeMeasurements)
    }

    @Test
    fun processGyroscopeMeasurement_whenInitializationCompleted_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed1 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            processor.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        processor.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)

        val kinematics: BodyKinematics? = processor.getPrivateProperty("kinematics")
        requireNotNull(kinematics)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = GyroscopeSensorMeasurement(
            wx, wy, wz, bx, by, bz, timestamp,
            accuracy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )
        processor.processGyroscopeMeasurement(measurement)

        // check
        assertEquals(wy.toDouble() + by.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, processor.numberOfProcessedGyroscopeMeasurements)

        verify(exactly = 1) {
            gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        assertEquals(baseNoiseLevel, processor.gyroscopeBaseNoiseLevel)
        val angularSpeed2 = processor.gyroscopeBaseNoiseLevelAsMeasurement
        requireNotNull(angularSpeed2)
        assertEquals(baseNoiseLevel, angularSpeed2.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed2.unit)
        val angularSpeed3 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed3))
        assertEquals(angularSpeed2, angularSpeed3)
    }

    @Test
    fun processGyroscopeMeasurement_whenStaticInterval_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed1 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            processor.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        processor.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.STATIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)

        val kinematics: BodyKinematics? = processor.getPrivateProperty("kinematics")
        requireNotNull(kinematics)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = GyroscopeSensorMeasurement(
            wx, wy, wz, bx, by, bz, timestamp,
            accuracy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )
        processor.processGyroscopeMeasurement(measurement)

        // check
        assertEquals(wy.toDouble() + by.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, processor.numberOfProcessedGyroscopeMeasurements)

        verify(exactly = 1) {
            gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        assertEquals(baseNoiseLevel, processor.gyroscopeBaseNoiseLevel)
        val angularSpeed2 = processor.gyroscopeBaseNoiseLevelAsMeasurement
        requireNotNull(angularSpeed2)
        assertEquals(baseNoiseLevel, angularSpeed2.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed2.unit)
        val angularSpeed3 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed3))
        assertEquals(angularSpeed2, angularSpeed3)
    }

    @Test
    fun processGyroscopeMeasurement_whenDynamicInterval_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed1 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            processor.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        processor.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.DYNAMIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)

        val kinematics: BodyKinematics? = processor.getPrivateProperty("kinematics")
        requireNotNull(kinematics)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = GyroscopeSensorMeasurement(
            wx, wy, wz, bx, by, bz, timestamp,
            accuracy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )
        processor.processGyroscopeMeasurement(measurement)

        // check
        assertEquals(wy.toDouble() + by.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, processor.numberOfProcessedGyroscopeMeasurements)

        verify(exactly = 1) {
            gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        assertEquals(baseNoiseLevel, processor.gyroscopeBaseNoiseLevel)
        val angularSpeed2 = processor.gyroscopeBaseNoiseLevelAsMeasurement
        requireNotNull(angularSpeed2)
        assertEquals(baseNoiseLevel, angularSpeed2.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed2.unit)
        val angularSpeed3 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed3))
        assertEquals(angularSpeed2, angularSpeed3)
    }

    @Test
    fun processGyroscopeMeasurement_whenGyroscopeBaseNoiseLevelAlreadySet_increasesMeasurements() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)
        assertNull(processor.gyroscopeBaseNoiseLevel)
        assertNull(processor.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed1 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            processor.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        processor.setPrivateProperty("gyroscopeBaseNoiseLevel", baseNoiseLevel)
        processor.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedGyroscopeMeasurements)

        val kinematics: BodyKinematics? = processor.getPrivateProperty("kinematics")
        requireNotNull(kinematics)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = GyroscopeSensorMeasurement(
            wx, wy, wz, bx, by, bz, timestamp,
            accuracy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )
        processor.processGyroscopeMeasurement(measurement)

        // check
        assertEquals(wy.toDouble() + by.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, processor.numberOfProcessedGyroscopeMeasurements)

        verify(exactly = 0) {
            gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        assertEquals(baseNoiseLevel, processor.gyroscopeBaseNoiseLevel)
        val angularSpeed2 = processor.gyroscopeBaseNoiseLevelAsMeasurement
        requireNotNull(angularSpeed2)
        assertEquals(baseNoiseLevel, angularSpeed2.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed2.unit)
        val angularSpeed3 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(processor.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed3))
        assertEquals(angularSpeed2, angularSpeed3)
    }

    @Test
    fun processMagnetometerMeasurement_whenInitializing_accumulatedAngularSpeedAndIncreatesMeasurements() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
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
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)

        val magneticFluxDensity: BodyMagneticFluxDensity? = processor.getPrivateProperty(
            "magneticFluxDensity")
        requireNotNull(magneticFluxDensity)
        assertEquals(0.0, magneticFluxDensity.bx, 0.0)
        assertEquals(0.0, magneticFluxDensity.by, 0.0)
        assertEquals(0.0, magneticFluxDensity.bz, 0.0)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = MagnetometerSensorMeasurement(
            bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp,
            accuracy, MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )
        processor.processMagnetometerMeasurement(measurement)

        // check
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            by.toDouble() + hardIronY.toDouble()), magneticFluxDensity.bx,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            bx.toDouble() + hardIronX.toDouble()), magneticFluxDensity.by,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            -bz.toDouble() - hardIronZ.toDouble()), magneticFluxDensity.bz,
            0.0)
        assertEquals(1, processor.numberOfProcessedMagnetometerMeasurements)
    }

    @Test
    fun processMagnetometerMeasurement_whenInitializationCompleted_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity1))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            processor.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val avgNorm = randomizer.nextDouble()
        every { magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        every { magnetometerAccumulatedNoiseEstimatorSpy.avgNorm }.returns(avgNorm)
        processor.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)

        val magneticFluxDensity: BodyMagneticFluxDensity? = processor.getPrivateProperty(
            "magneticFluxDensity")
        requireNotNull(magneticFluxDensity)
        assertEquals(0.0, magneticFluxDensity.bx, 0.0)
        assertEquals(0.0, magneticFluxDensity.by, 0.0)
        assertEquals(0.0, magneticFluxDensity.bz, 0.0)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = MagnetometerSensorMeasurement(
            bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp,
            accuracy, MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )
        processor.processMagnetometerMeasurement(measurement)

        // check
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            by.toDouble() + hardIronY.toDouble()), magneticFluxDensity.bx,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            bx.toDouble() + hardIronX.toDouble()), magneticFluxDensity.by,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            -bz.toDouble() - hardIronZ.toDouble()), magneticFluxDensity.bz,
            0.0)
        assertEquals(1, processor.numberOfProcessedMagnetometerMeasurements)

        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.avgNorm
        }
        assertEquals(baseNoiseLevel, processor.magnetometerBaseNoiseLevel)
        val magneticFluxDensity2 = processor.magnetometerBaseNoiseLevelAsMeasurement
        requireNotNull(magneticFluxDensity2)
        assertEquals(baseNoiseLevel, magneticFluxDensity2.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity2.unit)
        val magneticFluxDensity3 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getMagnetometerBaseNoiseLevelAsMeasurement(
            magneticFluxDensity3))
        assertEquals(magneticFluxDensity2, magneticFluxDensity3)

        assertEquals(avgNorm, processor.initialMagneticFluxDensityNorm)
        val magneticFluxDensity4 = processor.initialMagneticFluxDensityNormAsMeasurement
        requireNotNull(magneticFluxDensity4)
        assertEquals(avgNorm, magneticFluxDensity4.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity4.unit)
        val magneticFluxDensity5 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getInitialMagneticFluxDensityNormAsMeasurement(
            magneticFluxDensity5))
        assertEquals(magneticFluxDensity4, magneticFluxDensity5)
    }

    @Test
    fun processMagnetometerMeasurement_whenStaticInterval_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity1))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            processor.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val avgNorm = randomizer.nextDouble()
        every { magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        every { magnetometerAccumulatedNoiseEstimatorSpy.avgNorm }.returns(avgNorm)
        processor.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.STATIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)

        val magneticFluxDensity: BodyMagneticFluxDensity? = processor.getPrivateProperty(
            "magneticFluxDensity")
        requireNotNull(magneticFluxDensity)
        assertEquals(0.0, magneticFluxDensity.bx, 0.0)
        assertEquals(0.0, magneticFluxDensity.by, 0.0)
        assertEquals(0.0, magneticFluxDensity.bz, 0.0)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = MagnetometerSensorMeasurement(
            bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp,
            accuracy, MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )
        processor.processMagnetometerMeasurement(measurement)

        // check
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            by.toDouble() + hardIronY.toDouble()), magneticFluxDensity.bx,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            bx.toDouble() + hardIronX.toDouble()), magneticFluxDensity.by,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            -bz.toDouble() - hardIronZ.toDouble()), magneticFluxDensity.bz,
            0.0)
        assertEquals(1, processor.numberOfProcessedMagnetometerMeasurements)

        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.avgNorm
        }
        assertEquals(baseNoiseLevel, processor.magnetometerBaseNoiseLevel)
        val magneticFluxDensity2 = processor.magnetometerBaseNoiseLevelAsMeasurement
        requireNotNull(magneticFluxDensity2)
        assertEquals(baseNoiseLevel, magneticFluxDensity2.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity2.unit)
        val magneticFluxDensity3 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity3))
        assertEquals(magneticFluxDensity2, magneticFluxDensity3)

        assertEquals(avgNorm, processor.initialMagneticFluxDensityNorm)
        val magneticFluxDensity4 = processor.initialMagneticFluxDensityNormAsMeasurement
        requireNotNull(magneticFluxDensity4)
        assertEquals(avgNorm, magneticFluxDensity4.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity4.unit)
        val magneticFluxDensity5 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getInitialMagneticFluxDensityNormAsMeasurement(
            magneticFluxDensity5))
        assertEquals(magneticFluxDensity4, magneticFluxDensity5)
    }

    @Test
    fun processMagnetometerMeasurement_whenDynamicInterval_setsGyroscopeBaseNoiseLevel() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity1))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            processor.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val avgNorm = randomizer.nextDouble()
        every { magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        every { magnetometerAccumulatedNoiseEstimatorSpy.avgNorm }.returns(avgNorm)
        processor.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.DYNAMIC_INTERVAL, processor.status)
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)

        val magneticFluxDensity: BodyMagneticFluxDensity? = processor.getPrivateProperty(
            "magneticFluxDensity")
        requireNotNull(magneticFluxDensity)
        assertEquals(0.0, magneticFluxDensity.bx, 0.0)
        assertEquals(0.0, magneticFluxDensity.by, 0.0)
        assertEquals(0.0, magneticFluxDensity.bz, 0.0)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = MagnetometerSensorMeasurement(
            bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp,
            accuracy, MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )
        processor.processMagnetometerMeasurement(measurement)

        // check
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            by.toDouble() + hardIronY.toDouble()), magneticFluxDensity.bx,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            bx.toDouble() + hardIronX.toDouble()), magneticFluxDensity.by,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            -bz.toDouble() - hardIronZ.toDouble()), magneticFluxDensity.bz,
            0.0)
        assertEquals(1, processor.numberOfProcessedMagnetometerMeasurements)

        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.avgNorm
        }
        assertEquals(baseNoiseLevel, processor.magnetometerBaseNoiseLevel)
        val magneticFluxDensity2 = processor.magnetometerBaseNoiseLevelAsMeasurement
        requireNotNull(magneticFluxDensity2)
        assertEquals(baseNoiseLevel, magneticFluxDensity2.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity2.unit)
        val magneticFluxDensity3 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity3))
        assertEquals(magneticFluxDensity2, magneticFluxDensity3)

        assertEquals(avgNorm, processor.initialMagneticFluxDensityNorm)
        val magneticFluxDensity4 = processor.initialMagneticFluxDensityNormAsMeasurement
        requireNotNull(magneticFluxDensity4)
        assertEquals(avgNorm, magneticFluxDensity4.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity4.unit)
        val magneticFluxDensity5 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getInitialMagneticFluxDensityNormAsMeasurement(
            magneticFluxDensity5))
        assertEquals(magneticFluxDensity4, magneticFluxDensity5)
    }

    @Test
    fun processMagnetometerMeasurement_whenMagnetometerBaseNoiseLevelAlreadySet_increasesMeasurements() {
        val processor = AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor()

        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)
        assertNull(processor.magnetometerBaseNoiseLevel)
        assertNull(processor.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity1))
        assertNull(processor.initialMagneticFluxDensityNorm)
        assertNull(processor.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(processor.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            processor.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        processor.setPrivateProperty(
            "measurementsGenerator",
            measurementsGeneratorSpy
        )

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            processor.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val avgNorm = randomizer.nextDouble()
        processor.setPrivateProperty("magnetometerBaseNoiseLevel", baseNoiseLevel)
        processor.setPrivateProperty("initialMagneticFluxDensityNorm", avgNorm)
        processor.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        assertEquals(Status.INITIALIZATION_COMPLETED, processor.status)
        assertEquals(0, processor.numberOfProcessedMagnetometerMeasurements)

        val magneticFluxDensity: BodyMagneticFluxDensity? = processor.getPrivateProperty(
            "magneticFluxDensity")
        requireNotNull(magneticFluxDensity)
        assertEquals(0.0, magneticFluxDensity.bx, 0.0)
        assertEquals(0.0, magneticFluxDensity.by, 0.0)
        assertEquals(0.0, magneticFluxDensity.bz, 0.0)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accuracy = SensorAccuracy.HIGH
        val measurement = MagnetometerSensorMeasurement(
            bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp,
            accuracy, MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )
        processor.processMagnetometerMeasurement(measurement)

        // check
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            by.toDouble() + hardIronY.toDouble()), magneticFluxDensity.bx,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            bx.toDouble() + hardIronX.toDouble()), magneticFluxDensity.by,
            0.0)
        assertEquals(MagneticFluxDensityConverter.microTeslaToTesla(
            -bz.toDouble() - hardIronZ.toDouble()), magneticFluxDensity.bz,
            0.0)
        assertEquals(1, processor.numberOfProcessedMagnetometerMeasurements)

        verify(exactly = 0) {
            magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }
        verify(exactly = 0) {
            magnetometerAccumulatedNoiseEstimatorSpy.avgNorm
        }
        assertEquals(baseNoiseLevel, processor.magnetometerBaseNoiseLevel)
        val magneticFluxDensity2 = processor.magnetometerBaseNoiseLevelAsMeasurement
        requireNotNull(magneticFluxDensity2)
        assertEquals(baseNoiseLevel, magneticFluxDensity2.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity2.unit)
        val magneticFluxDensity3 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity3))
        assertEquals(magneticFluxDensity2, magneticFluxDensity3)

        assertEquals(avgNorm, processor.initialMagneticFluxDensityNorm)
        val magneticFluxDensity4 = processor.initialMagneticFluxDensityNormAsMeasurement
        requireNotNull(magneticFluxDensity4)
        assertEquals(avgNorm, magneticFluxDensity4.value.toDouble(),
            0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity4.unit)
        val magneticFluxDensity5 = MagneticFluxDensity(0.0,
            MagneticFluxDensityUnit.TESLA)
        assertTrue(processor.getInitialMagneticFluxDensityNormAsMeasurement(
            magneticFluxDensity5))
        assertEquals(magneticFluxDensity4, magneticFluxDensity5)
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