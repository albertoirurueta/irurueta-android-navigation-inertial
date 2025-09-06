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
package com.irurueta.android.navigation.inertial.calibration.intervals.measurements

import android.content.Context
import android.hardware.Sensor
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.testutils.callPrivateFunc
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.ArgumentCaptor
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initializationStartedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationStartedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var initializationCompletedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationCompletedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var errorListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnErrorListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staticIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalDetectedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var dynamicIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalDetectedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staticIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalSkippedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var dynamicIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalSkippedListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedAccelerometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedGyroscopeMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var generatedMagnetometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var resetListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnResetListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accelerometerMeasurementListener:
            AccelerometerSensorCollector.OnMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var gyroscopeMeasurementListener:
            GyroscopeSensorCollector.OnMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var magnetometerMeasurementListener:
            MagnetometerSensorCollector.OnMeasurementListener

    //    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener

    //    @MockK
    @Mock
    private lateinit var sensor: Sensor

    //    @MockK
    @Mock
    private lateinit var internalGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenAccelerometerSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenAccelerometerSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGyroscopeSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGyroscopeSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenMagnetometerSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenMagnetometerSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenInitializationStartedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenInitializationCompletedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenErrorListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenStaticIntervalDetectedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenDynamicIntervalDetectedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenStaticIntervalSkippedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenDynamicIntervalSkippedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGeneratedAccelerometerMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGeneratedGyroscopeMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertNull(generator.generatedMagnetometerMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGeneratedMagnetometerMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener,
            generatedMagnetometerMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenResetListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
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
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertSame(resetListener, generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenAccelerometerMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
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
            resetListener,
            accelerometerMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenGyroscopeMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
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
            resetListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertNull(generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenMagnetometerMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
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
            resetListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            magnetometerMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertSame(magnetometerMeasurementListener, generator.magnetometerMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun constructor_whenAccuracyChangedListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
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
            resetListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            magnetometerMeasurementListener,
            accuracyChangedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            generator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.magnetometerSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertSame(magnetometerMeasurementListener, generator.magnetometerMeasurementListener)
        assertSame(accuracyChangedListener, generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertNull(generator.magnetometerSensor)
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            generator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(generator.threshold)
        assertNull(generator.thresholdAsMeasurement)
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))
        assertEquals(0, generator.processedStaticSamples)
        assertEquals(0, generator.processedDynamicSamples)
        assertFalse(generator.isStaticIntervalSkipped)
        assertFalse(generator.isDynamicIntervalSkipped)
        assertNull(generator.accelerometerAverageTimeInterval)
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(generator.accelerometerTimeIntervalVariance)
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity))
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationStartedListener)

        // set new value
        generator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, generator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationCompletedListener)

        // set new value
        generator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.errorListener)

        // set new value
        generator.errorListener = errorListener

        // check
        assertSame(errorListener, generator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalDetectedListener)

        // set new value
        generator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalDetectedListener)

        // set new value
        generator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalSkippedListener)

        // set new value
        generator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalSkippedListener)

        // set new value
        generator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedAccelerometerMeasurementListener)

        // set new value
        generator.generatedAccelerometerMeasurementListener =
            generatedAccelerometerMeasurementListener

        // check
        assertSame(
            generatedAccelerometerMeasurementListener,
            generator.generatedAccelerometerMeasurementListener
        )
    }

    @Test
    fun generatedGyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedGyroscopeMeasurementListener)

        // set new value
        generator.generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedMagnetometerMeasurementListener)

        // set new value
        generator.generatedMagnetometerMeasurementListener =
            generatedMagnetometerMeasurementListener

        // check
        assertSame(
            generatedMagnetometerMeasurementListener,
            generator.generatedMagnetometerMeasurementListener
        )
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.resetListener)

        // set new value
        generator.resetListener = resetListener

        // check
        assertSame(resetListener, generator.resetListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.accelerometerMeasurementListener)

        // set new value
        generator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.gyroscopeMeasurementListener)

        // set new value
        generator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.magnetometerMeasurementListener)

        // set new value
        generator.magnetometerMeasurementListener = magnetometerMeasurementListener

        // check
        assertSame(magnetometerMeasurementListener, generator.magnetometerMeasurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.accuracyChangedListener)

        // set new value
        generator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, generator.accuracyChangedListener)
    }

    @Test
    fun accelerometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.accelerometerSensor)

        // set new value
        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
        whenever(accelerometerCollectorSpy.sensor).thenReturn(sensor)
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
//        every { accelerometerCollectorSpy.sensor }.returns(sensor)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        // check
        assertSame(sensor, generator.accelerometerSensor)
    }

    @Test
    fun gyroscopeSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.gyroscopeSensor)

        // set new value
        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
        whenever(gyroscopeCollectorSpy.sensor).thenReturn(sensor)
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
//        every { gyroscopeCollectorSpy.sensor }.returns(sensor)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        // check
        assertSame(sensor, generator.gyroscopeSensor)
    }

    @Test
    fun magnetometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.magnetometerSensor)

        // set new value
        val magnetometerCollector: MagnetometerSensorCollector? =
            generator.getPrivateProperty("magnetometerCollector")
        requireNotNull(magnetometerCollector)
        val magnetometerCollectorSpy = spy(magnetometerCollector)
        whenever(magnetometerCollectorSpy.sensor).thenReturn(sensor)
//        val magnetometerCollectorSpy = spyk(magnetometerCollector)
//        every { magnetometerCollectorSpy.sensor }.returns(sensor)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollectorSpy)

        // check
        assertSame(sensor, generator.magnetometerSensor)
    }

    @Test
    fun minStaticSamples_whenNotRunningAndValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)

        // set new value
        generator.minStaticSamples = MIN_STATIC_SAMPLES

        // check
        assertEquals(MIN_STATIC_SAMPLES, generator.minStaticSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun minStaticSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)

        // set new value
        generator.minStaticSamples = 2
    }

    @Test(expected = IllegalStateException::class)
    fun minStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)

        // set running
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        // set new value
        generator.minStaticSamples = MIN_STATIC_SAMPLES
    }

    @Test
    fun maxDynamicSamples_whenNotRunningAndValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)

        // set new value
        generator.maxDynamicSamples = MAX_DYNAMIC_SAMPLES

        // check
        assertEquals(MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun maxDynamicSamples_whenInvalidValue_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)

        // set new value
        generator.maxDynamicSamples = 2
    }

    @Test(expected = IllegalStateException::class)
    fun maxDynamicSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)

        // set running
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        // set new value
        generator.maxDynamicSamples = MAX_DYNAMIC_SAMPLES
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)

        // set new value
        generator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, generator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)

        generator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)

        // set running
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        generator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )

        // set new value
        generator.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, generator.initialStaticSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )

        generator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        generator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )

        // check default value
        generator.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, generator.thresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )

        // set new value
        generator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        // set new value
        generator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        generator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(INSTANTANEOUS_NOISE_LEVEL_FACTOR, generator.instantaneousNoiseLevelFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )

        generator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        generator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        generator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        generator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        val baseNoiseLevelAbsoluteThreshold1 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
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
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)

        // set new value
        val baseNoiseLevelAbsoluteThreshold3 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = baseNoiseLevelAbsoluteThreshold3

        // check
        val baseNoiseLevelAbsoluteThreshold4 =
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
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
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold5)
        assertEquals(baseNoiseLevelAbsoluteThreshold4, baseNoiseLevelAbsoluteThreshold5)
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevel)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.accelerometerBaseNoiseLevel).thenReturn(baseNoiseLevel1)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevel }.returns(baseNoiseLevel1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevel)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val baseNoiseLevel2 = generator.accelerometerBaseNoiseLevel
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val baseNoiseLevel1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.accelerometerBaseNoiseLevelAsMeasurement).thenReturn(
            baseNoiseLevel1
        )
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        /*        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelAsMeasurement }.returns(
                    baseNoiseLevel1
                )*/
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val baseNoiseLevel2 = generator.accelerometerBaseNoiseLevelAsMeasurement
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        val baseNoiseLevel1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel1))
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        doAnswer { invocation ->
            val result = invocation.getArgument<Acceleration>(0)
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }.whenever(measurementsGeneratorSpy).getAccelerometerBaseNoiseLevelAsMeasurement(any())
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
/*        every { measurementsGeneratorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }*/
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(0.0, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel.unit)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        assertTrue(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(value, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel.unit)
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.accelerometerBaseNoiseLevelPsd).thenReturn(baseNoiseLevelPsd1)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelPsd }.returns(baseNoiseLevelPsd1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelPsd)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val baseNoiseLevelPsd2 = generator.accelerometerBaseNoiseLevelPsd
        requireNotNull(baseNoiseLevelPsd2)
        assertEquals(baseNoiseLevelPsd1, baseNoiseLevelPsd2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.accelerometerBaseNoiseLevelRootPsd).thenReturn(baseNoiseLevelRootPsd1)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
/*        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelRootPsd }.returns(
            baseNoiseLevelRootPsd1
        )*/
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val baseNoiseLevelRootPsd2 = generator.accelerometerBaseNoiseLevelRootPsd
        requireNotNull(baseNoiseLevelRootPsd2)
        assertEquals(baseNoiseLevelRootPsd1, baseNoiseLevelRootPsd2, 0.0)
    }

    @Test
    fun threshold_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.threshold)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun threshold_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val threshold1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.threshold).thenReturn(threshold1)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.threshold }.returns(threshold1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.threshold)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val threshold2 = generator.threshold
        requireNotNull(threshold2)
        assertEquals(threshold1, threshold2, 0.0)
    }

    @Test
    fun thresholdAsMeasurement_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.thresholdAsMeasurement)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun thresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val threshold1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.thresholdAsMeasurement).thenReturn(threshold1)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.thresholdAsMeasurement }.returns(threshold1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.thresholdAsMeasurement)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val threshold2 = generator.thresholdAsMeasurement
        requireNotNull(threshold2)
        assertSame(threshold1, threshold2)
    }

    @Test
    fun getThresholdAsMeasurement_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getThresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        doAnswer { invocation ->
            val result = invocation.getArgument<Acceleration>(0)
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }.whenever(measurementsGeneratorSpy).getThresholdAsMeasurement(any())
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
/*        every { measurementsGeneratorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }*/
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        assertTrue(generator.getThresholdAsMeasurement(threshold))
        assertEquals(value, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
    }

    @Test
    fun processedStaticSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(0, generator.processedStaticSamples)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.processedStaticSamples).thenReturn(value)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.processedStaticSamples }.returns(value)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertEquals(value, generator.processedStaticSamples)
        verify(measurementsGeneratorSpy, only()).processedStaticSamples
//        verify(exactly = 1) { measurementsGeneratorSpy.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertEquals(0, generator.processedDynamicSamples)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.processedDynamicSamples).thenReturn(value)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.processedDynamicSamples }.returns(value)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertEquals(value, generator.processedDynamicSamples)
        verify(measurementsGeneratorSpy, only()).processedDynamicSamples
//        verify(exactly = 1) { measurementsGeneratorSpy.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertFalse(generator.isStaticIntervalSkipped)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.isStaticIntervalSkipped).thenReturn(true)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.isStaticIntervalSkipped }.returns(true)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertTrue(generator.isStaticIntervalSkipped)
        verify(measurementsGeneratorSpy, only()).isStaticIntervalSkipped
//        verify(exactly = 1) { measurementsGeneratorSpy.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertFalse(generator.isDynamicIntervalSkipped)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.isDynamicIntervalSkipped).thenReturn(true)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.isDynamicIntervalSkipped }.returns(true)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertTrue(generator.isDynamicIntervalSkipped)
        verify(measurementsGeneratorSpy, only()).isDynamicIntervalSkipped
//        verify(exactly = 1) { measurementsGeneratorSpy.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerAverageTimeInterval)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeInterval_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        whenever(timeIntervalEstimatorSpy.averageTimeInterval).thenReturn(value1)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
//        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerAverageTimeInterval)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val value2 = generator.accelerometerAverageTimeInterval
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        whenever(timeIntervalEstimatorSpy.averageTimeIntervalAsTime).thenReturn(time1)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
//        every { timeIntervalEstimatorSpy.averageTimeIntervalAsTime }.returns(time1)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerAverageTimeIntervalAsTime)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val time2 = generator.accelerometerAverageTimeIntervalAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        doAnswer { invocation ->
            val result = invocation.getArgument<Time>(0)
            result.value = value
            result.unit = TimeUnit.SECOND
        }.whenever(timeIntervalEstimatorSpy).getAverageTimeIntervalAsTime(any())
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
/*        every { timeIntervalEstimatorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }*/
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        assertTrue(generator.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalVariance)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        whenever(timeIntervalEstimatorSpy.timeIntervalVariance).thenReturn(value1)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
//        every { timeIntervalEstimatorSpy.timeIntervalVariance }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalVariance)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val value2 = generator.accelerometerTimeIntervalVariance
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        whenever(timeIntervalEstimatorSpy.timeIntervalStandardDeviation).thenReturn(value1)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
//        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviation }.returns(value1)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalStandardDeviation)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val value2 = generator.accelerometerTimeIntervalStandardDeviation
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        whenever(timeIntervalEstimatorSpy.timeIntervalStandardDeviationAsTime).thenReturn(time1)
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
//        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviationAsTime }.returns(time1)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        val time2 = generator.accelerometerTimeIntervalStandardDeviationAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spy(timeIntervalEstimator)
        doAnswer { invocation ->
            val result = invocation.getArgument<Time>(0)
            result.value = value
            result.unit = TimeUnit.SECOND
        }.whenever(timeIntervalEstimatorSpy).getTimeIntervalStandardDeviationAsTime(any())
//        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
/*        every { timeIntervalEstimatorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
        }*/
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        // check
        assertTrue(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun status_whenUnreliable_returnsFailed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable", true)

        assertEquals(Status.FAILED, generator.status)
    }

    @Test
    fun status_whenReliableAndIdle_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.IDLE)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.IDLE)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun status_whenReliableAndInitializing_returnsInitializing() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
    }

    @Test
    fun status_whenReliableAndInitializationCompleted_returnsInitializationCompleted() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, generator.status)
    }

    @Test
    fun status_whenReliableAndStaticInterval_returnsStaticInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.STATIC_INTERVAL, generator.status)
    }

    @Test
    fun status_whenReliableAndDynamicInterval_returnsDynamicInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.DYNAMIC_INTERVAL, generator.status)
    }

    @Test
    fun status_whenReliableAndFailed_returnsFailed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.FAILED)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.FAILED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.FAILED, generator.status)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializingAndNoMeasurements_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerCollectorMeasurementListener"
            )
        requireNotNull(listener)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wy
        kinematics.angularRateY = wx
        kinematics.angularRateZ = -wz
        generator.setPrivateProperty("kinematics", kinematics)

        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val magneticFluxDensity = BodyMagneticFluxDensity(by, bx, -bz)
        generator.setPrivateProperty("magneticFluxDensity", magneticFluxDensity)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, biasX, biasY, biasZ, timestamp, accuracy)

        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(timestamp, initialAccelerometerTimestamp)
        assertEquals(1, generator.numberOfProcessedAccelerometerMeasurements)
        val captor = ArgumentCaptor.captor<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(measurementsGeneratorSpy, times(1)).process(captor.capture())
/*        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }*/

        val timedBodyKinematicsAndMagneticFluxDensity = captor.value
//        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(ay.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx, 0.0)
        assertEquals(ax.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy, 0.0)
        assertEquals(-az.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz, 0.0)
        assertEquals(wy, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX, 0.0)
        assertEquals(wx, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY, 0.0)
        assertEquals(-wz, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematicsAndMagneticFluxDensity.kinematics)
        assertEquals(by, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx, 0.0)
        assertEquals(bx, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by, 0.0)
        assertEquals(-bz, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz, 0.0)
        assertEquals(
            magneticFluxDensity,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity
        )
        assertEquals(0.0, timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds, 0.0)

        verifyNoInteractions(accelerometerTimeIntervalEstimatorSpy)
//        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializingAndAvailableMeasurements_addsTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerCollectorMeasurementListener"
            )
        requireNotNull(listener)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "numberOfProcessedAccelerometerMeasurements",
            1
        )

        assertEquals(Status.INITIALIZING, generator.status)
        assertEquals(1, generator.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wy
        kinematics.angularRateY = wx
        kinematics.angularRateZ = -wz
        generator.setPrivateProperty("kinematics", kinematics)

        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val magneticFluxDensity = BodyMagneticFluxDensity(by, bx, -bz)
        generator.setPrivateProperty("magneticFluxDensity", magneticFluxDensity)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, biasX, biasY, biasZ, timestamp, accuracy)

        val seconds = TimeConverter.nanosecondToSecond(timestamp.toDouble())
        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).addTimestamp(seconds)
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.addTimestamp(seconds) }

        assertEquals(2, generator.numberOfProcessedAccelerometerMeasurements)

        val captor = ArgumentCaptor.captor<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(measurementsGeneratorSpy, times(1)).process(captor.capture())
/*        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }*/

        val timedBodyKinematicsAndMagneticFluxDensity = captor.value
//        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(ay.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx, 0.0)
        assertEquals(ax.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy, 0.0)
        assertEquals(-az.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz, 0.0)
        assertEquals(wy, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX, 0.0)
        assertEquals(wx, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY, 0.0)
        assertEquals(-wz, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematicsAndMagneticFluxDensity.kinematics)
        assertEquals(by, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx, 0.0)
        assertEquals(bx, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by, 0.0)
        assertEquals(-bz, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz, 0.0)
        assertEquals(
            magneticFluxDensity,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity
        )
        assertEquals(seconds, timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds, 0.0)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializationCompleted_setsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerCollectorMeasurementListener"
            )
        requireNotNull(listener)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        whenever(accelerometerTimeIntervalEstimatorSpy.averageTimeInterval).thenReturn(timeInterval)
//        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }.returns(timeInterval)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
/*        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )*/
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wy
        kinematics.angularRateY = wx
        kinematics.angularRateZ = -wz
        generator.setPrivateProperty("kinematics", kinematics)

        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val magneticFluxDensity = BodyMagneticFluxDensity(by, bx, -bz)
        generator.setPrivateProperty("magneticFluxDensity", magneticFluxDensity)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, biasX, biasY, biasZ, timestamp, accuracy)

        val captor = ArgumentCaptor.captor<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(measurementsGeneratorSpy, times(1)).process(captor.capture())
/*        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }*/

        val timedBodyKinematicsAndMagneticFluxDensity = captor.value
//        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(ay.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx, 0.0)
        assertEquals(ax.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy, 0.0)
        assertEquals(-az.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz, 0.0)
        assertEquals(wy, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX, 0.0)
        assertEquals(wx, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY, 0.0)
        assertEquals(-wz, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematicsAndMagneticFluxDensity.kinematics)
        assertEquals(by, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx, 0.0)
        assertEquals(bx, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by, 0.0)
        assertEquals(-bz, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz, 0.0)
        assertEquals(
            magneticFluxDensity,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity
        )
        assertEquals(
            TimeConverter.nanosecondToSecond(timestamp.toDouble()),
            timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds,
            0.0
        )

        verify(accelerometerTimeIntervalEstimatorSpy, only()).averageTimeInterval
        verify(measurementsGeneratorSpy, times(1)).timeInterval = timeInterval
/*        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }*/

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerCollectorMeasurementListener"
            )
        requireNotNull(listener)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wy
        kinematics.angularRateY = wx
        kinematics.angularRateZ = -wz
        generator.setPrivateProperty("kinematics", kinematics)

        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val magneticFluxDensity = BodyMagneticFluxDensity(by, bx, -bz)
        generator.setPrivateProperty("magneticFluxDensity", magneticFluxDensity)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, biasX, biasY, biasZ, timestamp, accuracy)

        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(timestamp, initialAccelerometerTimestamp)
        assertEquals(1, generator.numberOfProcessedAccelerometerMeasurements)
        val captor = ArgumentCaptor.captor<TimedBodyKinematicsAndMagneticFluxDensity>()
        verify(measurementsGeneratorSpy, times(1)).process(captor.capture())
//        val slot = slot<TimedBodyKinematicsAndMagneticFluxDensity>()
//        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematicsAndMagneticFluxDensity = captor.value
//        val timedBodyKinematicsAndMagneticFluxDensity = slot.captured
        assertEquals(ay.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fx, 0.0)
        assertEquals(ax.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fy, 0.0)
        assertEquals(-az.toDouble(), timedBodyKinematicsAndMagneticFluxDensity.kinematics.fz, 0.0)
        assertEquals(wy, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateX, 0.0)
        assertEquals(wx, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateY, 0.0)
        assertEquals(-wz, timedBodyKinematicsAndMagneticFluxDensity.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematicsAndMagneticFluxDensity.kinematics)
        assertEquals(by, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bx, 0.0)
        assertEquals(bx, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.by, 0.0)
        assertEquals(-bz, timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity.bz, 0.0)
        assertEquals(
            magneticFluxDensity,
            timedBodyKinematicsAndMagneticFluxDensity.magneticFluxDensity
        )
        assertEquals(0.0, timedBodyKinematicsAndMagneticFluxDensity.timestampSeconds, 0.0)

        verifyNoInteractions(accelerometerTimeIntervalEstimatorSpy)
//        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
        verify(accelerometerMeasurementListener, only()).onMeasurement(
            ax,
            ay,
            az,
            biasX,
            biasY,
            biasZ,
            timestamp,
            accuracy
        )
/*        verify(exactly = 1) {
            accelerometerMeasurementListener.onMeasurement(
                ax,
                ay,
                az,
                biasX,
                biasY,
                biasZ,
                timestamp,
                accuracy
            )
        }*/
    }

    @Test
    fun collectorAccuracyChangedListener_whenUnreliableAndNoListener_setsStopsAndSetsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val unreliable1: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "collectorAccuracyChangedListener"
            )
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        val unreliable2: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        verify(accelerometerCollectorSpy, times(1)).stop()
        verify(gyroscopeCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
    }

    @Test
    fun collectorAccuracyChangedListener_whenUnreliableAndListenersAvailable_setsStopsAndSetsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class, generator, "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val unreliable1: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "collectorAccuracyChangedListener"
            )
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        val unreliable2: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        verify(accelerometerCollectorSpy, times(1)).stop()
        verify(gyroscopeCollectorSpy, times(1)).stop()
        verify(errorListener, only()).onError(generator, ErrorReason.UNRELIABLE_SENSOR)
        verify(accuracyChangedListener, only()).onAccuracyChanged(SensorAccuracy.UNRELIABLE)
        //verify(exactly = 1) { accelerometerCollectorSpy.stop() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
//        verify(exactly = 1) { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
//        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE) }
    }

    @Test
    fun collectorAccuracyChangedListener_whenNotUnreliableAndListenersAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener,
            accuracyChangedListener = accuracyChangedListener
        )

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "collectorAccuracyChangedListener"
            )
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)

        verifyNoInteractions(errorListener)
        verify(accuracyChangedListener, only()).onAccuracyChanged(SensorAccuracy.HIGH)
//        verify { errorListener wasNot Called }
//        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH) }
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
        doReturn(true).whenever(accelerometerCollectorSpy).start()
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
//        every { accelerometerCollectorSpy.start() }.returns(true)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
        doReturn(true).whenever(gyroscopeCollectorSpy).start()
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
//        every { gyroscopeCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val magnetometerCollector: MagnetometerSensorCollector? =
            generator.getPrivateProperty("magnetometerCollector")
        requireNotNull(magnetometerCollector)
        val magnetometerCollectorSpy = spy(magnetometerCollector)
        doReturn(true).whenever(magnetometerCollectorSpy).start()
//        val magnetometerCollectorSpy = spyk(magnetometerCollector)
//        every { magnetometerCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollectorSpy)

        assertFalse(generator.running)

        // start
        generator.start()

        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).reset()
        verify(measurementsGeneratorSpy, only()).reset()
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
//        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertTrue(generator.running)

        verify(accelerometerCollectorSpy, only()).start()
        verify(gyroscopeCollectorSpy, only()).start()
        verify(magnetometerCollectorSpy, only()).start()
//        verify(exactly = 1) { accelerometerCollectorSpy.start() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.start() }
//        verify(exactly = 1) { magnetometerCollectorSpy.start() }
    }

    @Test
    fun start_whenAccelerometerCollectorDoesNotStart_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
        doReturn(false).whenever(accelerometerCollectorSpy).start()
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
//        every { accelerometerCollectorSpy.start() }.returns(false)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
        doReturn(true).whenever(gyroscopeCollectorSpy).start()
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
//        every { gyroscopeCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val magnetometerCollector: MagnetometerSensorCollector? =
            generator.getPrivateProperty("magnetometerCollector")
        requireNotNull(magnetometerCollector)
        val magnetometerCollectorSpy = spy(magnetometerCollector)
        doReturn(true).whenever(magnetometerCollectorSpy).start()
//        val magnetometerCollectorSpy = spyk(magnetometerCollector)
//        every { magnetometerCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollectorSpy)

        assertFalse(generator.running)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).reset()
        verify(measurementsGeneratorSpy, only()).reset()
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
//        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertFalse(generator.running)

        verify(accelerometerCollectorSpy, only()).start()
        verifyNoInteractions(gyroscopeCollectorSpy)
        verifyNoInteractions(magnetometerCollectorSpy)
//        verify(exactly = 1) { accelerometerCollectorSpy.start() }
//        verify { gyroscopeCollectorSpy wasNot Called }
//        verify { magnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenGyroscopeCollectorDoesNotStart_stopsAndThrowsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
        doReturn(true).whenever(accelerometerCollectorSpy).start()
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
//        every { accelerometerCollectorSpy.start() }.returns(true)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
        doReturn(false).whenever(gyroscopeCollectorSpy).start()
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
//        every { gyroscopeCollectorSpy.start() }.returns(false)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val magnetometerCollector: MagnetometerSensorCollector? =
            generator.getPrivateProperty("magnetometerCollector")
        requireNotNull(magnetometerCollector)
        val magnetometerCollectorSpy = spy(magnetometerCollector)
        doReturn(true).whenever(magnetometerCollectorSpy).start()
//        val magnetometerCollectorSpy = spyk(magnetometerCollector)
//        every { magnetometerCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollectorSpy)

        assertFalse(generator.running)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).reset()
        verify(measurementsGeneratorSpy, only()).reset()
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
//        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertFalse(generator.running)

        verify(accelerometerCollectorSpy, times(1)).start()
        verify(gyroscopeCollectorSpy, times(1)).start()
        verify(magnetometerCollectorSpy, never()).start()
//        verify(exactly = 1) { accelerometerCollectorSpy.start() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.start() }
//        verify(exactly = 0) { magnetometerCollectorSpy.start() }

        // also collectors were stopped
        verify(accelerometerCollectorSpy, times(1)).stop()
        verify(gyroscopeCollectorSpy, times(1)).stop()
        verify(magnetometerCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
//        verify(exactly = 1) { magnetometerCollectorSpy.stop() }
    }

    @Test
    fun start_whenMagnetometerCollectorDoesNotStart_stopsAndThrowsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
        doReturn(true).whenever(accelerometerCollectorSpy).start()
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
//        every { accelerometerCollectorSpy.start() }.returns(true)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
        doReturn(true).whenever(gyroscopeCollectorSpy).start()
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
//        every { gyroscopeCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val magnetometerCollector: MagnetometerSensorCollector? =
            generator.getPrivateProperty("magnetometerCollector")
        requireNotNull(magnetometerCollector)
        val magnetometerCollectorSpy = spy(magnetometerCollector)
        doReturn(false).whenever(magnetometerCollectorSpy).start()
//        val magnetometerCollectorSpy = spyk(magnetometerCollector)
//        every { magnetometerCollectorSpy.start() }.returns(false)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollectorSpy)

        assertFalse(generator.running)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).reset()
        verify(measurementsGeneratorSpy, only()).reset()
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
//        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertFalse(generator.running)

        verify(accelerometerCollectorSpy, times(1)).start()
        verify(gyroscopeCollectorSpy, times(1)).start()
        verify(magnetometerCollectorSpy, times(1)).start()
//        verify(exactly = 1) { accelerometerCollectorSpy.start() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.start() }
//        verify(exactly = 1) { magnetometerCollectorSpy.start() }

        // also collectors were stopped
        verify(accelerometerCollectorSpy, times(1)).stop()
        verify(gyroscopeCollectorSpy, times(1)).stop()
        verify(magnetometerCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
//        verify(exactly = 1) { magnetometerCollectorSpy.stop() }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verifyNoInteractions(accelerometerTimeIntervalEstimatorSpy)
        verifyNoInteractions(measurementsGeneratorSpy)
        verifyNoInteractions(accelerometerCollectorSpy)
        verifyNoInteractions(gyroscopeCollectorSpy)
//        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
//        verify { measurementsGeneratorSpy wasNot Called }
//        verify { accelerometerCollectorSpy wasNot Called }
//        verify { gyroscopeCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsCollectorAndSetsRunningToFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerCollector: AccelerometerSensorCollector? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spy(accelerometerCollector)
//        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerCollector",
            accelerometerCollectorSpy
        )

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spy(gyroscopeCollector)
//        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "running", true)
        assertTrue(generator.running)

        // stop
        generator.stop()

        // check
        assertFalse(generator.running)
        verify(accelerometerCollectorSpy, times(1)).stop()
        verify(gyroscopeCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
//        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
    }

    @Test
    fun reset_setsValuesToInitialStateAndResetsAccumulatedNoiseEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerTimeIntervalEstimator"
            )
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spy(accelerometerTimeIntervalEstimator)
//        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable", true)
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "initialAccelerometerTimestamp",
            1L
        )
        setPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "numberOfProcessedAccelerometerMeasurements",
            1
        )
        generator.setPrivateProperty("numberOfProcessedGyroscopeMeasurements", 1)
        generator.setPrivateProperty("numberOfProcessedMagnetometerMeasurements", 1)
        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized", true)

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            generator.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spy(gyroscopeAccumulatedNoiseEstimator)
//        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        generator.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )
        val randomizer = UniformRandomizer()
        val gyroscopeBaseNoiseLevel = randomizer.nextDouble()
        generator.setPrivateProperty("gyroscopeBaseNoiseLevel", gyroscopeBaseNoiseLevel)
        assertEquals(gyroscopeBaseNoiseLevel, generator.gyroscopeBaseNoiseLevel)

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            generator.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spy(magnetometerAccumulatedNoiseEstimator)
//        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        generator.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )
        val magnetometerBaseNoiseLevel = randomizer.nextDouble()
        generator.setPrivateProperty("magnetometerBaseNoiseLevel", magnetometerBaseNoiseLevel)
        assertEquals(magnetometerBaseNoiseLevel, generator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        generator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )
        assertEquals(initialMagneticFluxDensityNorm, generator.initialMagneticFluxDensityNorm)

        assertEquals(
            TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES,
            accelerometerTimeIntervalEstimatorSpy.totalSamples
        )

        // reset
        callPrivateFunc(
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator::class,
            generator,
            "reset"
        )

        assertEquals(Integer.MAX_VALUE, accelerometerTimeIntervalEstimatorSpy.totalSamples)
        verify(accelerometerTimeIntervalEstimatorSpy, times(1)).reset()
        verify(measurementsGeneratorSpy, only()).reset()
//        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
//        verify(exactly = 1) { measurementsGeneratorSpy.reset() }
        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)
        val initialAccelerometerTimestamp: Long? = getPrivateProperty(
            CalibrationMeasurementGenerator::class,
            generator,
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(0L, initialAccelerometerTimestamp)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(
            generator.getGyroscopeBaseNoiseLevelAsMeasurement(
                AngularSpeed(
                    0.0,
                    AngularSpeedUnit.RADIANS_PER_SECOND
                )
            )
        )
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(
            generator.getMagnetometerBaseNoiseLevelAsMeasurement(
                MagneticFluxDensity(
                    0.0,
                    MagneticFluxDensityUnit.TESLA
                )
            )
        )
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(
            generator.getInitialMagneticFluxDensityNormAsMeasurement(
                MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
            )
        )
        verify(magnetometerAccumulatedNoiseEstimatorSpy, only()).reset()
        verify(gyroscopeAccumulatedNoiseEstimatorSpy, only()).reset()
//        verify(exactly = 1) { magnetometerAccumulatedNoiseEstimatorSpy.reset() }
//        verify(exactly = 1) { gyroscopeAccumulatedNoiseEstimatorSpy.reset() }

        val initialized: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun mapErrorReason_whenNotUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val unreliable: Boolean? =
            getPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        var result: ErrorReason? = callPrivateFuncWithResult(
            CalibrationMeasurementGenerator::class,
            generator,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION, result)

        result = callPrivateFuncWithResult(
            CalibrationMeasurementGenerator::class,
            generator,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION, result)
    }

    @Test
    fun mapErrorReason_whenUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        setPrivateProperty(CalibrationMeasurementGenerator::class, generator, "unreliable", true)

        var result: ErrorReason? = callPrivateFuncWithResult(
            CalibrationMeasurementGenerator::class,
            generator,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)

        result = callPrivateFuncWithResult(
            CalibrationMeasurementGenerator::class,
            generator,
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)
    }

    @Test
    fun onInitializationStarted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)
    }

    @Test
    fun onInitializationStarted_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                initializationStartedListener = initializationStartedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onInitializationStarted(internalGenerator)

        verify(initializationStartedListener, only()).onInitializationStarted(generator)
//        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(generator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)
    }

    @Test
    fun onInitializationCompleted_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                initializationCompletedListener = initializationCompletedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)

        verify(initializationCompletedListener, only()).onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
/*        verify(exactly = 1) {
            initializationCompletedListener.onInitializationCompleted(
                generator,
                baseNoiseLevel
            )
        }*/
    }

    @Test
    fun onError_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
    }

    @Test
    fun onError_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                errorListener = errorListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(errorListener, only()).onError(
            generator,
            ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
        )
/*        verify(exactly = 1) {
            errorListener.onError(
                generator,
                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }*/
    }

    @Test
    fun onStaticIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                staticIntervalDetectedListener = staticIntervalDetectedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)

        verify(staticIntervalDetectedListener, only()).onStaticIntervalDetected(generator)
//        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(generator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)

        verify(dynamicIntervalDetectedListener, only()).onDynamicIntervalDetected(generator)
//        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(generator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                staticIntervalSkippedListener = staticIntervalSkippedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)

        verify(staticIntervalSkippedListener, only()).onStaticIntervalSkipped(generator)
//        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(generator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)

        verify(dynamicIntervalSkippedListener, only()).onDynamicIntervalSkipped(generator)
//        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )

        verify(generatedAccelerometerMeasurementListener, only()).onGeneratedAccelerometerMeasurement(
            generator,
            measurement
        )
/*        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                generator,
                measurement
            )
        }*/
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )

        verify(generatedGyroscopeMeasurementListener, only()).onGeneratedGyroscopeMeasurement(
            generator,
            measurement
        )
/*        verify(exactly = 1) {
            generatedGyroscopeMeasurementListener.onGeneratedGyroscopeMeasurement(
                generator,
                measurement
            )
        }*/
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        measurementsGeneratorListener.onGeneratedMagnetometerMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        measurementsGeneratorListener.onGeneratedMagnetometerMeasurement(
            internalGenerator,
            measurement
        )

        verify(generatedMagnetometerMeasurementListener, only()).onGeneratedMagnetometerMeasurement(
            generator,
            measurement
        )
/*        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                generator,
                measurement
            )
        }*/
    }

    @Test
    fun onReset_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)
    }

    @Test
    fun onReset_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
                context,
                resetListener = resetListener
            )

        val measurementsGeneratorListener: AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        measurementsGeneratorListener.onReset(internalGenerator)

        verify(resetListener, only()).onReset(generator)
//        verify(exactly = 1) { resetListener.onReset(generator) }
    }

    @Test
    fun onGyroscopeMeasurementListener_whenNoListener_setsAngularRates() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertNull(generator.gyroscopeMeasurementListener)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)

        val gyroscopeCollectorMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("gyroscopeCollectorMeasurementListener")
        requireNotNull(gyroscopeCollectorMeasurementListener)

        val kinematics: BodyKinematics? = generator.getPrivateProperty("kinematics")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeCollectorMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals(wy.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, generator.numberOfProcessedGyroscopeMeasurements)
    }

    @Test
    fun onGyroscopeMeasurementListener_whenListener_setsAngularRates() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )

        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)

        val gyroscopeCollectorMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("gyroscopeCollectorMeasurementListener")
        requireNotNull(gyroscopeCollectorMeasurementListener)

        val kinematics: BodyKinematics? = generator.getPrivateProperty("kinematics")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeCollectorMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals(wy.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, generator.numberOfProcessedGyroscopeMeasurements)

        verify(gyroscopeMeasurementListener, only()).onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )
/*        verify(exactly = 1) {
            gyroscopeMeasurementListener.onMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )
        }*/
    }

    @Test
    fun onGyroscopeMeasurementListener_whenInitializing_accumulatesNoise() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            generator.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spy(gyroscopeAccumulatedNoiseEstimator)
//        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        generator.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        val gyroscopeCollectorMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("gyroscopeCollectorMeasurementListener")
        requireNotNull(gyroscopeCollectorMeasurementListener)

        val kinematics: BodyKinematics? = generator.getPrivateProperty("kinematics")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeCollectorMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals(wy.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, generator.numberOfProcessedGyroscopeMeasurements)

        verify(gyroscopeAccumulatedNoiseEstimatorSpy, times(1)).addTriad(
            wy.toDouble(),
            wx.toDouble(),
            -wz.toDouble()
        )
/*        verify(exactly = 1) {
            gyroscopeAccumulatedNoiseEstimatorSpy.addTriad(
                wy.toDouble(),
                wx.toDouble(),
                -wz.toDouble()
            )
        }*/
    }

    @Test
    fun onGyroscopeMeasurementListener_whenInitializationCompleted_setsGyroscopeBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed1 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val gyroscopeAccumulatedNoiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            generator.getPrivateProperty("gyroscopeAccumulatedNoiseEstimator")
        requireNotNull(gyroscopeAccumulatedNoiseEstimator)
        val gyroscopeAccumulatedNoiseEstimatorSpy = spy(gyroscopeAccumulatedNoiseEstimator)
//        val gyroscopeAccumulatedNoiseEstimatorSpy = spyk(gyroscopeAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        whenever(gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm).thenReturn(baseNoiseLevel)
//        every { gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(baseNoiseLevel)
        generator.setPrivateProperty(
            "gyroscopeAccumulatedNoiseEstimator",
            gyroscopeAccumulatedNoiseEstimatorSpy
        )

        val gyroscopeCollectorMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("gyroscopeCollectorMeasurementListener")
        requireNotNull(gyroscopeCollectorMeasurementListener)

        val kinematics: BodyKinematics? = generator.getPrivateProperty("kinematics")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeCollectorMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals(wy.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wx.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(-wz.toDouble(), kinematics.angularRateZ, 0.0)
        assertEquals(1, generator.numberOfProcessedGyroscopeMeasurements)

        verify(gyroscopeAccumulatedNoiseEstimatorSpy, times(2)).standardDeviationNorm
/*        verify(exactly = 1) {
            gyroscopeAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }*/
        assertEquals(baseNoiseLevel, generator.gyroscopeBaseNoiseLevel)
        val angularSpeed2 = generator.gyroscopeBaseNoiseLevelAsMeasurement
        requireNotNull(angularSpeed2)
        assertEquals(baseNoiseLevel, angularSpeed2.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed2.unit)
        val angularSpeed3 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed3))
        assertEquals(angularSpeed2, angularSpeed3)
    }

    @Test
    fun onMagnetometerMeasurementListener_whenNoListener_setsMagneticFluxDensity() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertNull(generator.magnetometerMeasurementListener)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)

        val magnetometerCollectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("magnetometerCollectorMeasurementListener")
        requireNotNull(magnetometerCollectorMeasurementListener)

        val magneticFluxDensity: BodyMagneticFluxDensity? =
            generator.getPrivateProperty("magneticFluxDensity")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        magnetometerCollectorMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        // check
        val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
        val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
        val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())
        assertEquals(byTesla, magneticFluxDensity.bx, 0.0)
        assertEquals(bxTesla, magneticFluxDensity.by, 0.0)
        assertEquals(-bzTesla, magneticFluxDensity.bz, 0.0)
        assertEquals(1, generator.numberOfProcessedMagnetometerMeasurements)
    }

    @Test
    fun onMagnetometerMeasurementListener_whenListener_setsMagneticFluxDensity() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            magnetometerMeasurementListener = magnetometerMeasurementListener
        )

        assertSame(magnetometerMeasurementListener, generator.magnetometerMeasurementListener)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)

        val magnetometerCollectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("magnetometerCollectorMeasurementListener")
        requireNotNull(magnetometerCollectorMeasurementListener)

        val magneticFluxDensity: BodyMagneticFluxDensity? =
            generator.getPrivateProperty("magneticFluxDensity")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        magnetometerCollectorMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        // check
        val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
        val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
        val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())
        assertEquals(byTesla, magneticFluxDensity.bx, 0.0)
        assertEquals(bxTesla, magneticFluxDensity.by, 0.0)
        assertEquals(-bzTesla, magneticFluxDensity.bz, 0.0)
        assertEquals(1, generator.numberOfProcessedMagnetometerMeasurements)

        verify(magnetometerMeasurementListener, only()).onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )
/*        verify(exactly = 1) {
            magnetometerMeasurementListener.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )
        }*/
    }

    @Test
    fun onMagnetometerMeasurementListener_whenInitializing_accumulatesNoise() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZING)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            generator.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spy(magnetometerAccumulatedNoiseEstimator)
//        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        generator.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        val magnetometerCollectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("magnetometerCollectorMeasurementListener")
        requireNotNull(magnetometerCollectorMeasurementListener)

        val magneticFluxDensity: BodyMagneticFluxDensity? =
            generator.getPrivateProperty("magneticFluxDensity")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        magnetometerCollectorMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        // check
        val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
        val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
        val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())
        assertEquals(byTesla, magneticFluxDensity.bx, 0.0)
        assertEquals(bxTesla, magneticFluxDensity.by, 0.0)
        assertEquals(-bzTesla, magneticFluxDensity.bz, 0.0)
        assertEquals(1, generator.numberOfProcessedMagnetometerMeasurements)

        verify(magnetometerAccumulatedNoiseEstimatorSpy, times(1)).addTriad(
            bxTesla,
            byTesla,
            bzTesla
        )
/*        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.addTriad(
                bxTesla,
                byTesla,
                bzTesla
            )
        }*/
    }

    @Test
    fun onMagnetometerMeasurementListener_whenInitializationCompleted_setsMagnetometerBaseNoiseLevelAndInitialMagneticFluxDensityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity1))
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity1))

        val measurementsGenerator: AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spy(measurementsGenerator)
        whenever(measurementsGeneratorSpy.status).thenReturn(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
//        val measurementsGeneratorSpy = spyk(measurementsGenerator)
//        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val magnetometerAccumulatedNoiseEstimator: AccumulatedMagneticFluxDensityTriadNoiseEstimator? =
            generator.getPrivateProperty("magnetometerAccumulatedNoiseEstimator")
        requireNotNull(magnetometerAccumulatedNoiseEstimator)
        val magnetometerAccumulatedNoiseEstimatorSpy = spy(magnetometerAccumulatedNoiseEstimator)
//        val magnetometerAccumulatedNoiseEstimatorSpy = spyk(magnetometerAccumulatedNoiseEstimator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        whenever(magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm).thenReturn(baseNoiseLevel)
        whenever(magnetometerAccumulatedNoiseEstimatorSpy.avgNorm).thenReturn(initialMagneticFluxDensityNorm)
/*        every { magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm }.returns(
            baseNoiseLevel
        )
        every { magnetometerAccumulatedNoiseEstimatorSpy.avgNorm }
            .returns(initialMagneticFluxDensityNorm)*/
        generator.setPrivateProperty(
            "magnetometerAccumulatedNoiseEstimator",
            magnetometerAccumulatedNoiseEstimatorSpy
        )

        val magnetometerCollectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("magnetometerCollectorMeasurementListener")
        requireNotNull(magnetometerCollectorMeasurementListener)

        val magneticFluxDensity: BodyMagneticFluxDensity? =
            generator.getPrivateProperty("magneticFluxDensity")
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
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        magnetometerCollectorMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        // check
        val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
        val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
        val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())
        assertEquals(byTesla, magneticFluxDensity.bx, 0.0)
        assertEquals(bxTesla, magneticFluxDensity.by, 0.0)
        assertEquals(-bzTesla, magneticFluxDensity.bz, 0.0)
        assertEquals(1, generator.numberOfProcessedMagnetometerMeasurements)

        verify(magnetometerAccumulatedNoiseEstimatorSpy, times(2)).standardDeviationNorm
/*        verify(exactly = 1) {
            magnetometerAccumulatedNoiseEstimatorSpy.standardDeviationNorm
        }*/

        assertEquals(baseNoiseLevel, generator.magnetometerBaseNoiseLevel)
        val magneticFluxDensity2 = generator.magnetometerBaseNoiseLevelAsMeasurement
        requireNotNull(magneticFluxDensity2)
        assertEquals(baseNoiseLevel, magneticFluxDensity2.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity2.unit)
        val magneticFluxDensity3 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(generator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity3))
        assertEquals(magneticFluxDensity2, magneticFluxDensity3)
        assertEquals(initialMagneticFluxDensityNorm, generator.initialMagneticFluxDensityNorm)
        val magneticFluxDensity4 = generator.initialMagneticFluxDensityNormAsMeasurement
        requireNotNull(magneticFluxDensity4)
        assertEquals(initialMagneticFluxDensityNorm, magneticFluxDensity4.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, magneticFluxDensity4.unit)
        assertTrue(generator.getInitialMagneticFluxDensityNormAsMeasurement(magneticFluxDensity3))
        assertEquals(magneticFluxDensity4, magneticFluxDensity3)
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