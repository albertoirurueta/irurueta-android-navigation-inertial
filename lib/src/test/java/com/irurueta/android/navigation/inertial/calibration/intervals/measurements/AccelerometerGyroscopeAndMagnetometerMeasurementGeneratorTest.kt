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

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockk
import io.mockk.mockkStatic
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test

class AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var initializationStartedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationStartedListener

    @MockK(relaxUnitFun = true)
    private lateinit var initializationCompletedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationCompletedListener

    @MockK(relaxUnitFun = true)
    private lateinit var errorListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnErrorListener

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalDetectedListener

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalDetectedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalDetectedListener

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalSkippedListener

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalSkippedListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalSkippedListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedAccelerometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedGyroscopeMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var generatedMagnetometerMeasurementListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var resetListener:
            AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnResetListener

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor

    @MockK
    private lateinit var accelerometerCollector: AccelerometerSensorCollector

    @MockK
    private lateinit var gyroscopeCollector: GyroscopeSensorCollector

    @MockK
    private lateinit var magnetometerCollector: MagnetometerSensorCollector

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }.returns(
            magnetometerSensor
        )

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
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            generator.windowSize
        )
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
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold1,
            baseNoiseLevelAbsoluteThreshold2
        )
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
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
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertSame(accelerometerSensor, generator.accelerometerSensor)
        assertSame(gyroscopeSensor, generator.gyroscopeSensor)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertSame(magnetometerSensor, generator.magnetometerSensor)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0, MagneticFluxDensityUnit.TESLA
        )
        assertFalse(
            generator.getMagnetometerBaseNoiseLevelAsMeasurement(
                magneticFluxDensity
            )
        )
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(
            generator.getInitialMagneticFluxDensityNormAsMeasurement(
                magneticFluxDensity
            )
        )
    }

    @Test
    fun constructor_whenNonDefaultValues_setsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER.value) }.returns(
            accelerometerSensor
        )
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }.returns(
            gyroscopeSensor
        )
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER.value) }.returns(
            magnetometerSensor
        )

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
        assertSame(
            staticIntervalDetectedListener,
            generator.staticIntervalDetectedListener
        )
        assertSame(
            dynamicIntervalDetectedListener,
            generator.dynamicIntervalDetectedListener
        )
        assertSame(
            staticIntervalSkippedListener,
            generator.staticIntervalSkippedListener
        )
        assertSame(
            dynamicIntervalSkippedListener,
            generator.dynamicIntervalSkippedListener
        )
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
        assertEquals(
            MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
            generator.minStaticSamples
        )
        assertEquals(
            MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
            generator.maxDynamicSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            generator.windowSize
        )
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
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold1,
            baseNoiseLevelAbsoluteThreshold2
        )
        assertNull(generator.accelerometerBaseNoiseLevel)
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val baseNoiseLevel = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
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
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
        assertSame(accelerometerSensor, generator.accelerometerSensor)
        assertSame(gyroscopeSensor, generator.gyroscopeSensor)
        assertEquals(0, generator.numberOfProcessedGyroscopeMeasurements)
        assertNull(generator.gyroscopeBaseNoiseLevel)
        assertNull(generator.gyroscopeBaseNoiseLevelAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(generator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertSame(magnetometerSensor, generator.magnetometerSensor)
        assertEquals(0, generator.numberOfProcessedMagnetometerMeasurements)
        assertNull(generator.magnetometerBaseNoiseLevel)
        assertNull(generator.magnetometerBaseNoiseLevelAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(
            0.0, MagneticFluxDensityUnit.TESLA
        )
        assertFalse(
            generator.getMagnetometerBaseNoiseLevelAsMeasurement(
                magneticFluxDensity
            )
        )
        assertNull(generator.initialMagneticFluxDensityNorm)
        assertNull(generator.initialMagneticFluxDensityNormAsMeasurement)
        assertFalse(
            generator.getInitialMagneticFluxDensityNormAsMeasurement(
                magneticFluxDensity
            )
        )
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationStartedListener)

        // set new value
        generator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(
            initializationStartedListener,
            generator.initializationStartedListener
        )
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationCompletedListener)

        // set new value
        generator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(
            initializationCompletedListener,
            generator.initializationCompletedListener
        )
    }

    @Test
    fun errorListener_setsExpectedValue() {
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
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalDetectedListener)

        // set new value
        generator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(
            staticIntervalDetectedListener,
            generator.staticIntervalDetectedListener
        )
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalDetectedListener)

        // set new value
        generator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(
            dynamicIntervalDetectedListener,
            generator.dynamicIntervalDetectedListener
        )
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalSkippedListener)

        // set new value
        generator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(
            staticIntervalSkippedListener,
            generator.staticIntervalSkippedListener
        )
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalSkippedListener)

        // set new value
        generator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(
            dynamicIntervalSkippedListener,
            generator.dynamicIntervalSkippedListener
        )
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
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
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedGyroscopeMeasurementListener)

        // set new value
        generator.generatedGyroscopeMeasurementListener =
            generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            generator.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check default value
        assertNull(generator.resetListener)

        // set new value
        generator.resetListener = resetListener

        // check
        assertSame(resetListener, generator.resetListener)
    }

    @Test
    fun minStaticSamples_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val minStaticSamples = randomizer.nextInt()
        every { processor.minStaticSamples }.returns(minStaticSamples)

        // check
        assertEquals(minStaticSamples, generator.minStaticSamples)
    }

    @Test
    fun minStaticSamples_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val minStaticSamples = randomizer.nextInt()
        justRun { processor.minStaticSamples = minStaticSamples }

        generator.minStaticSamples = minStaticSamples

        verify(exactly = 1) { generator.minStaticSamples = minStaticSamples }
    }

    @Test
    fun minStaticSamples_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val minStaticSamples = randomizer.nextInt()
        assertThrows(IllegalStateException::class.java) {
            generator.minStaticSamples = minStaticSamples
        }
    }

    @Test
    fun maxDynamicSamples_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val maxDynamicSamples = randomizer.nextInt()
        every { processor.maxDynamicSamples }.returns(maxDynamicSamples)

        // check
        assertEquals(maxDynamicSamples, generator.maxDynamicSamples)
    }

    @Test
    fun maxDynamicSamples_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val maxDynamicSamples = randomizer.nextInt()
        justRun { processor.maxDynamicSamples = maxDynamicSamples }

        generator.maxDynamicSamples = maxDynamicSamples

        verify(exactly = 1) { generator.maxDynamicSamples = maxDynamicSamples }
    }

    @Test
    fun maxDynamicSamples_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val maxDynamicSamples = randomizer.nextInt()
        assertThrows(IllegalStateException::class.java) {
            generator.maxDynamicSamples = maxDynamicSamples
        }
    }

    @Test
    fun windowSize_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val windowSize = randomizer.nextInt()
        every { processor.windowSize }.returns(windowSize)

        // check
        assertEquals(windowSize, generator.windowSize)
    }

    @Test
    fun windowSize_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val windowSize = randomizer.nextInt()
        justRun { processor.windowSize = windowSize }

        generator.windowSize = windowSize

        verify(exactly = 1) { generator.windowSize = windowSize }
    }

    @Test
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val windowSize = randomizer.nextInt()
        assertThrows(IllegalStateException::class.java) {
            generator.windowSize = windowSize
        }
    }

    @Test
    fun initialStaticSamples_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val initialStaticSamples = randomizer.nextInt()
        every { processor.initialStaticSamples }.returns(initialStaticSamples)

        // check
        assertEquals(initialStaticSamples, generator.initialStaticSamples)
    }

    @Test
    fun initialStaticSamples_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val initialStaticSample = randomizer.nextInt()
        justRun { processor.initialStaticSamples = initialStaticSample }

        generator.initialStaticSamples = initialStaticSample

        verify(exactly = 1) { generator.initialStaticSamples = initialStaticSample }
    }

    @Test
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val initialStaticSamples = randomizer.nextInt()
        assertThrows(IllegalStateException::class.java) {
            generator.initialStaticSamples = initialStaticSamples
        }
    }

    @Test
    fun thresholdFactor_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val thresholdFactor = randomizer.nextDouble()
        every { processor.thresholdFactor }.returns(thresholdFactor)

        // check
        assertEquals(thresholdFactor, generator.thresholdFactor, 0.0)
    }

    @Test
    fun thresholdFactor_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val thresholdFactor = randomizer.nextDouble()
        justRun { processor.thresholdFactor = thresholdFactor }

        generator.thresholdFactor = thresholdFactor

        verify(exactly = 1) { generator.thresholdFactor = thresholdFactor }
    }

    @Test
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val thresholdFactor = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            generator.thresholdFactor = thresholdFactor
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val instantaneousNoiseLevelFactor = randomizer.nextDouble()
        every { processor.instantaneousNoiseLevelFactor }
            .returns(instantaneousNoiseLevelFactor)

        // check
        assertEquals(
            instantaneousNoiseLevelFactor,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val instantaneousNoiseLevelFactor = randomizer.nextDouble()
        justRun { processor.instantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor }

        generator.instantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor

        verify(exactly = 1) {
            generator.instantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val instantaneousNoiseLevelFactor = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            generator.instantaneousNoiseLevelFactor = instantaneousNoiseLevelFactor
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelAbsoluteThreshold = randomizer.nextDouble()
        every { processor.baseNoiseLevelAbsoluteThreshold }
            .returns(baseNoiseLevelAbsoluteThreshold)

        // check
        assertEquals(
            baseNoiseLevelAbsoluteThreshold,
            generator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelAbsoluteThreshold = randomizer.nextDouble()
        justRun { processor.baseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold }

        generator.baseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold

        verify(exactly = 1) {
            generator.baseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val baseNoiseLevelAbsoluteThreshold = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            generator.baseNoiseLevelAbsoluteThreshold = baseNoiseLevelAbsoluteThreshold
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_getsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        every { processor.baseNoiseLevelAbsoluteThresholdAsMeasurement }
            .returns(acceleration)

        // check
        assertSame(
            acceleration,
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        )
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenNotRunning_setsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        assertFalse(generator.running)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        justRun { processor.baseNoiseLevelAbsoluteThresholdAsMeasurement = acceleration }

        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = acceleration

        verify(exactly = 1) {
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = acceleration
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        assertThrows(IllegalStateException::class.java) {
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = acceleration
        }
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement_callsProcessor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        justRun { processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration) }

        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration)

        verify(exactly = 1) {
            processor.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration)
        }
    }

    @Test
    fun accelerometerBaseNoiseLevel_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerBaseNoiseLevel = randomizer.nextDouble()
        every { processor.accelerometerBaseNoiseLevel }
            .returns(accelerometerBaseNoiseLevel)

        // check
        assertEquals(
            accelerometerBaseNoiseLevel,
            generator.accelerometerBaseNoiseLevel
        )

        verify(exactly = 1) { processor.accelerometerBaseNoiseLevel }
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        every { processor.accelerometerBaseNoiseLevelAsMeasurement }
            .returns(acceleration)

        // check
        assertSame(
            acceleration,
            generator.accelerometerBaseNoiseLevelAsMeasurement
        )

        verify(exactly = 1) { processor.accelerometerBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_callsProcessor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        every { processor.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration) }
            .returns(true)

        assertTrue(generator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))

        verify(exactly = 1) {
            processor.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration)
        }
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerBaseNoiseLevelPsd = randomizer.nextDouble()
        every { processor.accelerometerBaseNoiseLevelPsd }
            .returns(accelerometerBaseNoiseLevelPsd)

        // check
        assertEquals(
            accelerometerBaseNoiseLevelPsd,
            generator.accelerometerBaseNoiseLevelPsd
        )

        verify(exactly = 1) { processor.accelerometerBaseNoiseLevelPsd }
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerBaseNoiseLevelRootPsd = randomizer.nextDouble()
        every { processor.accelerometerBaseNoiseLevelRootPsd }
            .returns(accelerometerBaseNoiseLevelRootPsd)

        // check
        assertEquals(
            accelerometerBaseNoiseLevelRootPsd,
            generator.accelerometerBaseNoiseLevelRootPsd
        )

        verify(exactly = 1) { processor.accelerometerBaseNoiseLevelRootPsd }
    }

    @Test
    fun threshold_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { processor.threshold }.returns(threshold)

        // check
        assertEquals(threshold, generator.threshold)

        verify(exactly = 1) { processor.threshold }
    }

    @Test
    fun thresholdAsMeasurement_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        every { processor.thresholdAsMeasurement }.returns(acceleration)

        // check
        assertSame(acceleration, generator.thresholdAsMeasurement)

        verify(exactly = 1) { processor.thresholdAsMeasurement }
    }

    @Test
    fun getThresholdAsMeasurement_callsProcessor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val acceleration = Acceleration(
            0.0,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        every { processor.getThresholdAsMeasurement(acceleration) }
            .returns(true)

        assertTrue(generator.getThresholdAsMeasurement(acceleration))

        verify(exactly = 1) {
            processor.getThresholdAsMeasurement(acceleration)
        }
    }

    @Test
    fun processedStaticSamples_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val processedStaticSamples = randomizer.nextInt()
        every { processor.processedStaticSamples }.returns(processedStaticSamples)

        // check
        assertEquals(processedStaticSamples, generator.processedStaticSamples)

        verify(exactly = 1) { processor.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val processedDynamicSamples = randomizer.nextInt()
        every { processor.processedDynamicSamples }.returns(processedDynamicSamples)

        // check
        assertEquals(processedDynamicSamples, generator.processedDynamicSamples)

        verify(exactly = 1) { processor.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        every { processor.isStaticIntervalSkipped }.returns(true)

        // check
        assertTrue(generator.isStaticIntervalSkipped)

        verify(exactly = 1) { processor.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        every { processor.isDynamicIntervalSkipped }.returns(true)

        // check
        assertTrue(generator.isDynamicIntervalSkipped)

        verify(exactly = 1) { processor.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerAverageTimeInterval = randomizer.nextDouble()
        every { processor.accelerometerAverageTimeInterval }
            .returns(accelerometerAverageTimeInterval)

        // check
        assertEquals(
            accelerometerAverageTimeInterval,
            generator.accelerometerAverageTimeInterval
        )

        verify(exactly = 1) { processor.accelerometerAverageTimeInterval }
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val time = Time(0.0, TimeUnit.SECOND)
        every { processor.accelerometerAverageTimeIntervalAsTime }
            .returns(time)

        // check
        assertSame(time, generator.accelerometerAverageTimeIntervalAsTime)

        verify(exactly = 1) { processor.accelerometerAverageTimeIntervalAsTime }
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_callsProcessor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val time = Time(0.0, TimeUnit.SECOND)
        every { processor.getAccelerometerAverageTimeIntervalAsTime(time) }
            .returns(true)

        assertTrue(generator.getAccelerometerAverageTimeIntervalAsTime(time))

        verify(exactly = 1) {
            processor.getAccelerometerAverageTimeIntervalAsTime(time)
        }
    }

    @Test
    fun accelerometerTimeIntervalVariance_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerTimeIntervalVariance = randomizer.nextDouble()
        every { processor.accelerometerTimeIntervalVariance }
            .returns(accelerometerTimeIntervalVariance)

        // check
        assertEquals(
            accelerometerTimeIntervalVariance,
            generator.accelerometerTimeIntervalVariance
        )

        verify(exactly = 1) { processor.accelerometerTimeIntervalVariance }
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val accelerometerTimeIntervalStandardDeviation = randomizer.nextDouble()
        every { processor.accelerometerTimeIntervalStandardDeviation }
            .returns(accelerometerTimeIntervalStandardDeviation)

        // check
        assertEquals(
            accelerometerTimeIntervalStandardDeviation,
            generator.accelerometerTimeIntervalStandardDeviation
        )
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val time = Time(0.0, TimeUnit.SECOND)
        every { processor.accelerometerTimeIntervalStandardDeviationAsTime }
            .returns(time)

        // check
        assertSame(
            time,
            generator.accelerometerTimeIntervalStandardDeviationAsTime
        )

        verify(exactly = 1) { processor.accelerometerTimeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_callsProcessor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val time = Time(0.0, TimeUnit.SECOND)
        every { processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time) }
            .returns(true)

        assertTrue(generator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        verify(exactly = 1) {
            processor.getAccelerometerTimeIntervalStandardDeviationAsTime(time)
        }
    }

    @Test
    fun numberOfProcessedAccelerometerMeasurements_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val numberOfProcessedAccelerometerMeasurements = randomizer.nextInt()
        every { processor.numberOfProcessedAccelerometerMeasurements }
            .returns(numberOfProcessedAccelerometerMeasurements)

        // check
        assertEquals(
            numberOfProcessedAccelerometerMeasurements,
            generator.numberOfProcessedAccelerometerMeasurements
        )

        verify(exactly = 1) {
            processor.numberOfProcessedAccelerometerMeasurements
        }
    }

    @Test
    fun status_returnsExpectedValue() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val status = Status.IDLE
        every { processor.status }.returns(status)

        // check
        assertSame(status, generator.status)

        verify(exactly = 1) { processor.status }
    }

    @Test
    fun accelerometerSensor_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)

        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // check
        assertSame(accelerometerSensor, generator.accelerometerSensor)
    }

    @Test
    fun accelerometerAccuracyChangedListener_whenAccuracyReliable_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val accelerometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerAccuracyChangedListener"
            )
        requireNotNull(accelerometerAccuracyChangedListener)

        val collector = mockk<AccelerometerSensorCollector>()
        accelerometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.HIGH
        )

        verify { processor wasNot Called }
    }

    @Test
    fun accelerometerAccuracyChangedListener_whenAccuracyUnreliableWithListener_stopsCollectorAndNotifiesUnreliableSensor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val accelerometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerAccuracyChangedListener"
            )
        requireNotNull(accelerometerAccuracyChangedListener)

        justRun { gyroscopeCollector.stop() }
        justRun { accelerometerCollector.stop() }
        justRun { magnetometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<AccelerometerSensorCollector>()
        accelerometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify(exactly = 1) { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun accelerometerAccuracyChangedListener_whenAccuracyUnreliableWithoutListener_stopsCollector() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val accelerometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerAccuracyChangedListener"
            )
        requireNotNull(accelerometerAccuracyChangedListener)

        justRun { accelerometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { magnetometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<AccelerometerSensorCollector>()
        accelerometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify { errorListener wasNot Called }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun accelerometerCollectorMeasurementListener_processesAccelerometerMeasurement() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val accelerometerCollectorMeasurementListener: SensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector>? =
            getPrivateProperty(
                CalibrationMeasurementGenerator::class,
                generator,
                "accelerometerCollectorMeasurementListener"
            )
        requireNotNull(accelerometerCollectorMeasurementListener)

        val collector = mockk<AccelerometerSensorCollector>()
        val measurement = AccelerometerSensorMeasurement()
        justRun { processor.processAccelerometerMeasurement(measurement) }

        accelerometerCollectorMeasurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) {
            processor.processAccelerometerMeasurement(measurement)
        }
    }

    @Test
    fun gyroscopeAccuracyChangedListener_whenAccuracyReliable_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val gyroscopeAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            generator.getPrivateProperty("gyroscopeAccuracyChangedListener")
        requireNotNull(gyroscopeAccuracyChangedListener)

        val collector = mockk<GyroscopeSensorCollector>()
        gyroscopeAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.HIGH
        )

        verify { processor wasNot Called }
    }

    @Test
    fun gyroscopeAccuracyChangedListener_whenAccuracyUnreliableWithListener_stopsCollectorAndNotifiesUnreliableSensor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val gyroscopeAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            generator.getPrivateProperty("gyroscopeAccuracyChangedListener")
        requireNotNull(gyroscopeAccuracyChangedListener)

        justRun { magnetometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { accelerometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<GyroscopeSensorCollector>()
        gyroscopeAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify(exactly = 1) { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun gyroscopeAccuracyChangedListener_whenAccuracyUnreliableWithoutListener_stopsCollector() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val gyroscopeAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            generator.getPrivateProperty("gyroscopeAccuracyChangedListener")
        requireNotNull(gyroscopeAccuracyChangedListener)

        justRun { accelerometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { magnetometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<GyroscopeSensorCollector>()
        gyroscopeAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify { errorListener wasNot Called }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun gyroscopeCollectorMeasurementListener_processesAccelerometerMeasurement() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val gyroscopeCollectorMeasurementListener: SensorCollector.OnMeasurementListener<GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            generator.getPrivateProperty("gyroscopeCollectorMeasurementListener")
        requireNotNull(gyroscopeCollectorMeasurementListener)

        val collector = mockk<GyroscopeSensorCollector>()
        val measurement = GyroscopeSensorMeasurement()
        justRun { processor.processGyroscopeMeasurement(measurement) }

        gyroscopeCollectorMeasurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) {
            processor.processGyroscopeMeasurement(measurement)
        }
    }

    @Test
    fun magnetometerAccuracyChangedListener_whenAccuracyReliable_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val magnetometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<MagnetometerSensorMeasurement, MagnetometerSensorCollector>? =
            generator.getPrivateProperty("magnetometerAccuracyChangedListener")
        requireNotNull(magnetometerAccuracyChangedListener)

        val collector = mockk<MagnetometerSensorCollector>()
        magnetometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.HIGH
        )

        verify { processor wasNot Called }
    }

    @Test
    fun magnetometerAccuracyChangedListener_whenAccuracyUnreliableWithListener_stopsCollectorAndNotifiesUnreliableSensor() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val magnetometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<MagnetometerSensorMeasurement, MagnetometerSensorCollector>? =
            generator.getPrivateProperty("magnetometerAccuracyChangedListener")
        requireNotNull(magnetometerAccuracyChangedListener)

        justRun { magnetometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { accelerometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<MagnetometerSensorCollector>()
        magnetometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify(exactly = 1) { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun magnetometerAccuracyChangedListener_whenAccuracyUnreliableWithoutListener_stopsCollector() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context
        )

        // set processor mock
        generator.setPrivateProperty("processor", processor)
        generator.setPrivateProperty("running", true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)

        val magnetometerAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener<MagnetometerSensorMeasurement, MagnetometerSensorCollector>? =
            generator.getPrivateProperty("magnetometerAccuracyChangedListener")
        requireNotNull(magnetometerAccuracyChangedListener)

        justRun { accelerometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { magnetometerCollector.stop() }
        justRun { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        justRun { processor.unreliable = true }

        val collector = mockk<MagnetometerSensorCollector>()
        magnetometerAccuracyChangedListener.onAccuracyChanged(
            collector,
            SensorAccuracy.UNRELIABLE
        )

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { processor.unreliable = true }
        verify { errorListener wasNot Called }
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun magnetometerCollectorMeasurementListener_processesAccelerometerMeasurement() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        // set processor mock
        generator.setPrivateProperty("processor", processor)

        val magnetometerCollectorMeasurementListener: SensorCollector.OnMeasurementListener<MagnetometerSensorMeasurement, MagnetometerSensorCollector>? =
            generator.getPrivateProperty("magnetometerCollectorMeasurementListener")
        requireNotNull(magnetometerCollectorMeasurementListener)

        val collector = mockk<MagnetometerSensorCollector>()
        val measurement = MagnetometerSensorMeasurement()
        justRun { processor.processMagnetometerMeasurement(measurement) }

        magnetometerCollectorMeasurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) {
            processor.processMagnetometerMeasurement(measurement)
        }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)
        assertTrue(generator.running)

        assertThrows(IllegalStateException::class.java) {
            generator.start()
        }
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsCollector() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

            generator.setPrivateProperty("processor", processor)
            generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
            generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
            generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)
            justRun { processor.reset() }
            every { accelerometerCollector.start(startTimestamp) }.returns(true)
            every { gyroscopeCollector.start(startTimestamp) }.returns(true)
            every { magnetometerCollector.start(startTimestamp) }.returns(true)

            assertFalse(generator.running)

            // start
            generator.start()

            // check
            assertTrue(generator.running)
            verify(exactly = 1) { processor.reset() }
            verify(exactly = 1) { accelerometerCollector.start(startTimestamp) }
            verify(exactly = 1) { gyroscopeCollector.start(startTimestamp) }
            verify(exactly = 1) { magnetometerCollector.start(startTimestamp) }
        }
    }

    @Test
    fun start_whenAccelerometerCollectorStartFails_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

            generator.setPrivateProperty("processor", processor)
            generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
            justRun { processor.reset() }
            every { accelerometerCollector.start(startTimestamp) }.returns(false)

            assertFalse(generator.running)

            // start
            assertThrows(IllegalStateException::class.java) {
                generator.start()
            }

            // check
            assertFalse(generator.running)
            verify(exactly = 1) { processor.reset() }
            verify(exactly = 1) { accelerometerCollector.start(startTimestamp) }
            verify { gyroscopeCollector wasNot Called }
            verify { magnetometerCollector wasNot Called }
        }
    }

    @Test
    fun start_whenGyroscopeCollectorStartFails_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

            generator.setPrivateProperty("processor", processor)
            generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
            generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
            generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)
            justRun { processor.reset() }
            every { accelerometerCollector.start(startTimestamp) }.returns(true)
            every { gyroscopeCollector.start(startTimestamp) }.returns(false)
            justRun { gyroscopeCollector.stop() }
            justRun { accelerometerCollector.stop() }
            justRun { magnetometerCollector.stop() }

            assertFalse(generator.running)

            // start
            assertThrows(IllegalStateException::class.java) {
                generator.start()
            }

            // check
            assertFalse(generator.running)
            verify(exactly = 1) { processor.reset() }
            verify(exactly = 1) { accelerometerCollector.start(startTimestamp) }
            verify(exactly = 1) { gyroscopeCollector.start(startTimestamp) }
            verify(exactly = 1) { accelerometerCollector.stop() }
            verify(exactly = 1) { gyroscopeCollector.stop() }
            verify(exactly = 1) { magnetometerCollector.stop() }
        }
    }

    @Test
    fun start_whenMagnetometerCollectorStartFails_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

            generator.setPrivateProperty("processor", processor)
            generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
            generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
            generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)
            justRun { processor.reset() }
            every { accelerometerCollector.start(startTimestamp) }.returns(true)
            every { gyroscopeCollector.start(startTimestamp) }.returns(true)
            every { magnetometerCollector.start(startTimestamp) }.returns(false)
            justRun { gyroscopeCollector.stop() }
            justRun { accelerometerCollector.stop() }
            justRun { magnetometerCollector.stop() }

            assertFalse(generator.running)

            // start
            assertThrows(IllegalStateException::class.java) {
                generator.start()
            }

            // check
            assertFalse(generator.running)
            verify(exactly = 1) { processor.reset() }
            verify(exactly = 1) { accelerometerCollector.start(startTimestamp) }
            verify(exactly = 1) { gyroscopeCollector.start(startTimestamp) }
            verify(exactly = 1) { magnetometerCollector.start(startTimestamp) }
            verify(exactly = 1) { accelerometerCollector.stop() }
            verify(exactly = 1) { gyroscopeCollector.stop() }
            verify(exactly = 1) { magnetometerCollector.stop() }
        }
    }

    @Test
    fun stop_stopsCollector() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)
        assertTrue(generator.running)

        generator.setPrivateProperty("accelerometerCollector", accelerometerCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollector)
        generator.setPrivateProperty("magnetometerCollector", magnetometerCollector)
        justRun { accelerometerCollector.stop() }
        justRun { gyroscopeCollector.stop() }
        justRun { magnetometerCollector.stop() }

        generator.stop()

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { accelerometerCollector.stop() }
        verify(exactly = 1) { gyroscopeCollector.stop() }
        verify(exactly = 1) { magnetometerCollector.stop() }
    }

    @Test
    fun processorInitializationStarted_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(processor)
    }

    @Test
    fun processorInitializationStarted_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.initializationStartedListener
        requireNotNull(listener)

        listener.onInitializationStarted(processor)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(generator) }
    }

    @Test
    fun processorInitializationCompleted_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.initializationCompletedListener
        requireNotNull(listener)

        listener.onInitializationCompleted(processor, 0.0)
    }

    @Test
    fun processorInitializationCompleted_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.initializationCompletedListener
        requireNotNull(listener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        listener.onInitializationCompleted(processor, baseNoiseLevel)

        verify(exactly = 1) {
            initializationCompletedListener.onInitializationCompleted(generator, baseNoiseLevel)
        }
    }

    @Test
    fun processorError_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.errorListener
        requireNotNull(listener)

        listener.onError(processor, ErrorReason.UNRELIABLE_SENSOR)
    }

    @Test
    fun processorError_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            errorListener = errorListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.errorListener
        requireNotNull(listener)

        listener.onError(processor, ErrorReason.UNRELIABLE_SENSOR)

        verify(exactly = 1) {
            errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR)
        }
    }

    @Test
    fun staticIntervalDetected_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.staticIntervalDetectedListener
        requireNotNull(listener)

        listener.onStaticIntervalDetected(processor)
    }

    @Test
    fun staticIntervalDetected_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.staticIntervalDetectedListener
        requireNotNull(listener)

        listener.onStaticIntervalDetected(processor)

        verify(exactly = 1) {
            staticIntervalDetectedListener.onStaticIntervalDetected(generator)
        }
    }

    @Test
    fun dynamicIntervalDetected_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.dynamicIntervalDetectedListener
        requireNotNull(listener)

        listener.onDynamicIntervalDetected(processor)
    }

    @Test
    fun dynamicIntervalDetected_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.dynamicIntervalDetectedListener
        requireNotNull(listener)

        listener.onDynamicIntervalDetected(processor)

        verify(exactly = 1) {
            dynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
        }
    }

    @Test
    fun staticIntervalSkipped_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.staticIntervalSkippedListener
        requireNotNull(listener)

        listener.onStaticIntervalSkipped(processor)
    }

    @Test
    fun staticIntervalSkipped_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.staticIntervalSkippedListener
        requireNotNull(listener)

        listener.onStaticIntervalSkipped(processor)

        verify(exactly = 1) {
            staticIntervalSkippedListener.onStaticIntervalSkipped(generator)
        }
    }

    @Test
    fun dynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.dynamicIntervalSkippedListener
        requireNotNull(listener)

        listener.onDynamicIntervalSkipped(processor)
    }

    @Test
    fun dynamicIntervalSkipped_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.dynamicIntervalSkippedListener
        requireNotNull(listener)

        listener.onDynamicIntervalSkipped(processor)

        verify(exactly = 1) {
            dynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
        }
    }

    @Test
    fun generatedAccelerometerMeasurement_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedAccelerometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyKinematics()
        listener.onGeneratedAccelerometerMeasurement(processor, measurement)
    }

    @Test
    fun generatedAccelerometerMeasurement_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedAccelerometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyKinematics()
        listener.onGeneratedAccelerometerMeasurement(processor, measurement)

        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                generator, measurement
            )
        }
    }

    @Test
    fun generatedGyroscopeMeasurement_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedGyroscopeMeasurementListener
        requireNotNull(listener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        listener.onGeneratedGyroscopeMeasurement(processor, measurement)
    }

    @Test
    fun generatedGyroscopeMeasurement_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedGyroscopeMeasurementListener
        requireNotNull(listener)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        listener.onGeneratedGyroscopeMeasurement(processor, measurement)

        verify(exactly = 1) {
            generatedGyroscopeMeasurementListener.onGeneratedGyroscopeMeasurement(
                generator, measurement
            )
        }
    }

    @Test
    fun generatedMagnetometerMeasurement_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedMagnetometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        listener.onGeneratedMagnetometerMeasurement(processor, measurement)
    }

    @Test
    fun generatedMagnetometerMeasurement_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.generatedMagnetometerMeasurementListener
        requireNotNull(listener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        listener.onGeneratedMagnetometerMeasurement(processor, measurement)

        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                generator, measurement
            )
        }
    }

    @Test
    fun resetListener_whenNoListener_makesNoAction() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(context)

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.resetListener
        requireNotNull(listener)

        listener.onReset(processor)
    }

    @Test
    fun resetListener_whenListener_notifies() {
        val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
            context,
            resetListener = resetListener
        )

        val processor: AccelerometerGyroscopeAndMagnetometerMeasurementGeneratorProcessor? =
            generator.getPrivateProperty("processor")
        requireNotNull(processor)

        val listener = processor.resetListener
        requireNotNull(listener)

        listener.onReset(processor)

        verify(exactly = 1) { resetListener.onReset(generator) }
    }
}