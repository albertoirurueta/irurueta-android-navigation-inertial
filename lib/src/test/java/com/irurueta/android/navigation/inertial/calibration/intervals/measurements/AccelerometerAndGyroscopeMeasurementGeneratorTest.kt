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
import com.irurueta.android.navigation.inertial.callPrivateFunc
import com.irurueta.android.navigation.inertial.callPrivateFuncWithResult
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.generators.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerAndGyroscopeMeasurementGeneratorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenAccelerometerSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenAccelerometerSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenGyroscopeSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, generator.gyroscopeSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenGyroscopeSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertNull(generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenInitializationStartedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertNull(generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertNull(generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenErrorListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertNull(generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenStaticIntervalDetectedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertNull(generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenDynamicIntervalDetectedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertNull(generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenStaticIntervalSkippedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertNull(generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenDynamicIntervalSkippedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
        assertSame(initializationStartedListener, generator.initializationStartedListener)
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
        assertSame(errorListener, generator.errorListener)
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
        assertNull(generator.generatedAccelerometerMeasurementListener)
        assertNull(generator.generatedGyroscopeMeasurementListener)
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenGeneratedAccelerometerMeasurementListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenGeneratedGyroscopeMeasurementListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertNull(generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenResetListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
        val resetListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            resetListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertSame(resetListener, generator.resetListener)
        assertNull(generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenAccelerometerMeasurementListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
        val resetListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            resetListener,
            accelerometerMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertNull(generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenGyroscopeMeasurementListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
        val resetListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            resetListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertNull(generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun constructor_whenAccuracyChangedListener_setsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
        val resetListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            resetListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            accuracyChangedListener
        )

        // check default values
        assertSame(context, generator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            generator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.accelerometerSensorDelay)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            generator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, generator.gyroscopeSensorDelay)
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
        assertSame(resetListener, generator.resetListener)
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
        assertSame(accuracyChangedListener, generator.accuracyChangedListener)
        assertNull(generator.accelerometerSensor)
        assertNull(generator.gyroscopeSensor)
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)
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
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)
        assertFalse(generator.running)
        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>()
        generator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, generator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>()
        generator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, generator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.errorListener)

        // set new value
        val errorListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>()
        generator.errorListener = errorListener

        // check
        assertSame(errorListener, generator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>()
        generator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, generator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalDetectedListener)

        // set new value
        assertNull(generator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>()
        generator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, generator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>()
        generator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, generator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>()
        generator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, generator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedAccelerometerMeasurementListener)

        // set new value
        val generatedAccelerometerMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>()
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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.generatedGyroscopeMeasurementListener)

        // set new value
        val generatedGyroscopeMeasurementListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>()
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.resetListener)

        // set new value
        val resetListener = mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>()
        generator.resetListener = resetListener

        // check
        assertSame(resetListener, generator.resetListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        generator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, generator.accelerometerMeasurementListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.gyroscopeMeasurementListener)

        // set new value
        val gyroscopeMeasurementListener =
            mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        generator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        generator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, generator.accuracyChangedListener)
    }

    @Test
    fun accelerometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.accelerometerSensor)

        // set new value
        val accelerometerCollector: AccelerometerSensorCollector? =
            generator.getPrivateProperty("accelerometerCollector")
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        val sensor = mockk<Sensor>()
        every { accelerometerCollectorSpy.sensor }.returns(sensor)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        // check
        assertSame(sensor, generator.accelerometerSensor)
    }

    @Test
    fun gyroscopeSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertNull(generator.gyroscopeSensor)

        // set new value
        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        val sensor = mockk<Sensor>()
        every { gyroscopeCollectorSpy.sensor }.returns(sensor)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        // check
        assertSame(sensor, generator.gyroscopeSensor)
    }

    @Test
    fun minStaticSamples_whenNotRunningAndValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)

        // set new value
        generator.minStaticSamples = 2
    }

    @Test(expected = IllegalStateException::class)
    fun minStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.minStaticSamples)

        // set running
        generator.setPrivateProperty("running", true)

        // set new value
        generator.minStaticSamples = MIN_STATIC_SAMPLES
    }

    @Test
    fun maxDynamicSamples_whenNotRunningAndValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)

        // set new value
        generator.maxDynamicSamples = 2
    }

    @Test(expected = IllegalStateException::class)
    fun maxDynamicSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.maxDynamicSamples)

        // set running
        generator.setPrivateProperty("running", true)

        // set new value
        generator.maxDynamicSamples = MAX_DYNAMIC_SAMPLES
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)

        generator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.windowSize)

        // set running
        generator.setPrivateProperty("running", true)

        generator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            generator.initialStaticSamples
        )

        generator.setPrivateProperty("running", true)

        generator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            generator.thresholdFactor,
            0.0
        )

        generator.setPrivateProperty("running", true)

        // set new value
        generator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            generator.instantaneousNoiseLevelFactor,
            0.0
        )

        generator.setPrivateProperty("running", true)

        generator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        generator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

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
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevel)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevel_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevel }.returns(baseNoiseLevel1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevel)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val baseNoiseLevel2 = generator.accelerometerBaseNoiseLevel
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val baseNoiseLevel1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelAsMeasurement }.returns(
            baseNoiseLevel1
        )
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelAsMeasurement)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val baseNoiseLevel2 = generator.accelerometerBaseNoiseLevelAsMeasurement
        requireNotNull(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        val baseNoiseLevel1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel1))
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val baseNoiseLevel = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(0.0, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel.unit)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        assertTrue(generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel))

        // check
        assertEquals(value, baseNoiseLevel.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel.unit)
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelPsd)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelPsd }.returns(baseNoiseLevelPsd1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelPsd)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val baseNoiseLevelPsd2 = generator.accelerometerBaseNoiseLevelPsd
        requireNotNull(baseNoiseLevelPsd2)
        assertEquals(baseNoiseLevelPsd1, baseNoiseLevelPsd2, 0.0)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.accelerometerBaseNoiseLevelRootPsd }.returns(
            baseNoiseLevelRootPsd1
        )
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.accelerometerBaseNoiseLevelRootPsd)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val baseNoiseLevelRootPsd2 = generator.accelerometerBaseNoiseLevelRootPsd
        requireNotNull(baseNoiseLevelRootPsd2)
        assertEquals(baseNoiseLevelRootPsd1, baseNoiseLevelRootPsd2, 0.0)
    }

    @Test
    fun threshold_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.threshold)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun threshold_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val threshold1 = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.threshold }.returns(threshold1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.threshold)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val threshold2 = generator.threshold
        requireNotNull(threshold2)
        assertEquals(threshold1, threshold2, 0.0)
    }

    @Test
    fun thresholdAsMeasurement_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.thresholdAsMeasurement)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun thresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val threshold1 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.thresholdAsMeasurement }.returns(threshold1)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertNull(generator.thresholdAsMeasurement)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val threshold2 = generator.thresholdAsMeasurement
        requireNotNull(threshold2)
        assertSame(threshold1, threshold2)
    }

    @Test
    fun getThresholdAsMeasurement_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getThresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val threshold = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(generator.getThresholdAsMeasurement(threshold))

        assertEquals(0.0, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        assertTrue(generator.getThresholdAsMeasurement(threshold))
        assertEquals(value, threshold.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold.unit)
    }

    @Test
    fun processedStaticSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(0, generator.processedStaticSamples)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.processedStaticSamples }.returns(value)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertEquals(value, generator.processedStaticSamples)
        verify(exactly = 1) { measurementsGeneratorSpy.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertEquals(0, generator.processedDynamicSamples)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextInt()
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.processedDynamicSamples }.returns(value)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertEquals(value, generator.processedDynamicSamples)
        verify(exactly = 1) { measurementsGeneratorSpy.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertFalse(generator.isStaticIntervalSkipped)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.isStaticIntervalSkipped }.returns(true)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertTrue(generator.isStaticIntervalSkipped)
        verify(exactly = 1) { measurementsGeneratorSpy.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check default value
        assertFalse(generator.isDynamicIntervalSkipped)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.isDynamicIntervalSkipped }.returns(true)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        // check
        assertTrue(generator.isDynamicIntervalSkipped)
        verify(exactly = 1) { measurementsGeneratorSpy.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerAverageTimeInterval)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeInterval_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(value1)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerAverageTimeInterval)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val value2 = generator.accelerometerAverageTimeInterval
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerAverageTimeIntervalAsTime)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.averageTimeIntervalAsTime }.returns(time1)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerAverageTimeIntervalAsTime)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val time2 = generator.accelerometerAverageTimeIntervalAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
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
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        assertTrue(generator.getAccelerometerAverageTimeIntervalAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalVariance)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalVariance_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalVariance }.returns(value1)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalVariance)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val value2 = generator.accelerometerTimeIntervalVariance
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalStandardDeviation)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value1 = randomizer.nextDouble()
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviation }.returns(value1)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalStandardDeviation)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val value2 = generator.accelerometerTimeIntervalStandardDeviation
        requireNotNull(value2)
        assertEquals(value1, value2, 0.0)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
            "accelerometerTimeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time1 = Time(value, TimeUnit.SECOND)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.timeIntervalStandardDeviationAsTime }.returns(time1)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertNull(generator.accelerometerTimeIntervalStandardDeviationAsTime)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        val time2 = generator.accelerometerTimeIntervalStandardDeviationAsTime
        requireNotNull(time2)
        assertSame(time1, time2)
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val timeIntervalEstimator: TimeIntervalEstimator? = generator.getPrivateProperty(
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
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(generator.getTimeIntervalStandardDeviationAsTime(time))

        assertEquals(0.0, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)

        // set as initialized
        generator.setPrivateProperty("initialized", true)

        // check
        assertTrue(generator.getTimeIntervalStandardDeviationAsTime(time))

        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
    }

    @Test
    fun status_whenUnreliable_returnsFailed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.setPrivateProperty("unreliable", true)

        assertEquals(Status.FAILED, generator.status)
    }

    @Test
    fun status_whenReliableAndIdle_returnsIdle() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.IDLE)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.IDLE, generator.status)
    }

    @Test
    fun status_whenReliableAndInitializing_returnsInitializing() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
    }

    @Test
    fun status_whenReliableAndInitializationCompleted_returnsInitializationCompleted() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, generator.status)
    }

    @Test
    fun status_whenReliableAndStaticInterval_returnsStaticInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.STATIC_INTERVAL)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.STATIC_INTERVAL, generator.status)
    }

    @Test
    fun status_whenReliableAndDynamicInterval_returnsDynamicInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.DYNAMIC_INTERVAL, generator.status)
    }

    @Test
    fun status_whenReliableAndFailed_returnsFailed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.FAILED)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.FAILED, generator.status)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializingAndNoMeasurements_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("accelerometerCollectorMeasurementListener")
        requireNotNull(listener)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wx
        kinematics.angularRateY = wy
        kinematics.angularRateZ = wz
        generator.setPrivateProperty("kinematics", kinematics)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)

        val initialAccelerometerTimestamp: Long? = generator.getPrivateProperty(
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(timestamp, initialAccelerometerTimestamp)
        assertEquals(1, generator.numberOfProcessedAccelerometerMeasurements)
        val slot = slot<TimedBodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematics = slot.captured
        assertEquals(ax.toDouble(), timedBodyKinematics.kinematics.fx, 0.0)
        assertEquals(ay.toDouble(), timedBodyKinematics.kinematics.fy, 0.0)
        assertEquals(az.toDouble(), timedBodyKinematics.kinematics.fz, 0.0)
        assertEquals(wx, timedBodyKinematics.kinematics.angularRateX, 0.0)
        assertEquals(wy, timedBodyKinematics.kinematics.angularRateY, 0.0)
        assertEquals(wz, timedBodyKinematics.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematics.kinematics)
        assertEquals(0.0, timedBodyKinematics.timestampSeconds, 0.0)

        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializingAndAvailableMeasurements_addsTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("accelerometerCollectorMeasurementListener")
        requireNotNull(listener)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        generator.setPrivateProperty(
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
        kinematics.angularRateX = wx
        kinematics.angularRateY = wy
        kinematics.angularRateZ = wz
        generator.setPrivateProperty("kinematics", kinematics)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)

        val seconds = TimeConverter.nanosecondToSecond(timestamp.toDouble())
        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.addTimestamp(seconds) }

        assertEquals(2, generator.numberOfProcessedAccelerometerMeasurements)

        val slot = slot<TimedBodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematics = slot.captured
        assertEquals(ax.toDouble(), timedBodyKinematics.kinematics.fx, 0.0)
        assertEquals(ay.toDouble(), timedBodyKinematics.kinematics.fy, 0.0)
        assertEquals(az.toDouble(), timedBodyKinematics.kinematics.fz, 0.0)
        assertEquals(wx, timedBodyKinematics.kinematics.angularRateX, 0.0)
        assertEquals(wy, timedBodyKinematics.kinematics.angularRateY, 0.0)
        assertEquals(wz, timedBodyKinematics.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematics.kinematics)
        assertEquals(seconds, timedBodyKinematics.timestampSeconds, 0.0)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenInitializationCompleted_setsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("accelerometerCollectorMeasurementListener")
        requireNotNull(listener)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        val randomizer = UniformRandomizer()
        val timeInterval = randomizer.nextDouble()
        every { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }.returns(timeInterval)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(
            TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED
        )
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZATION_COMPLETED, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wx
        kinematics.angularRateY = wy
        kinematics.angularRateZ = wz
        generator.setPrivateProperty("kinematics", kinematics)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)

        val slot = slot<TimedBodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematics = slot.captured
        assertEquals(ax.toDouble(), timedBodyKinematics.kinematics.fx, 0.0)
        assertEquals(ay.toDouble(), timedBodyKinematics.kinematics.fy, 0.0)
        assertEquals(az.toDouble(), timedBodyKinematics.kinematics.fz, 0.0)
        assertEquals(wx, timedBodyKinematics.kinematics.angularRateX, 0.0)
        assertEquals(wy, timedBodyKinematics.kinematics.angularRateY, 0.0)
        assertEquals(wz, timedBodyKinematics.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematics.kinematics)
        assertEquals(0.0, timedBodyKinematics.timestampSeconds, 0.0)

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.averageTimeInterval }
        verify(exactly = 1) { measurementsGeneratorSpy.timeInterval = timeInterval }

        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun onAccelerometerMeasurementListener_whenListenerAvailable_notifies() {
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )

        val listener: AccelerometerSensorCollector.OnMeasurementListener? =
            generator.getPrivateProperty("accelerometerCollectorMeasurementListener")
        requireNotNull(listener)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        every { measurementsGeneratorSpy.status }.returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        assertEquals(Status.INITIALIZING, generator.status)
        assertEquals(0, generator.numberOfProcessedAccelerometerMeasurements)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val kinematics = BodyKinematics()
        kinematics.angularRateX = wx
        kinematics.angularRateY = wy
        kinematics.angularRateZ = wz
        generator.setPrivateProperty("kinematics", kinematics)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)

        val initialAccelerometerTimestamp: Long? = generator.getPrivateProperty(
            "initialAccelerometerTimestamp"
        )
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(timestamp, initialAccelerometerTimestamp)
        assertEquals(1, generator.numberOfProcessedAccelerometerMeasurements)
        val slot = slot<TimedBodyKinematics>()
        verify(exactly = 1) { measurementsGeneratorSpy.process(capture(slot)) }

        val timedBodyKinematics = slot.captured
        assertEquals(ax.toDouble(), timedBodyKinematics.kinematics.fx, 0.0)
        assertEquals(ay.toDouble(), timedBodyKinematics.kinematics.fy, 0.0)
        assertEquals(az.toDouble(), timedBodyKinematics.kinematics.fz, 0.0)
        assertEquals(wx, timedBodyKinematics.kinematics.angularRateX, 0.0)
        assertEquals(wy, timedBodyKinematics.kinematics.angularRateY, 0.0)
        assertEquals(wz, timedBodyKinematics.kinematics.angularRateZ, 0.0)
        assertSame(kinematics, timedBodyKinematics.kinematics)
        assertEquals(0.0, timedBodyKinematics.timestampSeconds, 0.0)

        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
        verify(exactly = 1) {
            accelerometerMeasurementListener.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )
        }
    }

    @Test
    fun collectorAccuracyChangedListener_whenUnreliableAndNoListener_setsStopsAndSetsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val unreliable1: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            generator.getPrivateProperty("collectorAccuracyChangedListener")
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        val unreliable2: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
    }

    @Test
    fun collectorAccuracyChangedListener_whenUnreliableAndListenersAvailable_setsStopsAndSetsUnreliable() {
        val errorListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>(
                relaxUnitFun = true
            )
        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            errorListener = errorListener,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        val unreliable1: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            generator.getPrivateProperty("collectorAccuracyChangedListener")
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        val unreliable2: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
        verify(exactly = 1) { errorListener.onError(generator, ErrorReason.UNRELIABLE_SENSOR) }
        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE) }
    }

    @Test
    fun collectorAccuracyChangedListener_whenNotUnreliableAndListenersAvailable_makesNoAction() {
        val errorListener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>(
                relaxUnitFun = true
            )
        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            errorListener = errorListener,
            accuracyChangedListener = accuracyChangedListener
        )

        val collectorAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            generator.getPrivateProperty("collectorAccuracyChangedListener")
        requireNotNull(collectorAccuracyChangedListener)

        collectorAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)

        verify { errorListener wasNot Called }
        verify(exactly = 1) { accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH) }
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        every { accelerometerCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        every { gyroscopeCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        assertFalse(generator.running)

        // start
        generator.start()

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertTrue(generator.running)

        verify(exactly = 1) { accelerometerCollectorSpy.start() }
        verify(exactly = 1) { gyroscopeCollectorSpy.start() }
    }

    @Test
    fun start_whenAccelerometerCollectorDoesNotStart_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        every { accelerometerCollectorSpy.start() }.returns(false)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        every { gyroscopeCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        assertFalse(generator.running)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertFalse(generator.running)

        verify(exactly = 1) { accelerometerCollectorSpy.start() }
        verify { gyroscopeCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenGyroscopeCollectorDoesNotStart_stopsAndThrowsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        every { accelerometerCollectorSpy.start() }.returns(true)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? = generator.getPrivateProperty(
            "gyroscopeCollector"
        )
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        every { gyroscopeCollectorSpy.start() }.returns(false)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        assertFalse(generator.running)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }

        assertFalse(generator.running)

        verify(exactly = 1) { accelerometerCollectorSpy.start() }
        verify(exactly = 1) { gyroscopeCollectorSpy.start() }

        // also collectors were stopped
        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.setPrivateProperty("running", true)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        // start
        assertThrows(IllegalStateException::class.java) { generator.start() }

        verify { accelerometerTimeIntervalEstimatorSpy wasNot Called }
        verify { measurementsGeneratorSpy wasNot Called }
        verify { accelerometerCollectorSpy wasNot Called }
        verify { gyroscopeCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsCollectorAndSetsRunningToFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerCollector: AccelerometerSensorCollector? = generator.getPrivateProperty(
            "accelerometerCollector"
        )
        requireNotNull(accelerometerCollector)
        val accelerometerCollectorSpy = spyk(accelerometerCollector)
        generator.setPrivateProperty("accelerometerCollector", accelerometerCollectorSpy)

        val gyroscopeCollector: GyroscopeSensorCollector? =
            generator.getPrivateProperty("gyroscopeCollector")
        requireNotNull(gyroscopeCollector)
        val gyroscopeCollectorSpy = spyk(gyroscopeCollector)
        generator.setPrivateProperty("gyroscopeCollector", gyroscopeCollectorSpy)

        generator.setPrivateProperty("running", true)
        assertTrue(generator.running)

        // stop
        generator.stop()

        // check
        assertFalse(generator.running)
        verify(exactly = 1) { accelerometerCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeCollectorSpy.stop() }
    }

    @Test
    fun reset_setsValuesToInitialState() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val accelerometerTimeIntervalEstimator: TimeIntervalEstimator? =
            generator.getPrivateProperty("accelerometerTimeIntervalEstimator")
        requireNotNull(accelerometerTimeIntervalEstimator)
        val accelerometerTimeIntervalEstimatorSpy = spyk(accelerometerTimeIntervalEstimator)
        generator.setPrivateProperty(
            "accelerometerTimeIntervalEstimator",
            accelerometerTimeIntervalEstimatorSpy
        )

        val measurementsGenerator: AccelerometerAndGyroscopeMeasurementsGenerator? =
            generator.getPrivateProperty("measurementsGenerator")
        requireNotNull(measurementsGenerator)
        val measurementsGeneratorSpy = spyk(measurementsGenerator)
        generator.setPrivateProperty("measurementsGenerator", measurementsGeneratorSpy)

        generator.setPrivateProperty("unreliable", true)
        generator.setPrivateProperty("initialAccelerometerTimestamp", 1L)
        generator.setPrivateProperty("numberOfProcessedAccelerometerMeasurements", 1)
        generator.setPrivateProperty("initialized", true)

        assertEquals(
            TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES,
            accelerometerTimeIntervalEstimatorSpy.totalSamples
        )

        // reset
        callPrivateFunc(AccelerometerAndGyroscopeMeasurementGenerator::class, generator,"reset")

        assertEquals(Integer.MAX_VALUE, accelerometerTimeIntervalEstimatorSpy.totalSamples)
        verify(exactly = 1) { accelerometerTimeIntervalEstimatorSpy.reset() }
        verify(exactly = 1) { measurementsGeneratorSpy.reset() }
        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)
        val initialAccelerometerTimestamp: Long? =
            generator.getPrivateProperty("initialAccelerometerTimestamp")
        requireNotNull(initialAccelerometerTimestamp)
        assertEquals(0L, initialAccelerometerTimestamp)
        val numberOfProcessedAccelerometerMeasurements: Int? = generator.getPrivateProperty(
            "numberOfProcessedAccelerometerMeasurements"
        )
        requireNotNull(numberOfProcessedAccelerometerMeasurements)
        assertEquals(0, numberOfProcessedAccelerometerMeasurements)
        val initialized: Boolean? = generator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)
    }

    @Test
    fun mapErrorReason_whenNotUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val unreliable: Boolean? = generator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        var result: ErrorReason? = generator.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION, result)

        result = generator.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION, result)
    }

    @Test
    fun mapErrorReason_whenUnreliable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        generator.setPrivateProperty("unreliable", true)

        var result: ErrorReason? = generator.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)

        result = generator.callPrivateFuncWithResult(
            "mapErrorReason",
            TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED
        )
        requireNotNull(result)
        assertEquals(ErrorReason.UNRELIABLE_SENSOR, result)
    }

    @Test
    fun onInitializationStarted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onInitializationStarted(internalGenerator)
    }

    @Test
    fun onInitializationStarted_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                initializationStartedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onInitializationStarted(internalGenerator)

        verify(exactly = 1) { listener.onInitializationStarted(generator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)
    }

    @Test
    fun onInitializationCompleted_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                initializationCompletedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        measurementsGeneratorListener.onInitializationCompleted(internalGenerator, baseNoiseLevel)

        verify(exactly = 1) { listener.onInitializationCompleted(generator, baseNoiseLevel) }
    }

    @Test
    fun onError_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )
    }

    @Test
    fun onError_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(context, errorListener = listener)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onError(
            internalGenerator,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) {
            listener.onError(
                generator,
                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }
    }

    @Test
    fun onStaticIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)
    }

    @Test
    fun onStaticIntervalDetected_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                staticIntervalDetectedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onStaticIntervalDetected(internalGenerator)

        verify(exactly = 1) { listener.onStaticIntervalDetected(generator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                dynamicIntervalDetectedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onDynamicIntervalDetected(internalGenerator)

        verify(exactly = 1) { listener.onDynamicIntervalDetected(generator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                staticIntervalSkippedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onStaticIntervalSkipped(internalGenerator)

        verify(exactly = 1) { listener.onStaticIntervalSkipped(generator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                dynamicIntervalSkippedListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onDynamicIntervalSkipped(internalGenerator)

        verify(exactly = 1) { listener.onDynamicIntervalSkipped(generator) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                generatedAccelerometerMeasurementListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        measurementsGeneratorListener.onGeneratedAccelerometerMeasurement(
            internalGenerator,
            measurement
        )

        verify(exactly = 1) { listener.onGeneratedAccelerometerMeasurement(generator, measurement) }
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(
                context,
                generatedGyroscopeMeasurementListener = listener
            )

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        measurementsGeneratorListener.onGeneratedGyroscopeMeasurement(
            internalGenerator,
            measurement
        )

        verify(exactly = 1) { listener.onGeneratedGyroscopeMeasurement(generator, measurement) }
    }

    @Test
    fun onReset_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onReset(internalGenerator)
    }

    @Test
    fun onReset_whenListener_notifies() {
        val listener =
            mockk<AccelerometerAndGyroscopeMeasurementGenerator.OnResetListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator =
            AccelerometerAndGyroscopeMeasurementGenerator(context, resetListener = listener)

        val measurementsGeneratorListener: AccelerometerAndGyroscopeMeasurementsGeneratorListener? =
            generator.getPrivateProperty("measurementsGeneratorListener")
        requireNotNull(measurementsGeneratorListener)

        val internalGenerator = mockk<AccelerometerAndGyroscopeMeasurementsGenerator>()
        measurementsGeneratorListener.onReset(internalGenerator)

        verify(exactly = 1) { listener.onReset(generator) }
    }

    @Test
    fun onGyroscopeMeasurementListener_whenNoListener_setsAngularRates() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(context)

        assertNull(generator.gyroscopeMeasurementListener)

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
        assertEquals(wx.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wy.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(wz.toDouble(), kinematics.angularRateZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurementListener_whenListener_setsAngularRates() {
        val gyroscopeMeasurementListener =
            mockk<GyroscopeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val generator = AccelerometerAndGyroscopeMeasurementGenerator(
            context,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )

        assertSame(gyroscopeMeasurementListener, generator.gyroscopeMeasurementListener)

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
        assertEquals(wx.toDouble(), kinematics.angularRateX, 0.0)
        assertEquals(wy.toDouble(), kinematics.angularRateY, 0.0)
        assertEquals(wz.toDouble(), kinematics.angularRateZ, 0.0)

        verify(exactly = 1) {
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
        }
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