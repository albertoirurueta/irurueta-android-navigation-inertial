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

package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.intervals.AngularSpeedTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.AngularSpeedTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter
import com.irurueta.units.TimeUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.slot
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class GyroscopeIntervalDetectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var initializationStartedListener:
            IntervalDetector.OnInitializationStartedListener<GyroscopeIntervalDetector>

    @MockK(relaxUnitFun = true)
    private lateinit var initializationCompletedListener:
            IntervalDetector.OnInitializationCompletedListener<GyroscopeIntervalDetector>

    @MockK(relaxUnitFun = true)
    private lateinit var errorListener:
            IntervalDetector.OnErrorListener<GyroscopeIntervalDetector>

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalDetectedListener:
            IntervalDetector.OnStaticIntervalDetectedListener<GyroscopeIntervalDetector>

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalDetectedListener:
            IntervalDetector.OnDynamicIntervalDetectedListener<GyroscopeIntervalDetector>

    @MockK(relaxUnitFun = true)
    private lateinit var resetListener:
            IntervalDetector.OnResetListener<GyroscopeIntervalDetector>

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var internalDetector: AngularSpeedTriadStaticIntervalDetector

    @MockK
    private lateinit var timeIntervalEstimator: TimeIntervalEstimator

    @MockK
    private lateinit var collector: GyroscopeSensorCollector

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val detector = GyroscopeIntervalDetector(context)

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            detector.sensorType
        )
        assertEquals(SensorDelay.FASTEST, detector.sensorDelay)
        assertNull(detector.initializationStartedListener)
        assertNull(detector.initializationCompletedListener)
        assertNull(detector.errorListener)
        assertNull(detector.staticIntervalDetectedListener)
        assertNull(detector.dynamicIntervalDetectedListener)
        assertNull(detector.resetListener)
        assertSame(sensor, detector.sensor)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            detector.windowSize
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold1,
            baseNoiseLevelAbsoluteThreshold2
        )
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMeasurement)
        val baseNoiseLevel = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(detector.getBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMeasurement)
        val threshold = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMeasurement
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgX1.unit
        )
        val accumulatedAvgX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgXAsMeasurement(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMeasurement
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgY1.unit
        )
        val accumulatedAvgY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgYAsMeasurement(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMeasurement
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgZ1.unit
        )
        val accumulatedAvgZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgZAsMeasurement(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgTriad1.unit
        )
        val accumulatedAvgTriad2 = AngularSpeedTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMeasurement
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdX1.unit
        )
        val accumulatedStdX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdXAsMeasurement(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMeasurement
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdY1.unit
        )
        val accumulatedStdY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdYAsMeasurement(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMeasurement
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdZ1.unit
        )
        val accumulatedStdZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdZAsMeasurement(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdTriad1.unit
        )
        val accumulatedStdTriad2 = AngularSpeedTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMeasurement
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgX1.unit
        )
        val instantaneousAvgX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgXAsMeasurement(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMeasurement
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgY1.unit
        )
        val instantaneousAvgY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgYAsMeasurement(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMeasurement
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgZ1.unit
        )
        val instantaneousAvgZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgZAsMeasurement(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgTriad1.unit
        )
        val instantaneousAvgTriad2 = AngularSpeedTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMeasurement
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdX1.unit
        )
        val instantaneousStdX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdXAsMeasurement(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMeasurement
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdY1.unit
        )
        val instantaneousStdY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdYAsMeasurement(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMeasurement
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdZ1.unit
        )
        val instantaneousStdZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdZAsMeasurement(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdTriad1.unit
        )
        val instantaneousStdTriad2 = AngularSpeedTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(Status.IDLE, detector.status)
    }

    @Test
    fun constructor_whenNonDefaultValues_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value) }
            .returns(sensor)

        val detector = GyroscopeIntervalDetector(
            context,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            resetListener
        )

        // check default values
        assertSame(context, detector.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            detector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, detector.sensorDelay)
        assertSame(
            initializationStartedListener,
            detector.initializationStartedListener
        )
        assertSame(
            initializationCompletedListener,
            detector.initializationCompletedListener
        )
        assertSame(errorListener, detector.errorListener)
        assertSame(
            staticIntervalDetectedListener,
            detector.staticIntervalDetectedListener
        )
        assertSame(
            dynamicIntervalDetectedListener,
            detector.dynamicIntervalDetectedListener
        )
        assertSame(resetListener, detector.resetListener)
        assertSame(sensor, detector.sensor)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            detector.windowSize
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            detector.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(
            baseNoiseLevelAbsoluteThreshold2
        )
        assertEquals(
            baseNoiseLevelAbsoluteThreshold1,
            baseNoiseLevelAbsoluteThreshold2
        )
        assertNull(detector.baseNoiseLevel)
        assertNull(detector.baseNoiseLevelAsMeasurement)
        val baseNoiseLevel = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(detector.getBaseNoiseLevelAsMeasurement(baseNoiseLevel))
        assertNull(detector.baseNoiseLevelPsd)
        assertNull(detector.baseNoiseLevelRootPsd)
        assertNull(detector.threshold)
        assertNull(detector.thresholdAsMeasurement)
        val threshold = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(detector.getThresholdAsMeasurement(threshold))
        assertEquals(0.0, detector.accumulatedAvgX, 0.0)
        val accumulatedAvgX1 = detector.accumulatedAvgXAsMeasurement
        assertEquals(0.0, accumulatedAvgX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgX1.unit
        )
        val accumulatedAvgX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgXAsMeasurement(accumulatedAvgX2)
        assertEquals(accumulatedAvgX1, accumulatedAvgX2)
        assertEquals(0.0, detector.accumulatedAvgY, 0.0)
        val accumulatedAvgY1 = detector.accumulatedAvgYAsMeasurement
        assertEquals(0.0, accumulatedAvgY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgY1.unit
        )
        val accumulatedAvgY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgYAsMeasurement(accumulatedAvgY2)
        assertEquals(accumulatedAvgY1, accumulatedAvgY2)
        assertEquals(0.0, detector.accumulatedAvgZ, 0.0)
        val accumulatedAvgZ1 = detector.accumulatedAvgZAsMeasurement
        assertEquals(0.0, accumulatedAvgZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgZ1.unit
        )
        val accumulatedAvgZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedAvgZAsMeasurement(accumulatedAvgZ2)
        assertEquals(accumulatedAvgZ1, accumulatedAvgZ2)
        val accumulatedAvgTriad1 = detector.accumulatedAvgTriad
        assertEquals(0.0, accumulatedAvgTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedAvgTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedAvgTriad1.unit
        )
        val accumulatedAvgTriad2 = AngularSpeedTriad()
        detector.getAccumulatedAvgTriad(accumulatedAvgTriad2)
        assertEquals(accumulatedAvgTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.accumulatedStdX, 0.0)
        val accumulatedStdX1 = detector.accumulatedStdXAsMeasurement
        assertEquals(0.0, accumulatedStdX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdX1.unit
        )
        val accumulatedStdX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdXAsMeasurement(accumulatedStdX2)
        assertEquals(accumulatedStdX1, accumulatedStdX2)
        assertEquals(0.0, detector.accumulatedStdY, 0.0)
        val accumulatedStdY1 = detector.accumulatedStdYAsMeasurement
        assertEquals(0.0, accumulatedStdY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdY1.unit
        )
        val accumulatedStdY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdYAsMeasurement(accumulatedStdY2)
        assertEquals(accumulatedStdY1, accumulatedStdY2)
        assertEquals(0.0, detector.accumulatedStdZ, 0.0)
        val accumulatedStdZ1 = detector.accumulatedStdZAsMeasurement
        assertEquals(0.0, accumulatedStdZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdZ1.unit
        )
        val accumulatedStdZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getAccumulatedStdZAsMeasurement(accumulatedStdZ2)
        assertEquals(accumulatedStdZ1, accumulatedStdZ2)
        val accumulatedStdTriad1 = detector.accumulatedStdTriad
        assertEquals(0.0, accumulatedStdTriad1.valueX, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueY, 0.0)
        assertEquals(0.0, accumulatedStdTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            accumulatedStdTriad1.unit
        )
        val accumulatedStdTriad2 = AngularSpeedTriad()
        detector.getAccumulatedStdTriad(accumulatedStdTriad2)
        assertEquals(accumulatedStdTriad1, accumulatedAvgTriad2)
        assertEquals(0.0, detector.instantaneousAvgX, 0.0)
        val instantaneousAvgX1 = detector.instantaneousAvgXAsMeasurement
        assertEquals(0.0, instantaneousAvgX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgX1.unit
        )
        val instantaneousAvgX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgXAsMeasurement(instantaneousAvgX2)
        assertEquals(instantaneousAvgX1, instantaneousAvgX2)
        assertEquals(0.0, detector.instantaneousAvgY, 0.0)
        val instantaneousAvgY1 = detector.instantaneousAvgYAsMeasurement
        assertEquals(0.0, instantaneousAvgY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgY1.unit
        )
        val instantaneousAvgY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgYAsMeasurement(instantaneousAvgY2)
        assertEquals(instantaneousAvgY1, instantaneousAvgY2)
        assertEquals(0.0, detector.instantaneousAvgZ, 0.0)
        val instantaneousAvgZ1 = detector.instantaneousAvgZAsMeasurement
        assertEquals(0.0, instantaneousAvgZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgZ1.unit
        )
        val instantaneousAvgZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousAvgZAsMeasurement(instantaneousAvgZ2)
        assertEquals(instantaneousAvgZ1, instantaneousAvgZ2)
        val instantaneousAvgTriad1 = detector.instantaneousAvgTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousAvgTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousAvgTriad1.unit
        )
        val instantaneousAvgTriad2 = AngularSpeedTriad()
        detector.getInstantaneousAvgTriad(instantaneousAvgTriad2)
        assertEquals(instantaneousAvgTriad1, instantaneousAvgTriad2)
        assertEquals(0.0, detector.instantaneousStdX, 0.0)
        val instantaneousStdX1 = detector.instantaneousStdXAsMeasurement
        assertEquals(0.0, instantaneousStdX1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdX1.unit
        )
        val instantaneousStdX2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdXAsMeasurement(instantaneousStdX2)
        assertEquals(instantaneousStdX1, instantaneousStdX2)
        assertEquals(0.0, detector.instantaneousStdY, 0.0)
        val instantaneousStdY1 = detector.instantaneousStdYAsMeasurement
        assertEquals(0.0, instantaneousStdY1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdY1.unit
        )
        val instantaneousStdY2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdYAsMeasurement(instantaneousStdY2)
        assertEquals(instantaneousStdY1, instantaneousStdY2)
        assertEquals(0.0, detector.instantaneousStdZ, 0.0)
        val instantaneousStdZ1 = detector.instantaneousStdZAsMeasurement
        assertEquals(0.0, instantaneousStdZ1.value.toDouble(), 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdZ1.unit
        )
        val instantaneousStdZ2 = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.getInstantaneousStdZAsMeasurement(instantaneousStdZ2)
        assertEquals(instantaneousStdZ1, instantaneousStdZ2)
        val instantaneousStdTriad1 = detector.instantaneousStdTriad
        assertEquals(0.0, instantaneousAvgTriad1.valueX, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueY, 0.0)
        assertEquals(0.0, instantaneousStdTriad1.valueZ, 0.0)
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            instantaneousStdTriad1.unit
        )
        val instantaneousStdTriad2 = AngularSpeedTriad()
        detector.getInstantaneousStdTriad(instantaneousStdTriad2)
        assertEquals(instantaneousStdTriad1, instantaneousStdTriad2)
        assertNull(detector.averageTimeInterval)
        assertNull(detector.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))
        assertNull(detector.timeIntervalVariance)
        assertNull(detector.timeIntervalStandardDeviation)
        assertNull(detector.timeIntervalStandardDeviationAsTime)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0, detector.numberOfProcessedMeasurements)
        assertFalse(detector.running)
        assertEquals(Status.IDLE, detector.status)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.initializationStartedListener)

        // set new value
        detector.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, detector.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.initializationCompletedListener)

        // set new value
        detector.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, detector.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.errorListener)

        // set new value
        detector.errorListener = errorListener

        // check
        assertSame(errorListener, detector.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.staticIntervalDetectedListener)

        // set new value
        detector.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, detector.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.dynamicIntervalDetectedListener)

        // set new value
        detector.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, detector.dynamicIntervalDetectedListener)
    }

    @Test
    fun resetListener_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertNull(detector.resetListener)

        // set new value
        detector.resetListener = resetListener

        // check
        assertSame(resetListener, detector.resetListener)
    }

    @Test
    fun sensor_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val detector = GyroscopeIntervalDetector(context)
        assertSame(sensor, detector.sensor)
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
            detector.windowSize
        )

        // set new value
        detector.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, detector.windowSize)
    }

    @Test
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.windowSize = 0
        }
    }

    @Test
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.windowSize = WINDOW_SIZE
        }
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            detector.initialStaticSamples
        )

        // set new value
        detector.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, detector.initialStaticSamples)
    }

    @Test
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.initialStaticSamples = 0
        }
    }

    @Test
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.initialStaticSamples =
                TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES
        }
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            detector.thresholdFactor,
            0.0
        )

        // set new value
        detector.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, detector.thresholdFactor, 0.0)
    }

    @Test
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.thresholdFactor = 0.0
        }
    }

    @Test
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.thresholdFactor = TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        detector.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(
            INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            detector.instantaneousNoiseLevelFactor,
            0.0
        )
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.instantaneousNoiseLevelFactor = 0.0
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.instantaneousNoiseLevelFactor =
                TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        detector.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            detector.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.baseNoiseLevelAbsoluteThreshold = 0.0
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.baseNoiseLevelAbsoluteThreshold =
                TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        // check default value
        val baseNoiseLevelAbsoluteThreshold1 =
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold1.value.toDouble(),
            0.0
        )
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            baseNoiseLevelAbsoluteThreshold1.unit
        )
        val baseNoiseLevelAbsoluteThreshold2 =
            AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold2)
        assertEquals(baseNoiseLevelAbsoluteThreshold1, baseNoiseLevelAbsoluteThreshold2)

        // set new value
        val baseNoiseLevelAbsoluteThreshold3 = AngularSpeed(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        detector.baseNoiseLevelAbsoluteThresholdAsMeasurement = baseNoiseLevelAbsoluteThreshold3

        // check
        val baseNoiseLevelAbsoluteThreshold4 =
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            baseNoiseLevelAbsoluteThreshold4.value.toDouble(),
            0.0
        )
        assertEquals(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            baseNoiseLevelAbsoluteThreshold4.unit
        )
        val baseNoiseLevelAbsoluteThreshold5 =
            AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevelAbsoluteThreshold5)
        assertEquals(baseNoiseLevelAbsoluteThreshold4, baseNoiseLevelAbsoluteThreshold5)
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val detector = GyroscopeIntervalDetector(context)

        assertThrows(IllegalArgumentException::class.java) {
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement =
                AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            detector.baseNoiseLevelAbsoluteThresholdAsMeasurement = AngularSpeed(
                BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
        }
    }

    @Test
    fun baseNoiseLevel_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { internalDetector.baseNoiseLevel }.returns(baseNoiseLevel)

        assertEquals(baseNoiseLevel, detector.baseNoiseLevel)

        verify(exactly = 1) { internalDetector.baseNoiseLevel }
    }

    @Test
    fun baseNoiseLevel_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.baseNoiseLevel)
    }

    @Test
    fun baseNoiseLevelAsMeasurement_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.baseNoiseLevelAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.baseNoiseLevelAsMeasurement)

        verify(exactly = 1) { internalDetector.baseNoiseLevelAsMeasurement }
    }

    @Test
    fun baseNoiseLevelAsMeasurement_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.baseNoiseLevelAsMeasurement)
    }

    @Test
    fun getBaseNoiseLevelAsMeasurement_whenInitialized_returnsTrue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getBaseNoiseLevelAsMeasurement(angularSpeed) }

        assertTrue(detector.getBaseNoiseLevelAsMeasurement(angularSpeed))

        verify(exactly = 1) { internalDetector.getBaseNoiseLevelAsMeasurement(angularSpeed) }
    }

    @Test
    fun getBaseNoiseLevelAsMeasurement_whenNotInitialized_returnsFalse() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(detector.getBaseNoiseLevelAsMeasurement(angularSpeed))

        verify { internalDetector wasNot Called }
    }

    @Test
    fun baseNoiseLevelPsd_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd = randomizer.nextDouble()
        every { internalDetector.baseNoiseLevelPsd }.returns(baseNoiseLevelPsd)

        assertEquals(baseNoiseLevelPsd, detector.baseNoiseLevelPsd)

        verify(exactly = 1) { internalDetector.baseNoiseLevelPsd }
    }

    @Test
    fun baseNoiseLevelPsd_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.baseNoiseLevelPsd)
    }

    @Test
    fun baseNoiseLevelRootPsd_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd = randomizer.nextDouble()
        every { internalDetector.baseNoiseLevelRootPsd }.returns(baseNoiseLevelRootPsd)

        assertEquals(baseNoiseLevelRootPsd, detector.baseNoiseLevelRootPsd)

        verify(exactly = 1) { internalDetector.baseNoiseLevelRootPsd }
    }

    @Test
    fun baseNoiseLevelRootPsd_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.baseNoiseLevelRootPsd)
    }

    @Test
    fun threshold_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { internalDetector.threshold }.returns(threshold)

        assertEquals(threshold, detector.threshold)

        verify(exactly = 1) { internalDetector.threshold }
    }

    @Test
    fun threshold_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.threshold)
    }

    @Test
    fun thresholdAsMeasurement_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.thresholdAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.thresholdAsMeasurement)

        verify(exactly = 1) { internalDetector.thresholdAsMeasurement }
    }

    @Test
    fun thresholdAsMeasurement_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.thresholdAsMeasurement)
    }

    @Test
    fun getThresholdAsMeasurement_whenInitialized_returnsTrue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getThresholdAsMeasurement(angularSpeed) }

        assertTrue(detector.getThresholdAsMeasurement(angularSpeed))

        verify(exactly = 1) { internalDetector.getThresholdAsMeasurement(angularSpeed) }
    }

    @Test
    fun getThresholdAsMeasurement_whenNotInitialized_returnsFalse() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        assertFalse(detector.getThresholdAsMeasurement(angularSpeed))

        verify { internalDetector wasNot Called }
    }

    @Test
    fun accumulatedAvgX_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedAvgX = randomizer.nextDouble()
        every { internalDetector.accumulatedAvgX }.returns(accumulatedAvgX)

        assertEquals(accumulatedAvgX, detector.accumulatedAvgX, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedAvgX }
    }

    @Test
    fun accumulatedAvgXAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedAvgXAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedAvgXAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedAvgXAsMeasurement }
    }

    @Test
    fun getAccumulatedAvgXAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedAvgXAsMeasurement(angularSpeed) }

        detector.getAccumulatedAvgXAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedAvgXAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedAvgY_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedAvgY = randomizer.nextDouble()
        every { internalDetector.accumulatedAvgY }.returns(accumulatedAvgY)

        assertEquals(accumulatedAvgY, detector.accumulatedAvgY, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedAvgY }
    }

    @Test
    fun accumulatedAvgYAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedAvgYAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedAvgYAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedAvgYAsMeasurement }
    }

    @Test
    fun getAccumulatedAvgYAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedAvgYAsMeasurement(angularSpeed) }

        detector.getAccumulatedAvgYAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedAvgYAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedAvgZ_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedAvgZ = randomizer.nextDouble()
        every { internalDetector.accumulatedAvgZ }.returns(accumulatedAvgZ)

        assertEquals(accumulatedAvgZ, detector.accumulatedAvgZ, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedAvgZ }
    }

    @Test
    fun accumulatedAvgZAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedAvgZAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedAvgZAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedAvgZAsMeasurement }
    }

    @Test
    fun getAccumulatedAvgZAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedAvgZAsMeasurement(angularSpeed) }

        detector.getAccumulatedAvgZAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedAvgZAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedAvgTriad_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        every { internalDetector.accumulatedAvgTriad }.returns(triad)

        assertSame(triad, detector.accumulatedAvgTriad)

        verify(exactly = 1) { internalDetector.accumulatedAvgTriad }
    }

    @Test
    fun getAccumulatedAvgTriad_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        justRun { internalDetector.getAccumulatedAvgTriad(triad) }

        detector.getAccumulatedAvgTriad(triad)

        verify(exactly = 1) { internalDetector.getAccumulatedAvgTriad(triad) }
    }

    @Test
    fun accumulatedStdX_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedStdX = randomizer.nextDouble()
        every { internalDetector.accumulatedStdX }.returns(accumulatedStdX)

        assertEquals(accumulatedStdX, detector.accumulatedStdX, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedStdX }
    }

    @Test
    fun accumulatedStdXAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedStdXAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedStdXAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedStdXAsMeasurement }
    }

    @Test
    fun getAccumulatedStdXAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedStdXAsMeasurement(angularSpeed) }

        detector.getAccumulatedStdXAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedStdXAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedStdY_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedStdY = randomizer.nextDouble()
        every { internalDetector.accumulatedStdY }.returns(accumulatedStdY)

        assertEquals(accumulatedStdY, detector.accumulatedStdY, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedStdY }
    }

    @Test
    fun accumulatedStdYAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedStdYAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedStdYAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedStdYAsMeasurement }
    }

    @Test
    fun getAccumulatedStdYAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedStdYAsMeasurement(angularSpeed) }

        detector.getAccumulatedStdYAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedStdYAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedStdZ_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedStdZ = randomizer.nextDouble()
        every { internalDetector.accumulatedStdZ }.returns(accumulatedStdZ)

        assertEquals(accumulatedStdZ, detector.accumulatedStdZ, 0.0)

        verify(exactly = 1) { internalDetector.accumulatedStdZ }
    }

    @Test
    fun accumulatedStdZAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.accumulatedStdZAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.accumulatedStdZAsMeasurement)

        verify(exactly = 1) { internalDetector.accumulatedStdZAsMeasurement }
    }

    @Test
    fun getAccumulatedStdZAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getAccumulatedStdZAsMeasurement(angularSpeed) }

        detector.getAccumulatedStdZAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getAccumulatedStdZAsMeasurement(angularSpeed) }
    }

    @Test
    fun accumulatedStdTriad_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        every { internalDetector.accumulatedStdTriad }.returns(triad)

        assertSame(triad, detector.accumulatedStdTriad)

        verify(exactly = 1) { internalDetector.accumulatedStdTriad }
    }

    @Test
    fun getAccumulatedStdTriad_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        justRun { internalDetector.getAccumulatedStdTriad(triad) }

        detector.getAccumulatedStdTriad(triad)

        verify(exactly = 1) { internalDetector.getAccumulatedStdTriad(triad) }
    }

    @Test
    fun instantaneousAvgX_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        every { internalDetector.instantaneousAvgX }.returns(instantaneousAvgX)

        assertEquals(instantaneousAvgX, detector.instantaneousAvgX, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousAvgX }
    }

    @Test
    fun instantaneousAvgXAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousAvgXAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousAvgXAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousAvgXAsMeasurement }
    }

    @Test
    fun getInstantaneousAvgXAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousAvgXAsMeasurement(angularSpeed) }

        detector.getInstantaneousAvgXAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousAvgXAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousAvgY_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgY = randomizer.nextDouble()
        every { internalDetector.instantaneousAvgY }.returns(instantaneousAvgY)

        assertEquals(instantaneousAvgY, detector.instantaneousAvgY, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousAvgY }
    }

    @Test
    fun instantaneousAvgYAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousAvgYAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousAvgYAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousAvgYAsMeasurement }
    }

    @Test
    fun getInstantaneousAvgYAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousAvgYAsMeasurement(angularSpeed) }

        detector.getInstantaneousAvgYAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousAvgYAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousAvgZ_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgZ = randomizer.nextDouble()
        every { internalDetector.instantaneousAvgZ }.returns(instantaneousAvgZ)

        assertEquals(instantaneousAvgZ, detector.instantaneousAvgZ, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousAvgZ }
    }

    @Test
    fun instantaneousAvgZAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousAvgZAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousAvgZAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousAvgZAsMeasurement }
    }

    @Test
    fun getInstantaneousAvgZAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousAvgZAsMeasurement(angularSpeed) }

        detector.getInstantaneousAvgZAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousAvgZAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousAvgTriad_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        every { internalDetector.instantaneousAvgTriad }.returns(triad)

        assertSame(triad, detector.instantaneousAvgTriad)

        verify(exactly = 1) { internalDetector.instantaneousAvgTriad }
    }

    @Test
    fun getInstantaneousAvgTriad_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        justRun { internalDetector.getInstantaneousAvgTriad(triad) }

        detector.getInstantaneousAvgTriad(triad)

        verify(exactly = 1) { internalDetector.getInstantaneousAvgTriad(triad) }
    }

    @Test
    fun instantaneousStdX_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousStdX = randomizer.nextDouble()
        every { internalDetector.instantaneousStdX }.returns(instantaneousStdX)

        assertEquals(instantaneousStdX, detector.instantaneousStdX, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousStdX }
    }

    @Test
    fun instantaneousStdXAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousStdXAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousStdXAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousStdXAsMeasurement }
    }

    @Test
    fun getInstantaneousStdXAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousStdXAsMeasurement(angularSpeed) }

        detector.getInstantaneousStdXAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousStdXAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousStdY_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousStdY = randomizer.nextDouble()
        every { internalDetector.instantaneousStdY }.returns(instantaneousStdY)

        assertEquals(instantaneousStdY, detector.instantaneousStdY, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousStdY }
    }

    @Test
    fun instantaneousStdYAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousStdYAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousStdYAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousStdYAsMeasurement }
    }

    @Test
    fun getInstantaneousStdYAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousStdYAsMeasurement(angularSpeed) }

        detector.getInstantaneousStdYAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousStdYAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousStdZ_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousStdZ = randomizer.nextDouble()
        every { internalDetector.instantaneousStdZ }.returns(instantaneousStdZ)

        assertEquals(instantaneousStdZ, detector.instantaneousStdZ, 0.0)

        verify(exactly = 1) { internalDetector.instantaneousStdZ }
    }

    @Test
    fun instantaneousStdZAsMeasurement_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { internalDetector.instantaneousStdZAsMeasurement }.returns(angularSpeed)

        assertSame(angularSpeed, detector.instantaneousStdZAsMeasurement)

        verify(exactly = 1) { internalDetector.instantaneousStdZAsMeasurement }
    }

    @Test
    fun getInstantaneousStdZAsMeasurement_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        justRun { internalDetector.getInstantaneousStdZAsMeasurement(angularSpeed) }

        detector.getInstantaneousStdZAsMeasurement(angularSpeed)

        verify(exactly = 1) { internalDetector.getInstantaneousStdZAsMeasurement(angularSpeed) }
    }

    @Test
    fun instantaneousStdTriad_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        every { internalDetector.instantaneousStdTriad }.returns(triad)

        assertSame(triad, detector.instantaneousStdTriad)

        verify(exactly = 1) { internalDetector.instantaneousStdTriad }
    }

    @Test
    fun getInstantaneousStdTriad_callsInternalDetector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("internalDetector", internalDetector)

        val triad = AngularSpeedTriad()
        justRun { internalDetector.getInstantaneousStdTriad(triad) }

        detector.getInstantaneousStdTriad(triad)

        verify(exactly = 1) { internalDetector.getInstantaneousStdTriad(triad) }
    }

    @Test
    fun averageTimeInterval_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { timeIntervalEstimator.averageTimeInterval }.returns(averageTimeInterval)

        assertEquals(averageTimeInterval, detector.averageTimeInterval)

        verify(exactly = 1) { timeIntervalEstimator.averageTimeInterval }
    }

    @Test
    fun averageTimeInterval_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.averageTimeInterval)
    }

    @Test
    fun averageTimeIntervalAsTime_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)
        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        every { timeIntervalEstimator.averageTimeIntervalAsTime }.returns(time)

        assertSame(time, detector.averageTimeIntervalAsTime)

        verify(exactly = 1) { timeIntervalEstimator.averageTimeIntervalAsTime }
    }

    @Test
    fun averageTimeIntervalAsTime_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        assertNull(detector.averageTimeIntervalAsTime)
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenInitialized_returnsTrue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        justRun { timeIntervalEstimator.getAverageTimeIntervalAsTime(time) }

        assertTrue(detector.getAverageTimeIntervalAsTime(time))

        verify(exactly = 1) { timeIntervalEstimator.getAverageTimeIntervalAsTime(time) }
    }

    @Test
    fun getAverageTimeIntervalAsTime_whenNotInitialized_returnsFalse() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getAverageTimeIntervalAsTime(time))

        verify { timeIntervalEstimator wasNot Called }
    }

    @Test
    fun timeIntervalVariance_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        every { timeIntervalEstimator.timeIntervalVariance }
            .returns(timeIntervalVariance)

        assertEquals(timeIntervalVariance, detector.timeIntervalVariance)

        verify(exactly = 1) { timeIntervalEstimator.timeIntervalVariance }
    }

    @Test
    fun timeIntervalVariance_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        assertNull(detector.timeIntervalVariance)

        verify { timeIntervalEstimator wasNot Called }
    }

    @Test
    fun timeIntervalStandardDeviation_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        every { timeIntervalEstimator.timeIntervalStandardDeviation }
            .returns(timeIntervalStandardDeviation)

        assertEquals(
            timeIntervalStandardDeviation,
            detector.timeIntervalStandardDeviation
        )

        verify(exactly = 1) { timeIntervalEstimator.timeIntervalStandardDeviation }
    }

    @Test
    fun timeIntervalStandardDeviation_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        assertNull(detector.timeIntervalStandardDeviation)

        verify { timeIntervalEstimator wasNot Called }
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenInitialized_returnsExpectedValue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        every { timeIntervalEstimator.timeIntervalStandardDeviationAsTime }
            .returns(time)

        assertSame(time, detector.timeIntervalStandardDeviationAsTime)

        verify(exactly = 1) { timeIntervalEstimator.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_whenNotInitialized_returnsNull() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        assertNull(detector.timeIntervalStandardDeviationAsTime)

        verify { timeIntervalEstimator wasNot Called }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenInitialized_returnsTrue() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(
            IntervalDetector::class, detector, "initialized",
            true
        )
        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        justRun { timeIntervalEstimator.getTimeIntervalStandardDeviationAsTime(time) }

        assertTrue(detector.getTimeIntervalStandardDeviationAsTime(time))

        verify(exactly = 1) { timeIntervalEstimator.getTimeIntervalStandardDeviationAsTime(time) }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_whenNotInitialized_returnsFalse() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)

        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(detector.getTimeIntervalStandardDeviationAsTime(time))

        verify { timeIntervalEstimator wasNot Called }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val detector = GyroscopeIntervalDetector(context)

        setPrivateProperty(IntervalDetector::class, detector, "running", true)

        assertThrows(IllegalStateException::class.java) {
            detector.start()
        }
    }

    @Test
    fun start_whenNotRunningAndCollectorFails_resetsAndThrowsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val detector = GyroscopeIntervalDetector(context)

            detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)
            justRun { timeIntervalEstimator.totalSamples = Integer.MAX_VALUE }
            every { timeIntervalEstimator.reset() }.returns(true)
            detector.setPrivateProperty("internalDetector", internalDetector)
            justRun { internalDetector.reset() }
            setPrivateProperty(
                IntervalDetector::class, detector,
                "unreliable", true
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "initialTimestamp", 1L
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "numberOfProcessedMeasurements", 1
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "initialized", true
            )

            detector.setPrivateProperty("collector", collector)
            every { collector.start(startTimestamp) }.returns(false)

            assertThrows(IllegalStateException::class.java) {
                detector.start()
            }

            verify(exactly = 1) { timeIntervalEstimator.totalSamples = Integer.MAX_VALUE }
            verify(exactly = 1) { timeIntervalEstimator.reset() }
            verify(exactly = 1) { internalDetector.reset() }
            val unreliable: Boolean? = getPrivateProperty(
                IntervalDetector::class, detector, "unreliable"
            )
            requireNotNull(unreliable)
            assertFalse(unreliable)
            val initialTimestamp: Long? = getPrivateProperty(
                IntervalDetector::class, detector, "initialTimestamp"
            )
            requireNotNull(initialTimestamp)
            assertEquals(0L, initialTimestamp)
            assertEquals(0, detector.numberOfProcessedMeasurements)
            val initialized: Boolean? = getPrivateProperty(
                IntervalDetector::class,
                detector, "initialized"
            )
            requireNotNull(initialized)
            assertFalse(initialized)
            verify(exactly = 1) { collector.start(startTimestamp) }
        }
    }

    @Test
    fun start_whenNotRunningAndCollectorSucceeds_completesSuccessfully() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val detector = GyroscopeIntervalDetector(context)

            detector.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimator)
            justRun { timeIntervalEstimator.totalSamples = Integer.MAX_VALUE }
            every { timeIntervalEstimator.reset() }.returns(true)
            detector.setPrivateProperty("internalDetector", internalDetector)
            justRun { internalDetector.reset() }
            setPrivateProperty(
                IntervalDetector::class, detector,
                "unreliable", true
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "initialTimestamp", 1L
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "numberOfProcessedMeasurements", 1
            )
            setPrivateProperty(
                IntervalDetector::class, detector,
                "initialized", true
            )

            detector.setPrivateProperty("collector", collector)
            every { collector.start(startTimestamp) }.returns(true)

            detector.start()

            verify(exactly = 1) { timeIntervalEstimator.totalSamples = Integer.MAX_VALUE }
            verify(exactly = 1) { timeIntervalEstimator.reset() }
            verify(exactly = 1) { internalDetector.reset() }
            val unreliable: Boolean? = getPrivateProperty(
                IntervalDetector::class, detector, "unreliable"
            )
            requireNotNull(unreliable)
            assertFalse(unreliable)
            val initialTimestamp: Long? = getPrivateProperty(
                IntervalDetector::class, detector, "initialTimestamp"
            )
            requireNotNull(initialTimestamp)
            assertEquals(0L, initialTimestamp)
            assertEquals(0, detector.numberOfProcessedMeasurements)
            val initialized: Boolean? = getPrivateProperty(
                IntervalDetector::class,
                detector, "initialized"
            )
            requireNotNull(initialized)
            assertFalse(initialized)
            verify(exactly = 1) { collector.start(startTimestamp) }
        }
    }

    @Test
    fun stop_stopsCollector() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }
        setPrivateProperty(
            IntervalDetector::class, detector, "running",
            true
        )

        detector.stop()

        verify(exactly = 1) { collector.stop() }
        assertFalse(detector.running)
    }

    @Test
    fun internalDetectorListener_whenOnInitializationStartedAndNoListener_makesNoAction() {
        val detector = GyroscopeIntervalDetector(context)

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onInitializationStarted(internalDetector)
    }

    @Test
    fun internalDetectorListener_whenOnInitializationStartedAndListener_callsListener() {
        val detector = GyroscopeIntervalDetector(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onInitializationStarted(internalDetector)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(detector) }
    }

    @Test
    fun internalDetectorListener_whenOnInitializationCompletedAndNoListener_makesNoAction() {
        val detector = GyroscopeIntervalDetector(context)

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onInitializationCompleted(internalDetector, baseNoiseLevel)
    }

    @Test
    fun internalDetectorListener_whenOnInitializationCompletedAndListener_callsListener() {
        val detector = GyroscopeIntervalDetector(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onInitializationCompleted(internalDetector, baseNoiseLevel)

        verify(exactly = 1) {
            initializationCompletedListener.onInitializationCompleted(
                detector,
                baseNoiseLevel
            )
        }
    }

    @Test
    fun internalDetectorListener_whenOnErrorAndNoListener_stops() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedNoiseLevel = randomizer.nextDouble()
        val instantaneousNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onError(
            internalDetector,
            accumulatedNoiseLevel,
            instantaneousNoiseLevel,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) { collector.stop() }
    }

    @Test
    fun internalDetectorListener_whenOnErrorAndListener_stopsAndNotifies() {
        val detector = GyroscopeIntervalDetector(
            context,
            errorListener = errorListener
        )

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val accumulatedNoiseLevel = randomizer.nextDouble()
        val instantaneousNoiseLevel = randomizer.nextDouble()
        internalDetectorListener.onError(
            internalDetector,
            accumulatedNoiseLevel,
            instantaneousNoiseLevel,
            TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED
        )

        verify(exactly = 1) { collector.stop() }
        verify(exactly = 1) {
            errorListener.onError(
                detector,
                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            )
        }
    }

    @Test
    fun internalDetectorListener_whenOnStaticIntervalDetectedAndNoListener_makesNoAction() {
        val detector = GyroscopeIntervalDetector(context)

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        internalDetectorListener.onStaticIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ
        )
    }

    @Test
    fun internalDetectorListener_whenOnStaticIntervalDetectedAndListener_callsListener() {
        val detector = GyroscopeIntervalDetector(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        internalDetectorListener.onStaticIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ
        )

        verify(exactly = 1) {
            staticIntervalDetectedListener.onStaticIntervalDetected(
                detector,
                instantaneousAvgX,
                instantaneousAvgY,
                instantaneousAvgZ,
                instantaneousStdX,
                instantaneousStdY,
                instantaneousStdZ
            )
        }
    }

    @Test
    fun internalDetectorListener_whenOnDynamicIntervalDetectedAndNoListener_makesNoAction() {
        val detector = GyroscopeIntervalDetector(context)

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        val accumulatedAvgX = randomizer.nextDouble()
        val accumulatedAvgY = randomizer.nextDouble()
        val accumulatedAvgZ = randomizer.nextDouble()
        val accumulatedStdX = randomizer.nextDouble()
        val accumulatedStdY = randomizer.nextDouble()
        val accumulatedStdZ = randomizer.nextDouble()
        internalDetectorListener.onDynamicIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ,
            accumulatedAvgX,
            accumulatedAvgY,
            accumulatedAvgZ,
            accumulatedStdX,
            accumulatedStdY,
            accumulatedStdZ
        )
    }

    @Test
    fun internalDetectorListener_whenOnDynamicIntervalDetectedAndListener_callsListener() {
        val detector = GyroscopeIntervalDetector(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        val randomizer = UniformRandomizer()
        val instantaneousAvgX = randomizer.nextDouble()
        val instantaneousAvgY = randomizer.nextDouble()
        val instantaneousAvgZ = randomizer.nextDouble()
        val instantaneousStdX = randomizer.nextDouble()
        val instantaneousStdY = randomizer.nextDouble()
        val instantaneousStdZ = randomizer.nextDouble()
        val accumulatedAvgX = randomizer.nextDouble()
        val accumulatedAvgY = randomizer.nextDouble()
        val accumulatedAvgZ = randomizer.nextDouble()
        val accumulatedStdX = randomizer.nextDouble()
        val accumulatedStdY = randomizer.nextDouble()
        val accumulatedStdZ = randomizer.nextDouble()
        internalDetectorListener.onDynamicIntervalDetected(
            internalDetector,
            instantaneousAvgX,
            instantaneousAvgY,
            instantaneousAvgZ,
            instantaneousStdX,
            instantaneousStdY,
            instantaneousStdZ,
            accumulatedAvgX,
            accumulatedAvgY,
            accumulatedAvgZ,
            accumulatedStdX,
            accumulatedStdY,
            accumulatedStdZ
        )

        verify(exactly = 1) {
            dynamicIntervalDetectedListener.onDynamicIntervalDetected(
                detector,
                instantaneousAvgX,
                instantaneousAvgY,
                instantaneousAvgZ,
                instantaneousStdX,
                instantaneousStdY,
                instantaneousStdZ,
                accumulatedAvgX,
                accumulatedAvgY,
                accumulatedAvgZ,
                accumulatedStdX,
                accumulatedStdY,
                accumulatedStdZ
            )
        }
    }

    @Test
    fun internalDetectorListener_whenOnResetAndNoListener_makesNoAction() {
        val detector = GyroscopeIntervalDetector(context)

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onReset(internalDetector)
    }

    @Test
    fun internalDetectorListener_whenOnResetAndListener_callsListener() {
        val detector = GyroscopeIntervalDetector(
            context,
            resetListener = resetListener
        )

        val internalDetectorListener: AngularSpeedTriadStaticIntervalDetectorListener? =
            detector.getPrivateProperty("internalDetectorListener")
        requireNotNull(internalDetectorListener)

        val internalDetector: AngularSpeedTriadStaticIntervalDetector? =
            detector.getPrivateProperty("internalDetector")
        requireNotNull(internalDetector)

        internalDetectorListener.onReset(internalDetector)

        verify(exactly = 1) { resetListener.onReset(detector) }
    }

    @Test
    fun measurementListener_whenInitializingAndNoProcessedMeasurements_setsInitialTimestampAndIncreasesNumberOfProcessedMeasurements() {
        val detector = GyroscopeIntervalDetector(context)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)
        val collector: GyroscopeSensorCollector? = detector.getPrivateProperty(
            "collector"
        )
        requireNotNull(collector)

        detector.setPrivateProperty("internalDetector", internalDetector)
        every { internalDetector.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        every { internalDetector.process(any()) }.returns(true)
        setPrivateProperty(
            IntervalDetector::class, detector,
            "numberOfProcessedMeasurements", 0
        )

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz, timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val initialTimestamp: Long? = getPrivateProperty(
            IntervalDetector::class, detector, "initialTimestamp"
        )
        requireNotNull(initialTimestamp)
        assertEquals(timestamp, initialTimestamp)

        val slot = slot<AngularSpeedTriad>()
        verify { internalDetector.process(capture(slot)) }
        val triad = slot.captured
        assertEquals(wy.toDouble() + by.toDouble(), triad.valueX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), triad.valueY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), triad.valueZ, 0.0)

        assertEquals(1, detector.numberOfProcessedMeasurements)
    }

    @Test
    fun measurementListener_whenInitializingAndProcessedMeasurements_addsTimestampAndIncreasesNumberOfProcessedMeasurements() {
        val detector = GyroscopeIntervalDetector(context)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)
        val collector: GyroscopeSensorCollector? = detector.getPrivateProperty(
            "collector"
        )
        requireNotNull(collector)

        detector.setPrivateProperty("internalDetector", internalDetector)
        every { internalDetector.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZING)
        every { internalDetector.process(any()) }.returns(true)
        setPrivateProperty(
            IntervalDetector::class, detector,
            "numberOfProcessedMeasurements", 1
        )

        setPrivateProperty(
            IntervalDetector::class,
            detector,
            "timeIntervalEstimator",
            timeIntervalEstimator
        )
        every { timeIntervalEstimator.addTimestamp(any<Double>()) }.returns(true)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz, timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val initialTimestamp: Long? = getPrivateProperty(
            IntervalDetector::class, detector, "initialTimestamp"
        )
        requireNotNull(initialTimestamp)
        assertEquals(0L, initialTimestamp)

        val diffSeconds = TimeConverter.nanosecondToSecond(timestamp.toDouble())
        verify(exactly = 1) { timeIntervalEstimator.addTimestamp(diffSeconds) }

        val slot = slot<AngularSpeedTriad>()
        verify { internalDetector.process(capture(slot)) }
        val triad = slot.captured
        assertEquals(wy.toDouble() + by.toDouble(), triad.valueX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), triad.valueY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), triad.valueZ, 0.0)

        assertEquals(2, detector.numberOfProcessedMeasurements)
    }

    @Test
    fun measurementListener_whenInitializationCompletedAndProcessedMeasurements_setsTimeInterval() {
        val detector = GyroscopeIntervalDetector(context)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)
        val collector: GyroscopeSensorCollector? = detector.getPrivateProperty(
            "collector"
        )
        requireNotNull(collector)

        detector.setPrivateProperty("internalDetector", internalDetector)
        every { internalDetector.status }
            .returns(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED)
        every { internalDetector.process(any()) }.returns(true)
        setPrivateProperty(
            IntervalDetector::class, detector,
            "numberOfProcessedMeasurements", 0
        )

        setPrivateProperty(
            IntervalDetector::class,
            detector,
            "timeIntervalEstimator",
            timeIntervalEstimator
        )

        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { timeIntervalEstimator.averageTimeInterval }.returns(averageTimeInterval)
        justRun { internalDetector.timeInterval = averageTimeInterval }


        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz, timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val initialTimestamp: Long? = getPrivateProperty(
            IntervalDetector::class, detector, "initialTimestamp"
        )
        requireNotNull(initialTimestamp)
        assertEquals(0L, initialTimestamp)

        val slot = slot<AngularSpeedTriad>()
        verify { internalDetector.process(capture(slot)) }
        val triad = slot.captured
        assertEquals(wy.toDouble() + by.toDouble(), triad.valueX, 0.0)
        assertEquals(wx.toDouble() + bx.toDouble(), triad.valueY, 0.0)
        assertEquals(-wz.toDouble() - bz.toDouble(), triad.valueZ, 0.0)

        assertEquals(1, detector.numberOfProcessedMeasurements)
        verify(exactly = 1) { internalDetector.timeInterval = averageTimeInterval }

        val initialized: Boolean? = getPrivateProperty(
            IntervalDetector::class,
            detector, "initialized"
        )
        requireNotNull(initialized)
        assertTrue(initialized)
    }

    @Test
    fun accuracyChangedListener_whenUnreliableAndNoListener_stopsAndMarksAsUnreliable() {
        val detector = GyroscopeIntervalDetector(context)

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.UNRELIABLE)

        verify(exactly = 1) { collector.stop() }
        val unreliable: Boolean? = getPrivateProperty(
            IntervalDetector::class, detector, "unreliable")
        requireNotNull(unreliable)
        assertTrue(unreliable)
    }

    @Test
    fun accuracyChangedListener_whenUnreliableAndListener_stopsMarksAsUnreliableAndNotifies() {
        val detector = GyroscopeIntervalDetector(
            context,
            errorListener = errorListener
        )

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.UNRELIABLE)

        verify(exactly = 1) { collector.stop() }
        val unreliable: Boolean? = getPrivateProperty(
            IntervalDetector::class, detector, "unreliable")
        requireNotNull(unreliable)
        assertTrue(unreliable)

        verify(exactly = 1) { errorListener.onError(detector, ErrorReason.UNRELIABLE_SENSOR) }
    }

    @Test
    fun accuracyChangedListener_whenNotUnreliable_makesNoAction() {
        val detector = GyroscopeIntervalDetector(
            context,
            errorListener = errorListener
        )

        detector.setPrivateProperty("collector", collector)
        justRun { collector.stop() }

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            detector.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.HIGH)

        verify { collector wasNot Called }
        val unreliable: Boolean? = getPrivateProperty(
            IntervalDetector::class, detector, "unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        verify { errorListener wasNot Called }
    }


    private companion object {
        const val WINDOW_SIZE = 201

        const val INITIAL_STATIC_SAMPLES = 10000

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 100.0
    }
}