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
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.Time
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

class GyroscopeNormEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var completedListener: AccumulatedMeasurementEstimator
    .OnEstimationCompletedListener<GyroscopeNormEstimator>

    @MockK
    private lateinit var unreliableListener: AccumulatedMeasurementEstimator
    .OnUnreliableListener<GyroscopeNormEstimator>

    @MockK
    private lateinit var processor: GyroscopeNormProcessor

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

        val estimator = GyroscopeNormEstimator(context)

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenSensorType_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE.value)
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(
            context,
            GyroscopeSensorType.GYROSCOPE
        )

        assertSame(context, estimator.context)
        assertEquals(GyroscopeSensorType.GYROSCOPE, estimator.sensorType)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(
            context,
            sensorDelay = SensorDelay.NORMAL
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenNegativeMaxSamples_throwsIllegalArgumentException() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeNormEstimator(
                context,
                maxSamples = -1
            )
        }
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(
            context,
            maxSamples = MAX_SAMPLES
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenNegativeMaxDurationMillis_throwsIllegalArgumentException() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeNormEstimator(
                context,
                maxDurationMillis = -1
            )
        }
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(
            context,
            maxDurationMillis = MAX_DURATION_MILLIS
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenStopMode_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(
            context,
            stopMode = StopMode.MAX_DURATION_ONLY
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_DURATION_ONLY, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenCompletedListener_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context, completedListener = completedListener)

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
    }

    @Test
    fun constructor_whenUnreliableListener_setsExpectedValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context, unreliableListener = unreliableListener)

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
            estimator.maxSamples
        )
        assertEquals(
            BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
        assertEquals(0L, estimator.initialTimestampNanos)
        assertEquals(0L, estimator.endTimestampNanos)
        assertEquals(0L, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0L, estimator.elapsedTimeNanos)
        val elapsedTime = Time(0.0, TimeUnit.NANOSECOND)
        assertEquals(elapsedTime, estimator.elapsedTime)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)
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

        val estimator = GyroscopeNormEstimator(context)

        assertSame(sensor, estimator.sensor)
    }

    @Test
    fun running_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        every {
            sensorManager.registerListener(
                any(), any<Sensor>(),
                any()
            )
        }.returns(true)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = GyroscopeNormEstimator(context)

        // check default value
        assertFalse(estimator.running)

        // set running to true
        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())
            estimator.start()
        }

        assertTrue(estimator.running)

        // set running to false
        estimator.stop()
        assertFalse(estimator.running)
    }

    @Test
    fun averageNorm_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageNorm = randomizer.nextDouble()
        every { processor.averageNorm }.returns(averageNorm)

        // check
        assertEquals(averageNorm, estimator.averageNorm)
        verify(exactly = 1) { processor.averageNorm }
    }

    @Test
    fun averageNormAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageNorm = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { processor.averageNormAsMeasurement }.returns(averageNorm)

        // check
        assertSame(averageNorm, estimator.averageNormAsMeasurement)
        verify(exactly = 1) { processor.averageNormAsMeasurement }
    }

    @Test
    fun getAverageNormAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageNorm = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { processor.getAverageNormAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getAverageNormAsMeasurement(capture(slot)) }
        assertSame(averageNorm, slot.captured)
    }

    @Test
    fun normVariance_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val normVariance = randomizer.nextDouble()
        every { processor.normVariance }.returns(normVariance)

        // check
        assertEquals(normVariance, estimator.normVariance)
        verify(exactly = 1) { processor.normVariance }
    }

    @Test
    fun normStandardDeviation_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val normStandardDeviation = randomizer.nextDouble()
        every { processor.normStandardDeviation }.returns(normStandardDeviation)

        // check
        assertEquals(normStandardDeviation, estimator.normStandardDeviation)
        verify(exactly = 1) { processor.normStandardDeviation }
    }

    @Test
    fun normStandardDeviationAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val normStandardDeviation = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { processor.normStandardDeviationAsMeasurement }.returns(normStandardDeviation)

        // check
        assertSame(normStandardDeviation, estimator.normStandardDeviationAsMeasurement)
        verify(exactly = 1) { processor.normStandardDeviationAsMeasurement }
    }

    @Test
    fun getNormStandardDeviationAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val normStandardDeviation = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        every { processor.getNormStandardDeviationAsMeasurement(any()) }
            .returns(true)

        // check
        assertTrue(estimator.getNormStandardDeviationAsMeasurement(normStandardDeviation))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) {
            processor.getNormStandardDeviationAsMeasurement(
                capture(slot)
            )
        }
        assertSame(normStandardDeviation, slot.captured)
    }

    @Test
    fun psd_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val psd = randomizer.nextDouble()
        every { processor.psd }.returns(psd)

        // check
        assertEquals(psd, estimator.psd)
        verify(exactly = 1) { processor.psd }
    }

    @Test
    fun rootPsd_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val rootPsd = randomizer.nextDouble()
        every { processor.rootPsd }.returns(rootPsd)

        // check
        assertEquals(rootPsd, estimator.rootPsd)
        verify(exactly = 1) { processor.rootPsd }
    }

    @Test
    fun initialTimestampNanos_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val initialTimestampNanos = randomizer.nextLong()
        every { processor.initialTimestampNanos }.returns(initialTimestampNanos)

        // check
        assertEquals(initialTimestampNanos, estimator.initialTimestampNanos)
        verify(exactly = 1) { processor.initialTimestampNanos }
    }

    @Test
    fun endTimestampNanos_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val endTimestampNanos = randomizer.nextLong()
        every { processor.endTimestampNanos }.returns(endTimestampNanos)

        // check
        assertEquals(endTimestampNanos, estimator.endTimestampNanos)
        verify(exactly = 1) { processor.endTimestampNanos }
    }

    @Test
    fun numberOfProcessedMeasurements_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val numberOfProcessedMeasurements = randomizer.nextLong()
        every { processor.numberOfProcessedMeasurements }.returns(numberOfProcessedMeasurements)

        // check
        assertEquals(
            numberOfProcessedMeasurements,
            estimator.numberOfProcessedMeasurements
        )
        verify(exactly = 1) { processor.numberOfProcessedMeasurements }
    }

    @Test
    fun maxSamples_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val maxSamples = randomizer.nextInt(1, 100)
        every { processor.maxSamples }.returns(maxSamples)

        // check
        assertEquals(maxSamples, estimator.maxSamples)
        verify(exactly = 1) { processor.maxSamples }
    }

    @Test
    fun maxDurationMillis_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val maxDurationMillis = randomizer.nextLong(1, 100)
        every { processor.maxDurationMillis }.returns(maxDurationMillis)

        // check
        assertEquals(maxDurationMillis, estimator.maxDurationMillis)
        verify(exactly = 1) { processor.maxDurationMillis }
    }

    @Test
    fun stopMode_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.stopMode }.returns(StopMode.MAX_SAMPLES_ONLY)

        // check
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        verify(exactly = 1) { processor.stopMode }
    }

    @Test
    fun resultAvailable_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.resultAvailable }.returns(true)

        // check
        assertTrue(estimator.resultAvailable)
        verify(exactly = 1) { processor.resultAvailable }
    }

    @Test
    fun resultUnreliable_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("resultUnreliable", true)

        // check
        assertTrue(estimator.resultUnreliable)
    }

    @Test
    fun averageTimeInterval_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { processor.averageTimeInterval }.returns(averageTimeInterval)

        // check
        assertEquals(averageTimeInterval, estimator.averageTimeInterval)
        verify(exactly = 1) { processor.averageTimeInterval }
    }

    @Test
    fun averageTimeIntervalAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTimeInterval = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )
        every { processor.averageTimeIntervalAsTime }.returns(averageTimeInterval)

        // check
        assertSame(averageTimeInterval, estimator.averageTimeIntervalAsTime)
        verify(exactly = 1) { processor.averageTimeIntervalAsTime }
    }

    @Test
    fun getAverageTimeIntervalAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.getAverageTimeIntervalAsTime(any()) }.returns(true)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertTrue(estimator.getAverageTimeIntervalAsTime(time))

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getAverageTimeIntervalAsTime(capture(slot)) }
        assertSame(time, slot.captured)
    }

    @Test
    fun timeIntervalVariance_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        every { processor.timeIntervalVariance }.returns(timeIntervalVariance)

        // check
        assertEquals(timeIntervalVariance, estimator.timeIntervalVariance)
        verify(exactly = 1) { processor.timeIntervalVariance }
    }

    @Test
    fun timeIntervalStandardDeviation_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        every { processor.timeIntervalStandardDeviation }.returns(timeIntervalStandardDeviation)

        // check
        assertEquals(timeIntervalStandardDeviation, estimator.timeIntervalStandardDeviation)
        verify(exactly = 1) { processor.timeIntervalStandardDeviation }
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )
        every { processor.timeIntervalStandardDeviationAsTime }
            .returns(timeIntervalStandardDeviation)

        // check
        assertSame(timeIntervalStandardDeviation, estimator.timeIntervalStandardDeviationAsTime)
        verify(exactly = 1) { processor.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.getTimeIntervalStandardDeviationAsTime(any()) }.returns(true)

        // check
        val time = Time(0.0, TimeUnit.SECOND)
        assertTrue(estimator.getTimeIntervalStandardDeviationAsTime(time))

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getTimeIntervalStandardDeviationAsTime(capture(slot)) }
        assertSame(time, slot.captured)
    }

    @Test
    fun elapsedTimeNanos_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val elapsedTimeNanos = randomizer.nextLong()
        every { processor.elapsedTimeNanos }.returns(elapsedTimeNanos)

        // check
        assertEquals(elapsedTimeNanos, estimator.elapsedTimeNanos)
        verify(exactly = 1) { processor.elapsedTimeNanos }
    }

    @Test
    fun elapsedTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val elapsedTime = Time(
            randomizer.nextDouble(),
            TimeUnit.NANOSECOND
        )
        every { processor.elapsedTime }.returns(elapsedTime)

        // check
        assertSame(elapsedTime, estimator.elapsedTime)
        verify(exactly = 1) { processor.elapsedTime }
    }

    @Test
    fun getElapsedTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(context)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val elapsedTime = Time(
            randomizer.nextDouble(),
            TimeUnit.NANOSECOND
        )
        every { processor.getElapsedTime(any()) }.answers {
            val time = firstArg<Time>()
            time.value = elapsedTime.value
            time.unit = elapsedTime.unit
        }

        // check
        val time = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time)
        assertEquals(elapsedTime, time)

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getElapsedTime(capture(slot)) }
        assertSame(time, slot.captured)
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("running", true)

        assertThrows(IllegalStateException::class.java) {
            estimator.start()
        }

        verify(exactly = 0) {
            sensorManager.registerListener(
                any(),
                any<Sensor>(),
                any()
            )
        }
    }

    @Test
    fun start_whenNotRunningButSensorCollectorFails_throwsIllegalStateException() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        every {
            sensorManager.registerListener(
                any(), any<Sensor>(),
                any()
            )
        }.returns(false)

        val estimator = GyroscopeNormEstimator(context)

        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())
            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }

        verify(exactly = 1) {
            sensorManager.registerListener(
                any(),
                sensor,
                SensorDelay.FASTEST.value
            )
        }
    }

    @Test
    fun start_whenSucceeds_startsCollector() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        every {
            sensorManager.registerListener(
                any(), any<Sensor>(),
                any()
            )
        }.returns(true)

        val estimator = GyroscopeNormEstimator(context)

        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())
            estimator.start()
        }

        assertTrue(estimator.running)
        verify(exactly = 1) {
            sensorManager.registerListener(
                any(),
                sensor,
                SensorDelay.FASTEST.value
            )
        }
    }

    @Test
    fun stop_whenNotRunning_throwsIllegalStateException() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        // not running yet
        assertFalse(estimator.running)

        // stop should do nothing
        assertThrows(IllegalStateException::class.java) {
            estimator.stop()
        }

        verify(exactly = 0) {
            sensorManager.unregisterListener(any(), any<Sensor>())
        }
    }

    @Test
    fun stop_whenRunning_stopsCollector() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // stop
        estimator.stop()

        assertFalse(estimator.running)
        verify(exactly = 1) {
            sensorManager.unregisterListener(any(), any<Sensor>())
        }
    }

    @Test
    fun reset_resetsProcessor() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        justRun { processor.reset() }

        // reset
        estimator.reset()

        verify(exactly = 1) { processor.reset() }
    }

    @Test
    fun accuracyChangedListener_whenReliable_doesNothing() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context, unreliableListener = unreliableListener)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.HIGH)
        verify { unreliableListener wasNot Called }
    }

    @Test
    fun accuracyChangedListener_whenUnreliableWithListener_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context, unreliableListener = unreliableListener)

        justRun { unreliableListener.onUnreliable(estimator) }

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.UNRELIABLE)
        verify(exactly = 1) { unreliableListener.onUnreliable(estimator) }
    }

    @Test
    fun accuracyChangedListener_whenUnreliableWithoutListener_doesNothing() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("accuracyChangedListener")
        requireNotNull(accuracyChangedListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        accuracyChangedListener.onAccuracyChanged(collector, SensorAccuracy.UNRELIABLE)
        verify { unreliableListener wasNot Called }
    }

    @Test
    fun measurementListener_whenUnreliable_marksResultAsUnreliable() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNormEstimator(context, completedListener = completedListener)

        estimator.setPrivateProperty("processor", processor)

        every { processor.process(any()) }.returns(false)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.UNRELIABLE)
        measurementListener.onMeasurement(collector, measurement)

        assertTrue(estimator.resultUnreliable)
        verify(exactly = 1) { processor.process(measurement) }
        verify { completedListener wasNot Called }
    }

    @Test
    fun measurementListener_whenComplete_stopsAndNotifiesCompletion() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = GyroscopeNormEstimator(context, completedListener = completedListener)

        estimator.setPrivateProperty("processor", processor)
        estimator.setPrivateProperty("running", true)

        every { processor.process(any()) }.returns(true)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        assertTrue(estimator.running)
        justRun { completedListener.onEstimationCompleted(estimator) }

        val measurement = GyroscopeSensorMeasurement()
        measurementListener.onMeasurement(collector, measurement)

        assertFalse(estimator.resultUnreliable)
        assertFalse(estimator.running)

        verify(exactly = 1) { processor.process(measurement) }
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }
        verify(exactly = 1) {
            sensorManager.unregisterListener(any(), any<Sensor>())
        }
    }

    @Test
    fun measurementListener_whenCompleteAndNoListener_makesNoAction() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = GyroscopeNormEstimator(context)

        estimator.setPrivateProperty("processor", processor)
        estimator.setPrivateProperty("running", true)

        every { processor.process(any()) }.returns(true)

        val measurementListener: SensorCollector.OnMeasurementListener<
                GyroscopeSensorMeasurement, GyroscopeSensorCollector>? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val collector: GyroscopeSensorCollector? = estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        assertTrue(estimator.running)

        val measurement = GyroscopeSensorMeasurement()
        measurementListener.onMeasurement(collector, measurement)

        assertFalse(estimator.resultUnreliable)
        assertFalse(estimator.running)

        verify(exactly = 1) { processor.process(measurement) }
        verify { completedListener wasNot Called }
        verify(exactly = 1) {
            sensorManager.unregisterListener(any(), any<Sensor>())
        }
    }

    private companion object {
        const val MAX_SAMPLES = 100

        const val MAX_DURATION_MILLIS = 1000L
    }
}