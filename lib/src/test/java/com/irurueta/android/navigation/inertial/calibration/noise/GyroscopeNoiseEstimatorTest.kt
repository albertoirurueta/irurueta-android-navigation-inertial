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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
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
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test

class GyroscopeNoiseEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var completedListener: AccumulatedTriadEstimator
    .OnEstimationCompletedListener<GyroscopeNoiseEstimator>

    @MockK
    private lateinit var unreliableListener: AccumulatedTriadEstimator
    .OnUnreliableListener<GyroscopeNoiseEstimator>

    @MockK
    private lateinit var processor: GyroscopeNoiseProcessor

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

        val estimator = GyroscopeNoiseEstimator(context)

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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorType.GYROSCOPE
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        val estimator = GyroscopeNoiseEstimator(
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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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
            GyroscopeNoiseEstimator(
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

        val estimator = GyroscopeNoiseEstimator(
            context,
            maxSamples = MAX_SAMPLES
        )

        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            MAX_SAMPLES,
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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        assertThrows(java.lang.IllegalArgumentException::class.java) {
            GyroscopeNoiseEstimator(
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

        val estimator = GyroscopeNoiseEstimator(
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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        val estimator = GyroscopeNoiseEstimator(
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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        val estimator = GyroscopeNoiseEstimator(context, completedListener = completedListener)

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
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        val estimator = GyroscopeNoiseEstimator(
            context,
            unreliableListener = unreliableListener
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
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertSame(sensor, estimator.sensor)
        assertFalse(estimator.running)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(
            0.0,
            AngularSpeedUnit.RADIANS_PER_SECOND
        )
        assertFalse(estimator.getAverageXAsMeasurement(angularSpeed))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(angularSpeed))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(angularSpeed))
        assertNull(estimator.averageTriad)
        val triad = AngularSpeedTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(angularSpeed))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(angularSpeed))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(angularSpeed))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(angularSpeed))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

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
    fun averageX_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageX = randomizer.nextDouble()
        every { processor.averageX }.returns(averageX)

        // check
        assertEquals(averageX, estimator.averageX)
        verify(exactly = 1) { processor.averageX }
    }

    @Test
    fun averageXAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageX = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.averageXAsMeasurement }.returns(averageX)

        // check
        assertEquals(averageX, estimator.averageXAsMeasurement)
        verify(exactly = 1) { processor.averageXAsMeasurement }
    }

    @Test
    fun getAverageXAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageX = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getAverageXAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageXAsMeasurement(averageX))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getAverageXAsMeasurement(capture(slot)) }
        assertSame(averageX, slot.captured)
    }

    @Test
    fun averageY_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageY = randomizer.nextDouble()
        every { processor.averageY }.returns(averageY)

        // check
        assertEquals(averageY, estimator.averageY)
        verify(exactly = 1) { processor.averageY }
    }

    @Test
    fun averageYAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageY = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.averageYAsMeasurement }.returns(averageY)

        // check
        assertEquals(averageY, estimator.averageYAsMeasurement)
        verify(exactly = 1) { processor.averageYAsMeasurement }
    }

    @Test
    fun getAverageYAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageY = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getAverageYAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageYAsMeasurement(averageY))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getAverageYAsMeasurement(capture(slot)) }
        assertSame(averageY, slot.captured)
    }

    @Test
    fun averageZ_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageZ = randomizer.nextDouble()
        every { processor.averageZ }.returns(averageZ)

        // check
        assertEquals(averageZ, estimator.averageZ)
        verify(exactly = 1) { processor.averageZ }
    }

    @Test
    fun averageZAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageZ = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.averageZAsMeasurement }.returns(averageZ)

        // check
        assertEquals(averageZ, estimator.averageZAsMeasurement)
        verify(exactly = 1) { processor.averageZAsMeasurement }
    }

    @Test
    fun getAverageZAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageZ = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getAverageZAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageZAsMeasurement(averageZ))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getAverageZAsMeasurement(capture(slot)) }
        assertSame(averageZ, slot.captured)
    }

    @Test
    fun averageTriad_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTriad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        every { processor.averageTriad }.returns(averageTriad)

        // check
        val result = estimator.averageTriad
        assertSame(averageTriad, result)
        verify(exactly = 1) { processor.averageTriad }
    }

    @Test
    fun getAverageTriad_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTriad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        every { processor.getAverageTriad(averageTriad) }.returns(true)

        // check
        assertTrue(estimator.getAverageTriad(averageTriad))

        val slot = slot<AngularSpeedTriad>()
        verify(exactly = 1) { processor.getAverageTriad(capture(slot)) }
        assertSame(averageTriad, slot.captured)
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

        val estimator = GyroscopeNoiseEstimator(context)

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
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageNorm = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.averageNormAsMeasurement }.returns(averageNorm)

        // check
        assertEquals(averageNorm, estimator.averageNormAsMeasurement)
        verify(exactly = 1) { processor.averageNormAsMeasurement }
    }

    @Test
    fun getAverageNormAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

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
    fun varianceX_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val varianceX = randomizer.nextDouble()
        every { processor.varianceX }.returns(varianceX)

        // check
        assertEquals(varianceX, estimator.varianceX)
        verify(exactly = 1) { processor.varianceX }
    }

    @Test
    fun varianceY_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val varianceY = randomizer.nextDouble()
        every { processor.varianceY }.returns(varianceY)

        // check
        assertEquals(varianceY, estimator.varianceY)
        verify(exactly = 1) { processor.varianceY }
    }

    @Test
    fun varianceZ_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val varianceZ = randomizer.nextDouble()
        every { processor.varianceZ }.returns(varianceZ)

        // check
        assertEquals(varianceZ, estimator.varianceZ)
        verify(exactly = 1) { processor.varianceZ }
    }

    @Test
    fun standardDeviationX_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationX = randomizer.nextDouble()
        every { processor.standardDeviationX }.returns(standardDeviationX)

        // check
        assertEquals(standardDeviationX, estimator.standardDeviationX)
        verify(exactly = 1) { processor.standardDeviationX }
    }

    @Test
    fun standardDeviationXAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationX = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.standardDeviationXAsMeasurement }.returns(standardDeviationX)

        // check
        assertEquals(standardDeviationX, estimator.standardDeviationXAsMeasurement)
        verify(exactly = 1) { processor.standardDeviationXAsMeasurement }
    }

    @Test
    fun getStandardDeviationXAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationX = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getStandardDeviationXAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getStandardDeviationXAsMeasurement(standardDeviationX))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getStandardDeviationXAsMeasurement(capture(slot)) }
        assertSame(standardDeviationX, slot.captured)
    }

    @Test
    fun standardDeviationY_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationY = randomizer.nextDouble()
        every { processor.standardDeviationY }.returns(standardDeviationY)

        // check
        assertEquals(standardDeviationY, estimator.standardDeviationY)
        verify(exactly = 1) { processor.standardDeviationY }
    }

    @Test
    fun standardDeviationYAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationY = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.standardDeviationYAsMeasurement }.returns(standardDeviationY)

        // check
        assertEquals(standardDeviationY, estimator.standardDeviationYAsMeasurement)
        verify(exactly = 1) { processor.standardDeviationYAsMeasurement }
    }

    @Test
    fun getStandardDeviationYAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationY = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getStandardDeviationYAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getStandardDeviationYAsMeasurement(standardDeviationY))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getStandardDeviationYAsMeasurement(capture(slot)) }
        assertSame(standardDeviationY, slot.captured)
    }

    @Test
    fun standardDeviationZ_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationZ = randomizer.nextDouble()
        every { processor.standardDeviationZ }.returns(standardDeviationZ)

        // check
        assertEquals(standardDeviationZ, estimator.standardDeviationZ)
        verify(exactly = 1) { processor.standardDeviationZ }
    }

    @Test
    fun standardDeviationZAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationZ = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.standardDeviationZAsMeasurement }.returns(standardDeviationZ)

        // check
        assertEquals(standardDeviationZ, estimator.standardDeviationZAsMeasurement)
        verify(exactly = 1) { processor.standardDeviationZAsMeasurement }
    }

    @Test
    fun getStandardDeviationZAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationZ = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getStandardDeviationZAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getStandardDeviationZAsMeasurement(standardDeviationZ))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getStandardDeviationZAsMeasurement(capture(slot)) }
        assertSame(standardDeviationZ, slot.captured)
    }

    @Test
    fun standardDeviationTriad_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationTriad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        every { processor.standardDeviationTriad }.returns(standardDeviationTriad)

        // check
        val result = estimator.standardDeviationTriad
        assertSame(standardDeviationTriad, result)
        verify(exactly = 1) { processor.standardDeviationTriad }
    }

    @Test
    fun getStandardDeviationTriad_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationTriad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        every { processor.getStandardDeviationTriad(standardDeviationTriad) }.returns(true)

        // check
        assertTrue(estimator.getStandardDeviationTriad(standardDeviationTriad))

        val slot = slot<AngularSpeedTriad>()
        verify(exactly = 1) { processor.getStandardDeviationTriad(capture(slot)) }
        assertSame(standardDeviationTriad, slot.captured)
    }

    @Test
    fun standardDeviationNorm_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationNorm = randomizer.nextDouble()
        every { processor.standardDeviationNorm }.returns(standardDeviationNorm)

        // check
        assertEquals(standardDeviationNorm, estimator.standardDeviationNorm)
        verify(exactly = 1) { processor.standardDeviationNorm }
    }

    @Test
    fun standardDeviationNormAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationNorm = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.standardDeviationNormAsMeasurement }.returns(standardDeviationNorm)

        // check
        assertEquals(standardDeviationNorm, estimator.standardDeviationNormAsMeasurement)
        verify(exactly = 1) { processor.standardDeviationNormAsMeasurement }
    }

    @Test
    fun getStandardDeviationNormAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val standardDeviationNorm = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getStandardDeviationNormAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) { processor.getStandardDeviationNormAsMeasurement(capture(slot)) }
        assertSame(standardDeviationNorm, slot.captured)
    }

    @Test
    fun averageStandardDeviation_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageStandardDeviation = randomizer.nextDouble()
        every { processor.averageStandardDeviation }.returns(averageStandardDeviation)

        // check
        assertEquals(averageStandardDeviation, estimator.averageStandardDeviation)
        verify(exactly = 1) { processor.averageStandardDeviation }
    }

    @Test
    fun averageStandardDeviationAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageStandardDeviation = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.averageStandardDeviationAsMeasurement }
            .returns(averageStandardDeviation)

        // check
        assertEquals(averageStandardDeviation, estimator.averageStandardDeviationAsMeasurement)
        verify(exactly = 1) { processor.averageStandardDeviationAsMeasurement }
    }

    @Test
    fun getAverageStandardDeviationAsMeasurement_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageStandardDeviation = AngularSpeed(
            randomizer.nextDouble(),
            AngularSpeedUnit.RADIANS_PER_SECOND
        )

        every { processor.getAverageStandardDeviationAsMeasurement(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation))

        val slot = slot<AngularSpeed>()
        verify(exactly = 1) {
            processor.getAverageStandardDeviationAsMeasurement(capture(slot))
        }
        assertSame(averageStandardDeviation, slot.captured)
    }

    @Test
    fun psdX_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val psdX = randomizer.nextDouble()
        every { processor.psdX }.returns(psdX)

        // check
        assertEquals(psdX, estimator.psdX)
        verify(exactly = 1) { processor.psdX }
    }

    @Test
    fun psdY_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val psdY = randomizer.nextDouble()
        every { processor.psdY }.returns(psdY)

        // check
        assertEquals(psdY, estimator.psdY)
        verify(exactly = 1) { processor.psdY }
    }

    @Test
    fun psdZ_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val psdZ = randomizer.nextDouble()
        every { processor.psdZ }.returns(psdZ)

        // check
        assertEquals(psdZ, estimator.psdZ)
        verify(exactly = 1) { processor.psdZ }
    }

    @Test
    fun rootPsdX_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val rootPsdX = randomizer.nextDouble()
        every { processor.rootPsdX }.returns(rootPsdX)

        // check
        assertEquals(rootPsdX, estimator.rootPsdX)
        verify(exactly = 1) { processor.rootPsdX }
    }

    @Test
    fun rootPsdY_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val rootPsdY = randomizer.nextDouble()
        every { processor.rootPsdY }.returns(rootPsdY)

        // check
        assertEquals(rootPsdY, estimator.rootPsdY)
        verify(exactly = 1) { processor.rootPsdY }
    }

    @Test
    fun rootPsdZ_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val rootPsdZ = randomizer.nextDouble()
        every { processor.rootPsdZ }.returns(rootPsdZ)

        // check
        assertEquals(rootPsdZ, estimator.rootPsdZ)
        verify(exactly = 1) { processor.rootPsdZ }
    }

    @Test
    fun averageNoisePsd_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageNoisePsd = randomizer.nextDouble()
        every { processor.averageNoisePsd }.returns(averageNoisePsd)

        // check
        assertEquals(averageNoisePsd, estimator.averageNoisePsd)
        verify(exactly = 1) { processor.averageNoisePsd }
    }

    @Test
    fun noiseRootPsdNorm_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val noiseRootPsdNorm = randomizer.nextDouble()
        every { processor.noiseRootPsdNorm }.returns(noiseRootPsdNorm)

        // check
        assertEquals(noiseRootPsdNorm, estimator.noiseRootPsdNorm)
        verify(exactly = 1) { processor.noiseRootPsdNorm }
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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val numberOfProcessedMeasurements = randomizer.nextLong(1, 1000)
        every { processor.numberOfProcessedMeasurements }
            .returns(numberOfProcessedMeasurements)

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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.maxSamples }.returns(MAX_SAMPLES)

        // check
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        every { processor.maxDurationMillis }.returns(MAX_DURATION_MILLIS)

        // check
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val stopMode = StopMode.MAX_SAMPLES_ONLY
        every { processor.stopMode }.returns(stopMode)

        // check
        assertEquals(stopMode, estimator.stopMode)
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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

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
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTimeInterval = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )
        every { processor.averageTimeIntervalAsTime }.returns(averageTimeInterval)

        // check
        assertEquals(averageTimeInterval, estimator.averageTimeIntervalAsTime)
        verify(exactly = 1) { processor.averageTimeIntervalAsTime }
    }

    @Test
    fun getAverageTimeIntervalAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val averageTimeInterval = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )

        every { processor.getAverageTimeIntervalAsTime(any()) }.returns(true)

        // check
        assertTrue(estimator.getAverageTimeIntervalAsTime(averageTimeInterval))

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getAverageTimeIntervalAsTime(capture(slot)) }
        assertSame(averageTimeInterval, slot.captured)
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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

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
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )
        every { processor.timeIntervalStandardDeviationAsTime }
            .returns(timeIntervalStandardDeviation)

        // check
        assertEquals(
            timeIntervalStandardDeviation,
            estimator.timeIntervalStandardDeviationAsTime
        )
        verify(exactly = 1) { processor.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )

        every { processor.getTimeIntervalStandardDeviationAsTime(any()) }.returns(true)

        // check
        assertTrue(estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation))

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getTimeIntervalStandardDeviationAsTime(capture(slot)) }
        assertSame(timeIntervalStandardDeviation, slot.captured)
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

        val estimator = GyroscopeNoiseEstimator(context)

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
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val elapsedTime = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )
        every { processor.elapsedTime }.returns(elapsedTime)

        // check
        assertEquals(elapsedTime, estimator.elapsedTime)
        verify(exactly = 1) { processor.elapsedTime }
    }

    @Test
    fun getElapsedTime_returnsExpectedValue() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value
            )
        }.returns(
            sensor
        )

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        val randomizer = UniformRandomizer()
        val elapsedTime = Time(
            randomizer.nextDouble(),
            TimeUnit.SECOND
        )

        justRun { processor.getElapsedTime(any()) }

        // check
        estimator.getElapsedTime(elapsedTime)

        val slot = slot<Time>()
        verify(exactly = 1) { processor.getElapsedTime(capture(slot)) }
        assertSame(elapsedTime, slot.captured)
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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("running", true)

        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
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
                any(), any<Sensor>(), any()
            )
        }.returns(false)

        val estimator = GyroscopeNoiseEstimator(context)

        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }

        verify(exactly = 1) {
            sensorManager.registerListener(
                any(),
                any<Sensor>(),
                any()
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
                any(), any<Sensor>(), any()
            )
        }.returns(true)

        val estimator = GyroscopeNoiseEstimator(context)

        mockkStatic(SystemClock::class) {
            every { SystemClock.elapsedRealtimeNanos() }.returns(System.nanoTime())

            estimator.start()
        }

        // check
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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("running", true)

        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) {
            sensorManager.unregisterListener(any(), sensor)
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

        val estimator = GyroscopeNoiseEstimator(context)

        estimator.setPrivateProperty("processor", processor)

        justRun { processor.reset() }

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

        val estimator = GyroscopeNoiseEstimator(
            context,
            unreliableListener = unreliableListener
        )

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

        val estimator = GyroscopeNoiseEstimator(
            context,
            unreliableListener = unreliableListener
        )

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

        val estimator = GyroscopeNoiseEstimator(context)

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

        val estimator = GyroscopeNoiseEstimator(context, completedListener = completedListener)

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

        val estimator = GyroscopeNoiseEstimator(context, completedListener = completedListener)

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
            sensorManager.unregisterListener(
                any(),
                sensor
            )
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

        val estimator = GyroscopeNoiseEstimator(context)

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
            sensorManager.unregisterListener(
                any(),
                sensor
            )
        }
    }

    private companion object {
        const val MAX_SAMPLES = 100

        const val MAX_DURATION_MILLIS = 1000L
    }
}