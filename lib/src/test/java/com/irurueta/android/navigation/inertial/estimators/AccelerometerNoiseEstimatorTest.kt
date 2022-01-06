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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.ECEFGravity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationTriadNoiseEstimator
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerNoiseEstimatorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        // check default values
        assertSame(context, estimator.context)
        assertEquals(AccelerometerSensorCollector.SensorType.ACCELEROMETER, estimator.sensorType)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(AccumulatedTriadEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            AccumulatedTriadEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun constructor_whenSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(AccumulatedTriadEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            AccumulatedTriadEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(AccumulatedTriadEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            AccumulatedTriadEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenNegativeMaxSamples_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            -1
        )
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            AccumulatedTriadEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenNegativeMaxDurationMillis_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            -1L
        )
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun constructor_whenStopMode_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun constructor_whenCompletedListener_setsExpectedValues() {
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun constructor_whenUnreliableListener_setsExpectedValues() {
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>()
        val unreliableListener =
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<AccelerometerNoiseEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener,
            unreliableListener
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageXAsMeasurement(acceleration))
        assertNull(estimator.averageY)
        assertNull(estimator.averageYAsMeasurement)
        assertFalse(estimator.getAverageYAsMeasurement(acceleration))
        assertNull(estimator.averageZ)
        assertNull(estimator.averageZAsMeasurement)
        assertFalse(estimator.getAverageZAsMeasurement(acceleration))
        assertNull(estimator.averageTriad)
        val triad = AccelerationTriad()
        assertFalse(estimator.getAverageTriad(triad))
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.varianceX)
        assertNull(estimator.varianceY)
        assertNull(estimator.varianceZ)
        assertNull(estimator.standardDeviationX)
        assertNull(estimator.standardDeviationXAsMeasurement)
        assertFalse(estimator.getStandardDeviationXAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationY)
        assertNull(estimator.standardDeviationYAsMeasurement)
        assertFalse(estimator.getStandardDeviationYAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationZ)
        assertNull(estimator.standardDeviationZAsMeasurement)
        assertFalse(estimator.getStandardDeviationZAsMeasurement(acceleration))
        assertNull(estimator.standardDeviationTriad)
        assertFalse(estimator.getStandardDeviationTriad(triad))
        assertNull(estimator.standardDeviationNorm)
        assertNull(estimator.standardDeviationNormAsMeasurement)
        assertFalse(estimator.getStandardDeviationNormAsMeasurement(acceleration))
        assertNull(estimator.averageStandardDeviation)
        assertNull(estimator.averageStandardDeviationAsMeasurement)
        assertFalse(estimator.getAverageStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psdX)
        assertNull(estimator.psdY)
        assertNull(estimator.psdZ)
        assertNull(estimator.rootPsdX)
        assertNull(estimator.rootPsdY)
        assertNull(estimator.rootPsdZ)
        assertNull(estimator.averageNoisePsd)
        assertNull(estimator.noiseRootPsdNorm)
        assertNull(estimator.averageTimeInterval)
        assertNull(estimator.averageTimeIntervalAsTime)
        val time1 = Time(0.0, TimeUnit.SECOND)
        assertFalse(estimator.getAverageTimeIntervalAsTime(time1))
        assertNull(estimator.timeIntervalVariance)
        assertNull(estimator.timeIntervalStandardDeviation)
        assertNull(estimator.timeIntervalStandardDeviationAsTime)
        assertFalse(estimator.getTimeIntervalStandardDeviationAsTime(time1))
        assertEquals(0L, estimator.elapsedTimeNanos)
        assertEquals(Time(0.0, TimeUnit.NANOSECOND), estimator.elapsedTime)
        val time2 = Time(1.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(time2)
        assertEquals(estimator.elapsedTime, time2)
    }

    @Test
    fun completedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        // check default value
        assertNull(estimator.completedListener)

        // set new value
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>()
        estimator.completedListener = completedListener

        // check
        assertSame(completedListener, estimator.completedListener)
    }

    @Test
    fun unreliableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        // check default value
        assertNull(estimator.unreliableListener)

        // set new value
        val unreliableListener =
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<AccelerometerNoiseEstimator>>()
        estimator.unreliableListener = unreliableListener

        // check
        assertSame(unreliableListener, estimator.unreliableListener)
    }

    @Test
    fun start_startsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)
        verify(exactly = 1) { collectorSpy.start() }
    }

    @Test
    fun start_whenDefaultStopMode_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            AccumulatedTriadEstimator::class,
            estimator,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            AccumulatedTriadEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        assertEquals(0.0, noiseEstimatorSpy.timeInterval, 0.0)

        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        assertEquals(estimator.maxSamples, timeIntervalEstimatorSpy.totalSamples)
    }

    @Test
    fun start_whenMaxDurationOnlyStopMode_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context, stopMode = StopMode.MAX_DURATION_ONLY)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            AccumulatedTriadEstimator::class,
            estimator,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            AccumulatedTriadEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        assertEquals(0.0, noiseEstimatorSpy.timeInterval, 0.0)

        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        assertEquals(Integer.MAX_VALUE, timeIntervalEstimatorSpy.totalSamples)
    }

    @Test
    fun start_whenResultUnreliable_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        setPrivateProperty(AccumulatedTriadEstimator::class, estimator, "resultUnreliable", true)
        assertTrue(estimator.resultUnreliable)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        assertFalse(estimator.resultUnreliable)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        // start again
        estimator.start()
    }

    @Test
    fun stop_whenAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)
        verify(exactly = 1) { collectorSpy.start() }

        // stop
        estimator.stop()

        assertFalse(estimator.running)
        verify(exactly = 1) { collectorSpy.stop() }
    }

    @Test
    fun stop_whenNotAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertFalse(estimator.running)

        // stop
        estimator.stop()

        assertFalse(estimator.running)
        verify(exactly = 1) { collectorSpy.stop() }
    }

    @Test
    fun onMeasurement_whenUnreliableAccuracy_makesResultUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        assertFalse(estimator.resultUnreliable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.UNRELIABLE

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp, accuracy)

        assertTrue(estimator.resultUnreliable)
    }

    @Test
    fun onMeasurement_whenFirstMeasurement_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val initialTimestampNanos1: Long? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos1)
        assertEquals(0L, initialTimestampNanos1)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val timestamp2 = SystemClock.elapsedRealtimeNanos() + 1000000
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        val initialTimestampNanos2: Long? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos2)
        assertEquals(timestamp1, initialTimestampNanos2)

        // calling again with another timestamp, makes no effect
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        val initialTimestampNanos3: Long? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos3)
        assertEquals(timestamp1, initialTimestampNanos3)
    }

    @Test
    fun onMeasurement_addsMeasurementToNoiseAndTimeIntervalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val noiseEstimator: AccumulatedAccelerationTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            AccumulatedTriadEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp, accuracy)

        verify(exactly = 1) {
            noiseEstimatorSpy.addTriad(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble()
            )
        }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(0.0) }
    }

    @Test
    fun onMeasurement_updatesEndTimestampNanos() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        val endTimestampNanos1: Long? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "endTimestampNanos")
        requireNotNull(endTimestampNanos1)
        assertEquals(0L, endTimestampNanos1)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp, accuracy)

        val endTimestampNanos2: Long? =
            getPrivateProperty(AccumulatedTriadEstimator::class, estimator, "endTimestampNanos")
        requireNotNull(endTimestampNanos2)
        assertEquals(timestamp, endTimestampNanos2)
    }

    @Test
    fun onMeasurement_increasesNumberOfProcessedMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        // check default value
        assertEquals(0, estimator.numberOfProcessedMeasurements)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp, accuracy)

        // check
        assertEquals(1, estimator.numberOfProcessedMeasurements)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context, stopMode = StopMode.MAX_SAMPLES_ONLY)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        for (i in 1..maxSamples) {
            measurementListener.onMeasurement(
                ax,
                ay,
                az,
                null,
                null,
                null,
                timestamp + i * MILLIS_TO_NANOS,
                accuracy
            )

            assertEquals(i, estimator.numberOfProcessedMeasurements)
        }

        assertEquals(maxSamples, estimator.numberOfProcessedMeasurements)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxSamples(estimator, gravity)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndListener() {
        val completedListener =
            mockk<AccumulatedTriadEstimator
            .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            stopMode = StopMode.MAX_SAMPLES_ONLY,
            completedListener = completedListener
        )

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        for (i in 1..maxSamples) {
            measurementListener.onMeasurement(
                ax,
                ay,
                az,
                null,
                null,
                null,
                timestamp + i * MILLIS_TO_NANOS,
                accuracy
            )

            assertEquals(i, estimator.numberOfProcessedMeasurements)
        }

        assertEquals(maxSamples, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxSamples(estimator, gravity)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context, stopMode = StopMode.MAX_DURATION_ONLY)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, gravity)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndListener() {
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            stopMode = StopMode.MAX_DURATION_ONLY,
            completedListener = completedListener
        )

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, gravity)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AccelerometerNoiseEstimator(context, stopMode = StopMode.MAX_SAMPLES_OR_DURATION)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, gravity)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndListener() {
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<AccelerometerNoiseEstimator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(
            context,
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            completedListener = completedListener
        )

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, gravity)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndNoListener_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNoiseEstimator(context)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                AccumulatedTriadEstimator::class,
                estimator,
                "accuracyChangedListener"
            )

        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        assertTrue(estimator.resultUnreliable)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndListener_setsResultAsUnreliable() {
        val unreliableListener =
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<AccelerometerNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AccelerometerNoiseEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                AccumulatedTriadEstimator::class,
                estimator,
                "accuracyChangedListener"
            )

        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        assertTrue(estimator.resultUnreliable)
        verify(exactly = 1) { unreliableListener.onUnreliable(estimator) }
    }

    @Test
    fun onAccuracyChanged_whenNotUnreliable_makesNoAction() {
        val unreliableListener =
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<AccelerometerNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AccelerometerNoiseEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                AccumulatedTriadEstimator::class,
                estimator,
                "accuracyChangedListener"
            )
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.MEDIUM)

        // check
        assertFalse(estimator.resultUnreliable)
        verify(exactly = 0) { unreliableListener.onUnreliable(estimator) }
    }

    private fun checkResultMaxSamples(
        estimator: AccelerometerNoiseEstimator,
        gravity: ECEFGravity
    ) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageX = estimator.averageX
        requireNotNull(averageX)
        assertEquals(gravity.gx, averageX, ABSOLUTE_ERROR)

        val averageX1 = estimator.averageXAsMeasurement
        requireNotNull(averageX1)
        val averageX2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageXAsMeasurement(averageX2))
        assertEquals(averageX1, averageX2)
        assertEquals(averageX, averageX1.value.toDouble(), 0.0)

        val averageY = estimator.averageY
        requireNotNull(averageY)
        assertEquals(gravity.gy, averageY, ABSOLUTE_ERROR)

        val averageY1 = estimator.averageYAsMeasurement
        requireNotNull(averageY1)
        val averageY2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageYAsMeasurement(averageY2))
        assertEquals(averageY1, averageY2)
        assertEquals(averageY, averageY1.value.toDouble(), 0.0)

        val averageZ = estimator.averageZ
        requireNotNull(averageZ)
        assertEquals(gravity.gz, averageZ, ABSOLUTE_ERROR)

        val averageZ1 = estimator.averageZAsMeasurement
        requireNotNull(averageZ1)
        val averageZ2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageZAsMeasurement(averageZ2))
        assertEquals(averageZ1, averageZ2)
        assertEquals(averageZ, averageZ1.value.toDouble(), 0.0)

        val averageTriad1 = estimator.averageTriad
        requireNotNull(averageTriad1)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageTriad1.unit)
        assertEquals(averageX, averageTriad1.valueX, 0.0)
        assertEquals(averageY, averageTriad1.valueY, 0.0)
        assertEquals(averageZ, averageTriad1.valueZ, 0.0)
        val averageTriad2 = AccelerationTriad()
        assertTrue(estimator.getAverageTriad(averageTriad2))
        assertEquals(averageTriad1, averageTriad2)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(gravity.norm, averageNorm, ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)

        val varianceX = estimator.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)

        val varianceY = estimator.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)

        val varianceZ = estimator.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val standardDeviationX = estimator.standardDeviationX
        requireNotNull(standardDeviationX)
        assertEquals(0.0, standardDeviationX, ABSOLUTE_ERROR)
        val standardDeviationX1 = estimator.standardDeviationXAsMeasurement
        requireNotNull(standardDeviationX1)
        val standardDeviationX2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationXAsMeasurement(standardDeviationX2))
        assertEquals(standardDeviationX1, standardDeviationX2)
        assertEquals(standardDeviationX, standardDeviationX1.value.toDouble(), 0.0)

        val standardDeviationY = estimator.standardDeviationY
        requireNotNull(standardDeviationY)
        assertEquals(0.0, standardDeviationY, ABSOLUTE_ERROR)
        val standardDeviationY1 = estimator.standardDeviationYAsMeasurement
        requireNotNull(standardDeviationY1)
        val standardDeviationY2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationYAsMeasurement(standardDeviationY2))
        assertEquals(standardDeviationY1, standardDeviationY2)
        assertEquals(standardDeviationY, standardDeviationY1.value.toDouble(), 0.0)

        val standardDeviationZ = estimator.standardDeviationZ
        requireNotNull(standardDeviationZ)
        assertEquals(0.0, standardDeviationZ, ABSOLUTE_ERROR)
        val standardDeviationZ1 = estimator.standardDeviationZAsMeasurement
        requireNotNull(standardDeviationZ1)
        val standardDeviationZ2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationZAsMeasurement(standardDeviationZ2))
        assertEquals(standardDeviationZ1, standardDeviationZ2)
        assertEquals(standardDeviationZ, standardDeviationZ1.value.toDouble(), 0.0)

        val standardDeviationTriad1 = estimator.standardDeviationTriad
        requireNotNull(standardDeviationTriad1)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, standardDeviationTriad1.unit)
        assertEquals(standardDeviationX, standardDeviationTriad1.valueX, 0.0)
        assertEquals(standardDeviationY, standardDeviationTriad1.valueY, 0.0)
        assertEquals(standardDeviationZ, standardDeviationTriad1.valueZ, 0.0)
        val standardDeviationTriad2 = AccelerationTriad()
        assertTrue(estimator.getStandardDeviationTriad(standardDeviationTriad2))
        assertEquals(standardDeviationTriad1, standardDeviationTriad2)

        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        assertEquals(0.0, standardDeviationNorm, ABSOLUTE_ERROR)
        val standardDeviationNorm1 = estimator.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNorm1)
        val standardDeviationNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm2))
        assertEquals(standardDeviationNorm1, standardDeviationNorm2)
        assertEquals(standardDeviationNorm, standardDeviationNorm1.value.toDouble(), 0.0)

        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)
        assertEquals(0.0, averageStandardDeviation, ABSOLUTE_ERROR)
        val averageStandardDeviation1 = estimator.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviation1)
        val averageStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation2))
        assertEquals(averageStandardDeviation1, averageStandardDeviation2)
        assertEquals(averageStandardDeviation, averageStandardDeviation1.value.toDouble(), 0.0)

        val psdX = estimator.psdX
        requireNotNull(psdX)
        assertTrue(psdX > 0.0)

        val psdY = estimator.psdY
        requireNotNull(psdY)
        assertTrue(psdY > 0.0)

        val psdZ = estimator.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val rootPsdX = estimator.rootPsdX
        requireNotNull(rootPsdX)
        assertTrue(rootPsdX > 0.0)

        val rootPsdY = estimator.rootPsdY
        requireNotNull(rootPsdY)
        assertTrue(rootPsdY > 0.0)

        val rootPsdZ = estimator.rootPsdZ
        requireNotNull(rootPsdZ)
        assertTrue(rootPsdZ > 0.0)

        val averageNoisePsd = estimator.averageNoisePsd
        requireNotNull(averageNoisePsd)
        assertTrue(averageNoisePsd > 0.0)

        val noiseRootPsdNorm = estimator.noiseRootPsdNorm
        requireNotNull(noiseRootPsdNorm)
        assertTrue(noiseRootPsdNorm > 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getAverageTimeIntervalAsTime(averageTimeInterval2))
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)

        val timeIntervalVariance = estimator.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance, LARGE_ABSOLUTE_ERROR)

        val timeIntervalStandardDeviation = estimator.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation)
        assertEquals(0.0, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR)
        val timeIntervalStandardDeviation1 = estimator.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviation1)
        val timeIntervalStandardDeviation2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2))
        assertEquals(timeIntervalStandardDeviation1, timeIntervalStandardDeviation2)
        assertEquals(
            timeIntervalStandardDeviation,
            timeIntervalStandardDeviation1.value.toDouble(),
            0.0
        )

        val elapsedTimeNanos = estimator.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        val elapsedTime1 = estimator.elapsedTime
        val elapsedTime2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(elapsedTime2)
        assertEquals(elapsedTime1, elapsedTime2)
        assertEquals(elapsedTimeNanos.toDouble(), elapsedTime1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, elapsedTime1.unit)
    }

    private fun checkResultMaxDuration(
        estimator: AccelerometerNoiseEstimator,
        gravity: ECEFGravity
    ) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageX = estimator.averageX
        requireNotNull(averageX)
        assertEquals(gravity.gx, averageX, ABSOLUTE_ERROR)

        val averageX1 = estimator.averageXAsMeasurement
        requireNotNull(averageX1)
        val averageX2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageXAsMeasurement(averageX2))
        assertEquals(averageX1, averageX2)
        assertEquals(averageX, averageX1.value.toDouble(), 0.0)

        val averageY = estimator.averageY
        requireNotNull(averageY)
        assertEquals(gravity.gy, averageY, ABSOLUTE_ERROR)

        val averageY1 = estimator.averageYAsMeasurement
        requireNotNull(averageY1)
        val averageY2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageYAsMeasurement(averageY2))
        assertEquals(averageY1, averageY2)
        assertEquals(averageY, averageY1.value.toDouble(), 0.0)

        val averageZ = estimator.averageZ
        requireNotNull(averageZ)
        assertEquals(gravity.gz, averageZ, ABSOLUTE_ERROR)

        val averageZ1 = estimator.averageZAsMeasurement
        requireNotNull(averageZ1)
        val averageZ2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageZAsMeasurement(averageZ2))
        assertEquals(averageZ1, averageZ2)
        assertEquals(averageZ, averageZ1.value.toDouble(), 0.0)

        val averageTriad1 = estimator.averageTriad
        requireNotNull(averageTriad1)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageTriad1.unit)
        assertEquals(averageX, averageTriad1.valueX, 0.0)
        assertEquals(averageY, averageTriad1.valueY, 0.0)
        assertEquals(averageZ, averageTriad1.valueZ, 0.0)
        val averageTriad2 = AccelerationTriad()
        assertTrue(estimator.getAverageTriad(averageTriad2))
        assertEquals(averageTriad1, averageTriad2)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(gravity.norm, averageNorm, ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)

        val varianceX = estimator.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, ABSOLUTE_ERROR)

        val varianceY = estimator.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, ABSOLUTE_ERROR)

        val varianceZ = estimator.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, ABSOLUTE_ERROR)

        val standardDeviationX = estimator.standardDeviationX
        requireNotNull(standardDeviationX)
        assertEquals(0.0, standardDeviationX, ABSOLUTE_ERROR)
        val standardDeviationX1 = estimator.standardDeviationXAsMeasurement
        requireNotNull(standardDeviationX1)
        val standardDeviationX2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationXAsMeasurement(standardDeviationX2))
        assertEquals(standardDeviationX1, standardDeviationX2)
        assertEquals(standardDeviationX, standardDeviationX1.value.toDouble(), 0.0)

        val standardDeviationY = estimator.standardDeviationY
        requireNotNull(standardDeviationY)
        assertEquals(0.0, standardDeviationY, ABSOLUTE_ERROR)
        val standardDeviationY1 = estimator.standardDeviationYAsMeasurement
        requireNotNull(standardDeviationY1)
        val standardDeviationY2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationYAsMeasurement(standardDeviationY2))
        assertEquals(standardDeviationY1, standardDeviationY2)
        assertEquals(standardDeviationY, standardDeviationY1.value.toDouble(), 0.0)

        val standardDeviationZ = estimator.standardDeviationZ
        requireNotNull(standardDeviationZ)
        assertEquals(0.0, standardDeviationZ, ABSOLUTE_ERROR)
        val standardDeviationZ1 = estimator.standardDeviationZAsMeasurement
        requireNotNull(standardDeviationZ1)
        val standardDeviationZ2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationZAsMeasurement(standardDeviationZ2))
        assertEquals(standardDeviationZ1, standardDeviationZ2)
        assertEquals(standardDeviationZ, standardDeviationZ1.value.toDouble(), 0.0)

        val standardDeviationTriad1 = estimator.standardDeviationTriad
        requireNotNull(standardDeviationTriad1)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, standardDeviationTriad1.unit)
        assertEquals(standardDeviationX, standardDeviationTriad1.valueX, 0.0)
        assertEquals(standardDeviationY, standardDeviationTriad1.valueY, 0.0)
        assertEquals(standardDeviationZ, standardDeviationTriad1.valueZ, 0.0)
        val standardDeviationTriad2 = AccelerationTriad()
        assertTrue(estimator.getStandardDeviationTriad(standardDeviationTriad2))
        assertEquals(standardDeviationTriad1, standardDeviationTriad2)

        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        assertEquals(0.0, standardDeviationNorm, ABSOLUTE_ERROR)
        val standardDeviationNorm1 = estimator.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNorm1)
        val standardDeviationNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm2))
        assertEquals(standardDeviationNorm1, standardDeviationNorm2)
        assertEquals(standardDeviationNorm, standardDeviationNorm1.value.toDouble(), 0.0)

        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)
        assertEquals(0.0, averageStandardDeviation, ABSOLUTE_ERROR)
        val averageStandardDeviation1 = estimator.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviation1)
        val averageStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation2))
        assertEquals(averageStandardDeviation1, averageStandardDeviation2)
        assertEquals(averageStandardDeviation, averageStandardDeviation1.value.toDouble(), 0.0)

        val psdX = estimator.psdX
        requireNotNull(psdX)
        assertEquals(0.0, psdX, 0.0)

        val psdY = estimator.psdY
        requireNotNull(psdY)
        assertEquals(0.0, psdY, 0.0)

        val psdZ = estimator.psdZ
        requireNotNull(psdZ)
        assertEquals(0.0, psdZ, 0.0)

        val rootPsdX = estimator.rootPsdX
        requireNotNull(rootPsdX)
        assertEquals(0.0, rootPsdX, 0.0)

        val rootPsdY = estimator.rootPsdY
        requireNotNull(rootPsdY)
        assertEquals(0.0, rootPsdY, 0.0)

        val rootPsdZ = estimator.rootPsdZ
        requireNotNull(rootPsdZ)
        assertEquals(0.0, rootPsdZ, 0.0)

        val averageNoisePsd = estimator.averageNoisePsd
        requireNotNull(averageNoisePsd)
        assertEquals(0.0, averageNoisePsd, 0.0)

        val noiseRootPsdNorm = estimator.noiseRootPsdNorm
        requireNotNull(noiseRootPsdNorm)
        assertEquals(0.0, noiseRootPsdNorm, 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getAverageTimeIntervalAsTime(averageTimeInterval2))
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)

        val timeIntervalVariance = estimator.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertTrue(timeIntervalVariance > 0.0)

        val timeIntervalStandardDeviation = estimator.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation)
        assertTrue(timeIntervalStandardDeviation > 0.0)
        val timeIntervalStandardDeviation1 = estimator.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviation1)
        val timeIntervalStandardDeviation2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2))
        assertEquals(timeIntervalStandardDeviation1, timeIntervalStandardDeviation2)
        assertEquals(
            timeIntervalStandardDeviation,
            timeIntervalStandardDeviation1.value.toDouble(),
            0.0
        )

        val elapsedTimeNanos = estimator.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        val elapsedTime1 = estimator.elapsedTime
        val elapsedTime2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(elapsedTime2)
        assertEquals(elapsedTime1, elapsedTime2)
        assertEquals(elapsedTimeNanos.toDouble(), elapsedTime1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, elapsedTime1.unit)
    }

    private fun getGravity(): ECEFGravity {
        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val longitude =
            Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
        val nedPosition = NEDPosition(latitude, longitude, height)
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition, nedVelocity,
            ecefPosition, ecefVelocity
        )
        return ECEFGravityEstimator.estimateGravityAndReturnNew(
            ecefPosition.x,
            ecefPosition.y,
            ecefPosition.z
        )
    }

    private companion object {
        const val MAX_SAMPLES = 100

        const val MAX_DURATION_MILLIS = 1000L

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -50.0
        const val MAX_HEIGHT = 400.0

        const val MILLIS_TO_NANOS = 1000000

        const val ABSOLUTE_ERROR = 1e-6

        const val LARGE_ABSOLUTE_ERROR = 5e-5
    }
}