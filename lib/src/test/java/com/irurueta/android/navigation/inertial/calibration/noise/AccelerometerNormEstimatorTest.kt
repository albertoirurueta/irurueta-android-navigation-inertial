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
package com.irurueta.android.navigation.inertial.calibration.noise

import android.content.Context
import android.hardware.Sensor
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.ECEFGravity
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class AccelerometerNormEstimatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(BaseAccumulatedEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(BaseAccumulatedEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(BaseAccumulatedEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            -1
        )
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedEstimator.DEFAULT_MAX_DURATION_MILLIS,
            estimator.maxDurationMillis
        )
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            -1L
        )
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val completedListener =
            mockk<AccumulatedMeasurementEstimator
            .OnEstimationCompletedListener<AccelerometerNormEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val completedListener =
            mockk<AccumulatedMeasurementEstimator
            .OnEstimationCompletedListener<AccelerometerNormEstimator>>()
        val unreliableListener =
            mockk<AccumulatedMeasurementEstimator
            .OnUnreliableListener<AccelerometerNormEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener,
            unreliableListener
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertNull(estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val completedListener =
            mockk<AccumulatedMeasurementEstimator
            .OnEstimationCompletedListener<AccelerometerNormEstimator>>()
        val unreliableListener =
            mockk<AccumulatedMeasurementEstimator
            .OnUnreliableListener<AccelerometerNormEstimator>>()
        val measurementListener = mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener,
            unreliableListener,
            measurementListener
        )

        // check values
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertSame(measurementListener, estimator.measurementListener)
        assertNull(estimator.sensor)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageNorm)
        assertNull(estimator.averageNormAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageNormAsMeasurement(acceleration))
        assertNull(estimator.normVariance)
        assertNull(estimator.normStandardDeviation)
        assertNull(estimator.normStandardDeviationAsMeasurement)
        assertFalse(estimator.getNormStandardDeviationAsMeasurement(acceleration))
        assertNull(estimator.psd)
        assertNull(estimator.rootPsd)
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
        val estimator = AccelerometerNormEstimator(context)

        // check default value
        assertNull(estimator.completedListener)

        // set new value
        val completedListener = mockk<AccumulatedMeasurementEstimator
        .OnEstimationCompletedListener<AccelerometerNormEstimator>>()
        estimator.completedListener = completedListener

        // check
        assertSame(completedListener, estimator.completedListener)
    }

    @Test
    fun unreliableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check default value
        assertNull(estimator.unreliableListener)

        // set new value
        val unreliableListener = mockk<AccumulatedMeasurementEstimator
        .OnUnreliableListener<AccelerometerNormEstimator>>()
        estimator.unreliableListener = unreliableListener

        // check
        assertSame(unreliableListener, estimator.unreliableListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check default value
        assertNull(estimator.measurementListener)

        // set new value
        val measurementListener = mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        estimator.measurementListener = measurementListener

        // check
        assertSame(measurementListener, estimator.measurementListener)
    }

    @Test
    fun sensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check
        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        val sensor = mockk<Sensor>()
        every { collectorSpy.sensor }.returns(sensor)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertSame(sensor, estimator.sensor)
    }

    @Test
    fun start_whenSensorAvailable_startsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)
        verify(exactly = 1) { collectorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenSensorUnavailable_startsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("collector", collectorSpy)

        assertFalse(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenDefaultStopMode_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("collector", collectorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedEstimator::class,
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
        val estimator = AccelerometerNormEstimator(context, stopMode = StopMode.MAX_DURATION_ONLY)

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("collector", collectorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedEstimator::class,
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
        val estimator = AccelerometerNormEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("collector", collectorSpy)

        setPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "resultUnreliable",
            true
        )
        assertTrue(estimator.resultUnreliable)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        assertFalse(estimator.resultUnreliable)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        // start again
        estimator.start()
    }

    @Test
    fun stop_whenAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        assertSame(context, collector.context)
        assertEquals(collector.sensorType, estimator.sensorType)
        assertEquals(collector.sensorDelay, estimator.sensorDelay)
        assertNotNull(collector.measurementListener)
        assertNotNull(collector.accuracyChangedListener)

        val collectorSpy = spyk(collector)
        every { collectorSpy.start() }.returns(true)
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
        val estimator = AccelerometerNormEstimator(context)

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
    fun onMeasurement_whenMeasurementListener_notifies() {
        val measurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AccelerometerNoiseEstimator(context, measurementListener = measurementListener)

        val accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(accelerometerMeasurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val bx = 1.0f
        val by = 2.0f
        val bz = 3.0f
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        accelerometerMeasurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)

        verify(exactly = 1) {
            measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)
        }
    }

    @Test
    fun onMeasurement_whenUnreliableAccuracy_makesResultUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        assertFalse(estimator.resultUnreliable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
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
        val estimator = AccelerometerNormEstimator(context)

        val initialTimestampNanos1: Long? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "initialTimestampNanos"
        )
        requireNotNull(initialTimestampNanos1)
        assertEquals(0L, initialTimestampNanos1)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        val initialTimestampNanos2: Long? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "initialTimestampNanos"
        )
        requireNotNull(initialTimestampNanos2)
        assertEquals(timestamp1, initialTimestampNanos2)

        // calling again with another timestamp, makes no effect
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        val initialTimestampNanos3: Long? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "initialTimestampNanos"
        )
        requireNotNull(initialTimestampNanos3)
        assertEquals(timestamp1, initialTimestampNanos3)
    }

    @Test
    fun onMeasurement_addsMeasurementToNoiseAndTimeIntervalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(
                BaseAccumulatedEstimator::class,
                estimator,
                "timeIntervalEstimator"
            )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        verify(exactly = 1) { noiseEstimatorSpy.addMeasurement(a) }
        verify(exactly = 0) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }

        // set another measurement
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        verify(exactly = 2) { noiseEstimatorSpy.addMeasurement(a) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }
    }

    @Test
    fun onMeasurement_updatesEndTimestampNanos() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        val endTimestampNanos1: Long? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "endTimestampNanos"
        )
        requireNotNull(endTimestampNanos1)
        assertEquals(0L, endTimestampNanos1)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp, accuracy)

        val endTimestampNanos2: Long? = getPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "endTimestampNanos"
        )
        requireNotNull(endTimestampNanos2)
        assertEquals(timestamp, endTimestampNanos2)
    }

    @Test
    fun onMeasurement_increasesNumberOfProcessedMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check default value
        assertEquals(0, estimator.numberOfProcessedMeasurements)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
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
        val estimator = AccelerometerNormEstimator(context, stopMode = StopMode.MAX_SAMPLES_ONLY)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
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
        checkResultMaxSamples(estimator, a)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndListener() {
        val completedListener = mockk<AccumulatedMeasurementEstimator
        .OnEstimationCompletedListener<AccelerometerNormEstimator>>(
            relaxUnitFun = true
        )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
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
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
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
                timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS,
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
        checkResultMaxSamples(estimator, a)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
            context,
            stopMode = StopMode.MAX_DURATION_ONLY,
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
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, a)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndListener() {
        val completedListener = mockk<AccumulatedMeasurementEstimator
        .OnEstimationCompletedListener<AccelerometerNormEstimator>>(
            relaxUnitFun = true
        )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
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
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, a)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            AccelerometerNormEstimator(context, stopMode = StopMode.MAX_SAMPLES_OR_DURATION)

        val collector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, a)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndListener() {
        val completedListener = mockk<AccumulatedMeasurementEstimator
        .OnEstimationCompletedListener<AccelerometerNormEstimator>>(
            relaxUnitFun = true
        )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(
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
            estimator.getPrivateProperty("accelerometerMeasurementListener")
        requireNotNull(measurementListener)

        val gravity = getGravity()
        val ax = gravity.gx.toFloat()
        val ay = gravity.gy.toFloat()
        val az = gravity.gz.toFloat()
        val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(ax, ay, az, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, a)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndNoListener_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                BaseAccumulatedEstimator::class,
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
            mockk<AccumulatedMeasurementEstimator.OnUnreliableListener<AccelerometerNormEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                BaseAccumulatedEstimator::class,
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
            mockk<AccumulatedMeasurementEstimator.OnUnreliableListener<AccelerometerNormEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AccelerometerNormEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                BaseAccumulatedEstimator::class,
                estimator,
                "accuracyChangedListener"
            )
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.MEDIUM)

        // check
        assertFalse(estimator.resultUnreliable)
        verify(exactly = 0) { unreliableListener.onUnreliable(estimator) }
    }

    private fun checkResultMaxSamples(estimator: AccelerometerNormEstimator, a: Double) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(a, averageNorm, ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageNorm1.unit)

        val normVariance = estimator.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val normStandardDeviation = estimator.normStandardDeviation
        requireNotNull(normStandardDeviation)
        assertEquals(0.0, normStandardDeviation, ABSOLUTE_ERROR)
        val normStandardDeviation1 = estimator.normStandardDeviationAsMeasurement
        requireNotNull(normStandardDeviation1)
        val normStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getNormStandardDeviationAsMeasurement(normStandardDeviation2))
        assertEquals(normStandardDeviation1, normStandardDeviation2)
        assertEquals(
            normStandardDeviation,
            normStandardDeviation1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, normStandardDeviation1.unit)

        val psd = estimator.psd
        requireNotNull(psd)
        assertTrue(psd > 0.0)

        val rootPsd = estimator.rootPsd
        requireNotNull(rootPsd)
        assertTrue(rootPsd > 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getAverageTimeIntervalAsTime(averageTimeInterval2))
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.unit)

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
        assertEquals(TimeUnit.SECOND, timeIntervalStandardDeviation1.unit)

        val elapsedTimeNanos = estimator.elapsedTimeNanos
        assertTrue(elapsedTimeNanos > 0L)
        val elapsedTime1 = estimator.elapsedTime
        val elapsedTime2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getElapsedTime(elapsedTime2)
        assertEquals(elapsedTime1, elapsedTime2)
        assertEquals(elapsedTimeNanos.toDouble(), elapsedTime1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.NANOSECOND, elapsedTime1.unit)
    }

    private fun checkResultMaxDuration(estimator: AccelerometerNormEstimator, a: Double) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(a, averageNorm, ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, averageNorm1.unit)

        val normVariance = estimator.normVariance
        requireNotNull(normVariance)
        assertEquals(0.0, normVariance, ABSOLUTE_ERROR)

        val normStandardDeviation = estimator.normStandardDeviation
        requireNotNull(normStandardDeviation)
        assertEquals(0.0, normStandardDeviation, ABSOLUTE_ERROR)
        val normStandardDeviation1 = estimator.normStandardDeviationAsMeasurement
        requireNotNull(normStandardDeviation1)
        val normStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(estimator.getNormStandardDeviationAsMeasurement(normStandardDeviation2))
        assertEquals(normStandardDeviation1, normStandardDeviation2)
        assertEquals(
            normStandardDeviation,
            normStandardDeviation1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, normStandardDeviation1.unit)

        val psd = estimator.psd
        requireNotNull(psd)
        assertEquals(0.0, psd, 0.0)

        val rootPsd = estimator.rootPsd
        requireNotNull(rootPsd)
        assertEquals(0.0, rootPsd, 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        assertTrue(estimator.getAverageTimeIntervalAsTime(averageTimeInterval2))
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.unit)

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
        assertEquals(TimeUnit.SECOND, timeIntervalStandardDeviation1.unit)

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
        return GravityHelper.getGravityForPosition(nedPosition)
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

        const val TIME_INTERVAL_MILLIS = 20L

        const val MILLIS_TO_NANOS = 1000000

        const val ABSOLUTE_ERROR = 1e-12

        const val LARGE_ABSOLUTE_ERROR = 6e-4
    }
}