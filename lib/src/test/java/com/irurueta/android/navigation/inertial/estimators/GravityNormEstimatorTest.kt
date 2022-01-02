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
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
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
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
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
class GravityNormEstimatorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        // check default values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(GravityNormEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(GravityNormEstimator.DEFAULT_MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        val estimator = GravityNormEstimator(context, SensorDelay.NORMAL)

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(GravityNormEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(GravityNormEstimator.DEFAULT_MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        GravityNormEstimator(context, SensorDelay.NORMAL, -1)
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context, SensorDelay.NORMAL, MAX_SAMPLES)

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(GravityNormEstimator.DEFAULT_MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        GravityNormEstimator(context, SensorDelay.NORMAL, MAX_SAMPLES, -1L)
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(context, SensorDelay.NORMAL, MAX_SAMPLES, MAX_DURATION_MILLIS)

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        val estimator =
            GravityNormEstimator(
                context,
                SensorDelay.NORMAL,
                MAX_SAMPLES,
                MAX_DURATION_MILLIS,
                GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY
            )

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertNull(estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        val completedListener = mockk<GravityNormEstimator.OnEstimationCompletedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                SensorDelay.NORMAL,
                MAX_SAMPLES,
                MAX_DURATION_MILLIS,
                GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY,
                completedListener
            )

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertNull(estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        val completedListener = mockk<GravityNormEstimator.OnEstimationCompletedListener>()
        val unreliableListener = mockk<GravityNormEstimator.OnUnreliableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                SensorDelay.NORMAL,
                MAX_SAMPLES,
                MAX_DURATION_MILLIS,
                GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY,
                completedListener,
                unreliableListener
            )

        // check values
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(MAX_SAMPLES, estimator.maxSamples)
        assertEquals(MAX_DURATION_MILLIS, estimator.maxDurationMillis)
        assertEquals(GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY, estimator.stopMode)
        assertSame(completedListener, estimator.completedListener)
        assertSame(unreliableListener, estimator.unreliableListener)
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageGravityNorm)
        assertNull(estimator.averageGravityNormAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(estimator.getAverageGravityNormAsAcceleration(acceleration))
        assertNull(estimator.gravityNormVariance)
        assertNull(estimator.gravityNormStandardDeviation)
        assertNull(estimator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(estimator.getGravityNormStandardDeviationAsAcceleration(acceleration))
        assertNull(estimator.gravityPsd)
        assertNull(estimator.gravityRootPsd)
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
        val estimator = GravityNormEstimator(context)

        // check default value
        assertNull(estimator.completedListener)

        // set new value
        val completedListener = mockk<GravityNormEstimator.OnEstimationCompletedListener>()
        estimator.completedListener = completedListener

        // check
        assertSame(completedListener, estimator.completedListener)
    }

    @Test
    fun unreliableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        // check default value
        assertNull(estimator.unreliableListener)

        // set new value
        val unreliableListener = mockk<GravityNormEstimator.OnUnreliableListener>()
        estimator.unreliableListener = unreliableListener

        // check
        assertSame(unreliableListener, estimator.unreliableListener)
    }

    @Test
    fun start_startsGravitySensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        assertSame(context, gravityCollector.context)
        assertEquals(gravityCollector.sensorDelay, estimator.sensorDelay)
        assertNotNull(gravityCollector.measurementListener)
        assertNotNull(gravityCollector.accuracyChangedListener)

        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)
        verify(exactly = 1) { gravityCollectorSpy.start() }
    }

    @Test
    fun start_whenDefaultStopMode_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            estimator.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        estimator.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

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
        val estimator = GravityNormEstimator(
            context,
            stopMode = GravityNormEstimator.StopMode.MAX_DURATION_ONLY
        )

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            estimator.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        estimator.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        verify(exactly = 1) { noiseEstimatorSpy.reset() }
        assertEquals(0.0, noiseEstimatorSpy.timeInterval, 0.0)

        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }
        assertEquals(Integer.MAX_VALUE, timeIntervalEstimatorSpy.totalSamples)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        // start again
        estimator.start()
    }

    @Test
    fun stop_whenAlreadyStarted_stopsGravitySensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        assertSame(context, gravityCollector.context)
        assertEquals(gravityCollector.sensorDelay, estimator.sensorDelay)
        assertNotNull(gravityCollector.measurementListener)
        assertNotNull(gravityCollector.accuracyChangedListener)

        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)
        verify(exactly = 1) { gravityCollectorSpy.start() }

        // stop
        estimator.stop()

        assertFalse(estimator.running)
        verify(exactly = 1) { gravityCollectorSpy.stop() }
    }

    @Test
    fun stop_whenNotAlreadyStarted_stopsGravitySensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        assertSame(context, gravityCollector.context)
        assertEquals(gravityCollector.sensorDelay, estimator.sensorDelay)
        assertNotNull(gravityCollector.measurementListener)
        assertNotNull(gravityCollector.accuracyChangedListener)

        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        assertFalse(estimator.running)

        // stop
        estimator.stop()

        assertFalse(estimator.running)
        verify(exactly = 1) { gravityCollectorSpy.stop() }
    }

    @Test
    fun onMeasurement_whenUnreliableAccuracy_makesResultUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        assertFalse(estimator.resultUnreliable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.UNRELIABLE

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp, accuracy)

        assertTrue(estimator.resultUnreliable)
    }

    @Test
    fun onMeasurement_whenFirstMeasurement_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val initialTimestampNanos1: Long? = estimator.getPrivateProperty("initialTimestampNanos")
        requireNotNull(initialTimestampNanos1)
        assertEquals(0L, initialTimestampNanos1)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val timestamp2 = SystemClock.elapsedRealtimeNanos() + 1000000
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp1, accuracy)

        val initialTimestampNanos2: Long? = estimator.getPrivateProperty("initialTimestampNanos")
        requireNotNull(initialTimestampNanos2)
        assertEquals(timestamp1, initialTimestampNanos2)

        // calling again with another timestamp, makes no effect
        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp2, accuracy)

        val initialTimestampNanos3: Long? = estimator.getPrivateProperty("initialTimestampNanos")
        requireNotNull(initialTimestampNanos3)
        assertEquals(timestamp1, initialTimestampNanos3)
    }

    @Test
    fun onMeasurement_addsMeasurementToNoiseAndTimeIntervalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val noiseEstimator: AccumulatedAccelerationMeasurementNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            estimator.getPrivateProperty("timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        estimator.setPrivateProperty("timeIntervalEstimator", timeIntervalEstimatorSpy)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp, accuracy)

        verify(exactly = 1) { noiseEstimatorSpy.addMeasurement(g) }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(0.0) }
    }

    @Test
    fun onMeasurement_updatesEndTimestampNanos() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        val endTimestampNanos1: Long? = estimator.getPrivateProperty("endTimestampNanos")
        requireNotNull(endTimestampNanos1)
        assertEquals(0L, endTimestampNanos1)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp, accuracy)

        val endTimestampNanos2: Long? = estimator.getPrivateProperty("endTimestampNanos")
        requireNotNull(endTimestampNanos2)
        assertEquals(timestamp, endTimestampNanos2)
    }

    @Test
    fun onMeasurement_increasesNumberOfProcessedMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        // check default value
        assertEquals(0, estimator.numberOfProcessedMeasurements)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        // set measurement
        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp, accuracy)

        // check
        assertEquals(1, estimator.numberOfProcessedMeasurements)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(context, stopMode = GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY)

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        for (i in 1..maxSamples) {
            gravityMeasurementListener.onMeasurement(
                gx,
                gy,
                gz,
                g,
                timestamp + i * MILLIS_TO_NANOS,
                accuracy
            )

            assertEquals(i, estimator.numberOfProcessedMeasurements)
        }

        assertEquals(maxSamples, estimator.numberOfProcessedMeasurements)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }

        // check result
        checkResultMaxSamples(estimator, g)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndListener() {
        val completedListener =
            mockk<GravityNormEstimator.OnEstimationCompletedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                stopMode = GravityNormEstimator.StopMode.MAX_SAMPLES_ONLY,
                completedListener = completedListener
            )

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        for (i in 1..maxSamples) {
            gravityMeasurementListener.onMeasurement(
                gx,
                gy,
                gz,
                g,
                timestamp + i * MILLIS_TO_NANOS,
                accuracy
            )

            assertEquals(i, estimator.numberOfProcessedMeasurements)
        }

        assertEquals(maxSamples, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }
        verify(exactly = 1) { completedListener.onEstimatedCompleted(estimator) }

        // check result
        checkResultMaxSamples(estimator, g)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                stopMode = GravityNormEstimator.StopMode.MAX_DURATION_ONLY
            )

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, g)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndListener() {
        val completedListener =
            mockk<GravityNormEstimator.OnEstimationCompletedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                stopMode = GravityNormEstimator.StopMode.MAX_DURATION_ONLY,
                completedListener = completedListener
            )

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * 1000000

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimatedCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, g)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                stopMode = GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION
            )

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * 1000000

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, g)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationOnlyAndListener() {
        val completedListener =
            mockk<GravityNormEstimator.OnEstimationCompletedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityNormEstimator(
                context,
                stopMode = GravityNormEstimator.StopMode.MAX_SAMPLES_OR_DURATION,
                completedListener = completedListener
            )

        val gravityCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravityCollector")
        requireNotNull(gravityCollector)
        val gravityCollectorSpy = spyk(gravityCollector)
        estimator.setPrivateProperty("gravityCollector", gravityCollectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("gravityMeasurementListener")
        requireNotNull(gravityMeasurementListener)

        val gravity = getGravity()
        val gx = gravity.gx.toFloat()
        val gy = gravity.gy.toFloat()
        val gz = gravity.gz.toFloat()
        val g = gravity.norm
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.MEDIUM

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp2 = timestamp1 + maxDurationMillis * 1000000

        gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { gravityCollectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimatedCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, g)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndNoListener_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val gravityAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            estimator.getPrivateProperty("gravityAccuracyChangedListener")
        requireNotNull(gravityAccuracyChangedListener)

        gravityAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        assertTrue(estimator.resultUnreliable)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndListener_setsResultAsUnreliable() {
        val unreliableListener =
            mockk<GravityNormEstimator.OnUnreliableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val gravityAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            estimator.getPrivateProperty("gravityAccuracyChangedListener")
        requireNotNull(gravityAccuracyChangedListener)

        gravityAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.UNRELIABLE)

        // check
        assertTrue(estimator.resultUnreliable)
        verify(exactly = 1) { unreliableListener.onUnreliable(estimator) }
    }

    @Test
    fun onAccuracyChanged_whenNotUnreliable_makesNoAction() {
        val unreliableListener =
            mockk<GravityNormEstimator.OnUnreliableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityNormEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val gravityAccuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            estimator.getPrivateProperty("gravityAccuracyChangedListener")
        requireNotNull(gravityAccuracyChangedListener)

        gravityAccuracyChangedListener.onAccuracyChanged(SensorAccuracy.MEDIUM)

        // check
        assertFalse(estimator.resultUnreliable)
        verify(exactly = 0) { unreliableListener.onUnreliable(estimator) }
    }

    private fun checkResultMaxSamples(estimator: GravityNormEstimator, g: Double) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageGravityNorm = estimator.averageGravityNorm
        requireNotNull(averageGravityNorm)
        assertEquals(g, averageGravityNorm, ABSOLUTE_ERROR)

        val averageGravityNorm1 = estimator.averageGravityNormAsAcceleration
        requireNotNull(averageGravityNorm1)
        val averageGravityNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getAverageGravityNormAsAcceleration(averageGravityNorm2)
        assertEquals(averageGravityNorm1, averageGravityNorm2)
        assertEquals(averageGravityNorm, averageGravityNorm1.value.toDouble(), 0.0)

        val gravityNormVariance = estimator.gravityNormVariance
        requireNotNull(gravityNormVariance)
        assertEquals(0.0, gravityNormVariance, ABSOLUTE_ERROR)

        val gravityNormStandardDeviation = estimator.gravityNormStandardDeviation
        requireNotNull(gravityNormStandardDeviation)
        assertEquals(0.0, gravityNormStandardDeviation, ABSOLUTE_ERROR)
        val gravityNormStandardDeviation1 = estimator.gravityNormStandardDeviationAsAcceleration
        requireNotNull(gravityNormStandardDeviation1)
        val gravityNormStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getGravityNormStandardDeviationAsAcceleration(gravityNormStandardDeviation2)
        assertEquals(gravityNormStandardDeviation1, gravityNormStandardDeviation2)
        assertEquals(
            gravityNormStandardDeviation,
            gravityNormStandardDeviation1.value.toDouble(),
            0.0
        )

        val gravityPsd = estimator.gravityPsd
        requireNotNull(gravityPsd)
        assertTrue(gravityPsd > 0.0)

        val gravityRootPsd = estimator.gravityRootPsd
        requireNotNull(gravityRootPsd)
        assertTrue(gravityRootPsd > 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval2)
        assertEquals(averageTimeInterval1, averageTimeInterval2)
        assertEquals(averageTimeInterval, averageTimeInterval1.value.toDouble(), 0.0)

        val timeIntervalVariance = estimator.timeIntervalVariance
        requireNotNull(timeIntervalVariance)
        assertEquals(0.0, timeIntervalVariance, SMALL_ABSOLUTE_ERROR)

        val timeIntervalStandardDeviation = estimator.timeIntervalStandardDeviation
        requireNotNull(timeIntervalStandardDeviation)
        assertEquals(0.0, timeIntervalStandardDeviation, SMALL_ABSOLUTE_ERROR)
        val timeIntervalStandardDeviation1 = estimator.timeIntervalStandardDeviationAsTime
        requireNotNull(timeIntervalStandardDeviation1)
        val timeIntervalStandardDeviation2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2)
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

    private fun checkResultMaxDuration(estimator: GravityNormEstimator, g: Double) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageGravityNorm = estimator.averageGravityNorm
        requireNotNull(averageGravityNorm)
        assertEquals(g, averageGravityNorm, ABSOLUTE_ERROR)

        val averageGravityNorm1 = estimator.averageGravityNormAsAcceleration
        requireNotNull(averageGravityNorm1)
        val averageGravityNorm2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getAverageGravityNormAsAcceleration(averageGravityNorm2)
        assertEquals(averageGravityNorm1, averageGravityNorm2)
        assertEquals(averageGravityNorm, averageGravityNorm1.value.toDouble(), 0.0)

        val gravityNormVariance = estimator.gravityNormVariance
        requireNotNull(gravityNormVariance)
        assertEquals(0.0, gravityNormVariance, ABSOLUTE_ERROR)

        val gravityNormStandardDeviation = estimator.gravityNormStandardDeviation
        requireNotNull(gravityNormStandardDeviation)
        assertEquals(0.0, gravityNormStandardDeviation, ABSOLUTE_ERROR)
        val gravityNormStandardDeviation1 = estimator.gravityNormStandardDeviationAsAcceleration
        requireNotNull(gravityNormStandardDeviation1)
        val gravityNormStandardDeviation2 =
            Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        estimator.getGravityNormStandardDeviationAsAcceleration(gravityNormStandardDeviation2)
        assertEquals(gravityNormStandardDeviation1, gravityNormStandardDeviation2)
        assertEquals(
            gravityNormStandardDeviation,
            gravityNormStandardDeviation1.value.toDouble(),
            0.0
        )

        val gravityPsd = estimator.gravityPsd
        requireNotNull(gravityPsd)
        assertEquals(gravityPsd, 0.0, 0.0)

        val gravityRootPsd = estimator.gravityRootPsd
        requireNotNull(gravityRootPsd)
        assertEquals(gravityRootPsd, 0.0, 0.0)

        val averageTimeInterval = estimator.averageTimeInterval
        requireNotNull(averageTimeInterval)
        assertTrue(averageTimeInterval > 0.0)
        val averageTimeInterval1 = estimator.averageTimeIntervalAsTime
        requireNotNull(averageTimeInterval1)
        val averageTimeInterval2 = Time(0.0, TimeUnit.NANOSECOND)
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval2)
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
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStandardDeviation2)
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

    companion object {
        const val MAX_SAMPLES = 100

        const val MAX_DURATION_MILLIS = 1000L

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -50.0
        const val MAX_HEIGHT = 400.0

        const val MILLIS_TO_NANOS = 1000000

        const val ABSOLUTE_ERROR = 1e-12

        const val SMALL_ABSOLUTE_ERROR = 5e-5
    }
}