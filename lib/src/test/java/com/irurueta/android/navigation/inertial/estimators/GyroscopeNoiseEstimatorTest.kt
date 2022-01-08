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
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
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
class GyroscopeNoiseEstimatorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        // check default values
        assertSame(context, estimator.context)
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, estimator.sensorType)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(BaseAccumulatedEstimator.DEFAULT_MAX_SAMPLES, estimator.maxSamples)
        assertEquals(
            BaseAccumulatedEstimator.DEFAULT_MAX_DURATION_MILLIS,
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
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            -1
        )
    }

    @Test
    fun constructor_whenMaxSamples_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        assertFalse(estimator.running)
        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)
        assertNull(estimator.averageX)
        assertNull(estimator.averageXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            -1L
        )
    }

    @Test
    fun constructor_whenMaxDurationMillis_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        .OnEstimationCompletedListener<GyroscopeNoiseEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            MAX_SAMPLES,
            MAX_DURATION_MILLIS,
            StopMode.MAX_SAMPLES_ONLY,
            completedListener
        )

        // check default values
        assertSame(context, estimator.context)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        .OnEstimationCompletedListener<GyroscopeNoiseEstimator>>()
        val unreliableListener = mockk<AccumulatedTriadEstimator
        .OnUnreliableListener<GyroscopeNoiseEstimator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
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
        val estimator = GyroscopeNoiseEstimator(context)

        // check default value
        assertNull(estimator.completedListener)

        // set new value
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<GyroscopeNoiseEstimator>>()
        estimator.completedListener = completedListener

        // check
        assertSame(completedListener, estimator.completedListener)
    }

    @Test
    fun unreliableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        // check default value
        assertNull(estimator.unreliableListener)

        // set new value
        val unreliableListener =
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<GyroscopeNoiseEstimator>>()
        estimator.unreliableListener = unreliableListener

        // check
        assertSame(unreliableListener, estimator.unreliableListener)
    }

    @Test
    fun start_startsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        val collector: GyroscopeSensorCollector? =
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
        val estimator = GyroscopeNoiseEstimator(context)

        val noiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "timeIntervalEstimator")
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
        val estimator = GyroscopeNoiseEstimator(context, stopMode = StopMode.MAX_DURATION_ONLY)

        val noiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "timeIntervalEstimator")
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
        val estimator = GyroscopeNoiseEstimator(context)

        setPrivateProperty(BaseAccumulatedEstimator::class, estimator, "resultUnreliable", true)
        assertTrue(estimator.resultUnreliable)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        assertFalse(estimator.resultUnreliable)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        assertFalse(estimator.running)

        estimator.start()

        assertTrue(estimator.running)

        // start again
        estimator.start()
    }

    @Test
    fun stop_whenAlreadyStarted_stopsSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        val collector: GyroscopeSensorCollector? =
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
        val estimator = GyroscopeNoiseEstimator(context)

        val collector: GyroscopeSensorCollector? =
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
        val estimator = GyroscopeNoiseEstimator(context)

        assertFalse(estimator.resultUnreliable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.UNRELIABLE

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp, accuracy)

        assertTrue(estimator.resultUnreliable)
        assertEquals(
            ECEFKinematicsEstimator.EARTH_ROTATION_RATE,
            kinematics.angularRateNorm,
            SMALL_ABSOLUTE_ERROR
        )
    }

    @Test
    fun onMeasurement_whenFirstMeasurement_setsInitialTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        val initialTimestampNanos1: Long? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos1)
        assertEquals(0L, initialTimestampNanos1)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        val accuracy = SensorAccuracy.HIGH

        // set measurement
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        val initialTimestampNanos2: Long? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos2)
        assertEquals(timestamp1, initialTimestampNanos2)

        // calling again with another timestamp, makes no effect
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        val initialTimestampNanos3: Long? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "initialTimestampNanos")
        requireNotNull(initialTimestampNanos3)
        assertEquals(timestamp1, initialTimestampNanos3)
    }

    @Test
    fun onMeasurement_addsMeasurementToNoiseAndTimeIntervalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        val noiseEstimator: AccumulatedAngularSpeedTriadNoiseEstimator? =
            estimator.getPrivateProperty("noiseEstimator")
        requireNotNull(noiseEstimator)
        val noiseEstimatorSpy = spyk(noiseEstimator)
        estimator.setPrivateProperty("noiseEstimator", noiseEstimatorSpy)

        val timeIntervalEstimator: TimeIntervalEstimator? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "timeIntervalEstimator")
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseAccumulatedEstimator::class,
            estimator,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        // set measurement
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        verify(exactly = 1) {
            noiseEstimatorSpy.addTriad(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble()
            )
        }
        verify(exactly = 0) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }

        // set another measurement
        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        verify(exactly = 2) {
            noiseEstimatorSpy.addTriad(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble()
            )
        }
        verify(exactly = 1) { timeIntervalEstimatorSpy.addTimestamp(any<Double>()) }

        assertEquals(
            ECEFKinematicsEstimator.EARTH_ROTATION_RATE,
            kinematics.angularRateNorm,
            SMALL_ABSOLUTE_ERROR
        )
    }

    @Test
    fun onMeasurement_updatesEndTimestampNanos() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        val endTimestampNanos1: Long? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "endTimestampNanos")
        requireNotNull(endTimestampNanos1)
        assertEquals(0L, endTimestampNanos1)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        // set measurement
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp, accuracy)

        val endTimestampNanos2: Long? =
            getPrivateProperty(BaseAccumulatedEstimator::class, estimator, "endTimestampNanos")
        requireNotNull(endTimestampNanos2)
        assertEquals(timestamp, endTimestampNanos2)
    }

    @Test
    fun onMeasurement_increasesNumberOfProcessedMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

        // check default value
        assertEquals(0, estimator.numberOfProcessedMeasurements)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        // set measurement
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp, accuracy)

        // check
        assertEquals(1, estimator.numberOfProcessedMeasurements)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context, stopMode = StopMode.MAX_SAMPLES_ONLY)

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        for (i in 1..maxSamples) {
            measurementListener.onMeasurement(
                wx,
                wy,
                wz,
                null,
                null,
                null,
                timestamp + i * TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS,
                accuracy
            )

            assertEquals(i, estimator.numberOfProcessedMeasurements)
        }

        assertEquals(maxSamples, estimator.numberOfProcessedMeasurements)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxSamples(estimator, kinematics)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOnlyAndListener() {
        val completedListener = mockk<AccumulatedTriadEstimator
        .OnEstimationCompletedListener<GyroscopeNoiseEstimator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            stopMode = StopMode.MAX_SAMPLES_ONLY,
            completedListener = completedListener
        )

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxSamples = estimator.maxSamples

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        for (i in 1..maxSamples) {
            measurementListener.onMeasurement(
                wx,
                wy,
                wz,
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
        checkResultMaxSamples(estimator, kinematics)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context, stopMode = StopMode.MAX_DURATION_ONLY)

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, kinematics)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxDurationOnlyAndListener() {
        val completedListener =
            mockk<AccumulatedTriadEstimator.OnEstimationCompletedListener<GyroscopeNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            stopMode = StopMode.MAX_DURATION_ONLY,
            completedListener = completedListener
        )

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, kinematics)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndNoListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GyroscopeNoiseEstimator(context, stopMode = StopMode.MAX_SAMPLES_OR_DURATION)

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check result
        checkResultMaxDuration(estimator, kinematics)
    }

    @Test
    fun onMeasurement_whenIsCompleteMaxSamplesOrDurationAndListener() {
        val completedListener =
            mockk<AccumulatedTriadEstimator.OnEstimationCompletedListener<GyroscopeNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(
            context,
            stopMode = StopMode.MAX_SAMPLES_OR_DURATION,
            completedListener = completedListener
        )

        val collector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        estimator.setPrivateProperty("collector", collectorSpy)

        val maxDurationMillis = estimator.maxDurationMillis

        assertEquals(0, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val measurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            estimator.getPrivateProperty("measurementListener")
        requireNotNull(measurementListener)

        val kinematics = getBodyKinematics()
        val wx = kinematics.angularRateX.toFloat()
        val wy = kinematics.angularRateY.toFloat()
        val wz = kinematics.angularRateZ.toFloat()
        val timestamp1 = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp1, accuracy)

        assertEquals(1, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        val timestamp2 = timestamp1 + TIME_INTERVAL_MILLIS * MILLIS_TO_NANOS
        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp2, accuracy)

        assertEquals(2, estimator.numberOfProcessedMeasurements)
        assertFalse(estimator.resultAvailable)

        // add another measurement after max duration
        val timestamp3 = timestamp1 + maxDurationMillis * MILLIS_TO_NANOS

        measurementListener.onMeasurement(wx, wy, wz, null, null, null, timestamp3, accuracy)

        assertEquals(3, estimator.numberOfProcessedMeasurements)
        assertTrue(estimator.resultAvailable)

        // check that after completion, collector was stopped
        verify(exactly = 1) { collectorSpy.stop() }

        // check that listener was called
        verify(exactly = 1) { completedListener.onEstimationCompleted(estimator) }

        // check result
        checkResultMaxDuration(estimator, kinematics)
    }

    @Test
    fun onAccuracyChanged_whenUnreliableAndNoListener_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context)

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
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<GyroscopeNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context, unreliableListener = unreliableListener)

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
            mockk<AccumulatedTriadEstimator.OnUnreliableListener<GyroscopeNoiseEstimator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GyroscopeNoiseEstimator(context, unreliableListener = unreliableListener)

        // check default value
        assertFalse(estimator.resultUnreliable)

        val accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? =
            getPrivateProperty(
                BaseAccumulatedEstimator::class,
                estimator,
                "accuracyChangedListener"
            )
        requireNotNull(accuracyChangedListener)

        accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)

        // check
        assertFalse(estimator.resultUnreliable)
        verify(exactly = 0) { unreliableListener.onUnreliable(estimator) }
    }

    private fun checkResultMaxSamples(
        estimator: GyroscopeNoiseEstimator,
        kinematics: BodyKinematics
    ) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageX = estimator.averageX
        requireNotNull(averageX)
        assertEquals(kinematics.angularRateX, averageX, 0.0)

        val averageX1 = estimator.averageXAsMeasurement
        requireNotNull(averageX1)
        val averageX2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageXAsMeasurement(averageX2))
        assertEquals(averageX1, averageX2)
        assertEquals(averageX, averageX1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageX1.unit)

        val averageY = estimator.averageY
        requireNotNull(averageY)
        assertEquals(kinematics.angularRateY, averageY, 0.0)

        val averageY1 = estimator.averageYAsMeasurement
        requireNotNull(averageY1)
        val averageY2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageYAsMeasurement(averageY2))
        assertEquals(averageY1, averageY2)
        assertEquals(averageY, averageY1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageY1.unit)

        val averageZ = estimator.averageZ
        requireNotNull(averageZ)
        assertEquals(kinematics.angularRateZ, averageZ, SMALL_ABSOLUTE_ERROR)

        val averageZ1 = estimator.averageZAsMeasurement
        requireNotNull(averageZ1)
        val averageZ2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageZAsMeasurement(averageZ2))
        assertEquals(averageZ1, averageZ2)
        assertEquals(averageZ, averageZ1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageZ1.unit)

        val averageTriad1 = estimator.averageTriad
        requireNotNull(averageTriad1)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageTriad1.unit)
        assertEquals(averageX, averageTriad1.valueX, 0.0)
        assertEquals(averageY, averageTriad1.valueY, 0.0)
        assertEquals(averageZ, averageTriad1.valueZ, 0.0)
        val averageTriad2 = AngularSpeedTriad()
        assertTrue(estimator.getAverageTriad(averageTriad2))
        assertEquals(averageTriad1, averageTriad2)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(kinematics.angularRateNorm, averageNorm, SMALL_ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageNorm1.unit)

        val varianceX = estimator.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, SMALL_ABSOLUTE_ERROR)

        val varianceY = estimator.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, SMALL_ABSOLUTE_ERROR)

        val varianceZ = estimator.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, SMALL_ABSOLUTE_ERROR)

        val standardDeviationX = estimator.standardDeviationX
        requireNotNull(standardDeviationX)
        assertEquals(0.0, standardDeviationX, SMALL_ABSOLUTE_ERROR)
        val standardDeviationX1 = estimator.standardDeviationXAsMeasurement
        requireNotNull(standardDeviationX1)
        val standardDeviationX2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationXAsMeasurement(standardDeviationX2))
        assertEquals(standardDeviationX1, standardDeviationX2)
        assertEquals(standardDeviationX, standardDeviationX1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationX1.unit)

        val standardDeviationY = estimator.standardDeviationY
        requireNotNull(standardDeviationY)
        assertEquals(0.0, standardDeviationY, SMALL_ABSOLUTE_ERROR)
        val standardDeviationY1 = estimator.standardDeviationYAsMeasurement
        requireNotNull(standardDeviationY1)
        val standardDeviationY2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationYAsMeasurement(standardDeviationY2))
        assertEquals(standardDeviationY1, standardDeviationY2)
        assertEquals(standardDeviationY, standardDeviationY1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationY1.unit)

        val standardDeviationZ = estimator.standardDeviationZ
        requireNotNull(standardDeviationZ)
        assertEquals(0.0, standardDeviationZ, SMALL_ABSOLUTE_ERROR)
        val standardDeviationZ1 = estimator.standardDeviationZAsMeasurement
        requireNotNull(standardDeviationZ1)
        val standardDeviationZ2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationZAsMeasurement(standardDeviationZ2))
        assertEquals(standardDeviationZ1, standardDeviationZ2)
        assertEquals(standardDeviationZ, standardDeviationZ1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationZ1.unit)

        val standardDeviationTriad1 = estimator.standardDeviationTriad
        requireNotNull(standardDeviationTriad1)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationTriad1.unit)
        assertEquals(standardDeviationX, standardDeviationTriad1.valueX, 0.0)
        assertEquals(standardDeviationY, standardDeviationTriad1.valueY, 0.0)
        assertEquals(standardDeviationZ, standardDeviationTriad1.valueZ, 0.0)
        val standardDeviationTriad2 = AngularSpeedTriad()
        assertTrue(estimator.getStandardDeviationTriad(standardDeviationTriad2))
        assertEquals(standardDeviationTriad1, standardDeviationTriad2)

        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        assertEquals(0.0, standardDeviationNorm, SMALL_ABSOLUTE_ERROR)
        val standardDeviationNorm1 = estimator.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNorm1)
        val standardDeviationNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm2))
        assertEquals(standardDeviationNorm1, standardDeviationNorm2)
        assertEquals(standardDeviationNorm, standardDeviationNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationNorm1.unit)

        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)
        assertEquals(0.0, averageStandardDeviation, SMALL_ABSOLUTE_ERROR)
        val averageStandardDeviation1 = estimator.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviation1)
        val averageStandardDeviation2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation2))
        assertEquals(averageStandardDeviation1, averageStandardDeviation2)
        assertEquals(averageStandardDeviation, averageStandardDeviation1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageStandardDeviation1.unit)

        val psdX = estimator.psdX
        requireNotNull(psdX)
        assertEquals(0.0, psdX, 0.0)

        val psdY = estimator.psdY
        requireNotNull(psdY)
        assertEquals(0.0, psdY, 0.0)

        val psdZ = estimator.psdZ
        requireNotNull(psdZ)
        assertTrue(psdZ > 0.0)

        val rootPsdX = estimator.rootPsdX
        requireNotNull(rootPsdX)
        assertEquals(0.0, rootPsdX, 0.0)

        val rootPsdY = estimator.rootPsdY
        requireNotNull(rootPsdY)
        assertEquals(0.0, rootPsdY, 0.0)

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

    private fun checkResultMaxDuration(
        estimator: GyroscopeNoiseEstimator,
        kinematics: BodyKinematics
    ) {
        assertFalse(estimator.running)
        assertTrue(estimator.resultAvailable)
        assertFalse(estimator.resultUnreliable)

        val averageX = estimator.averageX
        requireNotNull(averageX)
        assertEquals(kinematics.angularRateX, averageX, 0.0)

        val averageX1 = estimator.averageXAsMeasurement
        requireNotNull(averageX1)
        val averageX2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageXAsMeasurement(averageX2))
        assertEquals(averageX1, averageX2)
        assertEquals(averageX, averageX1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageX1.unit)

        val averageY = estimator.averageY
        requireNotNull(averageY)
        assertEquals(kinematics.angularRateY, averageY, 0.0)

        val averageY1 = estimator.averageYAsMeasurement
        requireNotNull(averageY1)
        val averageY2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageYAsMeasurement(averageY2))
        assertEquals(averageY1, averageY2)
        assertEquals(averageY, averageY1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageY1.unit)

        val averageZ = estimator.averageZ
        requireNotNull(averageZ)
        assertEquals(kinematics.angularRateZ, averageZ, SMALL_ABSOLUTE_ERROR)

        val averageZ1 = estimator.averageZAsMeasurement
        requireNotNull(averageZ1)
        val averageZ2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageZAsMeasurement(averageZ2))
        assertEquals(averageZ1, averageZ2)
        assertEquals(averageZ, averageZ1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageZ1.unit)

        val averageTriad1 = estimator.averageTriad
        requireNotNull(averageTriad1)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageTriad1.unit)
        assertEquals(averageX, averageTriad1.valueX, 0.0)
        assertEquals(averageY, averageTriad1.valueY, 0.0)
        assertEquals(averageZ, averageTriad1.valueZ, 0.0)
        val averageTriad2 = AngularSpeedTriad()
        assertTrue(estimator.getAverageTriad(averageTriad2))
        assertEquals(averageTriad1, averageTriad2)

        val averageNorm = estimator.averageNorm
        requireNotNull(averageNorm)
        assertEquals(kinematics.angularRateNorm, averageNorm, SMALL_ABSOLUTE_ERROR)

        val averageNorm1 = estimator.averageNormAsMeasurement
        requireNotNull(averageNorm1)
        val averageNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageNormAsMeasurement(averageNorm2))
        assertEquals(averageNorm1, averageNorm2)
        assertEquals(averageNorm, averageNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageNorm1.unit)

        val varianceX = estimator.varianceX
        requireNotNull(varianceX)
        assertEquals(0.0, varianceX, SMALL_ABSOLUTE_ERROR)

        val varianceY = estimator.varianceY
        requireNotNull(varianceY)
        assertEquals(0.0, varianceY, SMALL_ABSOLUTE_ERROR)

        val varianceZ = estimator.varianceZ
        requireNotNull(varianceZ)
        assertEquals(0.0, varianceZ, SMALL_ABSOLUTE_ERROR)

        val standardDeviationX = estimator.standardDeviationX
        requireNotNull(standardDeviationX)
        assertEquals(0.0, standardDeviationX, SMALL_ABSOLUTE_ERROR)
        val standardDeviationX1 = estimator.standardDeviationXAsMeasurement
        requireNotNull(standardDeviationX1)
        val standardDeviationX2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationXAsMeasurement(standardDeviationX2))
        assertEquals(standardDeviationX1, standardDeviationX2)
        assertEquals(standardDeviationX, standardDeviationX1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationX1.unit)

        val standardDeviationY = estimator.standardDeviationY
        requireNotNull(standardDeviationY)
        assertEquals(0.0, standardDeviationY, SMALL_ABSOLUTE_ERROR)
        val standardDeviationY1 = estimator.standardDeviationYAsMeasurement
        requireNotNull(standardDeviationY1)
        val standardDeviationY2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationYAsMeasurement(standardDeviationY2))
        assertEquals(standardDeviationY1, standardDeviationY2)
        assertEquals(standardDeviationY, standardDeviationY1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationY1.unit)

        val standardDeviationZ = estimator.standardDeviationZ
        requireNotNull(standardDeviationZ)
        assertEquals(0.0, standardDeviationZ, SMALL_ABSOLUTE_ERROR)
        val standardDeviationZ1 = estimator.standardDeviationZAsMeasurement
        requireNotNull(standardDeviationZ1)
        val standardDeviationZ2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationZAsMeasurement(standardDeviationZ2))
        assertEquals(standardDeviationZ1, standardDeviationZ2)
        assertEquals(standardDeviationZ, standardDeviationZ1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationZ1.unit)

        val standardDeviationTriad1 = estimator.standardDeviationTriad
        requireNotNull(standardDeviationTriad1)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationTriad1.unit)
        assertEquals(standardDeviationX, standardDeviationTriad1.valueX, 0.0)
        assertEquals(standardDeviationY, standardDeviationTriad1.valueY, 0.0)
        assertEquals(standardDeviationZ, standardDeviationTriad1.valueZ, 0.0)
        val standardDeviationTriad2 = AngularSpeedTriad()
        assertTrue(estimator.getStandardDeviationTriad(standardDeviationTriad2))
        assertEquals(standardDeviationTriad1, standardDeviationTriad2)

        val standardDeviationNorm = estimator.standardDeviationNorm
        requireNotNull(standardDeviationNorm)
        assertEquals(0.0, standardDeviationNorm, SMALL_ABSOLUTE_ERROR)
        val standardDeviationNorm1 = estimator.standardDeviationNormAsMeasurement
        requireNotNull(standardDeviationNorm1)
        val standardDeviationNorm2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getStandardDeviationNormAsMeasurement(standardDeviationNorm2))
        assertEquals(standardDeviationNorm1, standardDeviationNorm2)
        assertEquals(standardDeviationNorm, standardDeviationNorm1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, standardDeviationNorm1.unit)

        val averageStandardDeviation = estimator.averageStandardDeviation
        requireNotNull(averageStandardDeviation)
        assertEquals(0.0, averageStandardDeviation, SMALL_ABSOLUTE_ERROR)
        val averageStandardDeviation1 = estimator.averageStandardDeviationAsMeasurement
        requireNotNull(averageStandardDeviation1)
        val averageStandardDeviation2 = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(estimator.getAverageStandardDeviationAsMeasurement(averageStandardDeviation2))
        assertEquals(averageStandardDeviation1, averageStandardDeviation2)
        assertEquals(averageStandardDeviation, averageStandardDeviation1.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, averageStandardDeviation1.unit)

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

    private fun getBodyKinematics(): BodyKinematics {
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
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        return ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            c,
            c,
            ecefVelocity,
            ecefVelocity,
            ecefPosition
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

        const val TIME_INTERVAL_MILLIS = 20L

        const val TIME_INTERVAL_SECONDS = TIME_INTERVAL_MILLIS.toDouble() / 1000

        const val MILLIS_TO_NANOS = 1000000L

        const val LARGE_ABSOLUTE_ERROR = 6e-4

        const val SMALL_ABSOLUTE_ERROR = 1e-11
    }
}