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
package com.irurueta.android.navigation.inertial.calibration

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.*
import com.irurueta.android.navigation.inertial.calibration.intervals.AccelerometerIntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator
import com.irurueta.navigation.inertial.calibration.IMUErrors
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.lang.reflect.InvocationTargetException
import java.util.*
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class StaticIntervalAccelerometerCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(AccelerometerSensorCollector.SensorType.ACCELEROMETER, calibrator.sensorType)
        assertEquals(SensorDelay.FASTEST, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSolveCalibrationWhenEnoughMeasurements_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            false
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenGroundTruthInitialBias_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenLocation_returnsExpectedValues() {
        val location = getLocation()

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenUnreliableGravityNormEstimationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertNull(calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitialBiasAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenNewCalibrationMeasurementAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener = mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertNull(calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenAccelerometerMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener = mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            accelerometerMeasurementListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(accelerometerMeasurementListener, calibrator.accelerometerMeasurementListener)
        assertNull(calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenGravityMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener = mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            accelerometerMeasurementListener,
            gravityMeasurementListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(accelerometerMeasurementListener, calibrator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, calibrator.gravityMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener = mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        val qualityScoreMapper = mockk<QualityScoreMapper<StandardDeviationBodyKinematics>>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            unreliableGravityNormEstimationListener,
            initialBiasAvailableListener,
            newCalibrationMeasurementAvailableListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            accelerometerMeasurementListener,
            gravityMeasurementListener,
            qualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(accelerometerMeasurementListener, calibrator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, calibrator.gravityMeasurementListener)
        assertSame(qualityScoreMapper, calibrator.qualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasYAsMeasurement)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasZAsMeasurement)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.initialBiasAsTriad)
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))
        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertNull(calibrator.averageTimeInterval)
        assertNull(calibrator.averageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))
        assertNull(calibrator.timeIntervalVariance)
        assertNull(calibrator.timeIntervalStandardDeviation)
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.initialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsMeasurement)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertNull(calibrator.gravityPsd)
        assertNull(calibrator.gravityRootPsd)
        assertNull(calibrator.estimatedMa)
        assertNull(calibrator.estimatedSx)
        assertNull(calibrator.estimatedSy)
        assertNull(calibrator.estimatedSz)
        assertNull(calibrator.estimatedMxy)
        assertNull(calibrator.estimatedMxz)
        assertNull(calibrator.estimatedMyx)
        assertNull(calibrator.estimatedMyz)
        assertNull(calibrator.estimatedMzx)
        assertNull(calibrator.estimatedMzy)
        assertNull(calibrator.estimatedCovariance)
        assertNull(calibrator.estimatedChiSq)
        assertNull(calibrator.estimatedMse)
        assertNull(calibrator.estimatedBiasX)
        assertNull(calibrator.estimatedBiasY)
        assertNull(calibrator.estimatedBiasZ)
        assertNull(calibrator.estimatedBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener = mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun unreliableGravityNormEstimatorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.unreliableGravityNormEstimationListener)

        // set new value
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        calibrator.unreliableGravityNormEstimationListener = unreliableGravityNormEstimationListener

        // check
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
    }

    @Test
    fun initialBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasAvailableListener)

        // set new value
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>()
        calibrator.initialBiasAvailableListener = initialBiasAvailableListener

        // check
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
    }

    @Test
    fun newCalibrationMeasurementAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)

        // set new value
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>()
        calibrator.newCalibrationMeasurementAvailableListener =
            newCalibrationMeasurementAvailableListener

        // check
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
    }

    @Test
    fun readyToSolveCalibrationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener = mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        calibrator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, calibrator.accelerometerMeasurementListener)
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.gravityMeasurementListener)

        // set new value
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        calibrator.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, calibrator.gravityMeasurementListener)
    }

    @Test
    fun qualityScoreMapper_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNotNull(calibrator.qualityScoreMapper)

        // set new value
        val qualityScoreMapper = mockk<QualityScoreMapper<StandardDeviationBodyKinematics>>()
        calibrator.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, calibrator.qualityScoreMapper)
    }

    @Test
    fun isGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertFalse(calibrator.isGroundTruthInitialBias)

        // set new value
        calibrator.isGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.isGroundTruthInitialBias = true
    }

    @Test
    fun location_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)

        // set new value
        val location = getLocation()
        calibrator.location = location

        // check
        assertSame(location, calibrator.location)
        assertEquals(GravityHelper.getGravityNormForLocation(location), calibrator.gravityNorm)

        // set new value
        calibrator.location = null

        // check
        assertNull(calibrator.location)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        val location = getLocation()
        calibrator.location = location
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
            calibrator.initialStaticSamples
        )

        // set new value
        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES

        // check
        assertEquals(INITIAL_STATIC_SAMPLES, calibrator.initialStaticSamples)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
            calibrator.thresholdFactor,
            0.0
        )

        // set new value
        calibrator.thresholdFactor = THRESHOLD_FACTOR

        // check
        assertEquals(THRESHOLD_FACTOR, calibrator.thresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )

        // set new value
        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR

        // check
        assertEquals(
            INSTANTANEOUS_NOISE_LEVEL_FACTOR,
            calibrator.instantaneousNoiseLevelFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )

        // set new value
        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD

        // check
        assertEquals(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            calibrator.baseNoiseLevelAbsoluteThreshold,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val value1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, value1.unit)

        // set new value
        val value2 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        assertEquals(value2, value3)
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        val value = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val value1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(value1)

        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, value1.unit)

        // set new value
        val value2 = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value2

        // check
        val value3 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(value3)
        assertEquals(value2, value3)
    }

    @Test
    fun initialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.initialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.initialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialSx = 0.0
    }

    @Test
    fun initialSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.initialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.initialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialSy = 0.0
    }

    @Test
    fun initialSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.initialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialSz = 0.0
    }

    @Test
    fun initialMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.initialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMxy = 0.0
    }

    @Test
    fun initialMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.initialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMxz = 0.0
    }

    @Test
    fun initialMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.initialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMyx = 0.0
    }

    @Test
    fun initialMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.initialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMyz = 0.0
    }

    @Test
    fun initialMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.initialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMzx = 0.0
    }

    @Test
    fun initialMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.initialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMzy = 0.0
    }

    @Test
    fun setInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default values
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz)

        // check
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz)
    }

    @Test
    fun setInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default values
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)

        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun setInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default values
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun initialMa_whenValid_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(Matrix(MA_SIZE, MA_SIZE), calibrator.initialMa)
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val ma = Matrix(MA_SIZE, MA_SIZE)
        ma.setElementAtIndex(0, initialSx)
        ma.setElementAtIndex(1, initialMyx)
        ma.setElementAtIndex(2, initialMzx)

        ma.setElementAtIndex(3, initialMxy)
        ma.setElementAtIndex(4, initialSy)
        ma.setElementAtIndex(5, initialMzy)

        ma.setElementAtIndex(6, initialMxz)
        ma.setElementAtIndex(7, initialMyz)
        ma.setElementAtIndex(8, initialSz)

        calibrator.initialMa = ma

        // check
        assertEquals(ma, calibrator.initialMa)
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialMa_whenInvalidRowsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.initialMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialMa_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.initialMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun initialMa_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.initialMa = Matrix(MA_SIZE, MA_SIZE)
    }

    @Test
    fun getInitialMa_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        val ma = Matrix(MA_SIZE, MA_SIZE)
        calibrator.getInitialMa(ma)

        // check
        assertEquals(initialSx, ma.getElementAtIndex(0), 0.0)
        assertEquals(initialMyx, ma.getElementAtIndex(1), 0.0)
        assertEquals(initialMzx, ma.getElementAtIndex(2), 0.0)

        assertEquals(initialMxy, ma.getElementAtIndex(3), 0.0)
        assertEquals(initialSy, ma.getElementAtIndex(4), 0.0)
        assertEquals(initialMzy, ma.getElementAtIndex(5), 0.0)

        assertEquals(initialMxz, ma.getElementAtIndex(6), 0.0)
        assertEquals(initialMyz, ma.getElementAtIndex(7), 0.0)
        assertEquals(initialSz, ma.getElementAtIndex(8), 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getInitialMa_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.getInitialMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getInitialMa_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.getInitialMa(ma)
    }

    @Test
    fun isCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertFalse(calibrator.isCommonAxisUsed)

        // set new value
        calibrator.isCommonAxisUsed = true

        // check
        assertTrue(calibrator.isCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.isCommonAxisUsed = true
    }

    @Test
    fun requiredMeasurements_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )

        // set new value
        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS

        // check
        assertEquals(REQUIRED_MEASUREMENTS, calibrator.requiredMeasurements)
    }

    @Test(expected = IllegalArgumentException::class)
    fun requiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
    }

    @Test
    fun robustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.robustMethod)

        // set new value
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun robustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun robustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )

        // set new value
        calibrator.robustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun robustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )

        // set new value
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun robustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun robustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )

        // set new value
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun robustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun robustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.robustThreshold)

        // set new value
        calibrator.robustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.robustThreshold = null

        // check
        assertNull(calibrator.robustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun robustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun robustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_STOP_THRESHOLD_FACTOR, calibrator.robustStopThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.robustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)

        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun onInitializationStarted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val intervalDetectorInitializationStartedListener: IntervalDetector.OnInitializationStartedListener<AccelerometerIntervalDetector>? =
            getPrivateProperty(StaticIntervalCalibrator::class, calibrator, "intervalDetectorInitializationStartedListener")
        requireNotNull(intervalDetectorInitializationStartedListener)

        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorInitializationStartedListener.onInitializationStarted(intervalDetector)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val initializationStartedListener =
            mockk<StaticIntervalCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val intervalDetectorInitializationStartedListener: IntervalDetector.OnInitializationStartedListener<AccelerometerIntervalDetector>? =
            getPrivateProperty(StaticIntervalCalibrator::class, calibrator, "intervalDetectorInitializationStartedListener")
        requireNotNull(intervalDetectorInitializationStartedListener)

        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorInitializationStartedListener.onInitializationStarted(intervalDetector)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_setsGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check initial value
        assertNull(calibrator.gravityNorm)

        val intervalDetectorInitializationCompletedListener: IntervalDetector.OnInitializationCompletedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationCompletedListener")
        requireNotNull(intervalDetectorInitializationCompletedListener)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        every { gravityNormEstimatorSpy.averageNorm }.returns(gravityNorm)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorInitializationCompletedListener.onInitializationCompleted(
            intervalDetector,
            baseNoiseLevel
        )

        // check
        assertEquals(gravityNorm, calibrator.gravityNorm)
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_setsGravityNormAndNotifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        // check initial value
        assertNull(calibrator.gravityNorm)

        val intervalDetectorInitializationCompletedListener: IntervalDetector.OnInitializationCompletedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationCompletedListener")
        requireNotNull(intervalDetectorInitializationCompletedListener)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        every { gravityNormEstimatorSpy.averageNorm }.returns(gravityNorm)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorInitializationCompletedListener.onInitializationCompleted(
            intervalDetector,
            baseNoiseLevel
        )

        // check
        assertEquals(gravityNorm, calibrator.gravityNorm)

        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsCollectorAndGravityNormEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetectorErrorListener: IntervalDetector.OnErrorListener<AccelerometerIntervalDetector>? =
            getPrivateProperty(StaticIntervalCalibrator::class, calibrator, "intervalDetectorErrorListener")
        requireNotNull(intervalDetectorErrorListener)

        intervalDetectorErrorListener.onError(
            intervalDetectorSpy,
            ErrorReason.UNRELIABLE_SENSOR
        )

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
    }

    @Test
    fun onError_whenListenersAvailable_stopsAndNotifies() {
        val errorListener =
            mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val stoppedListener =
            mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            errorListener = errorListener,
            stoppedListener = stoppedListener
        )

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetectorErrorListener: IntervalDetector.OnErrorListener<AccelerometerIntervalDetector>? =
            getPrivateProperty(StaticIntervalCalibrator::class, calibrator, "intervalDetectorErrorListener")
        requireNotNull(intervalDetectorErrorListener)

        intervalDetectorErrorListener.onError(
            intervalDetectorSpy,
            ErrorReason.UNRELIABLE_SENSOR
        )

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                StaticIntervalCalibrator.CalibratorErrorReason.UNRELIABLE_SENSOR
            )
        }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_addsOneMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.measurements.isEmpty())

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
            intervalDetector,
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0
        )

        assertEquals(1, calibrator.measurements.size)
        val measurement = calibrator.measurements[0]
        val kinematics = measurement.kinematics

        assertEquals(7.0, kinematics.fx, 0.0)
        assertEquals(8.0, kinematics.fy, 0.0)
        assertEquals(9.0, kinematics.fz, 0.0)
        assertEquals(0.0, kinematics.angularRateX, 0.0)
        assertEquals(0.0, kinematics.angularRateY, 0.0)
        assertEquals(0.0, kinematics.angularRateZ, 0.0)

        val stdNorm = sqrt(10.0.pow(2.0) + 11.0.pow(2.0) + 12.0.pow(2.0))
        assertEquals(stdNorm, measurement.specificForceStandardDeviation, 0.0)
        assertEquals(0.0, measurement.angularRateStandardDeviation, 0.0)

        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenNewCalibrationMeasurementAvailable_notifies() {
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalCalibrator.OnNewCalibrationMeasurementAvailableListener<StaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            newCalibrationMeasurementAvailableListener = newCalibrationMeasurementAvailableListener
        )

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val intervalDetector = mockk<AccelerometerIntervalDetector>()
        intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
            intervalDetector,
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0
        )

        assertEquals(1, calibrator.measurements.size)
        val measurement = calibrator.measurements[0]

        val measurementSize = calibrator.measurements.size
        assertEquals(1, measurementSize)

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        verify(exactly = 1) {
            newCalibrationMeasurementAvailableListener.onNewCalibrationMeasurementAvailable(
                calibrator,
                measurement,
                measurementSize,
                reqMeasurements
            )
        }

        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenEnoughMeasurementsAndNotSolveCalibrator_stopsCollectorAndGravityNormEstimatorAndBuildInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            location = location,
            solveCalibrationWhenEnoughMeasurements = false
        )

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val internalCalibrator1: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNull(internalCalibrator1)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        for (i in 1..reqMeasurements) {
            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0
            )
        }

        assertEquals(reqMeasurements, calibrator.measurements.size)
        assertTrue(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }

        val internalCalibrator2: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNotNull(internalCalibrator2)

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenersAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            location = location,
            solveCalibrationWhenEnoughMeasurements = false,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            stoppedListener = stoppedListener
        )

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val internalCalibrator1: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNull(internalCalibrator1)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        for (i in 1..reqMeasurements) {
            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0
            )
        }

        assertEquals(reqMeasurements, calibrator.measurements.size)
        assertTrue(calibrator.running)
        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenSolveCalibrationEnabled_solvesCalibration() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            location = location,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialBias)

        val nedPosition = location.toNEDPosition()

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMaGeneral()
        val mg = generateMg()
        val gg = generateGg()

        val accelNoiseRootPSD = 0.0
        val gyroNoiseRootPSD = 0.0
        val accelQuantLevel = 0.0
        val gyroQuantLevel = 0.0

        val errors = IMUErrors(
            ba,
            bg,
            ma,
            mg,
            gg,
            accelNoiseRootPSD,
            gyroNoiseRootPSD,
            accelQuantLevel,
            gyroQuantLevel
        )

        val randomizer = UniformRandomizer()
        val random = Random()

        for (i in 1..reqMeasurements) {
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val nedC = CoordinateTransformation(
                roll,
                pitch,
                yaw,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame
            )

            val measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random)

            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                measuredKinematics.fx,
                measuredKinematics.fy,
                measuredKinematics.fz,
                ACCUMULATED_STD,
                ACCUMULATED_STD,
                ACCUMULATED_STD
            )
        }

        assertNotNull(calibrator.estimatedMa)
        assertNotNull(calibrator.estimatedSx)
        assertNotNull(calibrator.estimatedSy)
        assertNotNull(calibrator.estimatedSz)
        assertNotNull(calibrator.estimatedMxy)
        assertNotNull(calibrator.estimatedMxz)
        assertNotNull(calibrator.estimatedMyx)
        assertNotNull(calibrator.estimatedMyz)
        assertNotNull(calibrator.estimatedMzx)
        assertNotNull(calibrator.estimatedMzy)
        assertNotNull(calibrator.estimatedCovariance)
        assertNotNull(calibrator.estimatedChiSq)
        assertNotNull(calibrator.estimatedMse)
        assertNotNull(calibrator.estimatedBiasX)
        assertNotNull(calibrator.estimatedBiasY)
        assertNotNull(calibrator.estimatedBiasZ)
        assertNotNull(calibrator.estimatedBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasAsTriad)
        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedBiasAsTriad(triad))
        assertNotNull(calibrator.estimatedBiasStandardDeviationNorm)

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val errorListener =
            mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            location = location,
            solveCalibrationWhenEnoughMeasurements = true,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            stoppedListener = stoppedListener,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener,
            errorListener = errorListener
        )

        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialBias)

        val nedPosition = location.toNEDPosition()

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMaGeneral()
        val mg = generateMg()
        val gg = generateGg()

        val accelNoiseRootPSD = 0.0
        val gyroNoiseRootPSD = 0.0
        val accelQuantLevel = 0.0
        val gyroQuantLevel = 0.0

        val errors = IMUErrors(
            ba,
            bg,
            ma,
            mg,
            gg,
            accelNoiseRootPSD,
            gyroNoiseRootPSD,
            accelQuantLevel,
            gyroQuantLevel
        )

        val randomizer = UniformRandomizer()
        val random = Random()

        for (i in 1..reqMeasurements) {
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val nedC = CoordinateTransformation(
                roll,
                pitch,
                yaw,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame
            )

            val measuredKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random)

            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                measuredKinematics.fx,
                measuredKinematics.fy,
                measuredKinematics.fz,
                ACCUMULATED_STD,
                ACCUMULATED_STD,
                ACCUMULATED_STD
            )
        }

        assertNotNull(calibrator.estimatedMa)
        assertNotNull(calibrator.estimatedSx)
        assertNotNull(calibrator.estimatedSy)
        assertNotNull(calibrator.estimatedSz)
        assertNotNull(calibrator.estimatedMxy)
        assertNotNull(calibrator.estimatedMxz)
        assertNotNull(calibrator.estimatedMyx)
        assertNotNull(calibrator.estimatedMyz)
        assertNotNull(calibrator.estimatedMzx)
        assertNotNull(calibrator.estimatedMzy)
        assertNotNull(calibrator.estimatedCovariance)
        assertNotNull(calibrator.estimatedChiSq)
        assertNotNull(calibrator.estimatedMse)
        assertNotNull(calibrator.estimatedBiasX)
        assertNotNull(calibrator.estimatedBiasY)
        assertNotNull(calibrator.estimatedBiasZ)
        assertNotNull(calibrator.estimatedBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedBiasAsTriad)
        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedBiasAsTriad(triad))
        assertNotNull(calibrator.estimatedBiasStandardDeviationNorm)

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }
        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
        verify { errorListener wasNot Called }

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onMeasurement_whenFirstMeasurement_updatesInitialBiases() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialBiasX = calibrator.initialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.initialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.initialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(4.0, initialBiasX, 0.0)
        assertEquals(5.0, initialBiasY, 0.0)
        assertEquals(6.0, initialBiasZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoBiasX_updatesInitialBiases() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            null,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialBiasX = calibrator.initialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.initialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.initialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoBiasY_updatesInitialBiases() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            null,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialBiasX = calibrator.initialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.initialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.initialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoBiasZ_updatesInitialBiases() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            null,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialBiasX = calibrator.initialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.initialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.initialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndListener_updatesInitialBiasesAndNotifies() {
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialBiasAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initialBiasAvailableListener = initialBiasAvailableListener
        )

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            null,
            null,
            null,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialBiasX = calibrator.initialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.initialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.initialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)

        verify(exactly = 1) {
            initialBiasAvailableListener.onInitialBiasAvailable(
                calibrator,
                0.0,
                0.0,
                0.0
            )
        }
    }

    @Test
    fun onMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(2)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
    }

    @Test
    fun onEstimationCompleted_setsGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.gravityNorm)

        val gravityNormCompletedListener: AccumulatedMeasurementEstimator.OnEstimationCompletedListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormCompletedListener")
        requireNotNull(gravityNormCompletedListener)

        val gravityNormEstimator = mockk<GravityNormEstimator>()
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        every { gravityNormEstimator.averageNorm }.returns(gravityNorm)
        gravityNormCompletedListener.onEstimationCompleted(gravityNormEstimator)

        // check
        assertEquals(gravityNorm, calibrator.gravityNorm)
    }

    @Test
    fun onUnreliable_whenNoListenerAvailable_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertFalse(calibrator.resultUnreliable)

        val gravityNormUnreliableListener: AccumulatedMeasurementEstimator.OnUnreliableListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormUnreliableListener")
        requireNotNull(gravityNormUnreliableListener)

        val gravityNormEstimator = mockk<GravityNormEstimator>()
        gravityNormUnreliableListener.onUnreliable(gravityNormEstimator)

        assertTrue(calibrator.resultUnreliable)
    }

    @Test
    fun onUnreliable_whenListenerAvailable_setsResultUnreliableAndNotifies() {
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            unreliableGravityNormEstimationListener = unreliableGravityNormEstimationListener
        )

        // check default value
        assertFalse(calibrator.resultUnreliable)

        val gravityNormUnreliableListener: AccumulatedMeasurementEstimator.OnUnreliableListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormUnreliableListener")
        requireNotNull(gravityNormUnreliableListener)

        val gravityNormEstimator = mockk<GravityNormEstimator>()
        gravityNormUnreliableListener.onUnreliable(gravityNormEstimator)

        assertTrue(calibrator.resultUnreliable)
        verify(exactly = 1) {
            unreliableGravityNormEstimationListener.onUnreliableGravityEstimation(
                calibrator
            )
        }
    }

    @Test
    fun initialBiasX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasX)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)

        assertEquals(initialBiasX, calibrator.initialBiasX)
    }

    @Test
    fun initialBiasY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasY)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)

        assertEquals(initialBiasY, calibrator.initialBiasY)
    }

    @Test
    fun initialBiasZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasZ)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)

        assertEquals(initialBiasZ, calibrator.initialBiasZ)
    }

    @Test
    fun initialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasXAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)

        val bias = calibrator.initialBiasXAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)

        // check
        assertTrue(calibrator.getInitialBiasXAsMeasurement(bias))
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun initialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasYAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)

        val bias = calibrator.initialBiasYAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasYAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)

        // check
        assertTrue(calibrator.getInitialBiasYAsMeasurement(bias))
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun initialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasZAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)

        val bias = calibrator.initialBiasZAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasZAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)

        // check
        assertTrue(calibrator.getInitialBiasZAsMeasurement(bias))
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun initialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasAsTriad)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)

        assertNull(calibrator.initialBiasAsTriad)

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)

        assertNull(calibrator.initialBiasAsTriad)

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)

        // check
        val triad = calibrator.initialBiasAsTriad
        requireNotNull(triad)
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun getInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val triad = AccelerationTriad()
        assertFalse(calibrator.getInitialBiasAsTriad(triad))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)

        assertFalse(calibrator.getInitialBiasAsTriad(triad))

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)

        assertFalse(calibrator.getInitialBiasAsTriad(triad))

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)

        // check
        assertTrue(calibrator.getInitialBiasAsTriad(triad))
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun accelerometerSensor_getsIntervalDetectorSensor() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val sensor = mockk<Sensor>()
        every { intervalDetectorSpy.sensor }.returns(sensor)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(sensor, calibrator.accelerometerSensor)

        verify(exactly = 1) { intervalDetectorSpy.sensor }
    }

    @Test
    fun gravitySensor_getsIntervalDetectorSensor() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val sensor = mockk<Sensor>()
        every { gravityNormEstimatorSpy.sensor }.returns(sensor)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertSame(sensor, calibrator.gravitySensor)

        verify(exactly = 1) { gravityNormEstimatorSpy.sensor }
    }

    @Test
    fun baseNoiseLevel_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevel }
    }

    @Test
    fun baseNoiseLevelAsMeasurement_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.baseNoiseLevelAsMeasurement)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val baseNoiseLevel1 =
            Acceleration(baseNoiseLevel, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { intervalDetectorSpy.baseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val baseNoiseLevel2 = calibrator.baseNoiseLevelAsMeasurement
        assertSame(baseNoiseLevel1, baseNoiseLevel2)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelAsMeasurement }
    }

    @Test
    fun getBaseNoiseLevelAsMeasurement_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.getBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = baseNoiseLevel
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getBaseNoiseLevelAsMeasurement(acceleration))

        // check
        assertEquals(baseNoiseLevel, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) { intervalDetectorSpy.getBaseNoiseLevelAsMeasurement(acceleration) }
    }

    @Test
    fun baseNoiseLevelPsd_getsIntervalDetectorBaseNoiseLevelPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.baseNoiseLevelPsd)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevelPsd }.returns(baseNoiseLevelPsd)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevelPsd, calibrator.baseNoiseLevelPsd)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelPsd }
    }

    @Test
    fun baseNoiseLevelRootPsd_getsIntervalDetectorBaseNoiseLevelRootPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.baseNoiseLevelRootPsd)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevelRootPsd }.returns(baseNoiseLevelRootPsd)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevelRootPsd, calibrator.baseNoiseLevelRootPsd)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelRootPsd }
    }

    @Test
    fun threshold_getsIntervalDetectorThreshold() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.threshold)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { intervalDetectorSpy.threshold }.returns(threshold)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(threshold, calibrator.threshold)
        verify(exactly = 1) { intervalDetectorSpy.threshold }
    }

    @Test
    fun thresholdAsMeasurement_getsIntervalDetectorThresholdAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.thresholdAsMeasurement)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        val acceleration = Acceleration(threshold, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { intervalDetectorSpy.thresholdAsMeasurement }.returns(acceleration)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(acceleration, calibrator.thresholdAsMeasurement)
        verify(exactly = 1) { intervalDetectorSpy.thresholdAsMeasurement }
    }

    @Test
    fun getThresholdAsMeasurement_getsIntervalDetectorThresholdAsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { intervalDetectorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = threshold
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(threshold, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) { intervalDetectorSpy.getThresholdAsMeasurement(acceleration) }
    }

    @Test
    fun averageTimeInterval_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.averageTimeInterval)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { intervalDetectorSpy.averageTimeInterval }.returns(averageTimeInterval)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(averageTimeInterval, calibrator.averageTimeInterval)
        verify(exactly = 1) { intervalDetectorSpy.averageTimeInterval }
    }

    @Test
    fun averageTimeIntervalAsTime_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.averageTimeIntervalAsTime)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        val time = Time(averageTimeInterval, TimeUnit.SECOND)
        every { intervalDetectorSpy.averageTimeIntervalAsTime }.returns(time)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(time, calibrator.averageTimeIntervalAsTime)
        verify(exactly = 1) { intervalDetectorSpy.averageTimeIntervalAsTime }
    }

    @Test
    fun getAverageTimeIntervalAsTime_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { intervalDetectorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = averageTimeInterval
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getAverageTimeIntervalAsTime(time))
        assertEquals(averageTimeInterval, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { intervalDetectorSpy.getAverageTimeIntervalAsTime(time) }
    }

    @Test
    fun timeIntervalVariance_getsIntervalDetectorTimeIntervalVariance() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.timeIntervalVariance)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        every { intervalDetectorSpy.timeIntervalVariance }.returns(timeIntervalVariance)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(timeIntervalVariance, calibrator.timeIntervalVariance)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalVariance }
    }

    @Test
    fun timeIntervalStandardDeviation_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.timeIntervalStandardDeviation)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        every { intervalDetectorSpy.timeIntervalStandardDeviation }.returns(
            timeIntervalStandardDeviation
        )
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(timeIntervalStandardDeviation, calibrator.timeIntervalStandardDeviation)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalStandardDeviation }
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time = Time(value, TimeUnit.SECOND)
        every { intervalDetectorSpy.timeIntervalStandardDeviationAsTime }.returns(time)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(time, calibrator.timeIntervalStandardDeviationAsTime)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        every { intervalDetectorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { intervalDetectorSpy.getTimeIntervalStandardDeviationAsTime(time) }
    }

    @Test
    fun minimumRequiredMeasurements_whenCommonAxisAndKnownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.isCommonAxisUsed = true
        calibrator.isGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isCommonAxisUsed)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertEquals(7, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.isCommonAxisUsed = true
        calibrator.isGroundTruthInitialBias = false

        // check
        assertTrue(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenNotCommonAxisAndKnownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.isCommonAxisUsed = false
        calibrator.isGroundTruthInitialBias = true

        // check
        assertFalse(calibrator.isCommonAxisUsed)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenNotCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.isCommonAxisUsed = false
        calibrator.isGroundTruthInitialBias = false

        // check
        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertEquals(13, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun averageGravityNorm_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        every { gravityNormEstimatorSpy.averageNorm }.returns(gravityNorm)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertEquals(gravityNorm, calibrator.averageGravityNorm)
        verify(exactly = 1) { gravityNormEstimatorSpy.averageNorm }
    }

    @Test
    fun averageGravityNorm_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.averageGravityNorm)
    }

    @Test
    fun averageGravityNormAsMeasurement_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        val acceleration = Acceleration(gravityNorm, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { gravityNormEstimatorSpy.averageNormAsMeasurement }.returns(acceleration)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertSame(acceleration, gravityNormEstimatorSpy.averageNormAsMeasurement)
        verify(exactly = 1) { gravityNormEstimatorSpy.averageNormAsMeasurement }
    }

    @Test
    fun averageGravityNormAsMeasurement_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.averageGravityNormAsMeasurement)
    }

    @Test
    fun getAverageGravityNormAsMeasurement_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        every { gravityNormEstimatorSpy.getAverageNormAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = gravityNorm
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertEquals(gravityNorm, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) { gravityNormEstimatorSpy.getAverageNormAsMeasurement(acceleration) }
    }

    @Test
    fun getAverageGravityNormAsMeasurement_whenGravityNormNotEstimated_returnsFalse() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertEquals(0.0, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun gravityNormVariance_whenGravityNormEstimated_getsEstimatorVariance() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val gravityNormVariance = randomizer.nextDouble()
        every { gravityNormEstimatorSpy.normVariance }.returns(gravityNormVariance)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val result = calibrator.gravityNormVariance
        requireNotNull(result)
        assertEquals(gravityNormVariance, result, 0.0)
        verify(exactly = 1) { gravityNormEstimatorSpy.normVariance }
    }

    @Test
    fun gravityNormVariance_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertNull(calibrator.gravityNormVariance)
    }

    @Test
    fun gravityNormStandardDeviation_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val gravityNormStandardDeviation = randomizer.nextDouble()
        every { gravityNormEstimatorSpy.normStandardDeviation }.returns(gravityNormStandardDeviation)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val result = calibrator.gravityNormStandardDeviation
        requireNotNull(result)
        assertEquals(gravityNormStandardDeviation, result, 0.0)
        verify(exactly = 1) { gravityNormEstimatorSpy.normStandardDeviation }
    }

    @Test
    fun gravityNormStandardDeviation_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityNormStandardDeviation)
    }

    @Test
    fun gravityNormStandardDeviationAsMeasurement_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val gravityNormStandardDeviation = randomizer.nextDouble()
        val acceleration =
            Acceleration(gravityNormStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { gravityNormEstimatorSpy.normStandardDeviationAsMeasurement }.returns(acceleration)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val result = calibrator.gravityNormStandardDeviationAsMeasurement
        assertSame(acceleration, result)
        verify(exactly = 1) { gravityNormEstimatorSpy.normStandardDeviationAsMeasurement }
    }

    @Test
    fun gravityNormStandardDeviationAsMeasurement_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
    }

    @Test
    fun getGravityNormStandardDeviationAsMeasurement_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        every { gravityNormEstimatorSpy.getNormStandardDeviationAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = value
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
        assertEquals(value, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) {
            gravityNormEstimatorSpy.getNormStandardDeviationAsMeasurement(
                acceleration
            )
        }
    }

    @Test
    fun getGravityNormStandardDeviationAsMeasurement_whenGravityNormNotEstimated_returnsFalse() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
    }

    @Test
    fun gravityPsd_whenGravityNormEstimated_getsEstimatorPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val gravityPsd = randomizer.nextDouble()
        every { gravityNormEstimatorSpy.psd }.returns(gravityPsd)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertEquals(gravityPsd, calibrator.gravityPsd)
        verify(exactly = 1) { gravityNormEstimatorSpy.psd }
    }

    @Test
    fun gravityPsd_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityPsd)
    }

    @Test
    fun gravityRootPsd_whenGravityNormEstimated_getEstimatorRootPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val randomizer = UniformRandomizer()
        val gravityRootPsd = randomizer.nextDouble()
        every { gravityNormEstimatorSpy.rootPsd }.returns(gravityRootPsd)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertEquals(gravityRootPsd, calibrator.gravityRootPsd)
        verify(exactly = 1) { gravityNormEstimatorSpy.rootPsd }
    }

    @Test
    fun gravityRootPsd_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityRootPsd)
    }

    @Test
    fun start_whenNotRunningAndGravityNormNotEstimated_resetsAndStartsIntervalDetector() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context, location = location)

        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)

        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurements = calibrator.measurements
        val measurementsSpy = spyk(measurements)
        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "measurements", measurementsSpy)

        calibrator.setPrivateProperty("resultUnreliable", true)
        calibrator.setPrivateProperty("initialBiasX", 0.0)
        calibrator.setPrivateProperty("initialBiasY", 0.0)
        calibrator.setPrivateProperty("initialBiasZ", 0.0)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.start() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        calibrator.start()

        // check
        assertNull(calibrator.gravityNorm)
        verify(exactly = 1) { measurementsSpy.clear() }
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        assertTrue(calibrator.running)

        verify { gravityNormEstimatorSpy wasNot Called }
        verify(exactly = 1) { intervalDetectorSpy.start() }
    }

    @Test
    fun start_whenNotRunningAndGravityNormEstimated_resetsAndStartsIntervalDetector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurements = calibrator.measurements
        val measurementsSpy = spyk(measurements)
        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "measurements", measurementsSpy)

        calibrator.setPrivateProperty("resultUnreliable", true)
        calibrator.setPrivateProperty("initialBiasX", 0.0)
        calibrator.setPrivateProperty("initialBiasY", 0.0)
        calibrator.setPrivateProperty("initialBiasZ", 0.0)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        justRun { gravityNormEstimatorSpy.start() }
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.start() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        calibrator.start()

        // check
        assertNull(calibrator.gravityNorm)
        verify(exactly = 1) { measurementsSpy.clear() }
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { gravityNormEstimatorSpy.start() }
        verify(exactly = 1) { intervalDetectorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertFalse(calibrator.running)

        calibrator.start()

        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun stop_whenGravityNormEstimatorNotRunning_stopsIntervalDetector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.stop() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 0) { gravityNormEstimatorSpy.stop() }
        assertFalse(gravityNormEstimatorSpy.running)
    }

    @Test
    fun stop_whenGravityNormEstimatorRunning_stopsIntervalDetector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.stop() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        assertTrue(gravityNormEstimatorSpy.running)
    }

    @Test
    fun stop_whenListenerAvailable_notifies() {
        val stoppedListener =
            mockk<StaticIntervalCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, stoppedListener = stoppedListener)

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.stop() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        setPrivateProperty(StaticIntervalCalibrator::class, calibrator, "running", true)
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 0) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
        assertFalse(gravityNormEstimatorSpy.running)
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenNotReadyToSolveCalibration_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.calibrate()
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.setPrivateProperty("running", true)

        calibrator.calibrate()
    }

    @Test
    fun calibrate_whenReadyNotRunningAndNoInternalCalibrator_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        justRun { internalCalibrator.calibrate() }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }
        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
    }

    @Test
    fun calibrate_whenFailure_setsAsNotRunning() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        val errorListener =
            mockk<StaticIntervalCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, errorListener = errorListener)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                StaticIntervalCalibrator.CalibratorErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION
            )
        }
    }

    @Test
    fun estimatedBiasX_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasX)
    }

    @Test
    fun estimatedBiasX_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFx }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedBiasX)
    }

    @Test
    fun estimatedBiasX_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedBiasX)
    }

    @Test
    fun estimatedBiasY_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasY)
    }

    @Test
    fun estimatedBiasY_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFy }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedBiasY)
    }

    @Test
    fun estimatedBiasY_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedBiasY)
    }

    @Test
    fun estimatedBiasZ_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasZ)
    }

    @Test
    fun estimatedBiasZ_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFz }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedBiasZ)
    }

    @Test
    fun estimatedBiasZ_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedBiasZ)
    }

    @Test
    fun estimatedBiasXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasXAsMeasurement)
    }

    @Test
    fun estimatedBiasXAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFxAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasXAsMeasurement)
    }

    @Test
    fun estimatedBiasXAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasXAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasXAsMeasurement)
    }

    @Test
    fun getEstimatedBiasXAsMeasurement_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedBiasXAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedBiasXAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFxAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasXAsMeasurement(acceleration))

        assertEquals(estimatedBiasX, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun getEstimatedBiasXAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasXAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasXAsMeasurement(acceleration))

        assertEquals(estimatedBiasX, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedBiasYAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasYAsMeasurement)
    }

    @Test
    fun estimatedBiasYAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFyAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasYAsMeasurement)
    }

    @Test
    fun estimatedBiasYAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasYAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasYAsMeasurement)
    }

    @Test
    fun getEstimatedBiasYAsMeasurement_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedBiasYAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedBiasYAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFyAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasYAsMeasurement(acceleration))

        assertEquals(estimatedBiasY, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun getEstimatedBiasYAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasYAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasYAsMeasurement(acceleration))

        assertEquals(estimatedBiasY, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedBiasZAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasZAsMeasurement)
    }

    @Test
    fun estimatedBiasZAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFzAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasZAsMeasurement)
    }

    @Test
    fun estimatedBiasZAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasZAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedBiasZAsMeasurement)
    }

    @Test
    fun getEstimatedBiasZAsMeasurement_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedBiasZAsMeasurement_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFzAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasZAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedBiasZAsMeasurement_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasZAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedBiasZAsMeasurement(acceleration))

        assertEquals(estimatedBiasZ, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedBiasAsTriad_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedBiasAsTriad)
    }

    @Test
    fun estimatedBiasAsTriad_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        val triad = AccelerationTriad(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        every { internalCalibratorSpy.estimatedBiasAsTriad }.returns(triad)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedBiasAsTriad)
    }

    @Test
    fun estimatedBiasAsTriad_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        val triad = AccelerationTriad(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        every { internalCalibratorSpy.biasAsTriad }.returns(triad)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedBiasAsTriad)
    }

    @Test
    fun getEstimatedBiasAsTriad_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        val triad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
    }

    @Test
    fun getEstimatedBiasAsTriad_whenUnknownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasAsTriad(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                estimatedBiasX,
                estimatedBiasY,
                estimatedBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun getEstimatedBiasAsTriad_whenKnownBiasInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasAsTriad(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                estimatedBiasX,
                estimatedBiasY,
                estimatedBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun estimatedBiasStandardDeviationNorm_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun estimatedBiasStandardDeviationNorm_whenBiasUncertaintySourceInternalCalibrator_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasStandardDeviationNorm = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasStandardDeviationNorm }.returns(
            estimatedBiasStandardDeviationNorm
        )
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(
            estimatedBiasStandardDeviationNorm,
            calibrator.estimatedBiasStandardDeviationNorm
        )
    }

    @Test
    fun estimatedBiasStandardDeviationNorm_whenOtherInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndPositionBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndPositionBiasSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndPositionBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndGravityBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndGravityBiasSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndGravityBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthBiasAndGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndPositionBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndPositionBiasNotAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndPositionBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndGravityBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndGravityBiasSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndGravityBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNotNull(calibrator.gravityNorm)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as KnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthBiasAndGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(ROBUST_STOP_THRESHOLD_FACTOR, calibrator.robustStopThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(ROBUST_STOP_THRESHOLD_FACTOR, calibrator.robustStopThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = true
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = true)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(
                context,
                location = location,
                isGroundTruthInitialBias = false
            )

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNotNull(calibrator.location)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenRANSACGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenMSACGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenMSACGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.MSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROSACGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROSAC
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenLMedSGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val measurement = mockk<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.LMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialBiasX", initialBiasX)
        calibrator.setPrivateProperty("initialBiasY", initialBiasY)
        calibrator.setPrivateProperty("initialBiasZ", initialBiasZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val intervalDetector: AccelerometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        val internalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(initialSx, internalCalibrator2.initialSx, 0.0)
        assertEquals(initialSy, internalCalibrator2.initialSy, 0.0)
        assertEquals(initialSz, internalCalibrator2.initialSz, 0.0)
        assertEquals(initialMxy, internalCalibrator2.initialMxy, 0.0)
        assertEquals(initialMxz, internalCalibrator2.initialMxz, 0.0)
        assertEquals(initialMyx, internalCalibrator2.initialMyx, 0.0)
        assertEquals(initialMyz, internalCalibrator2.initialMyz, 0.0)
        assertEquals(initialMzx, internalCalibrator2.initialMzx, 0.0)
        assertEquals(initialMzy, internalCalibrator2.initialMzy, 0.0)
        assertEquals(ROBUST_CONFIDENCE, internalCalibrator2.confidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, internalCalibrator2.maxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, internalCalibrator2.preliminarySubsetSize)
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )
        assertEquals(calibrator.measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGravityMissingGravityNorm_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildInternalCalibrator_whenPROMedSGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerCalibrator(context, isGroundTruthInitialBias = false)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
        calibrator.robustMethod = RobustEstimatorMethod.PROMedS
        calibrator.robustConfidence = ROBUST_CONFIDENCE
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.robustThreshold = null
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertEquals(gravityNorm, calibrator.gravityNorm)
        assertNull(calibrator.baseNoiseLevel)
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
        val robustThreshold = calibrator.robustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
        assertNull(calibrator.baseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    private companion object {
        const val MA_SIZE = 3

        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_ANGLE_DEGREES = -180.0
        const val MAX_ANGLE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        const val WINDOW_SIZE = 51

        const val REQUIRED_MEASUREMENTS = 30

        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 15

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val TIME_INTERVAL_SECONDS = 0.02

        const val ACCUMULATED_STD = 0.007

        const val MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6

        const val DEG_TO_RAD = 0.01745329252

        const val ABSOLUTE_ERROR = 1e-6

        fun getLocation(): Location {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
            val longitudeDegrees =
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }

        fun generateBa(): Matrix {
            return Matrix.newFromArray(
                doubleArrayOf(
                    900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                    -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                    800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED
                )
            )
        }

        fun generateBg(): Matrix {
            return Matrix.newFromArray(
                doubleArrayOf(
                    -9 * DEG_TO_RAD / 3600.0,
                    13 * DEG_TO_RAD / 3600.0,
                    -8 * DEG_TO_RAD / 3600.0
                )
            )
        }

        private fun generateMaGeneral(): Matrix {
            val result = Matrix(3, 3)
            result.fromArray(
                doubleArrayOf(
                    500e-6, -300e-6, 200e-6,
                    -150e-6, -600e-6, 250e-6,
                    -250e-6, 100e-6, 450e-6
                ), false
            )

            return result
        }

        fun generateMg(): Matrix {
            val result = Matrix(3, 3)
            result.fromArray(
                doubleArrayOf(
                    400e-6, -300e-6, 250e-6,
                    0.0, -300e-6, -150e-6,
                    0.0, 0.0, -350e-6
                ), false
            )

            return result
        }

        fun generateGg(): Matrix {
            val result = Matrix(3, 3)
            val tmp = DEG_TO_RAD / (3600 * 9.80665)
            result.fromArray(
                doubleArrayOf(
                    0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                    -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                    0.3 * tmp, 1.1 * tmp, -1.3 * tmp
                ), false
            )

            return result
        }
    }
}