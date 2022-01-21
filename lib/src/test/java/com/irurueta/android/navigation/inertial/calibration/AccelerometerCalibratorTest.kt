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
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.every
import io.mockk.mockk
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

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
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenSolveCalibrationWhenEnoughMeasurements_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenGroundTruthInitialBias_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenLocation_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenUnreliableGravityNormEstimationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenInitialBiasAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenNewCalibrationMeasurementAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<AccelerometerCalibrator.OnStoppedListener>()
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenAccelerometerMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<AccelerometerCalibrator.OnStoppedListener>()
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
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenGravityMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<AccelerometerCalibrator.OnStoppedListener>()
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
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun constructor_whenQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<AccelerometerCalibrator.OnStoppedListener>()
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
        val calibrator = AccelerometerCalibrator(
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
        assertEquals(getGravityNormForLocation(location), calibrator.gravityNorm)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.resultUnreliable)
        assertNull(calibrator.initialBiasX)
        assertNull(calibrator.initialBiasY)
        assertNull(calibrator.initialBiasZ)
        assertNull(calibrator.initialBiasXAsAcceleration)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getInitialBiasXAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasYAsAcceleration)
        assertFalse(calibrator.getInitialBiasYAsAcceleration(acceleration))
        assertNull(calibrator.initialBiasZAsAcceleration)
        assertFalse(calibrator.getInitialBiasZAsAcceleration(acceleration))
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsAcceleration
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsAcceleration)
        assertFalse(calibrator.getBaseNoiseLevelAsAcceleration(acceleration))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsAcceleration)
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
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            AccelerometerCalibrator.KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            AccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.averageGravityNorm)
        assertNull(calibrator.averageGravityNormAsAcceleration)
        assertFalse(calibrator.getAverageNormAsAcceleration(acceleration))
        assertNull(calibrator.gravityNormVariance)
        assertNull(calibrator.gravityNormStandardDeviation)
        assertNull(calibrator.gravityNormStandardDeviationAsAcceleration)
        assertFalse(calibrator.getGravityNormStandardDeviationAsAcceleration(acceleration))
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
        assertNull(calibrator.estimatedBiasXAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasXAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasYAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasYAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasZAsAcceleration)
        assertFalse(calibrator.getEstimatedBiasZAsAcceleration(acceleration))
        assertNull(calibrator.estimatedBiasAsTriad)
        assertFalse(calibrator.getEstimatedBiasAsTriad(triad))
        assertNull(calibrator.estimatedBiasStandardDeviationNorm)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<AccelerometerCalibrator.OnInitializationStartedListener>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<AccelerometerCalibrator.OnInitializationCompletedListener>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener = mockk<AccelerometerCalibrator.OnErrorListener>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun unreliableGravityNormEstimatorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.unreliableGravityNormEstimationListener)

        // set new value
        val unreliableGravityNormEstimationListener =
            mockk<AccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
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
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialBiasAvailableListener)

        // set new value
        val initialBiasAvailableListener =
            mockk<AccelerometerCalibrator.OnInitialBiasAvailableListener>()
        calibrator.initialBiasAvailableListener = initialBiasAvailableListener

        // check
        assertSame(initialBiasAvailableListener, calibrator.initialBiasAvailableListener)
    }

    @Test
    fun newCalibrationMeasurementAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)

        // set new value
        val newCalibrationMeasurementAvailableListener =
            mockk<AccelerometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
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
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<AccelerometerCalibrator.OnReadyToSolveCalibrationListener>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<AccelerometerCalibrator.OnCalibrationSolvingStartedListener>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<AccelerometerCalibrator.OnCalibrationCompletedListener>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener = mockk<AccelerometerCalibrator.OnStoppedListener>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = AccelerometerCalibrator(context)

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
        val calibrator = AccelerometerCalibrator(context)

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
        val calibrator = AccelerometerCalibrator(context)

        // check default value
        assertNotNull(calibrator.qualityScoreMapper)

        // set new value
        val qualityScoreMapper = mockk<QualityScoreMapper<StandardDeviationBodyKinematics>>()
        calibrator.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, calibrator.qualityScoreMapper)
    }

    private companion object {
        const val MA_SIZE = 3

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        fun getGravityNormForLocation(location: Location): Double {
            val nedPosition = location.toNEDPosition()
            val nedVelocity = NEDVelocity()
            val ecefPosition = ECEFPosition()
            val ecefVelocity = ECEFVelocity()
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition,
                nedVelocity,
                ecefPosition,
                ecefVelocity
            )
            val gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                ecefPosition.x,
                ecefPosition.y,
                ecefPosition.z
            )
            return gravity.norm
        }
    }
}