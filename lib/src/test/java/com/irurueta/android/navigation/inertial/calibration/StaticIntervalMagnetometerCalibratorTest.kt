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
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.WrongSizeException
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.MagnetometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.callPrivateFuncWithResult
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.*
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.lang.reflect.InvocationTargetException
import java.util.*

@RunWith(RobolectricTestRunner::class)
class StaticIntervalMagnetometerCalibratorTest {

    @Test
    fun constructor_whenContextAndLocation_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertNotNull(calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenContextLocationAndTimestamp_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenWorldMagneticModel_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(context, location, timestamp, worldMagneticModel)

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAccelerometerSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenMagnetometerSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAccelerometerSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenMagnetometerSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenSolveCalibrationWhenEnoughMeasurements_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                false
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenIsMagnetometerGroundTruthInitialHardIron_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenStaticIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenDynamicIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenStaticIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenDynamicIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenGeneratedMagnetometerMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenInitialMagnetometerHardIronAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>()
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener,
                initialMagnetometerHardIronAvailableListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAccuracyChangedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>()
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
        val accuracyChanged = mockk<SensorCollector.OnAccuracyChangedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener,
                initialMagnetometerHardIronAvailableListener,
                accuracyChanged
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
        assertSame(accuracyChanged, calibrator.accuracyChangedListener)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenMagnetometerQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>()
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
        val accuracyChanged = mockk<SensorCollector.OnAccuracyChangedListener>()
        val magnetometerQualityScoreMapper =
            mockk<QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isMagnetometerGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                staticIntervalDetectedListener,
                dynamicIntervalDetectedListener,
                staticIntervalSkippedListener,
                dynamicIntervalSkippedListener,
                generatedMagnetometerMeasurementListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener,
                initialMagnetometerHardIronAvailableListener,
                accuracyChanged,
                magnetometerQualityScoreMapper
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
        assertSame(accuracyChanged, calibrator.accuracyChangedListener)
        assertSame(magnetometerQualityScoreMapper, calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val magneticFluxDensity = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.magnetometerSensor)
        val mg1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mg1, calibrator.magnetometerInitialMm)
        val mg2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mg2)
        assertEquals(mg1, mg2)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredMagnetometerMeasurements,
            calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMagnetometerMm)
        assertNull(calibrator.estimatedMagnetometerSx)
        assertNull(calibrator.estimatedMagnetometerSy)
        assertNull(calibrator.estimatedMagnetometerSz)
        assertNull(calibrator.estimatedMagnetometerMxy)
        assertNull(calibrator.estimatedMagnetometerMxz)
        assertNull(calibrator.estimatedMagnetometerMyx)
        assertNull(calibrator.estimatedMagnetometerMyz)
        assertNull(calibrator.estimatedMagnetometerMzx)
        assertNull(calibrator.estimatedMagnetometerMzy)
        assertNull(calibrator.estimatedMagnetometerCovariance)
        assertNull(calibrator.estimatedMagnetometerChiSq)
        assertNull(calibrator.estimatedMagnetometerMse)
        assertNull(calibrator.estimatedMagnetometerHardIronX)
        assertNull(calibrator.estimatedMagnetometerHardIronY)
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(magneticFluxDensity))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(magneticFluxDensity))
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)
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
        assertNull(calibrator.accelerometerBaseNoiseLevel)
        assertNull(calibrator.accelerometerBaseNoiseLevelAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerBaseNoiseLevelPsd)
        assertNull(calibrator.accelerometerBaseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMeasurement)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(0, calibrator.processedStaticSamples)
        assertEquals(0, calibrator.processedDynamicSamples)
        assertFalse(calibrator.isStaticIntervalSkipped)
        assertFalse(calibrator.isDynamicIntervalSkipped)
        assertNull(calibrator.accelerometerAverageTimeInterval)
        assertNull(calibrator.accelerometerAverageTimeIntervalAsTime)
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertNull(calibrator.accelerometerTimeIntervalVariance)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.generatedMagnetometerMeasurementListener)

        // set new value
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        calibrator.generatedMagnetometerMeasurementListener =
            generatedMagnetometerMeasurementListener

        // check
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
    }

    @Test
    fun readyToSolveCalibrationListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun initialMagnetometerHardIronAvailableListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)

        // set new value
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
        calibrator.initialMagnetometerHardIronAvailableListener =
            initialMagnetometerHardIronAvailableListener

        // check
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        calibrator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
    }

    @Test
    fun isMagnetometerGroundTruthInitialHardIron_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)

        // set new value
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
    }

    @Test(expected = IllegalStateException::class)
    fun isMagnetometerGroundTruthInitialHardIron_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isMagnetometerGroundTruthInitialHardIron = true
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val location1 = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location1)

        // check default value
        assertSame(location1, calibrator.location)
        assertFalse(calibrator.running)

        val location2 = mockk<Location>()
        calibrator.location = location2

        // check
        assertSame(location2, calibrator.location)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunning_throwsIllegalStateException() {
        val location1 = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location1)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check
        assertSame(location1, calibrator.location)
        assertTrue(calibrator.running)

        // set new value
        val location2 = mockk<Location>()
        calibrator.location = location2
    }

    @Test
    fun timestamp_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val timestamp1 = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp1)

        // check default value
        assertEquals(timestamp1, calibrator.timestamp)
        assertFalse(calibrator.running)

        val timestamp2 = Date(0)
        calibrator.timestamp = timestamp2

        // check
        assertEquals(timestamp2, calibrator.timestamp)
    }

    @Test(expected = IllegalStateException::class)
    fun timestamp_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val timestamp1 = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp1)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check
        assertEquals(timestamp1, calibrator.timestamp)
        assertTrue(calibrator.running)

        // set new value
        val timestamp2 = Date()
        calibrator.timestamp = timestamp2
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.worldMagneticModel)
        assertFalse(calibrator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        assertNull(calibrator.worldMagneticModel)
        assertTrue(calibrator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun magnetometerInitialMm_whenValid_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(Matrix(MM_SIZE, MM_SIZE), calibrator.magnetometerInitialMm)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

        // set new values
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

        val mm = Matrix(MM_SIZE, MM_SIZE)
        mm.setElementAtIndex(0, initialSx)
        mm.setElementAtIndex(1, initialMyx)
        mm.setElementAtIndex(2, initialMzx)

        mm.setElementAtIndex(3, initialMxy)
        mm.setElementAtIndex(4, initialSy)
        mm.setElementAtIndex(5, initialMzy)

        mm.setElementAtIndex(6, initialMxz)
        mm.setElementAtIndex(7, initialMyz)
        mm.setElementAtIndex(8, initialSz)

        calibrator.magnetometerInitialMm = mm

        // check
        assertEquals(mm, calibrator.magnetometerInitialMm)
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val mm = Matrix(1, MM_SIZE)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerInitialMm_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMm_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMm = Matrix(MM_SIZE, MM_SIZE)
    }

    @Test
    fun getMagnetometerInitialMm_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        val mm = Matrix(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm)

        // check
        assertEquals(initialSx, mm.getElementAtIndex(0), 0.0)
        assertEquals(initialMyx, mm.getElementAtIndex(1), 0.0)
        assertEquals(initialMzx, mm.getElementAtIndex(2), 0.0)

        assertEquals(initialMxy, mm.getElementAtIndex(3), 0.0)
        assertEquals(initialSy, mm.getElementAtIndex(4), 0.0)
        assertEquals(initialMzy, mm.getElementAtIndex(5), 0.0)

        assertEquals(initialMxz, mm.getElementAtIndex(6), 0.0)
        assertEquals(initialMyz, mm.getElementAtIndex(7), 0.0)
        assertEquals(initialSz, mm.getElementAtIndex(8), 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getMagnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val mm = Matrix(1, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getMagnetometerInitialMm_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test
    fun magnetometerInitialSx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialSx = 0.0
    }

    @Test
    fun magnetometerInitialSy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialSy = 0.0
    }

    @Test
    fun magnetometerInitialSz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialSz = 0.0
    }

    @Test
    fun magnetometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMxy = 0.0
    }

    @Test
    fun magnetometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxz_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMxz = 0.0
    }

    @Test
    fun magnetometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMyx = 0.0
    }

    @Test
    fun magnetometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMyz = 0.0
    }

    @Test
    fun magnetometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMzx = 0.0
    }

    @Test
    fun magnetometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerInitialMzy = 0.0
    }

    @Test
    fun setMagnetometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(initialSx, initialSy, initialSz)

        // check
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(initialSx, initialSy, initialSz)
    }

    @Test
    fun setMagnetometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)

        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
    fun isMagnetometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertFalse(calibrator.isMagnetometerCommonAxisUsed)

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isMagnetometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isMagnetometerCommonAxisUsed = true
    }

    @Test
    fun magnetometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerRobustMethod)

        // set new value
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun magnetometerRobustConfidence_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun magnetometerRobustMaxIterations_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )

        // set new value
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun magnetometerRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun magnetometerRobustThreshold_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerRobustThreshold)

        // set new value
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.magnetometerRobustThreshold = null

        // check
        assertNull(calibrator.magnetometerRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun magnetometerRobustThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun magnetometerRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustStopThresholdFactor_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdMeasurement_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val value = Acceleration(
            BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

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
    fun requiredMeasurements_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )

        // set new value
        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS

        // check
        assertEquals(REQUIRED_MEASUREMENTS, calibrator.requiredMeasurements)
    }

    @Test(expected = IllegalArgumentException::class)
    fun requiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
    }

    @Test
    fun onInitializationStarted_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            initializationStartedListener = initializationStartedListener
        )

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_makesNoAction() {
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            initializationCompletedListener = initializationCompletedListener
        )

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )

        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsGenerator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorErrorListener")
        requireNotNull(generatorErrorListener)

        generatorErrorListener.onError(generatorSpy, ErrorReason.UNRELIABLE_SENSOR)

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
    }

    @Test
    fun onError_whenListenersAvailable_stopsAndNotifies() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            errorListener = errorListener,
            stoppedListener = stoppedListener
        )

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorErrorListener")
        requireNotNull(generatorErrorListener)

        generatorErrorListener.onError(generatorSpy, ErrorReason.UNRELIABLE_SENSOR)

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                CalibratorErrorReason.UNRELIABLE_SENSOR
            )
        }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test
    fun onStaticIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedMeasurement_addsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])
    }

    @Test
    fun onGeneratedMeasurement_whenListenerAvailable_notifies() {
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<MagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])

        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            )
        }
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val nedPosition = location.toNEDPosition()

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val randomizer = UniformRandomizer()

        for (i in 1..reqMeasurements) {
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val cnb = CoordinateTransformation(
                roll,
                pitch,
                yaw,
                FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME
            )

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val measurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            StandardDeviationBodyMagneticFluxDensity(calibrator.magnetometerMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedMagnetometerMm)
        assertNotNull(calibrator.estimatedMagnetometerSx)
        assertNotNull(calibrator.estimatedMagnetometerSy)
        assertNotNull(calibrator.estimatedMagnetometerSz)
        assertNotNull(calibrator.estimatedMagnetometerMxy)
        assertNotNull(calibrator.estimatedMagnetometerMxz)
        assertNotNull(calibrator.estimatedMagnetometerMyx)
        assertNotNull(calibrator.estimatedMagnetometerMyz)
        assertNotNull(calibrator.estimatedMagnetometerMzx)
        assertNotNull(calibrator.estimatedMagnetometerMzy)
        assertNotNull(calibrator.estimatedMagnetometerCovariance)
        assertNotNull(calibrator.estimatedMagnetometerChiSq)
        assertNotNull(calibrator.estimatedMagnetometerMse)
        assertNotNull(calibrator.estimatedMagnetometerHardIronX)
        assertNotNull(calibrator.estimatedMagnetometerHardIronY)
        assertNotNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNotNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = true,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            stoppedListener = stoppedListener,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener,
            errorListener = errorListener
        )

        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val nedPosition = location.toNEDPosition()

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val randomizer = UniformRandomizer()

        for (i in 1..reqMeasurements) {
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val cnb = CoordinateTransformation(
                roll,
                pitch,
                yaw,
                FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME
            )

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val measurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            StandardDeviationBodyMagneticFluxDensity(calibrator.magnetometerMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedMagnetometerMm)
        assertNotNull(calibrator.estimatedMagnetometerSx)
        assertNotNull(calibrator.estimatedMagnetometerSy)
        assertNotNull(calibrator.estimatedMagnetometerSz)
        assertNotNull(calibrator.estimatedMagnetometerMxy)
        assertNotNull(calibrator.estimatedMagnetometerMxz)
        assertNotNull(calibrator.estimatedMagnetometerMyx)
        assertNotNull(calibrator.estimatedMagnetometerMyz)
        assertNotNull(calibrator.estimatedMagnetometerMzx)
        assertNotNull(calibrator.estimatedMagnetometerMzy)
        assertNotNull(calibrator.estimatedMagnetometerCovariance)
        assertNotNull(calibrator.estimatedMagnetometerChiSq)
        assertNotNull(calibrator.estimatedMagnetometerMse)
        assertNotNull(calibrator.estimatedMagnetometerHardIronX)
        assertNotNull(calibrator.estimatedMagnetometerHardIronY)
        assertNotNull(calibrator.estimatedMagnetometerHardIronZ)
        assertNotNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNotNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }
        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
        verify { errorListener wasNot Called }
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurement_updatesInitialHardIron() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        val initialHardIronX = calibrator.magnetometerInitialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.magnetometerInitialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.magnetometerInitialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronX.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronX, 0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronY.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronY, 0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronZ.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronZ, 0.0
        )
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronX_updatesInitialHardIron() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            null,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        val initialHardIronX = calibrator.magnetometerInitialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.magnetometerInitialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.magnetometerInitialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronY_updatesInitialHardIron() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            null,
            hardIronZ,
            timestamp,
            accuracy
        )

        val initialHardIronX = calibrator.magnetometerInitialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.magnetometerInitialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.magnetometerInitialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronZ_updatesInitialHardIron() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            null,
            timestamp,
            accuracy
        )

        val initialHardIronX = calibrator.magnetometerInitialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.magnetometerInitialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.magnetometerInitialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurementAndListener_updatesInitialHardIron() {
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            initialMagnetometerHardIronAvailableListener = initialMagnetometerHardIronAvailableListener
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        val initialHardIronX = calibrator.magnetometerInitialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.magnetometerInitialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.magnetometerInitialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronX.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronX, 0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronY.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronY, 0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.convert(
                hardIronZ.toDouble(),
                MagneticFluxDensityUnit.MICROTESLA,
                MagneticFluxDensityUnit.TESLA
            ), initialHardIronZ, 0.0
        )

        verify(exactly = 1) {
            initialMagnetometerHardIronAvailableListener.onInitialHardIronAvailable(
                calibrator,
                MagneticFluxDensityConverter.convert(
                    hardIronX.toDouble(),
                    MagneticFluxDensityUnit.MICROTESLA,
                    MagneticFluxDensityUnit.TESLA
                ),
                MagneticFluxDensityConverter.convert(
                    hardIronY.toDouble(),
                    MagneticFluxDensityUnit.MICROTESLA,
                    MagneticFluxDensityUnit.TESLA
                ),
                MagneticFluxDensityConverter.convert(
                    hardIronZ.toDouble(),
                    MagneticFluxDensityUnit.MICROTESLA,
                    MagneticFluxDensityUnit.TESLA
                )
            )
        }
    }

    @Test
    fun onMagnetometerMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            initialMagnetometerHardIronAvailableListener = initialMagnetometerHardIronAvailableListener
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(2)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorMagnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorMagnetometerMeasurementListener")
        requireNotNull(generatorMagnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorMagnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        verify { initialMagnetometerHardIronAvailableListener wasNot Called }
    }

    @Test
    fun magnetometerBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        verify(exactly = 1) { generatorSpy.magnetometerBaseNoiseLevel }
    }

    @Test
    fun magnetometerBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val baseNoiseLevel1 = MagneticFluxDensity(baseNoiseLevel, MagneticFluxDensityUnit.TESLA)
        every { generatorSpy.magnetometerBaseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val baseNoiseLevel2 = calibrator.magnetometerBaseNoiseLevelAsMeasurement
        assertSame(baseNoiseLevel1, baseNoiseLevel2)

        verify(exactly = 1) { generatorSpy.magnetometerBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getMagnetometerBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        // check default value
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.getMagnetometerBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = baseNoiseLevel
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))

        // check
        assertEquals(baseNoiseLevel, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
        verify(exactly = 1) { generatorSpy.getMagnetometerBaseNoiseLevelAsMeasurement(b) }
    }

    @Test
    fun accelerometerBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.accelerometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(baseNoiseLevel, calibrator.accelerometerBaseNoiseLevel)

        verify(exactly = 1) { generatorSpy.accelerometerBaseNoiseLevel }
    }

    @Test
    fun accelerometerBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val baseNoiseLevel1 =
            Acceleration(baseNoiseLevel, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { generatorSpy.accelerometerBaseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val baseNoiseLevel2 = calibrator.accelerometerBaseNoiseLevelAsMeasurement
        assertSame(baseNoiseLevel1, baseNoiseLevel2)
        verify(exactly = 1) { generatorSpy.accelerometerBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getAccelerometerBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = baseNoiseLevel
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))

        // check
        assertEquals(baseNoiseLevel, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) { generatorSpy.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration) }
    }

    @Test
    fun accelerometerBaseNoiseLevelPsd_getsGeneratorBaseNoiseLevelPsd() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd = randomizer.nextDouble()
        every { generatorSpy.accelerometerBaseNoiseLevelPsd }.returns(baseNoiseLevelPsd)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(baseNoiseLevelPsd, calibrator.accelerometerBaseNoiseLevelPsd)
        verify(exactly = 1) { generatorSpy.accelerometerBaseNoiseLevelPsd }
    }

    @Test
    fun accelerometerBaseNoiseLevelRootPsd_getsGeneratorBaseNoiseLevelRootPsd() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd = randomizer.nextDouble()
        every { generatorSpy.accelerometerBaseNoiseLevelRootPsd }.returns(baseNoiseLevelRootPsd)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(baseNoiseLevelRootPsd, calibrator.accelerometerBaseNoiseLevelRootPsd)
        verify(exactly = 1) { generatorSpy.accelerometerBaseNoiseLevelRootPsd }
    }

    @Test
    fun threshold_getsGeneratorThreshold() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.threshold)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { generatorSpy.threshold }.returns(threshold)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(threshold, calibrator.threshold)
        verify(exactly = 1) { generatorSpy.threshold }
    }

    @Test
    fun thresholdAsMeasurement_getsGeneratorThresholdAsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.thresholdAsMeasurement)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        val acceleration = Acceleration(threshold, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { generatorSpy.thresholdAsMeasurement }.returns(acceleration)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertSame(acceleration, calibrator.thresholdAsMeasurement)
        verify(exactly = 1) { generatorSpy.thresholdAsMeasurement }
    }

    @Test
    fun getThresholdAsMeasurement_getsGeneratorThresholdAsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { generatorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = threshold
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getThresholdAsMeasurement(acceleration))
        assertEquals(threshold, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
        verify(exactly = 1) { generatorSpy.getThresholdAsMeasurement(acceleration) }
    }

    @Test
    fun processedStaticSamples_getsGeneratorProcessedStaticSamples() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val processedStaticSamples = randomizer.nextInt()
        every { generatorSpy.processedStaticSamples }.returns(processedStaticSamples)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(processedStaticSamples, calibrator.processedStaticSamples)
        verify(exactly = 1) { generatorSpy.processedStaticSamples }
    }

    @Test
    fun processedDynamicSamples_getsGeneratorProcessedDynamicSamples() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val processedDynamicSamples = randomizer.nextInt()
        every { generatorSpy.processedDynamicSamples }.returns(processedDynamicSamples)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(processedDynamicSamples, calibrator.processedDynamicSamples)
        verify(exactly = 1) { generatorSpy.processedDynamicSamples }
    }

    @Test
    fun isStaticIntervalSkipped_getsGeneratorStaticIntervalSkipped() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.isStaticIntervalSkipped }.returns(true)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.isStaticIntervalSkipped)
        verify(exactly = 1) { generatorSpy.isStaticIntervalSkipped }
    }

    @Test
    fun isDynamicIntervalSkipped_getsGeneratorStaticIntervalSkipped() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.isDynamicIntervalSkipped }.returns(true)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.isDynamicIntervalSkipped)
        verify(exactly = 1) { generatorSpy.isDynamicIntervalSkipped }
    }

    @Test
    fun accelerometerAverageTimeInterval_getsGeneratorAccelerometerAverageTimeInterval() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { generatorSpy.accelerometerAverageTimeInterval }.returns(averageTimeInterval)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(averageTimeInterval, calibrator.accelerometerAverageTimeInterval)
        verify(exactly = 1) { generatorSpy.accelerometerAverageTimeInterval }
    }

    @Test
    fun accelerometerAverageTimeIntervalAsTime_getsGeneratorAccelerometerAverageTimeInterval() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        val time = Time(averageTimeInterval, TimeUnit.SECOND)
        every { generatorSpy.accelerometerAverageTimeIntervalAsTime }.returns(time)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertSame(time, calibrator.accelerometerAverageTimeIntervalAsTime)
        verify(exactly = 1) { generatorSpy.accelerometerAverageTimeIntervalAsTime }
    }

    @Test
    fun getAccelerometerAverageTimeIntervalAsTime_getsGeneratorAccelerometerAverageTimeInterval() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { generatorSpy.getAccelerometerAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = averageTimeInterval
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))
        assertEquals(averageTimeInterval, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { generatorSpy.getAccelerometerAverageTimeIntervalAsTime(time) }
    }

    @Test
    fun accelerometerTimeIntervalVariance_getsGeneratorAccelerometerTimeIntervalVariance() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalVariance)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        every { generatorSpy.accelerometerTimeIntervalVariance }.returns(timeIntervalVariance)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(timeIntervalVariance, calibrator.accelerometerTimeIntervalVariance)
        verify(exactly = 1) { generatorSpy.accelerometerTimeIntervalVariance }
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviation_getsGeneratorAccelerometerTimeIntervalStandardDeviation() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        every { generatorSpy.accelerometerTimeIntervalStandardDeviation }.returns(
            timeIntervalStandardDeviation
        )
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(
            timeIntervalStandardDeviation,
            calibrator.accelerometerTimeIntervalStandardDeviation
        )
        verify(exactly = 1) { generatorSpy.accelerometerTimeIntervalStandardDeviation }
    }

    @Test
    fun accelerometerTimeIntervalStandardDeviationAsTime_getsGeneratorAccelerometerTimeIntervalStandardDeviationAsTime() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time = Time(value, TimeUnit.SECOND)
        every { generatorSpy.accelerometerTimeIntervalStandardDeviationAsTime }.returns(time)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertSame(time, calibrator.accelerometerTimeIntervalStandardDeviationAsTime)
        verify(exactly = 1) { generatorSpy.accelerometerTimeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getAccelerometerTimeIntervalStandardDeviationAsTime_getsGeneratorAccelerometerTimeIntervalStandardDeviationAsTime() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        every { generatorSpy.getAccelerometerTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))
        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { generatorSpy.getAccelerometerTimeIntervalStandardDeviationAsTime(time) }
    }

    @Test
    fun numberOfProcessedMagnetometerMeasurements_getsGeneratorNumberOfProcessedMagnetometerMeasurements() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val numberOfProcessedMagnetometerMeasurements = randomizer.nextInt()
        every { generatorSpy.numberOfProcessedMagnetometerMeasurements }.returns(
            numberOfProcessedMagnetometerMeasurements
        )
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(
            numberOfProcessedMagnetometerMeasurements,
            calibrator.numberOfProcessedMagnetometerMeasurements
        )

        verify(exactly = 1) { generatorSpy.numberOfProcessedMagnetometerMeasurements }
    }

    @Test
    fun numberOfProcessedAccelerometerMeasurements_getsGeneratorNumberOfProcessedAccelerometerMeasurements() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val numberOfProcessedAccelerometerMeasurements = randomizer.nextInt()
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(
            numberOfProcessedAccelerometerMeasurements
        )
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(
            numberOfProcessedAccelerometerMeasurements,
            calibrator.numberOfProcessedAccelerometerMeasurements
        )

        verify(exactly = 1) { generatorSpy.numberOfProcessedAccelerometerMeasurements }
    }

    @Test
    fun magnetometerInitialHardIronX_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronX)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)

        assertEquals(initialHardIronX, calibrator.magnetometerInitialHardIronX)
    }

    @Test
    fun magnetometerInitialHardIronY_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronY)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)

        assertEquals(initialHardIronY, calibrator.magnetometerInitialHardIronY)
    }

    @Test
    fun magnetometerInitialHardIronZ_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronZ)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)

        assertEquals(initialHardIronZ, calibrator.magnetometerInitialHardIronZ)
    }

    @Test
    fun magnetometerInitialHardIronXAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)

        val hardIron = calibrator.magnetometerInitialHardIronXAsMeasurement
        requireNotNull(hardIron)
        assertEquals(initialHardIronX, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getMagnetometerInitialHardIronXAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)

        assertTrue(calibrator.getMagnetometerInitialHardIronXAsMeasurement(hardIron))
        assertEquals(initialHardIronX, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun magnetometerInitialHardIronYAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)

        val hardIron = calibrator.magnetometerInitialHardIronYAsMeasurement
        requireNotNull(hardIron)
        assertEquals(initialHardIronY, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getMagnetometerInitialHardIronYAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)

        assertTrue(calibrator.getMagnetometerInitialHardIronYAsMeasurement(hardIron))
        assertEquals(initialHardIronY, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun magnetometerInitialHardIronZAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)

        val hardIron = calibrator.magnetometerInitialHardIronZAsMeasurement
        requireNotNull(hardIron)
        assertEquals(initialHardIronZ, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getMagnetometerInitialHardIronZAsMeasurement_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)

        assertTrue(calibrator.getMagnetometerInitialHardIronZAsMeasurement(hardIron))
        assertEquals(initialHardIronZ, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun magnetometerInitialHardIronAsTriad_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)

        assertNull(calibrator.magnetometerInitialHardIronAsTriad)

        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)

        assertNull(calibrator.magnetometerInitialHardIronAsTriad)

        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)

        // check
        val triad = calibrator.magnetometerInitialHardIronAsTriad
        requireNotNull(triad)
        assertEquals(initialHardIronX, triad.valueX, 0.0)
        assertEquals(initialHardIronY, triad.valueY, 0.0)
        assertEquals(initialHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun getMagnetometerInitialHardIronAsTriad_getsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)

        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))

        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)

        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))

        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)

        // check
        assertTrue(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertEquals(initialHardIronX, triad.valueX, 0.0)
        assertEquals(initialHardIronY, triad.valueY, 0.0)
        assertEquals(initialHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(7, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(13, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsGenerator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertFalse(calibrator.running)

        val measurements = calibrator.magnetometerMeasurements
        val measurementsSpy = spyk(measurements)
        calibrator.setPrivateProperty("magnetometerMeasurements", measurementsSpy)

        calibrator.setPrivateProperty("magnetometerInitialHardIronX", 0.0)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", 0.0)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", 0.0)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.start() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        calibrator.start()

        // check
        verify(exactly = 1) { measurementsSpy.clear() }
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { generatorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertFalse(calibrator.running)

        calibrator.start()

        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun stop_whenNoListenerAvailable_stopsGenerator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
    }

    @Test
    fun stop_whenListenerAvailable_notifies() {
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            stoppedListener = stoppedListener
        )

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenNotReadyToSolveCalibration_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.calibrate()
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.calibrate()
    }

    @Test
    fun calibrate_whenReadyNotRunningAndNoInternalCalibrator_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        justRun { internalCalibrator.calibrate() }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

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
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            errorListener = errorListener
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                CalibratorErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION
            )
        }
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronXAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronXAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronXAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronXAsMeasurement)
    }

    @Test
    fun getEstimatedMagnetometerHardIronXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronXAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronXAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronX
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))

        assertEquals(estimatedHardIronX, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun getEstimatedMagnetometerHardIronXAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronXAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronX
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))

        assertEquals(estimatedHardIronX, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedMagnetometerHardIronYAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronYAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronYAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronYAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronYAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronYAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronYAsMeasurement)
    }

    @Test
    fun getEstimatedMagnetometerHardIronYAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronYAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronYAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronY
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))

        assertEquals(estimatedHardIronY, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun getEstimatedMagnetometerHardIronYAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronYAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronY
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))

        assertEquals(estimatedHardIronY, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedMagnetometerHardIronZAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronZAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronZAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronZAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronZAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronZAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(b, calibrator.estimatedMagnetometerHardIronZAsMeasurement)
    }

    @Test
    fun getEstimatedMagnetometerHardIronZAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronZAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronZAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))

        assertEquals(estimatedHardIronZ, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun getEstimatedMagnetometerHardIronZAsMeasurement_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronZAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))

        assertEquals(estimatedHardIronZ, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedMagnetometerHardIronAsTriad_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
    }

    @Test
    fun estimatedMagnetometerHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val estimatedHardIronY = randomizer.nextDouble()
        val estimatedHardIronZ = randomizer.nextDouble()
        val triad = MagneticFluxDensityTriad(
            MagneticFluxDensityUnit.TESLA,
            estimatedHardIronX,
            estimatedHardIronY,
            estimatedHardIronZ
        )
        every { internalCalibratorSpy.estimatedHardIronAsTriad }.returns(triad)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedMagnetometerHardIronAsTriad)
    }

    @Test
    fun estimatedMagnetometerHardIronAsTriad_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val estimatedHardIronY = randomizer.nextDouble()
        val estimatedHardIronZ = randomizer.nextDouble()
        val triad = MagneticFluxDensityTriad(
            MagneticFluxDensityUnit.TESLA,
            estimatedHardIronX,
            estimatedHardIronY,
            estimatedHardIronZ
        )
        every { internalCalibratorSpy.hardIronAsTriad }.returns(triad)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(triad, calibrator.estimatedMagnetometerHardIronAsTriad)
    }

    @Test
    fun getEstimatedMagnetometerHardIronAsTriad_whenNoInternalCalibrator_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
    }

    @Test
    fun getEstimatedMagnetometerHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val estimatedHardIronY = randomizer.nextDouble()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronAsTriad(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensityTriad
            result.setValueCoordinatesAndUnit(
                estimatedHardIronX,
                estimatedHardIronY,
                estimatedHardIronZ,
                MagneticFluxDensityUnit.TESLA
            )
            return@answers true
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))

        assertEquals(estimatedHardIronX, triad.valueX, 0.0)
        assertEquals(estimatedHardIronY, triad.valueY, 0.0)
        assertEquals(estimatedHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun getEstimatedMagnetometerHardIronAsTriad_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val estimatedHardIronY = randomizer.nextDouble()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronAsTriad(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensityTriad
            result.setValueCoordinatesAndUnit(
                estimatedHardIronX,
                estimatedHardIronY,
                estimatedHardIronZ,
                MagneticFluxDensityUnit.TESLA
            )
        }
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))

        assertEquals(estimatedHardIronX, triad.valueX, 0.0)
        assertEquals(estimatedHardIronY, triad.valueY, 0.0)
        assertEquals(estimatedHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenNonRobustGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
            calibrator.minimumRequiredMagnetometerMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenNonRobustGroundTruthHardIronSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
            calibrator.minimumRequiredMagnetometerMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenNonRobustGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.initialHardIronZ, 0.0)
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
            calibrator.minimumRequiredMagnetometerMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenNonRobustAndGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.initialHardIronZ, 0.0)
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
            calibrator.minimumRequiredMagnetometerMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

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
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = true

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("magnetometerInitialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialHardIronX, internalCalibrator2.hardIronX, 0.0)
        assertEquals(initialHardIronY, internalCalibrator2.hardIronY, 0.0)
        assertEquals(initialHardIronZ, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialHardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialHardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.magnetometerBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(ROBUST_STOP_THRESHOLD_FACTOR, calibrator.magnetometerRobustStopThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.magnetometerMeasurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.hardIronX, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronY, 0.0)
        assertEquals(0.0, internalCalibrator2.hardIronZ, 0.0)
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
        assertEquals(calibrator.magnetometerMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        for (i in 1..13) {
            calibrator.magnetometerMeasurements.add(measurement)
        }

        calibrator.isMagnetometerCommonAxisUsed = false

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.magnetometerRobustPreliminarySubsetSize)
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    private companion object {
        const val MM_SIZE = 3

        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

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

        const val MIN_HARD_IRON = -1e-5
        const val MAX_HARD_IRON = 1e-5

        const val MIN_SOFT_IRON = -1e-6
        const val MAX_SOFT_IRON = 1e-6

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val MAGNETOMETER_NOISE_STD = 200e-9

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

        fun generateHardIron(): DoubleArray {
            val result = DoubleArray(BodyMagneticFluxDensity.COMPONENTS)
            val randomizer = UniformRandomizer()
            randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON)
            return result
        }

        fun generateSoftIron(): Matrix? {
            return try {
                Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS,
                    MIN_SOFT_IRON,
                    MAX_SOFT_IRON
                )
            } catch (ignore: WrongSizeException) {
                // never happens
                null
            }
        }
    }
}