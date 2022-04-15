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
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerGyroscopeAndMagnetometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
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
import java.util.*
import kotlin.math.max
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location
        )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertNotNull(calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, calibrator.gyroscopeSensorType)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.magnetometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.generatedMagnetometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.magnetometerSensor)
        val accelerometerInitialMa1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val accelerometerInitialMa2 = calibrator.accelerometerInitialMa
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa2)
        val accelerometerInitialMa3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerInitialMa(accelerometerInitialMa3)
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa3)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        val gyroscopeInitialMg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialMg2 = calibrator.gyroscopeInitialMg
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg2)
        val gyroscopeInitialMg3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(gyroscopeInitialMg3)
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gyroscopeInitialGg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialGg2 = calibrator.gyroscopeInitialGg
        assertEquals(gyroscopeInitialGg1, gyroscopeInitialGg2)
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.magnetometerInitialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isAccelerometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isGyroscopeCommonAxisUsed
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isMagnetometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES,
            calibrator.isGDependentCrossBiasesEstimated
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            max(
                max(
                    calibrator.minimumRequiredAccelerometerMeasurements,
                    calibrator.minimumRequiredGyroscopeMeasurements
                ), calibrator.minimumRequiredMagnetometerMeasurements
            ), calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.accelerometerRobustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.accelerometerRobustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.gyroscopeRobustMethod)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        assertNull(calibrator.gyroscopeRobustThreshold)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedAccelerometerMa)
        assertNull(calibrator.estimatedAccelerometerSx)
        assertNull(calibrator.estimatedAccelerometerSy)
        assertNull(calibrator.estimatedAccelerometerSz)
        assertNull(calibrator.estimatedAccelerometerMxy)
        assertNull(calibrator.estimatedAccelerometerMxz)
        assertNull(calibrator.estimatedAccelerometerMyx)
        assertNull(calibrator.estimatedAccelerometerMyz)
        assertNull(calibrator.estimatedAccelerometerMzx)
        assertNull(calibrator.estimatedAccelerometerMzy)
        assertNull(calibrator.estimatedAccelerometerCovariance)
        assertNull(calibrator.estimatedAccelerometerChiSq)
        assertNull(calibrator.estimatedAccelerometerMse)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedGyroscopeMg)
        assertNull(calibrator.estimatedGyroscopeSx)
        assertNull(calibrator.estimatedGyroscopeSy)
        assertNull(calibrator.estimatedGyroscopeSz)
        assertNull(calibrator.estimatedGyroscopeMxy)
        assertNull(calibrator.estimatedGyroscopeMxz)
        assertNull(calibrator.estimatedGyroscopeMyx)
        assertNull(calibrator.estimatedGyroscopeMyz)
        assertNull(calibrator.estimatedGyroscopeMzx)
        assertNull(calibrator.estimatedGyroscopeMzy)
        assertNull(calibrator.estimatedGyroscopeGg)
        assertNull(calibrator.estimatedGyroscopeCovariance)
        assertNull(calibrator.estimatedGyroscopeChiSq)
        assertNull(calibrator.estimatedGyroscopeMse)
        assertNull(calibrator.estimatedGyroscopeBiasX)
        assertNull(calibrator.estimatedGyroscopeBiasY)
        assertNull(calibrator.estimatedGyroscopeBiasZ)
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)
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
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
        assertFalse(calibrator.isReadyToSolveMagnetometerCalibration)
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val accelerometerQualityScoreMapper = DefaultAccelerometerQualityScoreMapper()
        val gyroscopeQualityScoreMapper = DefaultGyroscopeQualityScoreMapper()
        val magnetometerQualityScoreMapper = DefaultMagnetometerQualityScoreMapper()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            timestamp,
            worldMagneticModel,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            isGyroscopeGroundTruthInitialBias = true,
            isMagnetometerGroundTruthInitialHardIron = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener,
            generatedMagnetometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            initialAccelerometerBiasAvailableListener,
            initialGyroscopeBiasAvailableListener,
            initialMagnetometerHardIronAvailableListener,
            accuracyChangedListener,
            accelerometerQualityScoreMapper,
            gyroscopeQualityScoreMapper,
            magnetometerQualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertEquals(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.magnetometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedAccelerometerMeasurementListener,
            calibrator.generatedAccelerometerMeasurementListener
        )
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(
            generatedMagnetometerMeasurementListener,
            calibrator.generatedMagnetometerMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertSame(
            initialMagnetometerHardIronAvailableListener,
            calibrator.initialMagnetometerHardIronAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertSame(accelerometerQualityScoreMapper, calibrator.accelerometerQualityScoreMapper)
        assertSame(gyroscopeQualityScoreMapper, calibrator.gyroscopeQualityScoreMapper)
        assertSame(magnetometerQualityScoreMapper, calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration))
        assertNull(calibrator.accelerometerInitialBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)
        val angularSpeed = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.magnetometerInitialHardIronXAsMeasurement)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getMagnetometerInitialHardIronXAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronYAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronYAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronZAsMeasurement)
        assertFalse(calibrator.getMagnetometerInitialHardIronZAsMeasurement(b))
        assertNull(calibrator.magnetometerInitialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getMagnetometerInitialHardIronAsTriad(triad))
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.magnetometerSensor)
        val accelerometerInitialMa1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val accelerometerInitialMa2 = calibrator.accelerometerInitialMa
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa2)
        val accelerometerInitialMa3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerInitialMa(accelerometerInitialMa3)
        assertEquals(accelerometerInitialMa1, accelerometerInitialMa3)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        val gyroscopeInitialMg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialMg2 = calibrator.gyroscopeInitialMg
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg2)
        val gyroscopeInitialMg3 =
            Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(gyroscopeInitialMg3)
        assertEquals(gyroscopeInitialMg1, gyroscopeInitialMg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gyroscopeInitialGg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gyroscopeInitialGg2 = calibrator.gyroscopeInitialGg
        assertEquals(gyroscopeInitialGg1, gyroscopeInitialGg2)
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.magnetometerInitialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isAccelerometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isGyroscopeCommonAxisUsed
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isMagnetometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES,
            calibrator.isGDependentCrossBiasesEstimated
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMagnetometerMeasurements
        )
        assertEquals(
            max(
                max(
                    calibrator.minimumRequiredAccelerometerMeasurements,
                    calibrator.minimumRequiredGyroscopeMeasurements
                ), calibrator.minimumRequiredMagnetometerMeasurements
            ), calibrator.minimumRequiredMeasurements
        )
        assertNull(calibrator.accelerometerRobustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.accelerometerRobustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.gyroscopeRobustMethod)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        assertNull(calibrator.gyroscopeRobustThreshold)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.magnetometerRobustMethod)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.magnetometerRobustConfidence,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.magnetometerRobustMaxIterations
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        assertNull(calibrator.magnetometerRobustThreshold)
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustThresholdFactor,
            0.0
        )
        assertEquals(
            SingleSensorStaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedAccelerometerMa)
        assertNull(calibrator.estimatedAccelerometerSx)
        assertNull(calibrator.estimatedAccelerometerSy)
        assertNull(calibrator.estimatedAccelerometerSz)
        assertNull(calibrator.estimatedAccelerometerMxy)
        assertNull(calibrator.estimatedAccelerometerMxz)
        assertNull(calibrator.estimatedAccelerometerMyx)
        assertNull(calibrator.estimatedAccelerometerMyz)
        assertNull(calibrator.estimatedAccelerometerMzx)
        assertNull(calibrator.estimatedAccelerometerMzy)
        assertNull(calibrator.estimatedAccelerometerCovariance)
        assertNull(calibrator.estimatedAccelerometerChiSq)
        assertNull(calibrator.estimatedAccelerometerMse)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertNull(calibrator.estimatedGyroscopeMg)
        assertNull(calibrator.estimatedGyroscopeSx)
        assertNull(calibrator.estimatedGyroscopeSy)
        assertNull(calibrator.estimatedGyroscopeSz)
        assertNull(calibrator.estimatedGyroscopeMxy)
        assertNull(calibrator.estimatedGyroscopeMxz)
        assertNull(calibrator.estimatedGyroscopeMyx)
        assertNull(calibrator.estimatedGyroscopeMyz)
        assertNull(calibrator.estimatedGyroscopeMzx)
        assertNull(calibrator.estimatedGyroscopeMzy)
        assertNull(calibrator.estimatedGyroscopeGg)
        assertNull(calibrator.estimatedGyroscopeCovariance)
        assertNull(calibrator.estimatedGyroscopeChiSq)
        assertNull(calibrator.estimatedGyroscopeMse)
        assertNull(calibrator.estimatedGyroscopeBiasX)
        assertNull(calibrator.estimatedGyroscopeBiasY)
        assertNull(calibrator.estimatedGyroscopeBiasZ)
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(angularSpeed))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)
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
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertNull(calibrator.magnetometerBaseNoiseLevel)
        assertNull(calibrator.magnetometerBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getMagnetometerBaseNoiseLevelAsMeasurement(b))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertEquals(0, calibrator.numberOfProcessedMagnetometerMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertTrue(calibrator.magnetometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
        assertFalse(calibrator.isReadyToSolveMagnetometerCalibration)
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val location1 = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location1)

        // check default value
        assertSame(location1, calibrator.location)
        assertFalse(calibrator.running)

        // set new value
        val location2 = getLocation()
        calibrator.location = location2

        // check
        assertSame(location2, calibrator.location)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunning_throwsIllegalStateException() {
        val location1 = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location1)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val location2 = getLocation()
        calibrator.location = location2
    }

    @Test
    fun timestamp_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNotNull(calibrator.timestamp)
        assertFalse(calibrator.running)

        // set new value
        val timestamp = Date()
        calibrator.timestamp = timestamp

        // check
        assertSame(timestamp, calibrator.timestamp)
    }

    @Test(expected = IllegalStateException::class)
    fun timestamp_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val timestamp = Date()
        calibrator.timestamp = timestamp
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.worldMagneticModel)
        assertFalse(calibrator.running)

        //set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.generatedAccelerometerMeasurementListener)

        // set new value
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        calibrator.generatedAccelerometerMeasurementListener =
            generatedAccelerometerMeasurementListener

        // check
        assertSame(
            generatedAccelerometerMeasurementListener,
            calibrator.generatedAccelerometerMeasurementListener
        )
    }

    @Test
    fun generatedGyroscopeMeasurementListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.generatedGyroscopeMeasurementListener)

        // set new value
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        calibrator.generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.generatedMagnetometerMeasurementListener)

        // set new value
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>()
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun initialAccelerometerBiasAvailableListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)

        // set new value
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        calibrator.initialAccelerometerBiasAvailableListener =
            initialAccelerometerBiasAvailableListener

        // check
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
    }

    @Test
    fun initialGyroscopeBiasAvailableListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)

        // set new value
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        calibrator.initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener

        // check
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
    }

    @Test
    fun initialMagnetometerHardIronAvailableListener_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialMagnetometerHardIronAvailableListener)

        // set new value
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>()
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        calibrator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
    }

    @Test
    fun isAccelerometerGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        // set new value
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        calibrator.isAccelerometerGroundTruthInitialBias = true
    }

    @Test
    fun isGyroscopeGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)

        // set new value
        calibrator.isGyroscopeGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isGyroscopeGroundTruthInitialBias = true
    }

    @Test
    fun isMagnetometerGroundTruthInitialHardIron_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isMagnetometerGroundTruthInitialHardIron = true
    }

    @Test
    fun accelerometerInitialMa_whenValid_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(Matrix(MA_SIZE, MA_SIZE), calibrator.accelerometerInitialMa)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

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

        calibrator.accelerometerInitialMa = ma

        // check
        assertEquals(ma, calibrator.accelerometerInitialMa)
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)

    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidRowsSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val ma = Matrix(1, MA_SIZE)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMa_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMa = Matrix(MA_SIZE, MA_SIZE)
    }

    @Test
    fun getAccelerometerInitialMa_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.getAccelerometerInitialMa(ma)

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
    fun getAccelerometerInitialMa_whenInvalidRowSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val ma = Matrix(1, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerInitialMa_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test
    fun accelerometerInitialSx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.accelerometerInitialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialSx = 0.0
    }

    @Test
    fun accelerometerInitialSy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.accelerometerInitialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialSy = 0.0
    }

    @Test
    fun accelerometerInitialSz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.accelerometerInitialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialSz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialSz = 0.0
    }

    @Test
    fun accelerometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.accelerometerInitialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMxy = 0.0
    }

    @Test
    fun accelerometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.accelerometerInitialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMxz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMxz = 0.0
    }

    @Test
    fun accelerometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.accelerometerInitialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMyx = 0.0
    }

    @Test
    fun accelerometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.accelerometerInitialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMyz = 0.0
    }

    @Test
    fun accelerometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.accelerometerInitialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMzx = 0.0
    }

    @Test
    fun accelerometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.accelerometerInitialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMzy = 0.0
    }

    @Test
    fun setAccelerometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setAccelerometerInitialScalingFactors(initialSx, initialSy, initialSz)

        // check
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        calibrator.setAccelerometerInitialScalingFactors(initialSx, initialSy, initialSz)
    }

    @Test
    fun setAccelerometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setAccelerometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)

        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        calibrator.setAccelerometerInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun setAccelerometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)

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
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
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
        assertEquals(initialSx, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(initialSy, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(initialSz, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(initialMxy, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(initialMxz, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(initialMyx, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(initialMyz, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(initialMzx, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(initialMzy, calibrator.accelerometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        calibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
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
    fun gyroscopeInitialMg_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)

        // set new value
        val mg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg3)
        calibrator.gyroscopeInitialMg = mg3

        // check
        val mg4 = calibrator.gyroscopeInitialMg
        assertEquals(mg3, mg4)
        assertEquals(mg3.getElementAt(0, 0), calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(mg3.getElementAt(1, 0), calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(mg3.getElementAt(2, 0), calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(mg3.getElementAt(0, 1), calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(mg3.getElementAt(1, 1), calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(mg3.getElementAt(2, 1), calibrator.gyroscopeInitialMzy, 0.0)
        assertEquals(mg3.getElementAt(0, 2), calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(mg3.getElementAt(1, 2), calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(mg3.getElementAt(2, 2), calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMg_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidRows_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mg1 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mg1 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test
    fun getGyroscopeInitialMg_whenValid_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)

        // set new value
        val mg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg3)
        calibrator.gyroscopeInitialMg = mg3

        // check
        val mg4 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg4)
        assertEquals(mg3, mg4)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getGyroscopeInitialMg_whenInvalidRows_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // set new value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg1)
        calibrator.gyroscopeInitialMg = mg1

        // get
        val mg2 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getGyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // set new value
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, mg1)
        calibrator.gyroscopeInitialMg = mg1

        // get
        val mg2 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.getGyroscopeInitialMg(mg2)
    }

    @Test
    fun gyroscopeInitialSx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        calibrator.gyroscopeInitialSx = gyroscopeInitialSx

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        calibrator.gyroscopeInitialSx = gyroscopeInitialSx
    }

    @Test
    fun gyroscopeInitialSy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSy = randomizer.nextDouble()
        calibrator.gyroscopeInitialSy = gyroscopeInitialSy

        // check
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSy = randomizer.nextDouble()
        calibrator.gyroscopeInitialSy = gyroscopeInitialSy
    }

    @Test
    fun gyroscopeInitialSz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.gyroscopeInitialSz = gyroscopeInitialSz

        // check
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialSz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.gyroscopeInitialSz = gyroscopeInitialSz
    }

    @Test
    fun gyroscopeInitialMxy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxy = gyroscopeInitialMxy

        // check
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMxy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxy = gyroscopeInitialMxy
    }

    @Test
    fun gyroscopeInitialMxz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxz = gyroscopeInitialMxz

        // check
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMxz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMxz = gyroscopeInitialMxz
    }

    @Test
    fun gyroscopeInitialMyx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyx = gyroscopeInitialMyx

        // check
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMyx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyx = gyroscopeInitialMyx
    }

    @Test
    fun gyroscopeInitialMyz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyz = gyroscopeInitialMyz

        // check
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMyz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        calibrator.gyroscopeInitialMyz = gyroscopeInitialMyz
    }

    @Test
    fun gyroscopeInitialMzx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzx = gyroscopeInitialMzx

        // check
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMzx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzx = gyroscopeInitialMzx
    }

    @Test
    fun gyroscopeInitialMzy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzy = gyroscopeInitialMzy

        // check
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialMzy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.gyroscopeInitialMzy = gyroscopeInitialMzy
    }

    @Test
    fun setGyroscopeInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )
    }

    @Test
    fun setGyroscopeInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialCrossCouplingErrors(
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )

        // check
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialCrossCouplingErrors(
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    @Test
    fun setGyroscopeInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )

        // check
        assertEquals(gyroscopeInitialSx, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(gyroscopeInitialSy, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(gyroscopeInitialSz, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(gyroscopeInitialMxy, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(gyroscopeInitialMxz, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(gyroscopeInitialMyx, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(gyroscopeInitialMyz, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(gyroscopeInitialMzx, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(gyroscopeInitialMzy, calibrator.gyroscopeInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setGyroscopeInitialScalingFactorsAndCrossCouplingErrors_whenRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val gyroscopeInitialSx = randomizer.nextDouble()
        val gyroscopeInitialSy = randomizer.nextDouble()
        val gyroscopeInitialSz = randomizer.nextDouble()
        val gyroscopeInitialMxy = randomizer.nextDouble()
        val gyroscopeInitialMxz = randomizer.nextDouble()
        val gyroscopeInitialMyx = randomizer.nextDouble()
        val gyroscopeInitialMyz = randomizer.nextDouble()
        val gyroscopeInitialMzx = randomizer.nextDouble()
        val gyroscopeInitialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    @Test
    fun gyroscopeInitialGg_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)

        // set new value
        val gg3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg3)
        calibrator.gyroscopeInitialGg = gg3

        // check
        val gg4 = calibrator.gyroscopeInitialGg
        assertEquals(gg3, gg4)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeInitialGg_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidRows_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // set new value
        val gg = Matrix(1, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidColumns_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, 1)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test
    fun magnetometerInitialMm_whenValid_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mm = Matrix(1, MM_SIZE)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerInitialMm_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.magnetometerInitialMm = mm
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMm_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mm = Matrix(1, MM_SIZE)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getMagnetometerInitialMm_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.getMagnetometerInitialMm(mm)
    }

    @Test
    fun magnetometerInitialSx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = magnetometerInitialSx

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = magnetometerInitialSx
    }

    @Test
    fun magnetometerInitialSy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = magnetometerInitialSy

        // check
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = magnetometerInitialSy
    }

    @Test
    fun magnetometerInitialSz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = magnetometerInitialSz

        // check
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialSz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = magnetometerInitialSz
    }

    @Test
    fun magnetometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = magnetometerInitialMxy

        // check
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = magnetometerInitialMxy
    }

    @Test
    fun magnetometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = magnetometerInitialMxz

        // check
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMxz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = magnetometerInitialMxz
    }

    @Test
    fun magnetometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = magnetometerInitialMyx

        // check
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = magnetometerInitialMyx
    }

    @Test
    fun magnetometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = magnetometerInitialMyz

        // check
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = magnetometerInitialMyz
    }

    @Test
    fun magnetometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = magnetometerInitialMzx

        // check
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = magnetometerInitialMzx
    }

    @Test
    fun magnetometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = magnetometerInitialMzy

        // check
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = magnetometerInitialMzy
    }

    @Test
    fun setMagnetometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )
    }

    @Test
    fun setMagnetometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )

        // check
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialCrossCouplingErrors_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
    }

    @Test
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        assertFalse(calibrator.running)

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )

        // check
        assertEquals(magnetometerInitialSx, calibrator.magnetometerInitialSx, 0.0)
        assertEquals(magnetometerInitialSy, calibrator.magnetometerInitialSy, 0.0)
        assertEquals(magnetometerInitialSz, calibrator.magnetometerInitialSz, 0.0)
        assertEquals(magnetometerInitialMxy, calibrator.magnetometerInitialMxy, 0.0)
        assertEquals(magnetometerInitialMxz, calibrator.magnetometerInitialMxz, 0.0)
        assertEquals(magnetometerInitialMyx, calibrator.magnetometerInitialMyx, 0.0)
        assertEquals(magnetometerInitialMyz, calibrator.magnetometerInitialMyz, 0.0)
        assertEquals(magnetometerInitialMzx, calibrator.magnetometerInitialMzx, 0.0)
        assertEquals(magnetometerInitialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new values
        val randomizer = UniformRandomizer()
        val magnetometerInitialSx = randomizer.nextDouble()
        val magnetometerInitialSy = randomizer.nextDouble()
        val magnetometerInitialSz = randomizer.nextDouble()
        val magnetometerInitialMxy = randomizer.nextDouble()
        val magnetometerInitialMxz = randomizer.nextDouble()
        val magnetometerInitialMyx = randomizer.nextDouble()
        val magnetometerInitialMyz = randomizer.nextDouble()
        val magnetometerInitialMzx = randomizer.nextDouble()
        val magnetometerInitialMzy = randomizer.nextDouble()
        calibrator.setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
    }

    @Test
    fun isAccelerometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertFalse(calibrator.isAccelerometerCommonAxisUsed)

        // set new value
        calibrator.isAccelerometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isAccelerometerCommonAxisUsed = true
    }

    @Test
    fun isGyroscopeCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isGyroscopeCommonAxisUsed = true

        // check
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isGyroscopeCommonAxisUsed = true
    }

    @Test
    fun isMagnetometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isMagnetometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true
    }

    @Test
    fun isGDependentCrossBiasesEstimated_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default values
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
    }

    @Test(expected = IllegalStateException::class)
    fun isGDependentCrossBiasesEstimated_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenCommonAxisAndKnownBias_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(7, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenCommonAxisAndUnknownBias_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndKnownBias_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndUnknownBias_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(13, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(16, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndNoCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(7, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasNoCommonAxisAndCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(19, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasNoCommonAxisAndNoCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = true
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(10, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(19, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasCommonAxisAndNoCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(10, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasNoCommonAxisAndCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = true

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertTrue(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(22, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenNoGroundTruthInitialBiasNoCommonAxisAndNoCrossBiases_returnsExpctedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isGyroscopeGroundTruthInitialBias = false
        calibrator.isGyroscopeCommonAxisUsed = false
        calibrator.isGDependentCrossBiasesEstimated = false

        // check
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(13, calibrator.minimumRequiredGyroscopeMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(13, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun accelerometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accelerometerRobustMethod)

        // set new value
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.accelerometerRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun accelerometerRobustConfidence_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.accelerometerRobustConfidence,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.accelerometerRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun accelerometerRobustMaxIterations_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.accelerometerRobustMaxIterations
        )

        // set new value
        calibrator.accelerometerRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.accelerometerRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun accelerometerRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.accelerometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.accelerometerRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun accelerometerRobustThreshold_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.accelerometerRobustThreshold)

        // set new value
        calibrator.accelerometerRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.accelerometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.accelerometerRobustThreshold = null

        // check
        assertNull(calibrator.accelerometerRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun accelerometerRobustThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.accelerometerRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun accelerometerRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.accelerometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.accelerometerRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.accelerometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun gyroscopeRobustMethod_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.gyroscopeRobustMethod)

        // set new value
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMethod_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun gyroscopeRobustConfidence_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.gyroscopeRobustConfidence,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustConfidence_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun gyroscopeRobustMaxIterations_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.gyroscopeRobustMaxIterations
        )

        // set new value
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun gyroscopeRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )

        // set new value
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun gyroscopeRobustThreshold_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.gyroscopeRobustThreshold)

        // set new value
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.gyroscopeRobustThreshold = null

        // check
        assertNull(calibrator.gyroscopeRobustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun gyroscopeRobustThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun gyroscopeRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.gyroscopeRobustStopThresholdFactor,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.gyroscopeRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun magnetometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun magnetometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.magnetometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun magnetometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorInitializationStartedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationStartedListener? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_makesNoAction() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                initializationStartedListener = initializationStartedListener
            )

        val generatorInitializationStartedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationStartedListener? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorInitializationCompletedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationCompletedListener? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_notifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                initializationCompletedListener = initializationCompletedListener
            )

        val generatorInitializationCompletedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationCompletedListener? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnErrorListener? =
            calibrator.getPrivateProperty("generatorErrorListener")
        requireNotNull(generatorErrorListener)

        generatorErrorListener.onError(generatorSpy, ErrorReason.UNRELIABLE_SENSOR)

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
    }

    @Test
    fun onError_whenListenersAvailable_stopsGeneratoAndNotifies() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
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

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnErrorListener? =
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
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorStaticIntervalDetectedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                staticIntervalDetectedListener = staticIntervalDetectedListener
            )

        val generatorStaticIntervalDetectedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorDynamicIntervalDetectedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
            )

        val generatorDynamicIntervalDetectedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorStaticIntervalSkippedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context, location,
                staticIntervalSkippedListener = staticIntervalSkippedListener
            )

        val generatorStaticIntervalSkippedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        val generatorDynamicIntervalSkippedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
            )

        val generatorDynamicIntervalSkippedListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_addsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.accelerometerMeasurements.size)
        assertSame(measurement, calibrator.accelerometerMeasurements[0])
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenListenerAvailable_notifies() {
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedAccelerometerMeasurementListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
            )

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.accelerometerMeasurements.size)
        assertSame(measurement, calibrator.accelerometerMeasurements[0])

        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            )
        }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            accelerometerMeasurement
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val location = getLocation()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            solveCalibrationWhenEnoughMeasurements = false,
            location = location
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            accelerometerMeasurement
        )

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onGeneratedAccelerometerMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onGeneratedGyroscopeMeasurement_addsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.gyroscopeMeasurements.size)
        assertSame(measurement, calibrator.gyroscopeMeasurements[0])
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenListenerAvailable_notifies() {
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedGyroscopeMeasurementListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
            )

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.gyroscopeMeasurements.size)
        assertSame(measurement, calibrator.gyroscopeMeasurements[0])

        verify(exactly = 1) {
            generatedGyroscopeMeasurementListener.onGeneratedGyroscopeMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            )
        }
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            gyroscopeMeasurement
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            gyroscopeMeasurement
        )

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            calibrator.gyroscopeMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onGeneratedGyroscopeMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            calibrator.gyroscopeMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onGeneratedMagnetometerMeasurement_addsMeasurement() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenListenerAvailable_notifies() {
        val generatedMagnetometerMeasurementListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
            )

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerGyroscopeAndMagnetometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])

        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            )
        }
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generatorSpy,
            magnetometerMeasurement
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val location = getLocation()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            solveCalibrationWhenEnoughMeasurements = false,
            location = location
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val magnetometerMeasurement = StandardDeviationBodyMagneticFluxDensity()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)
        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generatorSpy,
            magnetometerMeasurement
        )

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedMagnetometerMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generatorSpy,
            calibrator.magnetometerMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onGeneratedMagnetometerMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>>(
                relaxUnitFun = true
            )

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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

        val ba = generateBa()
        val bg = generateBg()
        val ma = generateMa()
        val mg = generateGeneralMg()
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
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

        val hardIron = generateHardIron()
        val mm = generateSoftIron()

        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 0 until reqMeasurements) {
            // initial attitude of sequence
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

            val beforeQ = Quaternion()
            nedC.asRotation(beforeQ)

            val nedFrame = NEDFrame(nedPosition, nedC)
            val ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame)

            val trueBeforeGravityKinematics =
                ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    ecefFrame,
                    ecefFrame
                )
            val measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueBeforeGravityKinematics,
                errors,
                random
            )
            val beforeMeanFx = measuredBeforeGravityKinematics.fx
            val beforeMeanFy = measuredBeforeGravityKinematics.fy
            val beforeMeanFz = measuredBeforeGravityKinematics.fz

            val deltaRoll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES)
            )

            val oldNedFrame = NEDFrame(nedFrame)
            val newNedFrame = NEDFrame()
            val oldEcefFrame = ECEFFrame()
            val newEcefFrame = ECEFFrame()
            var oldRoll = roll - deltaRoll
            var oldPitch = pitch - deltaPitch
            var oldYaw = yaw - deltaYaw

            val trueSequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            val sequence = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
            sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz)

            val trueTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()
            val measuredTimedKinematicsList = mutableListOf<StandardDeviationTimedBodyKinematics>()

            val earthB = wmmEstimator.estimate(nedPosition, timestamp)
            val truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, roll, pitch, yaw)
            val measuredMagnetic =
                BodyMagneticFluxDensityGenerator.generate(truthMagnetic, hardIron, mm)

            val magnetometerMeasurement =
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, MAGNETOMETER_NOISE_STD)
            calibrator.magnetometerMeasurements.add(magnetometerMeasurement)

            for (j in 0 until reqMeasurements) {
                val newRoll = oldRoll + deltaRoll
                val newPitch = oldPitch + deltaPitch
                val newYaw = oldYaw + deltaYaw
                val newNedC = CoordinateTransformation(
                    newRoll,
                    newPitch,
                    newYaw,
                    FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME
                )
                val newNedPosition = oldNedFrame.position

                newNedFrame.position = newNedPosition
                newNedFrame.coordinateTransformation = newNedC

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)
                NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame)

                val timestampSeconds = j * TIME_INTERVAL_SECONDS

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                val trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    newEcefFrame,
                    oldEcefFrame
                )

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                val measuredKinematics = BodyKinematicsGenerator.generate(
                    TIME_INTERVAL_SECONDS,
                    trueKinematics,
                    errors,
                    random
                )

                val trueTimedKinematics = StandardDeviationTimedBodyKinematics(
                    trueKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                val measuredTimedKinematics = StandardDeviationTimedBodyKinematics(
                    measuredKinematics,
                    timestampSeconds,
                    specificForceStandardDeviation,
                    angularRateStandardDeviation
                )

                trueTimedKinematicsList.add(trueTimedKinematics)
                measuredTimedKinematicsList.add(measuredTimedKinematics)

                oldNedFrame.copyFrom(newNedFrame)
                oldRoll = newRoll
                oldPitch = newPitch
                oldYaw = newYaw
            }
            trueSequence.setItems(trueTimedKinematicsList)
            sequence.setItems(measuredTimedKinematicsList)

            val afterQ = Quaternion()
            QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, afterQ)

            val newNedC = CoordinateTransformation(
                afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
            newNedFrame.position = nedPosition
            newNedFrame.coordinateTransformation = newNedC

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame)

            val trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS,
                newEcefFrame,
                newEcefFrame
            )
            val measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                TIME_INTERVAL_SECONDS,
                trueAfterGravityKinematics,
                errors,
                random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)

            val accelerometerMeasurement = StandardDeviationBodyKinematics(
                measuredBeforeGravityKinematics,
                ACCUMULATED_STD,
                0.0
            )
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedMagnetometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMagnetometerMeasurement(
            generatorSpy,
            calibrator.magnetometerMeasurements.last()
        )

        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)

        assertNotNull(calibrator.estimatedAccelerometerMa)
        assertNotNull(calibrator.estimatedAccelerometerSx)
        assertNotNull(calibrator.estimatedAccelerometerSy)
        assertNotNull(calibrator.estimatedAccelerometerSz)
        assertNotNull(calibrator.estimatedAccelerometerMxy)
        assertNotNull(calibrator.estimatedAccelerometerMxz)
        assertNotNull(calibrator.estimatedAccelerometerMyx)
        assertNotNull(calibrator.estimatedAccelerometerMyz)
        assertNotNull(calibrator.estimatedAccelerometerMzx)
        assertNotNull(calibrator.estimatedAccelerometerMzy)
        assertNotNull(calibrator.estimatedAccelerometerCovariance)
        assertNotNull(calibrator.estimatedAccelerometerChiSq)
        assertNotNull(calibrator.estimatedAccelerometerMse)
        assertNotNull(calibrator.estimatedAccelerometerBiasX)
        assertNotNull(calibrator.estimatedAccelerometerBiasY)
        assertNotNull(calibrator.estimatedAccelerometerBiasZ)
        assertNotNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNotNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerometerTriad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerometerTriad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

        assertNotNull(calibrator.estimatedGyroscopeMg)
        assertNotNull(calibrator.estimatedGyroscopeSx)
        assertNotNull(calibrator.estimatedGyroscopeSy)
        assertNotNull(calibrator.estimatedGyroscopeSz)
        assertNotNull(calibrator.estimatedGyroscopeMxy)
        assertNotNull(calibrator.estimatedGyroscopeMxz)
        assertNotNull(calibrator.estimatedGyroscopeMyx)
        assertNotNull(calibrator.estimatedGyroscopeMyz)
        assertNotNull(calibrator.estimatedGyroscopeMzx)
        assertNotNull(calibrator.estimatedGyroscopeMzy)
        assertNotNull(calibrator.estimatedGyroscopeGg)
        assertNotNull(calibrator.estimatedGyroscopeCovariance)
        assertNotNull(calibrator.estimatedGyroscopeChiSq)
        assertNotNull(calibrator.estimatedGyroscopeMse)
        assertNotNull(calibrator.estimatedGyroscopeBiasX)
        assertNotNull(calibrator.estimatedGyroscopeBiasY)
        assertNotNull(calibrator.estimatedGyroscopeBiasZ)
        assertNotNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(bias))
        assertNotNull(calibrator.estimatedGyroscopeBiasAsTriad)
        val angularSpeedTriad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(angularSpeedTriad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)

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
    fun onAccelerometerMeasurement_whenFirstMeasurement_updatesInitialBiases() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.accelerometerInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.accelerometerInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.accelerometerInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(bx.toDouble(), initialBiasX, 0.0)
        assertEquals(by.toDouble(), initialBiasY, 0.0)
        assertEquals(bz.toDouble(), initialBiasZ, 0.0)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementAndNoBiasX_updatesInitialBiases() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            null,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.accelerometerInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.accelerometerInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.accelerometerInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementAndNoBiasY_updatesInitialBiases() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            null,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.accelerometerInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.accelerometerInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.accelerometerInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementAndNoBiasZ_updatesInitialBiases() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            null,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.accelerometerInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.accelerometerInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.accelerometerInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementAndListener_updatesInitialBiases() {
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
            )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.accelerometerInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.accelerometerInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.accelerometerInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(bx.toDouble(), initialBiasX, 0.0)
        assertEquals(by.toDouble(), initialBiasY, 0.0)
        assertEquals(bz.toDouble(), initialBiasZ, 0.0)

        verify(exactly = 1) {
            initialAccelerometerBiasAvailableListener.onInitialBiasAvailable(
                calibrator,
                bx.toDouble(),
                by.toDouble(),
                bz.toDouble()
            )
        }
    }

    @Test
    fun onAccelerometerMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(2)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorAccelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        verify { initialAccelerometerBiasAvailableListener wasNot Called }
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurement_updatesInitialBias() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.gyroscopeInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(bx.toDouble(), initialBiasX, 0.0)
        assertEquals(by.toDouble(), initialBiasY, 0.0)
        assertEquals(bz.toDouble(), initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasX_updatesInitialBias() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            null,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.gyroscopeInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasY_updatesInitialBias() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            null,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.gyroscopeInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasZ_updatesInitialBias() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            null,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.gyroscopeInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasX, 0.0)
        assertEquals(0.0, initialBiasY, 0.0)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndListener_updatesInitialBias() {
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        val initialBiasX = calibrator.gyroscopeInitialBiasX
        requireNotNull(initialBiasX)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(bx.toDouble(), initialBiasX, 0.0)
        assertEquals(by.toDouble(), initialBiasY, 0.0)
        assertEquals(bz.toDouble(), initialBiasZ, 0.0)

        verify(exactly = 1) {
            initialGyroscopeBiasAvailableListener.onInitialBiasAvailable(
                calibrator,
                initialBiasX,
                initialBiasY,
                initialBiasZ
            )
        }
    }

    @Test
    fun onGyroscopeMeasurement_whenNotFirstMeasurement_updatesInitialBias() {
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
            context,
            location,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(2)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorGyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorGyroscopeMeasurementListener")
        requireNotNull(generatorGyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorGyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        verify { initialGyroscopeBiasAvailableListener wasNot Called }
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurement_updatesInitialHardIrons() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronX.toDouble()),
            initialHardIronX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronY.toDouble()),
            initialHardIronY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronZ.toDouble()),
            initialHardIronZ,
            0.0
        )
    }

    @Test
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronX_updatesInitialHardIrons() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronY_updatesInitialHardIrons() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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
    fun onMagnetometerMeasurement_whenFirstMeasurementAndNoHardIronZ_updatesInitialHardIrons() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(context, location)

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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
    fun onMagnetometerMeasurement_whenFirstMeasurementAndListener_updatesInitialHardIrons() {
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                initialMagnetometerHardIronAvailableListener = initialMagnetometerHardIronAvailableListener
            )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronX.toDouble()),
            initialHardIronX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronY.toDouble()),
            initialHardIronY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(hardIronZ.toDouble()),
            initialHardIronZ,
            0.0
        )

        verify(exactly = 1) {
            initialMagnetometerHardIronAvailableListener.onInitialHardIronAvailable(
                calibrator,
                initialHardIronX,
                initialHardIronY,
                initialHardIronZ
            )
        }
    }

    @Test
    fun onMagnetometerMeasurement_whenNoFirstMeasurement_makesNoAction() {
        val initialMagnetometerHardIronAvailableListener =
            mockk<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator.OnInitialMagnetometerHardIronAvailableListener>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator(
                context,
                location,
                initialMagnetometerHardIronAvailableListener = initialMagnetometerHardIronAvailableListener
            )

        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)

        val generator: AccelerometerGyroscopeAndMagnetometerMeasurementGenerator? =
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

    // TODO: gyroscopeBaseNoiseLevel_getsGeneratorBaseNoiseLevel

    private companion object {
        const val MA_SIZE = 3
        const val MM_SIZE = 3

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        const val TIME_INTERVAL_SECONDS = 0.02

        const val MIN_ANGLE_DEGREES = -180.0
        const val MAX_ANGLE_DEGREES = 180.0

        const val MIN_ANGLE_VARIATION_DEGREES = -2.0
        const val MAX_ANGLE_VARIATION_DEGREES = 2.0

        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val WINDOW_SIZE = 51

        const val REQUIRED_MEASUREMENTS = 30

        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 19

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val ACCUMULATED_STD = 0.007

        const val MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6
        const val DEG_TO_RAD = 0.01745329252

        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_HARD_IRON = -1e-5
        const val MAX_HARD_IRON = 1e-5

        const val MIN_SOFT_IRON = -1e-6
        const val MAX_SOFT_IRON = 1e-6

        const val MAGNETOMETER_NOISE_STD = 200e-9

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

        fun generateMa(): Matrix {
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

        fun generateGeneralMg(): Matrix {
            val result = Matrix(3, 3)
            result.fromArray(
                doubleArrayOf(
                    400e-6, -300e-6, 250e-6,
                    -300e-6, -300e-6, -150e-6,
                    250e-6, -150e-6, -350e-6
                ), false
            )
            return result
        }

        fun getAccelNoiseRootPSD(): Double {
            return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED
        }

        fun getGyroNoiseRootPSD(): Double {
            return 0.01 * DEG_TO_RAD / 60.0
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