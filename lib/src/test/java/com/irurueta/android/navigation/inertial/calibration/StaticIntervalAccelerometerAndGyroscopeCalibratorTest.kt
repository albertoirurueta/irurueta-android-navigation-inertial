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
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerAndGyroscopeMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndPositionAccelerometerCalibrator
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownPositionAccelerometerCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasEasyGyroscopeCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
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
class StaticIntervalAccelerometerAndGyroscopeCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, calibrator.gyroscopeSensorType)
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertNull(calibrator.gravityNorm)
        assertFalse(calibrator.accelerometerResultUnreliable)
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
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.gravitySensor)
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
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isAccelerometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isGyroscopeCommonAxisUsed
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
            max(
                calibrator.minimumRequiredAccelerometerMeasurements,
                calibrator.minimumRequiredGyroscopeMeasurements
            ), calibrator.minimumRequiredMeasurements
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
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
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
    fun constructor_whenAllParameters_setsExpectedValues() {
        val location = getLocation()
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnUnreliableGravityEstimationListener>()
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val accelerometerQualityScoreMapper = DefaultAccelerometerQualityScoreMapper()
        val gyroscopeQualityScoreMapper = DefaultGyroscopeQualityScoreMapper()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            isGyroscopeGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimationListener,
            initialAccelerometerBiasAvailableListener,
            initialGyroscopeBiasAvailableListener,
            accuracyChangedListener,
            accelerometerQualityScoreMapper,
            gyroscopeQualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(location, calibrator.location)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertSame(accelerometerQualityScoreMapper, calibrator.accelerometerQualityScoreMapper)
        assertSame(gyroscopeQualityScoreMapper, calibrator.gyroscopeQualityScoreMapper)
        assertNotNull(calibrator.gravityNorm)
        assertFalse(calibrator.accelerometerResultUnreliable)
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
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        assertNull(calibrator.gravitySensor)
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
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isAccelerometerCommonAxisUsed
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            calibrator.isGyroscopeCommonAxisUsed
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
            max(
                calibrator.minimumRequiredAccelerometerMeasurements,
                calibrator.minimumRequiredGyroscopeMeasurements
            ), calibrator.minimumRequiredMeasurements
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
        assertNull(calibrator.gyroscopeBaseNoiseLevel)
        assertNull(calibrator.gyroscopeBaseNoiseLevelAsMeasurement)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(angularSpeed))
        assertEquals(0, calibrator.numberOfProcessedGyroscopeMeasurements)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveAccelerometerCalibration)
        assertFalse(calibrator.isReadyToSolveGyroscopeCalibration)
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.generatedAccelerometerMeasurementListener)

        // set new value
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedAccelerometerMeasurementListener>()
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.generatedGyroscopeMeasurementListener)

        // set new value
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        calibrator.generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener

        // check
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
    }

    @Test
    fun readyToSolveCalibrationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun unreliableGravityNormEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.unreliableGravityNormEstimationListener)

        // set new value
        val unreliableGravityNormEstimationListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnUnreliableGravityEstimationListener>()
        calibrator.unreliableGravityNormEstimationListener = unreliableGravityNormEstimationListener

        // check
        assertSame(
            unreliableGravityNormEstimationListener,
            calibrator.unreliableGravityNormEstimationListener
        )
    }

    @Test
    fun initialAccelerometerBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)

        // set new value
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialAccelerometerBiasAvailableListener>()
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)

        // set new value
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        calibrator.initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener

        // check
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        calibrator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
    }

    @Test
    fun isAccelerometerGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        // set new value
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isAccelerometerGroundTruthInitialBias = true
    }

    @Test
    fun isGyroscopeGroundTruthInitialBias_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)

        // set new value
        calibrator.isGyroscopeGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isGyroscopeGroundTruthInitialBias = true
    }

    @Test
    fun location_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val location = getLocation()
        calibrator.location = location
    }

    @Test
    fun accelerometerInitialMa_whenValid_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMa_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerInitialMa_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test
    fun accelerometerInitialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val mg1 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val mg1 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test
    fun getGyroscopeInitialMg_whenValid_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // set new value
        val gg = Matrix(1, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, 1)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test
    fun isAccelerometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerCommonAxisUsed)

        // set new value
        calibrator.isAccelerometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
    fun isGDependentCrossBiasesEstimated_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(7, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.isAccelerometerCommonAxisUsed = true
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndKnownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(10, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredAccelerometerMeasurements_whenNotCommonAxisAndUnknownBias_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.isAccelerometerCommonAxisUsed = false
        calibrator.isAccelerometerGroundTruthInitialBias = false

        // check
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertEquals(13, calibrator.minimumRequiredAccelerometerMeasurements)
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
    fun averageGravityNorm_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.averageGravityNorm)
    }

    @Test
    fun averageGravityNormAsMeasurement_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        val acceleration = Acceleration(gravityNorm, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { gravityNormEstimatorSpy.averageNormAsMeasurement }.returns(acceleration)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        assertSame(acceleration, calibrator.averageGravityNormAsMeasurement)
        verify(exactly = 1) { gravityNormEstimatorSpy.averageNormAsMeasurement }
    }

    @Test
    fun averageGravityNormAsMeasurement_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.averageGravityNormAsMeasurement)
    }

    @Test
    fun getAverageGravityNormAsMeasurement_whenGravityNormEstimated_getsEstimatorAverageNorm() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
    fun getAverageGravityNormAsMeasurement_whenGravityNormNotEstimated_returnsNull() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAverageGravityNormAsMeasurement(acceleration))
        assertEquals(0.0, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun gravityNormVariance_whenGravityNormEstimated_getsEstimatorVariance() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertNull(calibrator.gravityNormVariance)
    }

    @Test
    fun gravityNormStandardDeviation_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityNormStandardDeviation)
    }

    @Test
    fun gravityNormStandardDeviationAsMeasurement_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityNormStandardDeviationAsMeasurement)
    }

    @Test
    fun getGravityNormStandardDeviationAsMeasurement_whenGravityNormEstimated_getsEstimatorStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getGravityNormStandardDeviationAsMeasurement(acceleration))
    }

    @Test
    fun gravityPsd_whenGravityNormEstimated_getsEstimatorPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityPsd)
    }

    @Test
    fun gravityRootPsd_whenGravityNormEstimated_getEstimatorRootPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator =
            StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.isGravityNormEstimated)

        assertNull(calibrator.gravityRootPsd)
    }

    @Test
    fun accelerometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerRobustMethod)

        // set new value
        calibrator.accelerometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.accelerometerRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.accelerometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeRobustMethod)

        // set new value
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorInitializationStartedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_makesNoAction() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val generatorInitializationStartedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationStartedListener? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorInitializationCompletedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_notifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val generatorInitializationCompletedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnInitializationCompletedListener? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )

        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsGeneratorAndGravityNormEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generatorErrorListener: AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener? =
            calibrator.getPrivateProperty("generatorErrorListener")
        requireNotNull(generatorErrorListener)

        generatorErrorListener.onError(generatorSpy, ErrorReason.UNRELIABLE_SENSOR)

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
    }

    @Test
    fun onError_whenListenersAvailable_stopsAndNotifies() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
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

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generatorErrorListener: AccelerometerAndGyroscopeMeasurementGenerator.OnErrorListener? =
            calibrator.getPrivateProperty("generatorErrorListener")
        requireNotNull(generatorErrorListener)

        generatorErrorListener.onError(generatorSpy, ErrorReason.UNRELIABLE_SENSOR)

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { generatorSpy.stop() }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorStaticIntervalDetectedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val generatorStaticIntervalDetectedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorDynamicIntervalDetectedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val generatorDynamicIntervalDetectedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalDetectedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorStaticIntervalSkippedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val generatorStaticIntervalSkippedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnStaticIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generatorDynamicIntervalSkippedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val generatorDynamicIntervalSkippedListener: AccelerometerAndGyroscopeMeasurementGenerator.OnDynamicIntervalSkippedListener? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_addsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedAccelerometerMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
        )

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generator,
            measurement
        )

        assertEquals(1, calibrator.accelerometerMeasurements.size)
        assertSame(measurement, calibrator.accelerometerMeasurements[0])
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
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
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            accelerometerMeasurement
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val location = getLocation()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
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
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            accelerometerMeasurement
        )

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = true,
            location = location
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

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
    }

    @Test
    fun onGeneratedAccelerometerMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = true,
            location = location,
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

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
        )

        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerAndGyroscopeMeasurementGenerator>()
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
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
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
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            gyroscopeMeasurement
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = false,
            location = location,
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
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedGyroscopeMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedGyroscopeMeasurement(
            generatorSpy,
            gyroscopeMeasurement
        )

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        // gyroscope internal calibrator is only built once accelerometer calibration is solved
        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = true,
            location = location
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

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
    }

    @Test
    fun onGeneratedGyroscopeMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = true,
            location = location,
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

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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

        val generatorGeneratedMeasurementListener: AccelerometerAndGyroscopeMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener? =
            calibrator.getPrivateProperty("generatorGeneratedAccelerometerMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(
            generatorSpy,
            calibrator.accelerometerMeasurements.last()
        )

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
    fun onGravityNormCompletedListener_setsGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.gravityNorm)

        val gravityNormCompletedListener: AccumulatedMeasurementEstimator.OnEstimationCompletedListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormCompletedListener")
        requireNotNull(gravityNormCompletedListener)

        val randomizer = UniformRandomizer()
        val gravityNorm = randomizer.nextDouble()
        val gravityNormEstimator = mockk<GravityNormEstimator>()
        every { gravityNormEstimator.averageNorm }.returns(gravityNorm)
        gravityNormCompletedListener.onEstimationCompleted(gravityNormEstimator)

        assertEquals(gravityNorm, calibrator.gravityNorm)
    }

    @Test
    fun onGravityNormUnreliable_whenNoListenerAvailable_setsResultAsUnreliable() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertFalse(calibrator.accelerometerResultUnreliable)

        val gravityNormUnreliableListener: AccumulatedMeasurementEstimator.OnUnreliableListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormUnreliableListener")
        requireNotNull(gravityNormUnreliableListener)

        val gravityNormEstimator = mockk<GravityNormEstimator>()
        gravityNormUnreliableListener.onUnreliable(gravityNormEstimator)

        assertTrue(calibrator.accelerometerResultUnreliable)
    }

    @Test
    fun onGravityNormUnreliable_whenListenerAvailable_setsResultAsUnreliableAndNotifies() {
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerAndGyroscopeCalibrator.OnUnreliableGravityEstimationListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            unreliableGravityNormEstimationListener = unreliableGravityNormEstimatorListener
        )

        assertFalse(calibrator.accelerometerResultUnreliable)

        val gravityNormUnreliableListener: AccumulatedMeasurementEstimator.OnUnreliableListener<GravityNormEstimator>? =
            calibrator.getPrivateProperty("gravityNormUnreliableListener")
        requireNotNull(gravityNormUnreliableListener)

        val gravityNormEstimator = mockk<GravityNormEstimator>()
        gravityNormUnreliableListener.onUnreliable(gravityNormEstimator)

        assertTrue(calibrator.accelerometerResultUnreliable)

        verify(exactly = 1) {
            unreliableGravityNormEstimatorListener.onUnreliableGravityEstimation(
                calibrator
            )
        }
    }

    @Test
    fun gyroscopeBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        verify(exactly = 1) { generatorSpy.gyroscopeBaseNoiseLevel }
    }

    @Test
    fun gyroscopeBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val baseNoiseLevel1 = AngularSpeed(value, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { generatorSpy.gyroscopeBaseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val baseNoiseLevel2 = calibrator.gyroscopeBaseNoiseLevelAsMeasurement
        assertSame(baseNoiseLevel1, baseNoiseLevel2)

        verify(exactly = 1) { generatorSpy.gyroscopeBaseNoiseLevelAsMeasurement }
    }

    @Test
    fun getGyroscopeBaseNoiseLevelAsMeasurement_getsGeneratorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(w))

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        every { generatorSpy.getGyroscopeBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = value
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertTrue(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(w))

        // check
        assertEquals(value, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
        verify(exactly = 1) { generatorSpy.getGyroscopeBaseNoiseLevelAsMeasurement(w) }
    }

    @Test
    fun accelerometerBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.threshold)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.thresholdAsMeasurement)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalVariance)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
    fun numberOfProcessedGyroscopeMeasurements_getsGeneratorNumberOfProcessedMagnetometerMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val randomizer = UniformRandomizer()
        val numberOfProcessedGyroscopeMeasurements = randomizer.nextInt()
        every { generatorSpy.numberOfProcessedGyroscopeMeasurements }.returns(
            numberOfProcessedGyroscopeMeasurements
        )
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(
            numberOfProcessedGyroscopeMeasurements,
            calibrator.numberOfProcessedGyroscopeMeasurements
        )

        verify(exactly = 1) { generatorSpy.numberOfProcessedGyroscopeMeasurements }
    }

    @Test
    fun numberOfProcessedAccelerometerMeasurements_getsGeneratorNumberOfProcessedAccelerometerMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
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
    fun gyroscopeInitialBiasX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasX)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)

        assertEquals(initialBiasX, calibrator.gyroscopeInitialBiasX)
    }

    @Test
    fun gyroscopeInitialBiasY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasY)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)

        assertEquals(initialBiasY, calibrator.gyroscopeInitialBiasY)
    }

    @Test
    fun gyroscopeInitialBiasZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasZ)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)

        assertEquals(initialBiasZ, calibrator.gyroscopeInitialBiasZ)
    }

    @Test
    fun gyroscopeInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasXAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)

        val bias = calibrator.gyroscopeInitialBiasXAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun getGyroscopeInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasXAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)

        assertTrue(calibrator.getGyroscopeInitialBiasXAsMeasurement(bias))
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun gyroscopeInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasYAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)

        val bias = calibrator.gyroscopeInitialBiasYAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun getGyroscopeInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasYAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)

        assertTrue(calibrator.getGyroscopeInitialBiasYAsMeasurement(bias))
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun gyroscopeInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasZAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)

        val bias = calibrator.gyroscopeInitialBiasZAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun getGyroscopeInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeInitialBiasZAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)

        assertTrue(calibrator.getGyroscopeInitialBiasZAsMeasurement(bias))
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bias.unit)
    }

    @Test
    fun gyroscopeInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.gyroscopeInitialBiasAsTriad)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)

        assertNull(calibrator.gyroscopeInitialBiasAsTriad)

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)

        assertNull(calibrator.gyroscopeInitialBiasAsTriad)

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)

        val triad = calibrator.gyroscopeInitialBiasAsTriad
        requireNotNull(triad)
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.unit)
    }

    @Test
    fun getGyroscopeInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val triad = AngularSpeedTriad()
        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(triad))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)

        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(triad))

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)

        assertFalse(calibrator.getGyroscopeInitialBiasAsTriad(triad))

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)

        // check
        assertTrue(calibrator.getGyroscopeInitialBiasAsTriad(triad))
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.unit)
    }

    @Test
    fun accelerometerInitialBiasX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasX)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasX", initialBiasX)

        assertEquals(initialBiasX, calibrator.accelerometerInitialBiasX)
    }

    @Test
    fun accelerometerInitialBiasY_getExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasY)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasY", initialBiasY)

        assertEquals(initialBiasY, calibrator.accelerometerInitialBiasY)
    }

    @Test
    fun accelerometerInitialBiasZ_getExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasZ)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", initialBiasZ)

        assertEquals(initialBiasZ, calibrator.accelerometerInitialBiasZ)
    }

    @Test
    fun accelerometerInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasXAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasX", initialBiasX)

        val bias = calibrator.accelerometerInitialBiasXAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getAccelerometerInitialBiasXAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasXAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasX", initialBiasX)

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasXAsMeasurement(bias))
        assertEquals(initialBiasX, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun accelerometerInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasYAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasY", initialBiasY)

        val bias = calibrator.accelerometerInitialBiasYAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getAccelerometerInitialBiasYAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasYAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasY", initialBiasY)

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasYAsMeasurement(bias))
        assertEquals(initialBiasY, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun accelerometerInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasZAsMeasurement)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", initialBiasZ)

        val bias = calibrator.accelerometerInitialBiasZAsMeasurement
        requireNotNull(bias)
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun getAccelerometerInitialBiasZAsMeasurement_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val bias = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerInitialBiasZAsMeasurement(bias))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", initialBiasZ)

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasZAsMeasurement(bias))
        assertEquals(initialBiasZ, bias.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bias.unit)
    }

    @Test
    fun accelerometerInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerInitialBiasAsTriad)

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasX", initialBiasX)

        assertNull(calibrator.accelerometerInitialBiasAsTriad)

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasY", initialBiasY)

        assertNull(calibrator.accelerometerInitialBiasAsTriad)

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", initialBiasZ)

        // check
        val triad = calibrator.accelerometerInitialBiasAsTriad
        requireNotNull(triad)
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun getAccelerometerInitialBiasAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        // check default value
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))

        // set new value
        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasX", initialBiasX)

        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))

        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasY", initialBiasY)

        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))

        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", initialBiasZ)

        // check
        assertTrue(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertEquals(initialBiasX, triad.valueX, 0.0)
        assertEquals(initialBiasY, triad.valueY, 0.0)
        assertEquals(initialBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun start_whenNotRunningAndGravityNormNotEstimated_resetsAndStartsGenerator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context, location = location)

        assertFalse(calibrator.running)
        assertFalse(calibrator.isGravityNormEstimated)

        val gravityNorm = GravityHelper.getGravityNormForLocation(location)
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val accelerometerMeasurements = calibrator.accelerometerMeasurements
        val accelerometerMeasurementsSpy = spyk(accelerometerMeasurements)
        calibrator.setPrivateProperty("accelerometerMeasurements", accelerometerMeasurementsSpy)

        val gyroscopeMeasurements = calibrator.gyroscopeMeasurements
        val gyroscopeMeasurementsSpy = spyk(gyroscopeMeasurements)
        calibrator.setPrivateProperty("gyroscopeMeasurements", gyroscopeMeasurementsSpy)

        calibrator.setPrivateProperty("accelerometerResultUnreliable", true)
        calibrator.setPrivateProperty("accelerometerInitialBiasX", 0.0)
        calibrator.setPrivateProperty("accelerometerInitialBiasY", 0.0)
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", 0.0)

        calibrator.setPrivateProperty("gyroscopeInitialBiasX", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", 0.0)

        val accelerometerInternalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", accelerometerInternalCalibrator)

        val gyroscopeInternalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", gyroscopeInternalCalibrator)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.start() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        calibrator.start()

        // check
        assertNull(calibrator.gravityNorm)
        verify(exactly = 1) { accelerometerMeasurementsSpy.clear() }
        assertFalse(calibrator.accelerometerResultUnreliable)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))

        assertTrue(calibrator.running)

        verify { gravityNormEstimatorSpy wasNot Called }
        verify(exactly = 1) { generatorSpy.start() }
    }

    @Test
    fun start_whenNotRunningAndGravityNormEstimated_resetsAndStartsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertFalse(calibrator.running)
        assertTrue(calibrator.isGravityNormEstimated)

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        calibrator.setPrivateProperty("gravityNorm", gravityNorm)

        val accelerometerMeasurements = calibrator.accelerometerMeasurements
        val accelerometerMeasurementsSpy = spyk(accelerometerMeasurements)
        calibrator.setPrivateProperty("accelerometerMeasurements", accelerometerMeasurementsSpy)

        val gyroscopeMeasurements = calibrator.gyroscopeMeasurements
        val gyroscopeMeasurementsSpy = spyk(gyroscopeMeasurements)
        calibrator.setPrivateProperty("gyroscopeMeasurements", gyroscopeMeasurementsSpy)

        calibrator.setPrivateProperty("accelerometerResultUnreliable", true)
        calibrator.setPrivateProperty("accelerometerInitialBiasX", 0.0)
        calibrator.setPrivateProperty("accelerometerInitialBiasY", 0.0)
        calibrator.setPrivateProperty("accelerometerInitialBiasZ", 0.0)

        calibrator.setPrivateProperty("gyroscopeInitialBiasX", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", 0.0)

        val accelerometerInternalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", accelerometerInternalCalibrator)

        val gyroscopeInternalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", gyroscopeInternalCalibrator)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        justRun { gravityNormEstimatorSpy.start() }
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.start() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        calibrator.start()

        // check
        assertNull(calibrator.gravityNorm)
        verify(exactly = 1) { accelerometerMeasurementsSpy.clear() }
        assertFalse(calibrator.accelerometerResultUnreliable)
        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)
        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { gravityNormEstimatorSpy.start() }
        verify(exactly = 1) { generatorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertFalse(calibrator.running)

        calibrator.start()

        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun stop_whenGravityNormEstimatorNotRunning_stopsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

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
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 0) { gravityNormEstimatorSpy.stop() }
        assertFalse(gravityNormEstimatorSpy.running)
    }

    @Test
    fun stop_whenGravityNormEstimatorRunning_stopsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

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
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        assertTrue(gravityNormEstimatorSpy.running)
    }

    @Test
    fun stop_whenListenerAvailable_notifies() {
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            stoppedListener = stoppedListener
        )

        val generator: AccelerometerAndGyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

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
        verify(exactly = 1) { gravityNormEstimatorSpy.running }
        verify(exactly = 0) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
        assertFalse(gravityNormEstimatorSpy.running)
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenNotReadyToSolveCalibration_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        calibrator.calibrate()
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.calibrate()
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            location = location,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

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
        val random = Random()
        val randomizer = UniformRandomizer(random)

        val sqrtTimeInterval = sqrt(TIME_INTERVAL_SECONDS)
        val specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval
        val angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval

        val sequences =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..13) {
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
        assertFalse(calibrator.running)

        val accelerometerInternalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        justRun { accelerometerInternalCalibrator.calibrate() }
        every { accelerometerInternalCalibrator.estimatedSx }.returns(ma.getElementAt(0, 0))
        every { accelerometerInternalCalibrator.estimatedSy }.returns(ma.getElementAt(1, 1))
        every { accelerometerInternalCalibrator.estimatedSz }.returns(ma.getElementAt(2, 2))
        every { accelerometerInternalCalibrator.estimatedMxy }.returns(ma.getElementAt(0, 1))
        every { accelerometerInternalCalibrator.estimatedMxz }.returns(ma.getElementAt(0, 2))
        every { accelerometerInternalCalibrator.estimatedMyx }.returns(ma.getElementAt(1, 0))
        every { accelerometerInternalCalibrator.estimatedMyz }.returns(ma.getElementAt(1, 2))
        every { accelerometerInternalCalibrator.estimatedMzx }.returns(ma.getElementAt(2, 0))
        every { accelerometerInternalCalibrator.estimatedMzy }.returns(ma.getElementAt(2, 1))
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", accelerometerInternalCalibrator)

        val gyroscopeInternalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        justRun { gyroscopeInternalCalibrator.calibrate() }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", gyroscopeInternalCalibrator)

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
    fun calibrate_whenInternalAccelerometerFailure_setsAsNotRunning() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            location = location
        )

        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerAndGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            location = location,
            errorListener = errorListener
        )

        val accelerometerMeasurement = StandardDeviationBodyKinematics()
        val gyroscopeMeasurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(accelerometerMeasurement)
            calibrator.gyroscopeMeasurements.add(gyroscopeMeasurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<AccelerometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibrator)

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
    fun estimatedAccelerometerBiasX_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasX)
    }

    @Test
    fun estimatedAccelerometerBiasX_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFx }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedAccelerometerBiasX)
    }

    @Test
    fun estimatedAccelerometerBiasX_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedAccelerometerBiasX)
    }

    @Test
    fun estimatedAccelerometerBiasY_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasY)
    }

    @Test
    fun estimatedAccelerometerBiasY_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFy }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedAccelerometerBiasY)
    }

    @Test
    fun estimatedAccelerometerBiasY_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedAccelerometerBiasY)
    }

    @Test
    fun estimatedAccelerometerBiasZ_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasZ)
    }

    @Test
    fun estimatedAccelerometerBiasZ_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasFz }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedAccelerometerBiasZ)
    }

    @Test
    fun estimatedAccelerometerBiasZ_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedAccelerometerBiasZ)
    }

    @Test
    fun estimatedAccelerometerBiasXAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasXAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFxAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasXAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasXAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasXAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasXAsMeasurement)
    }

    @Test
    fun getEstimatedAccelerometerBiasXAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedAccelerometerBiasXAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFxAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))

        assertEquals(estimatedBiasX, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun getEstimatedAccelerometerBiasXAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasXAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))

        assertEquals(estimatedBiasX, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedAccelerometerBiasYAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasYAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFyAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasYAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasYAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasYAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasYAsMeasurement)
    }

    @Test
    fun getEstimatedAccelerometerBiasYAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedAccelerometerBiasYAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFyAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))

        assertEquals(estimatedBiasY, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun getEstimatedAccelerometerBiasYAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasYAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))

        assertEquals(estimatedBiasY, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedAccelerometerBiasZAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasZAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.estimatedBiasFzAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasZAsMeasurement)
    }

    @Test
    fun estimatedAccelerometerBiasZAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val acceleration = Acceleration(estimatedBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        every { internalCalibratorSpy.biasZAsAcceleration }.returns(acceleration)
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(acceleration, calibrator.estimatedAccelerometerBiasZAsMeasurement)
    }

    @Test
    fun getEstimatedAccelerometerBiasZAsMeasurement_whenNoAccelerometerInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
    }

    @Test
    fun getEstimatedAccelerometerBiasZAsMeasurement_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasFzAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))

        assertEquals(estimatedBiasZ, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun getEstimatedAccelerometerBiasZAsMeasurement_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasZAsAcceleration(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Acceleration
            result.value = estimatedBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
        }
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))

        assertEquals(estimatedBiasZ, acceleration.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration.unit)
    }

    @Test
    fun estimatedAccelerometerBiasAsTriad_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
    }

    @Test
    fun estimatedAccelerometerBiasAsTriad_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedAccelerometerBiasAsTriad)
    }

    @Test
    fun estimatedAccelerometerBiasAsTriad_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedAccelerometerBiasAsTriad)
    }

    @Test
    fun getEstimatedAccelerometerBiasAsTriad_whenNoAccelerometerInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))

        val triad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
    }

    @Test
    fun getEstimatedAccelerometerBiasAsTriad_whenUnknownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun getEstimatedAccelerometerBiasAsTriad_whenKnownBiasAccelerometerInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

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
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad.unit)
    }

    @Test
    fun estimatedAccelerometerBiasStandardDeviationNorm_whenNoAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("accelerometerInternalCalibrator"))

        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
    }

    @Test
    fun estimatedAccelerometerBiasStandardDeviationNorm_whenBiasUncertaintySourceAccelerometerInternalCalibrator_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAccelerometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasStandardDeviationNorm = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasStandardDeviationNorm }.returns(
            estimatedBiasStandardDeviationNorm
        )
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(
            estimatedBiasStandardDeviationNorm,
            calibrator.estimatedAccelerometerBiasStandardDeviationNorm
        )
    }

    @Test
    fun estimatedAccelerometerBiasStandardDeviationNorm_whenOtherAccelerometerInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasAndPositionAccelerometerCalibrator())
        calibrator.setPrivateProperty("accelerometerInternalCalibrator", internalCalibratorSpy)

        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
    }

    @Test
    fun estimatedGyroscopeBiasX_whenNoGyroscopeInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasX)
    }

    @Test
    fun estimatedGyroscopeBiasX_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasX }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedGyroscopeBiasX)
    }

    @Test
    fun estimatedGyroscopeBiasX_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedGyroscopeBiasX)
    }

    @Test
    fun estimatedGyroscopeBiasY_whenNoGyroscopeInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasY)
    }

    @Test
    fun estimatedGyroscopeBiasY_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasY }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedGyroscopeBiasY)
    }

    @Test
    fun estimatedGyroscopeBiasY_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedGyroscopeBiasY)
    }

    @Test
    fun estimatedGyroscopeBiasZ_whenNoGyroscopeInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasZ)
    }

    @Test
    fun estimatedGyroscopeBiasZ_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedBiasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedGyroscopeBiasZ)
    }

    @Test
    fun estimatedGyroscopeBiasZ_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedGyroscopeBiasZ)
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_whenNoGyroscopeInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasX, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.estimatedBiasAngularSpeedX }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasXAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasX, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.biasAngularSpeedX }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasXAsMeasurement)
    }

    @Test
    fun getEstimatedGyroscopeBiasXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasXAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasAngularSpeedX(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasX
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasX, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(w))

        assertEquals(estimatedBiasX, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun getEstimatedGyroscopeBiasXAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasAngularSpeedX(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasX
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasX, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(w))

        assertEquals(estimatedBiasX, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun estimatedGyroscopeBiasYAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasYAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasY, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.estimatedBiasAngularSpeedY }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasYAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasYAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasY, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.biasAngularSpeedY }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasYAsMeasurement)
    }

    @Test
    fun getEstimatedGyroscopeBiasYAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasYAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasAngularSpeedY(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasY
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasY, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(w))

        assertEquals(estimatedBiasY, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun getEstimatedGyroscopeBiasYAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasAngularSpeedY(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasY
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasY, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(w))

        assertEquals(estimatedBiasY, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun estimatedGyroscopeBiasZAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasZAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.estimatedBiasAngularSpeedZ }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasZAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasZAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        val w = AngularSpeed(estimatedBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND)
        every { internalCalibratorSpy.biasAngularSpeedZ }.returns(w)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(w, calibrator.estimatedGyroscopeBiasZAsMeasurement)
    }

    @Test
    fun getEstimatedGyroscopeBiasZAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasZAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasAngularSpeedZ(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasZ
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(w))

        assertEquals(estimatedBiasZ, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun getEstimatedGyroscopeBiasZAsMeasurement_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getBiasAngularSpeedZ(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeed
            result.value = estimatedBiasZ
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val w = AngularSpeed(estimatedBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertTrue(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(w))

        assertEquals(estimatedBiasZ, w.value.toDouble(), 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w.unit)
    }

    @Test
    fun estimatedGyroscopeBiasAsTriad_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
    }

    @Test
    fun estimatedGyroscopeBiasAsTriad_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        val triad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        every { internalCalibratorSpy.estimatedBiasAsTriad }.returns(triad)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedGyroscopeBiasAsTriad)
    }

    @Test
    fun estimatedGyroscopeBiasAsTriad_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        val triad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(triad, calibrator.estimatedGyroscopeBiasAsTriad)
    }

    @Test
    fun getEstimatedGyroscopeBiasAsTriad_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val triad = AngularSpeedTriad()
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))
    }

    @Test
    fun getEstimatedGyroscopeBiasAsTriad_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(EasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedBiasAsTriad(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AngularSpeedTriad
            result.setValueCoordinatesAndUnit(
                estimatedBiasX,
                estimatedBiasY,
                estimatedBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            return@answers true
        }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val triad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.unit)
    }

    @Test
    fun getEstimatedGyroscopeBiasAsTriad_whenKnownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerAndGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        val estimatedBiasY = randomizer.nextDouble()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        val triad = AngularSpeedTriad(
            AngularSpeedUnit.RADIANS_PER_SECOND,
            estimatedBiasX,
            estimatedBiasY,
            estimatedBiasZ
        )
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))

        assertEquals(estimatedBiasX, triad.valueX, 0.0)
        assertEquals(estimatedBiasY, triad.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.unit)
    }

    private companion object {
        const val MA_SIZE = 3

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

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 15

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val ACCUMULATED_STD = 0.007

        const val MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6
        const val DEG_TO_RAD = 0.01745329252

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
    }
}