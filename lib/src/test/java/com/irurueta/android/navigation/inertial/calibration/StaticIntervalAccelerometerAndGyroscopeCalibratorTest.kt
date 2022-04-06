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
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerAndGyroscopeMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.every
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.max

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

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(generatorSpy, accelerometerMeasurement)

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

        generatorGeneratedMeasurementListener.onGeneratedAccelerometerMeasurement(generatorSpy, accelerometerMeasurement)

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

    private companion object {
        const val MA_SIZE = 3

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

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
    }
}