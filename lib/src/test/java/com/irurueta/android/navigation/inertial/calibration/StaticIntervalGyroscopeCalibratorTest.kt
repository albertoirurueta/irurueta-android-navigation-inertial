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
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.GyroscopeMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.callPrivateFuncWithResult
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.gyroscope.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.lang.reflect.InvocationTargetException
import java.util.*
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class StaticIntervalGyroscopeCalibratorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAccelerometerSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenGyroscopeSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenAccelerometerSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenGyroscopeSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenSolveCalibrationWhenEnoughMeasurements_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            false
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertEquals(0, calibrator.numberOfProcessedAccelerometerMeasurements)
    }

    @Test
    fun constructor_whenIsGyroscopeGroundTruthInitialBias_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenStaticIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenDynamicIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenStaticIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenDynamicIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
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
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedGyroscopeMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenGeneratedGyroscopeMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenInitialGyroscopeBiasAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>()
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            initialGyroscopeBiasAvailableListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenAccuracyChangedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>()
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            initialGyroscopeBiasAvailableListener,
            accuracyChangedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertNotNull(calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun constructor_whenGyroscopeQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>()
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val gyroscopeQualityScoreMapper =
            mockk<QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedGyroscopeMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            initialGyroscopeBiasAvailableListener,
            accuracyChangedListener,
            gyroscopeQualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            calibrator.gyroscopeSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertEquals(SensorDelay.NORMAL, calibrator.gyroscopeSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertSame(
            generatedGyroscopeMeasurementListener,
            calibrator.generatedGyroscopeMeasurementListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            initialGyroscopeBiasAvailableListener,
            calibrator.initialGyroscopeBiasAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertSame(gyroscopeQualityScoreMapper, calibrator.gyroscopeQualityScoreMapper)
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        var acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val acceleration2 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration2)
        assertEquals(acceleration1, acceleration2)
        val accelerationTriad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, accelerationTriad1.valueX, 0.0)
        assertEquals(0.0, accelerationTriad1.valueY, 0.0)
        assertEquals(0.0, accelerationTriad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.unit)
        val accelerationTriad2 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(accelerationTriad2)
        assertEquals(accelerationTriad1, accelerationTriad2)
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
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gyroscopeSensor)
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.gyroscopeInitialMg
        assertEquals(mg1, mg2)
        val mg3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getGyroscopeInitialMg(mg3)
        assertEquals(mg1, mg3)
        assertEquals(0.0, calibrator.gyroscopeInitialSx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialSz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxy, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMxz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMyz, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzx, 0.0)
        assertEquals(0.0, calibrator.gyroscopeInitialMzy, 0.0)
        val gg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val gg2 = calibrator.gyroscopeInitialGg
        assertEquals(gg1, gg2)
        assertFalse(calibrator.isGyroscopeCommonAxisUsed)
        assertFalse(calibrator.isGDependentCrossBiasesEstimated)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.minimumRequiredGyroscopeMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
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
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration))
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val accelerationTriad = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(accelerationTriad))
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        val ma3 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma3)
        assertEquals(ma1, ma3)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
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
        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())
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
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedGyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.generatedGyroscopeMeasurementListener)

        // set new value
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>()
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun initialGyroscopeBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.initialGyroscopeBiasAvailableListener)

        // set new value
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>()
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        calibrator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
    }

    @Test
    fun accelerometerInitialBiasX_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasX = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasX = biasX

        // check
        assertEquals(biasX, calibrator.accelerometerInitialBiasX, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasX_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasX = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasX = biasX
    }

    @Test
    fun accelerometerInitialBiasY_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasY = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasY = biasY

        // check
        assertEquals(biasY, calibrator.accelerometerInitialBiasY, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasY_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasY = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasY = biasY
    }

    @Test
    fun accelerometerInitialBiasZ_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasZ = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasZ = biasZ

        // check
        assertEquals(biasZ, calibrator.accelerometerInitialBiasZ, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasZ_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val biasZ = randomizer.nextDouble()
        calibrator.accelerometerInitialBiasZ = biasZ
    }

    @Test
    fun accelerometerInitialBiasXAsMeasurement_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasXAsMeasurement = acceleration2

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasX, 0.0)
        val acceleration3 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(acceleration2, acceleration3)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasXAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasXAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasXAsMeasurement = acceleration2
    }

    @Test
    fun getAccelerometerInitialBiasXAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration1)
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasXAsMeasurement = acceleration2

        // check
        val acceleration3 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasXAsMeasurement(acceleration3)
        assertEquals(acceleration2, acceleration3)
    }

    @Test
    fun accelerometerInitialBiasYAsMeasurement_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasYAsMeasurement = acceleration2

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasY, 0.0)
        val acceleration3 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(acceleration2, acceleration3)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasYAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasYAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasYAsMeasurement = acceleration2
    }

    @Test
    fun getAccelerometerInitialBiasYAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration1)
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasYAsMeasurement = acceleration2

        // check
        val acceleration3 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasYAsMeasurement(acceleration3)
        assertEquals(acceleration2, acceleration3)
    }

    @Test
    fun accelerometerInitialBiasZAsMeasurement_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasZAsMeasurement = acceleration2

        // check
        assertEquals(value, calibrator.accelerometerInitialBiasZ, 0.0)
        val acceleration3 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(acceleration2, acceleration3)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasZAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        val acceleration1 = calibrator.accelerometerInitialBiasZAsMeasurement
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasZAsMeasurement = acceleration2
    }

    @Test
    fun getAccelerometerInitialBiasZAsMeasurement_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration1)
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val acceleration2 = Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.accelerometerInitialBiasZAsMeasurement = acceleration2

        // check
        val acceleration3 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.getAccelerometerInitialBiasZAsMeasurement(acceleration3)
        assertEquals(acceleration2, acceleration3)
    }

    @Test
    fun accelerometerInitialBiasAsTriad_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val triad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, triad1.valueX, 0.0)
        assertEquals(0.0, triad1.valueY, 0.0)
        assertEquals(0.0, triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val triad2 =
            AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, valueX, valueY, valueZ)
        calibrator.accelerometerInitialBiasAsTriad = triad2

        // check
        assertEquals(valueX, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(valueY, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(valueZ, calibrator.accelerometerInitialBiasZ, 0.0)
        val triad3 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(triad2, triad3)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialBiasAsTriad_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set running
        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // check default value
        val triad1 = calibrator.accelerometerInitialBiasAsTriad
        assertEquals(0.0, triad1.valueX, 0.0)
        assertEquals(0.0, triad1.valueY, 0.0)
        assertEquals(0.0, triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertTrue(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val triad2 =
            AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, valueX, valueY, valueZ)
        calibrator.accelerometerInitialBiasAsTriad = triad2
    }

    @Test
    fun getAccelerometerInitialBiasAsTriad_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val triad1 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(triad1)
        assertEquals(0.0, triad1.valueX, 0.0)
        assertEquals(0.0, triad1.valueY, 0.0)
        assertEquals(0.0, triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertFalse(calibrator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val triad2 =
            AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, valueX, valueY, valueZ)
        calibrator.accelerometerInitialBiasAsTriad = triad2

        // check
        assertEquals(valueX, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(valueY, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(valueZ, calibrator.accelerometerInitialBiasZ, 0.0)
        val triad3 = AccelerationTriad()
        calibrator.getAccelerometerInitialBiasAsTriad(triad3)
        assertEquals(triad2, triad3)
    }

    @Test
    fun isGyroscopeGroundTruthInitialBias_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isGyroscopeGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isGyroscopeGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isGyroscopeGroundTruthInitialBias = true
    }

    @Test
    fun gyroscopeInitialMg_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val mg1 = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialMg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val mg1 = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.gyroscopeInitialMg = mg1
    }

    @Test
    fun getGyroscopeInitialMg_whenValid_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set new value
        val gg = Matrix(1, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeInitialGg_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set new value
        val gg = Matrix(BodyKinematics.COMPONENTS, 1)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg
    }

    @Test
    fun isGyroscopeCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
    fun gyroscopeRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun gyroscopeRobustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun gyroscopeRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun gyroscopeRobustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun gyroscopeRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun gyroscopeRobustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun gyroscopeRobustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun gyroscopeRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.gyroscopeRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun gyroscopeRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun isAccelerometerGroundTruthInitialBias_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertFalse(calibrator.running)

        // set new value
        calibrator.isAccelerometerGroundTruthInitialBias = true

        // check
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerGroundTruthInitialBias_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isAccelerometerGroundTruthInitialBias = true
    }

    @Test
    fun accelerometerMa_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val ma2 = calibrator.accelerometerMa
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)

        // set new value
        val ma3 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, ma3)
        calibrator.accelerometerMa = ma3

        // check
        val ma4 = calibrator.accelerometerMa
        assertEquals(ma3, ma4)
        assertEquals(calibrator.accelerometerSx, ma4.getElementAt(0, 0), 0.0)
        assertEquals(calibrator.accelerometerMyx, ma4.getElementAt(1, 0), 0.0)
        assertEquals(calibrator.accelerometerMzx, ma4.getElementAt(2, 0), 0.0)
        assertEquals(calibrator.accelerometerMxy, ma4.getElementAt(0, 1), 0.0)
        assertEquals(calibrator.accelerometerSy, ma4.getElementAt(1, 1), 0.0)
        assertEquals(calibrator.accelerometerMzy, ma4.getElementAt(2, 1), 0.0)
        assertEquals(calibrator.accelerometerMxz, ma4.getElementAt(0, 2), 0.0)
        assertEquals(calibrator.accelerometerMyz, ma4.getElementAt(1, 2), 0.0)
        assertEquals(calibrator.accelerometerSz, ma4.getElementAt(2, 2), 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerMa_whenInvalidRows_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set new value
        val ma = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.accelerometerMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerMa_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set new value
        val ma = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.accelerometerMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometer_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val ma = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.accelerometerMa = ma
    }

    @Test
    fun getAccelerometerMa_whenValid_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // set new value
        val ma1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, ma1)
        calibrator.accelerometerMa = ma1

        // check
        val ma2 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma2)
        assertEquals(ma1, ma2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerMa_whenInvalidRows_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val ma = Matrix(1, BodyKinematics.COMPONENTS)
        calibrator.getAccelerometerMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerMa_whenInvalidColumns_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val ma = Matrix(BodyKinematics.COMPONENTS, 1)
        calibrator.getAccelerometerMa(ma)
    }

    @Test
    fun accelerometerSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSx = randomizer.nextDouble()
        calibrator.accelerometerSx = accelerometerSx

        // check
        assertEquals(accelerometerSx, calibrator.accelerometerSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSx = randomizer.nextDouble()
        calibrator.accelerometerSx = accelerometerSx
    }

    @Test
    fun accelerometerSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSy = randomizer.nextDouble()
        calibrator.accelerometerSy = accelerometerSy

        // check
        assertEquals(accelerometerSy, calibrator.accelerometerSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSy = randomizer.nextDouble()
        calibrator.accelerometerSy = accelerometerSy
    }

    @Test
    fun accelerometerSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSz = randomizer.nextDouble()
        calibrator.accelerometerSz = accelerometerSz

        // check
        assertEquals(accelerometerSz, calibrator.accelerometerSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSz = randomizer.nextDouble()
        calibrator.accelerometerSz = accelerometerSz
    }

    @Test
    fun accelerometerMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxy = randomizer.nextDouble()
        calibrator.accelerometerMxy = accelerometerMxy

        // check
        assertEquals(accelerometerMxy, calibrator.accelerometerMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxy = randomizer.nextDouble()
        calibrator.accelerometerMxy = accelerometerMxy
    }

    @Test
    fun accelerometerMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxz = randomizer.nextDouble()
        calibrator.accelerometerMxz = accelerometerMxz

        // check
        assertEquals(accelerometerMxz, calibrator.accelerometerMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxz = randomizer.nextDouble()
        calibrator.accelerometerMxz = accelerometerMxz
    }

    @Test
    fun accelerometerMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMyx = randomizer.nextDouble()
        calibrator.accelerometerMyx = accelerometerMyx

        // check
        assertEquals(accelerometerMyx, calibrator.accelerometerMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMyx = randomizer.nextDouble()
        calibrator.accelerometerMyx = accelerometerMyx
    }

    @Test
    fun accelerometerMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMyz = randomizer.nextDouble()
        calibrator.accelerometerMyz = accelerometerMyz

        // check
        assertEquals(accelerometerMyz, calibrator.accelerometerMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMyz = randomizer.nextDouble()
        calibrator.accelerometerMyz = accelerometerMyz
    }

    @Test
    fun accelerometerMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMzx = randomizer.nextDouble()
        calibrator.accelerometerMzx = accelerometerMzx

        // check
        assertEquals(accelerometerMzx, calibrator.accelerometerMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMzx = randomizer.nextDouble()
        calibrator.accelerometerMzx = accelerometerMzx
    }

    @Test
    fun accelerometerMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMzy = randomizer.nextDouble()
        calibrator.accelerometerMzy = accelerometerMzy

        // check
        assertEquals(accelerometerMzy, calibrator.accelerometerMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMzy = randomizer.nextDouble()
        calibrator.accelerometerMzy = accelerometerMzy
    }

    @Test
    fun setAccelerometerScalingFactors_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default values
        assertEquals(0.0, calibrator.accelerometerSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSx = randomizer.nextDouble()
        val accelerometerSy = randomizer.nextDouble()
        val accelerometerSz = randomizer.nextDouble()
        calibrator.setAccelerometerScalingFactors(accelerometerSx, accelerometerSy, accelerometerSz)

        // check
        assertEquals(accelerometerSx, calibrator.accelerometerSx, 0.0)
        assertEquals(accelerometerSy, calibrator.accelerometerSy, 0.0)
        assertEquals(accelerometerSz, calibrator.accelerometerSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerScalingFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerSx = randomizer.nextDouble()
        val accelerometerSy = randomizer.nextDouble()
        val accelerometerSz = randomizer.nextDouble()
        calibrator.setAccelerometerScalingFactors(accelerometerSx, accelerometerSy, accelerometerSz)
    }

    @Test
    fun setAccelerometerCrossCouplingErrors_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default values
        assertEquals(0.0, calibrator.accelerometerMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxy = randomizer.nextDouble()
        val accelerometerMxz = randomizer.nextDouble()
        val accelerometerMyx = randomizer.nextDouble()
        val accelerometerMyz = randomizer.nextDouble()
        val accelerometerMzx = randomizer.nextDouble()
        val accelerometerMzy = randomizer.nextDouble()
        calibrator.setAccelerometerCrossCouplingErrors(
            accelerometerMxy,
            accelerometerMxz,
            accelerometerMyx,
            accelerometerMyz,
            accelerometerMzx,
            accelerometerMzy
        )

        // check
        assertEquals(accelerometerMxy, calibrator.accelerometerMxy, 0.0)
        assertEquals(accelerometerMxz, calibrator.accelerometerMxz, 0.0)
        assertEquals(accelerometerMyx, calibrator.accelerometerMyx, 0.0)
        assertEquals(accelerometerMyz, calibrator.accelerometerMyz, 0.0)
        assertEquals(accelerometerMzx, calibrator.accelerometerMzx, 0.0)
        assertEquals(accelerometerMzy, calibrator.accelerometerMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setAccelerometerCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        val randomizer = UniformRandomizer()
        val accelerometerMxy = randomizer.nextDouble()
        val accelerometerMxz = randomizer.nextDouble()
        val accelerometerMyx = randomizer.nextDouble()
        val accelerometerMyz = randomizer.nextDouble()
        val accelerometerMzx = randomizer.nextDouble()
        val accelerometerMzy = randomizer.nextDouble()
        calibrator.setAccelerometerCrossCouplingErrors(
            accelerometerMxy,
            accelerometerMxz,
            accelerometerMyx,
            accelerometerMyz,
            accelerometerMzx,
            accelerometerMzy
        )
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
    }

    @Test
    fun onInitializationStarted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_notifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )

        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<GyroscopeMeasurementGenerator>? =
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            errorListener = errorListener,
            stoppedListener = stoppedListener
        )

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<GyroscopeMeasurementGenerator>? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<GyroscopeMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedMeasurement_addsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.gyroscopeMeasurements.size)
        assertSame(measurement, calibrator.gyroscopeMeasurements[0])
    }

    @Test
    fun onGeneratedMeasurement_whenListenerAvailable_notifies() {
        val generatedGyroscopeMeasurementListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnGeneratedGyroscopeMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            generatedGyroscopeMeasurementListener = generatedGyroscopeMeasurementListener
        )

        assertTrue(calibrator.gyroscopeMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<GyroscopeMeasurementGenerator>()
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

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
    fun onGeneratedMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = false,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener
        )

        assertEquals(
            StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
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
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val longitude =
            Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
        val nedPosition = NEDPosition(latitude, longitude, height)

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
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
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
            QuaternionIntegrator.integrateGyroSequence(
                trueSequence,
                beforeQ,
                QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ
            )

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
                TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errors, random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            BodyKinematicsSequence(calibrator.gyroscopeMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
        val triad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))
        assertNotNull(calibrator.estimatedGyroscopeBiasStandardDeviationNorm)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
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
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val longitude =
            Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
        val nedPosition = NEDPosition(latitude, longitude, height)

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
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
            )
            val deltaPitch = Math.toRadians(
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
            )
            val deltaYaw = Math.toRadians(
                randomizer.nextDouble(
                    MIN_ANGLE_VARIATION_DEGREES,
                    MAX_ANGLE_VARIATION_DEGREES
                )
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
            QuaternionIntegrator.integrateGyroSequence(
                trueSequence,
                beforeQ,
                QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ
            )

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
                TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errors, random
            )
            val afterMeanFx = measuredAfterGravityKinematics.fx
            val afterMeanFy = measuredAfterGravityKinematics.fy
            val afterMeanFz = measuredAfterGravityKinematics.fz

            sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz)

            sequences.add(sequence)
        }
        calibrator.gyroscopeMeasurements.addAll(sequences)

        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNull(gyroscopeInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            BodyKinematicsSequence(calibrator.gyroscopeMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { generatorSpy.stop() }

        gyroscopeInternalCalibrator =
            calibrator.getPrivateProperty("gyroscopeInternalCalibrator")
        assertNotNull(gyroscopeInternalCalibrator)

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
        val triad = AngularSpeedTriad()
        assertTrue(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))
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
    fun onAccelerometerMeasurement_whenFirstMeasurementAndNoGroundTruthBias_updatesEstimatedAccelerometerBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
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

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        val estimatedBiasX = calibrator.estimatedAccelerometerBiasX
        requireNotNull(estimatedBiasX)
        assertEquals(by.toDouble(), estimatedBiasX, 0.0)
        val estimatedBiasY = calibrator.estimatedAccelerometerBiasY
        requireNotNull(estimatedBiasY)
        assertEquals(bx.toDouble(), estimatedBiasY, 0.0)
        val estimatedBiasZ = calibrator.estimatedAccelerometerBiasZ
        requireNotNull(estimatedBiasZ)
        assertEquals(-bz.toDouble(), estimatedBiasZ, 0.0)
        var acceleration2 = calibrator.estimatedAccelerometerBiasXAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(estimatedBiasX, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasYAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(estimatedBiasY, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasZAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(estimatedBiasZ, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertEquals(estimatedBiasX, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertEquals(estimatedBiasY, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertEquals(estimatedBiasZ, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val triad2 = calibrator.estimatedAccelerometerBiasAsTriad
        requireNotNull(triad2)
        assertEquals(estimatedBiasX, triad2.valueX, 0.0)
        assertEquals(estimatedBiasY, triad2.valueY, 0.0)
        assertEquals(estimatedBiasZ, triad2.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
        assertEquals(triad2, triad1)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementNoGroundTruthBiasAndNoBiasX_updatesEstimatedAccelerometerBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
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

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        val estimatedBiasX = calibrator.estimatedAccelerometerBiasX
        requireNotNull(estimatedBiasX)
        assertEquals(0.0, estimatedBiasX, 0.0)
        val estimatedBiasY = calibrator.estimatedAccelerometerBiasY
        requireNotNull(estimatedBiasY)
        assertEquals(0.0, estimatedBiasY, 0.0)
        val estimatedBiasZ = calibrator.estimatedAccelerometerBiasZ
        requireNotNull(estimatedBiasZ)
        assertEquals(0.0, estimatedBiasZ, 0.0)
        var acceleration2 = calibrator.estimatedAccelerometerBiasXAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasYAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasZAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val triad2 = calibrator.estimatedAccelerometerBiasAsTriad
        requireNotNull(triad2)
        assertEquals(0.0, triad2.valueX, 0.0)
        assertEquals(0.0, triad2.valueY, 0.0)
        assertEquals(0.0, triad2.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
        assertEquals(triad2, triad1)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementNoGroundTruthBiasAndNoBiasY_updatesEstimatedAccelerometerBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
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

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        val estimatedBiasX = calibrator.estimatedAccelerometerBiasX
        requireNotNull(estimatedBiasX)
        assertEquals(0.0, estimatedBiasX, 0.0)
        val estimatedBiasY = calibrator.estimatedAccelerometerBiasY
        requireNotNull(estimatedBiasY)
        assertEquals(0.0, estimatedBiasY, 0.0)
        val estimatedBiasZ = calibrator.estimatedAccelerometerBiasZ
        requireNotNull(estimatedBiasZ)
        assertEquals(0.0, estimatedBiasZ, 0.0)
        var acceleration2 = calibrator.estimatedAccelerometerBiasXAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasYAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasZAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val triad2 = calibrator.estimatedAccelerometerBiasAsTriad
        requireNotNull(triad2)
        assertEquals(0.0, triad2.valueX, 0.0)
        assertEquals(0.0, triad2.valueY, 0.0)
        assertEquals(0.0, triad2.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
        assertEquals(triad2, triad1)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementNoGroundTruthBiasAndNoBiasZ_updatesEstimatedAccelerometerBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
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

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        val estimatedBiasX = calibrator.estimatedAccelerometerBiasX
        requireNotNull(estimatedBiasX)
        assertEquals(0.0, estimatedBiasX, 0.0)
        val estimatedBiasY = calibrator.estimatedAccelerometerBiasY
        requireNotNull(estimatedBiasY)
        assertEquals(0.0, estimatedBiasY, 0.0)
        val estimatedBiasZ = calibrator.estimatedAccelerometerBiasZ
        requireNotNull(estimatedBiasZ)
        assertEquals(0.0, estimatedBiasZ, 0.0)
        var acceleration2 = calibrator.estimatedAccelerometerBiasXAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasYAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasZAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(0.0, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertEquals(0.0, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val triad2 = calibrator.estimatedAccelerometerBiasAsTriad
        requireNotNull(triad2)
        assertEquals(0.0, triad2.valueX, 0.0)
        assertEquals(0.0, triad2.valueY, 0.0)
        assertEquals(0.0, triad2.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
        assertEquals(triad2, triad1)
    }

    @Test
    fun onAccelerometerMeasurement_whenFirstMeasurementAndGroundTruthBias_updatesEstimatedAccelerometerBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)
        calibrator.isAccelerometerGroundTruthInitialBias = true

        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)

        val randomizer = UniformRandomizer()
        val accelerometerInitialBiasX = randomizer.nextDouble()
        val accelerometerInitialBiasY = randomizer.nextDouble()
        val accelerometerInitialBiasZ = randomizer.nextDouble()

        calibrator.accelerometerInitialBiasX = accelerometerInitialBiasX
        calibrator.accelerometerInitialBiasY = accelerometerInitialBiasY
        calibrator.accelerometerInitialBiasZ = accelerometerInitialBiasZ

        assertEquals(accelerometerInitialBiasX, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(accelerometerInitialBiasY, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(accelerometerInitialBiasZ, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorAccelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorAccelerometerMeasurementListener")
        requireNotNull(generatorAccelerometerMeasurementListener)

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

        assertEquals(accelerometerInitialBiasX, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(accelerometerInitialBiasY, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(accelerometerInitialBiasZ, calibrator.accelerometerInitialBiasZ, 0.0)
        val estimatedBiasX = calibrator.estimatedAccelerometerBiasX
        requireNotNull(estimatedBiasX)
        assertEquals(accelerometerInitialBiasX, estimatedBiasX, 0.0)
        val estimatedBiasY = calibrator.estimatedAccelerometerBiasY
        requireNotNull(estimatedBiasY)
        assertEquals(accelerometerInitialBiasY, estimatedBiasY, 0.0)
        val estimatedBiasZ = calibrator.estimatedAccelerometerBiasZ
        requireNotNull(estimatedBiasZ)
        assertEquals(accelerometerInitialBiasZ, estimatedBiasZ, 0.0)
        var acceleration2 = calibrator.estimatedAccelerometerBiasXAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(accelerometerInitialBiasX, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasYAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(accelerometerInitialBiasY, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        acceleration2 = calibrator.estimatedAccelerometerBiasZAsMeasurement
        requireNotNull(acceleration2)
        assertEquals(accelerometerInitialBiasZ, acceleration2.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertEquals(accelerometerInitialBiasX, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertEquals(accelerometerInitialBiasY, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertEquals(accelerometerInitialBiasZ, acceleration1.value.toDouble(), 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.unit)
        val triad2 = calibrator.estimatedAccelerometerBiasAsTriad
        requireNotNull(triad2)
        assertEquals(accelerometerInitialBiasX, triad2.valueX, 0.0)
        assertEquals(accelerometerInitialBiasY, triad2.valueY, 0.0)
        assertEquals(accelerometerInitialBiasZ, triad2.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad2.unit)
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
        assertEquals(triad2, triad1)
    }

    @Test
    fun onAccelerometerMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        val acceleration1 = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        val triad1 = AccelerationTriad()
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))

        val generator: GyroscopeMeasurementGenerator? =
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

        assertEquals(0.0, calibrator.accelerometerInitialBiasX, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasY, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialBiasZ, 0.0)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasXAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasYAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasZAsMeasurement)
        assertNull(calibrator.estimatedAccelerometerBiasAsTriad)
        assertFalse(calibrator.getEstimatedAccelerometerBiasXAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasYAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasZAsMeasurement(acceleration1))
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad1))
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurement_updatesInitialBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
        assertEquals(by.toDouble(), initialBiasX, 0.0)
        assertEquals(bx.toDouble(), initialBiasY, 0.0)
        assertEquals(-bz.toDouble(), initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasX_updatesInitialBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
        assertEquals(0.0, initialBiasX, 0.0)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        assertEquals(0.0, initialBiasY, 0.0)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasY_updatesInitialBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
        assertEquals(0.0, initialBiasX, 0.0)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        assertEquals(0.0, initialBiasY, 0.0)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndNoBiasZ_updatesInitialBias() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
        assertEquals(0.0, initialBiasX, 0.0)
        val initialBiasY = calibrator.gyroscopeInitialBiasY
        requireNotNull(initialBiasY)
        assertEquals(0.0, initialBiasY, 0.0)
        val initialBiasZ = calibrator.gyroscopeInitialBiasZ
        requireNotNull(initialBiasZ)
        assertEquals(0.0, initialBiasZ, 0.0)
    }

    @Test
    fun onGyroscopeMeasurement_whenFirstMeasurementAndListener_updatesInitialBias() {
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
        assertEquals(by.toDouble(), initialBiasX, 0.0)
        assertEquals(bx.toDouble(), initialBiasY, 0.0)
        assertEquals(-bz.toDouble(), initialBiasZ, 0.0)

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
    fun onGyroscopeMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val initialGyroscopeBiasAvailableListener =
            mockk<StaticIntervalGyroscopeCalibrator.OnInitialGyroscopeBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            initialGyroscopeBiasAvailableListener = initialGyroscopeBiasAvailableListener
        )

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)

        val generator: GyroscopeMeasurementGenerator? =
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
    fun gyroscopeBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getGyroscopeBaseNoiseLevelAsMeasurement(w))

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration))

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.threshold)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.thresholdAsMeasurement)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val acceleration = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertFalse(calibrator.getThresholdAsMeasurement(acceleration))

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerAverageTimeIntervalAsTime(time))

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalVariance)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        assertNull(calibrator.accelerometerTimeIntervalStandardDeviation)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(time))

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
    fun start_whenNotRunning_resetsAndStartsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.running)

        val measurements = calibrator.gyroscopeMeasurements
        val measurementsSpy = spyk(measurements)
        calibrator.setPrivateProperty("gyroscopeMeasurements", measurementsSpy)

        calibrator.setPrivateProperty("gyroscopeInitialBiasX", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", 0.0)
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", 0.0)

        val internalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibrator)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.start() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        calibrator.start()

        // check
        verify(exactly = 1) { measurementsSpy.clear() }
        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { generatorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertFalse(calibrator.running)

        calibrator.start()

        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun stop_whenNoListenerAvailable_stopsGenerator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            stoppedListener = stoppedListener
        )

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.stop() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.calibrate()
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.calibrate()
    }

    @Test
    fun calibrate_whenReadyNotRunningAndNoInternalCalibrator_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..13) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..13) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        justRun { internalCalibrator.calibrate() }
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibrator)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..13) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalGyroscopeCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            errorListener = errorListener
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..13) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<GyroscopeNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibrator)

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
    fun estimatedGyroscopeBiasX_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasX)
    }

    @Test
    fun estimatedGyroscopeBiasX_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasX = randomizer.nextDouble()
        every { internalCalibratorSpy.biasX }.returns(estimatedBiasX)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasX, calibrator.estimatedGyroscopeBiasX)
    }

    @Test
    fun estimatedGyroscopeBiasY_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasY)
    }

    @Test
    fun estimatedGyroscopeBiasY_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasY = randomizer.nextDouble()
        every { internalCalibratorSpy.biasY }.returns(estimatedBiasY)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasY, calibrator.estimatedGyroscopeBiasY)
    }

    @Test
    fun estimatedGyroscopeBiasZ_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasZ)
    }

    @Test
    fun estimatedGyroscopeBiasZ_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        val internalCalibratorSpy = spyk(KnownBiasEasyGyroscopeCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedBiasZ = randomizer.nextDouble()
        every { internalCalibratorSpy.biasZ }.returns(estimatedBiasZ)
        calibrator.setPrivateProperty("gyroscopeInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedBiasZ, calibrator.estimatedGyroscopeBiasZ)
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasXAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasXAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasXAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasXAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasYAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasYAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasYAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasYAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasZAsMeasurement)
    }

    @Test
    fun estimatedGyroscopeBiasZAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val w = AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND)
        assertFalse(calibrator.getEstimatedGyroscopeBiasZAsMeasurement(w))
    }

    @Test
    fun getEstimatedGyroscopeBiasZAsMeasurement_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        assertNull(calibrator.estimatedGyroscopeBiasAsTriad)
    }

    @Test
    fun estimatedGyroscopeBiasAsTriad_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        assertNull(calibrator.getPrivateProperty("gyroscopeInternalCalibrator"))
        val triad = AngularSpeedTriad()
        assertFalse(calibrator.getEstimatedGyroscopeBiasAsTriad(triad))
    }

    @Test
    fun getEstimatedGyroscopeBiasAsTriad_whenUnknownBiasGyroscopeInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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

    @Test
    fun buildGyroscopeInternalCalibrator_whenNonRobustGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenNonRobustGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenNonRobustNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as EasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenNonRobustNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 = internalCalibrator as EasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenRANSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.RANSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenMSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.MSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.MSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROSAC
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenLMedSNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.LMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.biasX, 0.0)
        assertEquals(0.0, internalCalibrator2.biasY, 0.0)
        assertEquals(0.0, internalCalibrator2.biasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        assertNull(calibrator.gyroscopeInitialBiasX)
        assertNull(calibrator.gyroscopeInitialBiasY)
        assertNull(calibrator.gyroscopeInitialBiasZ)
        assertNull(calibrator.estimatedAccelerometerBiasX)
        assertNull(calibrator.estimatedAccelerometerBiasY)
        assertNull(calibrator.estimatedAccelerometerBiasZ)

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

        val randomizer = UniformRandomizer()
        val initialBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasX", initialBiasX)
        val initialBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasY", initialBiasY)
        val initialBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("gyroscopeInitialBiasZ", initialBiasZ)
        val accelerometerBiasX = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasX", accelerometerBiasX)
        val accelerometerBiasY = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasY", accelerometerBiasY)
        val accelerometerBiasZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("estimatedAccelerometerBiasZ", accelerometerBiasZ)

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
        assertEquals(accelerometerBiasX, internalCalibrator2.accelerometerBiasX, 0.0)
        assertEquals(accelerometerBiasY, internalCalibrator2.accelerometerBiasY, 0.0)
        assertEquals(accelerometerBiasZ, internalCalibrator2.accelerometerBiasZ, 0.0)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = true
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..16) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertTrue(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        calibrator.gyroscopeRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        val generator: GyroscopeMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val baseNoiseLevel = randomizer.nextDouble()
        every { generatorSpy.gyroscopeBaseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("generator", generatorSpy)

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.gyroscopeBaseNoiseLevel)

        val internalCalibrator: GyroscopeNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(calibrator.gyroscopeMeasurements, internalCalibrator2.sequences)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertTrue(internalCalibrator2.isGDependentCrossBiasesEstimated)
        assertEquals(gg, internalCalibrator2.initialGg)
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
        assertEquals(calibrator.gyroscopeMeasurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
        assertEquals(
            calibrator.minimumRequiredGyroscopeMeasurements,
            internalCalibrator2.minimumRequiredMeasurementsOrSequences
        )
    }

    @Test
    fun buildGyroscopeInternalCalibrator_whenPROMedSNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(
            context,
            isGyroscopeGroundTruthInitialBias = false
        )

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        for (i in 1..19) {
            calibrator.gyroscopeMeasurements.add(measurement)
        }

        calibrator.isGyroscopeCommonAxisUsed = true
        calibrator.isGDependentCrossBiasesEstimated = true
        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)
        calibrator.gyroscopeInitialGg = gg

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
        calibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.gyroscopeRobustMethod = RobustEstimatorMethod.PROMedS
        calibrator.gyroscopeRobustConfidence = ROBUST_CONFIDENCE
        calibrator.gyroscopeRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.gyroscopeRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.gyroscopeRobustThreshold = null
        calibrator.gyroscopeRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMedS, calibrator.gyroscopeRobustMethod)
        assertFalse(calibrator.isGyroscopeGroundTruthInitialBias)
        assertEquals(ROBUST_CONFIDENCE, calibrator.gyroscopeRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.gyroscopeRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.gyroscopeRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.gyroscopeRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.gyroscopeRobustThresholdFactor, 0.0)
        assertNull(calibrator.gyroscopeBaseNoiseLevel)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildGyroscopeInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    private companion object {
        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val WINDOW_SIZE = 51

        const val REQUIRED_MEASUREMENTS = 30

        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 20

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6
        const val DEG_TO_RAD = 0.01745329252

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0
        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0
        const val MIN_HEIGHT = -50.0
        const val MAX_HEIGHT = 50.0

        const val TIME_INTERVAL_SECONDS = 0.02

        const val MIN_ANGLE_DEGREES = -180.0
        const val MAX_ANGLE_DEGREES = 180.0

        const val MIN_ANGLE_VARIATION_DEGREES = -2.0
        const val MAX_ANGLE_VARIATION_DEGREES = 2.0

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