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
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.GyroscopeMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.MagnetometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class StaticIntervalGyroscopeCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, calibrator.gyroscopeSensorType)
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            false
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isGyroscopeGroundTruthInitialBias = true,
            initializationStartedListener
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        // set new value
        calibrator.isGDependentCrossBiasesEstimated = true
    }

    @Test
    fun minimumRequiredGyroscopeMeasurements_whenGroundTruthInitialBiasCommonAxisAndCrossBiases_returnsExpctedValue() {
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
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

    /*@Test
    fun estimatedGyroscopeMg_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalGyroscopeCalibrator(context)

        // check default values
        val mg1 = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        val mg2 = calibrator.estimatedGyroscopeMg
        assertEquals(mg1, mg2)
    }*/
    // TODO: setAccelerometerScalingFactors

    private companion object {
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
    }
}