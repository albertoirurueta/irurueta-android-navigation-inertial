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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.*
import io.mockk.mockk
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

    // TODO: gyroscopeInitialMg
}