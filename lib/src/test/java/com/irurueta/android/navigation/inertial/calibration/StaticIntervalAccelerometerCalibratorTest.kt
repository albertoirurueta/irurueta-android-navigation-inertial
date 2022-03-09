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
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator
import com.irurueta.navigation.inertial.calibration.IMUErrors
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class StaticIntervalAccelerometerCalibratorTest {

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenAccelerometerSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.accelerometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenAccelerometerSensorDelay_returnsExpectedValues() {
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
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
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
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isAccelerometerGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenIsAccelerometerGroundTruthInitialBias_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertNull(calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertTrue(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
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
            isAccelerometerGroundTruthInitialBias = true,
            location
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertNull(calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenStaticIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertNull(calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenDynamicIntervalDetectedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertNull(calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenStaticIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertNull(calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenDynamicIntervalSkippedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
        assertSame(location, calibrator.location)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
        assertNull(calibrator.generatedAccelerometerMeasurementListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenGeneratedAccelerometerMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
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
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertNull(calibrator.unreliableGravityNormEstimationListener)
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenUnreliableGravityNormEstimationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimatorListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimatorListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenInitialBiasAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimatorListener,
            initialBiasAvailableListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimatorListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(
            initialBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertNull(calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenAccuracyChangedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimatorListener,
            initialBiasAvailableListener,
            accuracyChangedListener
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimatorListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(
            initialBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertNotNull(calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun constructor_whenQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        val initialBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val qualityScoreMapper = mockk<QualityScoreMapper<StandardDeviationBodyKinematics>>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            solveCalibrationWhenEnoughMeasurements = false,
            isAccelerometerGroundTruthInitialBias = true,
            location,
            initializationStartedListener,
            initializationCompletedListener,
            errorListener,
            staticIntervalDetectedListener,
            dynamicIntervalDetectedListener,
            staticIntervalSkippedListener,
            dynamicIntervalSkippedListener,
            generatedAccelerometerMeasurementListener,
            readyToSolveCalibrationListener,
            calibrationSolvingStartedListener,
            calibrationCompletedListener,
            stoppedListener,
            unreliableGravityNormEstimatorListener,
            initialBiasAvailableListener,
            accuracyChangedListener,
            qualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.accelerometerSensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isAccelerometerGroundTruthInitialBias)
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
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(
            unreliableGravityNormEstimatorListener,
            calibrator.unreliableGravityNormEstimationListener
        )
        assertSame(
            initialBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
        assertSame(accuracyChangedListener, calibrator.accuracyChangedListener)
        assertSame(qualityScoreMapper, calibrator.accelerometerQualityScoreMapper)
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
        val triad = AccelerationTriad()
        assertFalse(calibrator.getAccelerometerInitialBiasAsTriad(triad))
        assertFalse(calibrator.isGravityNormEstimated)
        assertNull(calibrator.accelerometerSensor)
        assertNull(calibrator.gravitySensor)
        val ma1 = Matrix(MA_SIZE, MA_SIZE)
        assertEquals(ma1, calibrator.accelerometerInitialMa)
        val ma2 = Matrix.identity(MA_SIZE, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma2)
        assertEquals(ma1, ma2)
        assertEquals(0.0, calibrator.accelerometerInitialSx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialSz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxy, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMxz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMyz, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzx, 0.0)
        assertEquals(0.0, calibrator.accelerometerInitialMzy, 0.0)
        assertFalse(calibrator.isAccelerometerCommonAxisUsed)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredAccelerometerMeasurements
        )
        assertEquals(
            calibrator.minimumRequiredAccelerometerMeasurements,
            calibrator.minimumRequiredMeasurements
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        assertFalse(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
        assertTrue(calibrator.accelerometerMeasurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>()
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
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
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedAccelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.generatedAccelerometerMeasurementListener)

        // set new value
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>()
        calibrator.generatedAccelerometerMeasurementListener =
            generatedAccelerometerMeasurementListener

        // check
        assertSame(
            generatedAccelerometerMeasurementListener,
            calibrator.generatedAccelerometerMeasurementListener
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>()
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>()
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>()
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
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun unreliableGravityNormEstimatorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.unreliableGravityNormEstimationListener)

        // set new value
        val unreliableGravityNormEstimatorListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>()
        calibrator.unreliableGravityNormEstimationListener = unreliableGravityNormEstimatorListener

        // check
        assertSame(
            unreliableGravityNormEstimatorListener,
            calibrator.unreliableGravityNormEstimationListener
        )
    }

    @Test
    fun initialAccelerometerBiasAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertNull(calibrator.initialAccelerometerBiasAvailableListener)

        // set new value
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>()
        calibrator.initialAccelerometerBiasAvailableListener =
            initialAccelerometerBiasAvailableListener

        // check
        assertSame(
            initialAccelerometerBiasAvailableListener,
            calibrator.initialAccelerometerBiasAvailableListener
        )
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isAccelerometerGroundTruthInitialBias = true
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerInitialMa_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.accelerometerInitialMa = ma
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerInitialMa_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(1, MA_SIZE)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getAccelerometerInitialMa_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val ma = Matrix(MA_SIZE, 1)
        calibrator.getAccelerometerInitialMa(ma)
    }

    @Test
    fun accelerometerInitialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
    fun accelerometerInitialMxz_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerInitialMzy = 0.0
    }

    @Test
    fun setAccelerometerInitialScalingFactors_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
    fun isAccelerometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertFalse(calibrator.isAccelerometerCommonAxisUsed)

        // set new value
        calibrator.isAccelerometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isAccelerometerCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isAccelerometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.isAccelerometerCommonAxisUsed = true
    }

    @Test
    fun accelerometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun accelerometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        calibrator.accelerometerRobustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun accelerometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        calibrator.accelerometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
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
    fun baseNoiseLevelAbsoluteThresholdMeasurement_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
    fun requiredMeasurements_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val initializationStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_setsGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        // check initial value
        assertNull(calibrator.gravityNorm)

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        every { gravityNormEstimatorSpy.averageNorm }.returns(gravityNorm)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )

        // check
        assertEquals(gravityNorm, calibrator.gravityNorm)
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_setsGravityNormAndNotifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        // check initial value
        assertNull(calibrator.gravityNorm)

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        every { gravityNormEstimatorSpy.averageNorm }.returns(gravityNorm)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
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

        setPrivateProperty(
            StaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )
        assertTrue(calibrator.running)

        val generator: AccelerometerMeasurementGenerator? =
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

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<AccelerometerMeasurementGenerator>? =
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
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
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

        val generator: AccelerometerMeasurementGenerator? =
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

        val generatorErrorListener: SingleSensorCalibrationMeasurementGenerator.OnErrorListener<AccelerometerMeasurementGenerator>? =
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val staticIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val dynamicIntervalDetectedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val staticIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val dynamicIntervalSkippedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<AccelerometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedMeasurement_addsMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.accelerometerMeasurements.size)
        assertSame(measurement, calibrator.accelerometerMeasurements[0])
    }

    @Test
    fun onGeneratedMeasurement_whenListenerAvailable_notifies() {
        val generatedAccelerometerMeasurementListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnGeneratedAccelerometerMeasurementListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            generatedAccelerometerMeasurementListener = generatedAccelerometerMeasurementListener
        )

        assertTrue(calibrator.accelerometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val generator = mockk<AccelerometerMeasurementGenerator>()
        val measurement = StandardDeviationBodyKinematics()
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.accelerometerMeasurements.size)
        assertSame(measurement, calibrator.accelerometerMeasurements[0])

        verify(exactly = 1) {
            generatedAccelerometerMeasurementListener.onGeneratedAccelerometerMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            )
        }
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToCalibrate_stopsAndBuildCalibrator() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = false,
            location = location
        )

        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = StandardDeviationBodyKinematics()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        val location = getLocation()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            solveCalibrationWhenEnoughMeasurements = false,
            location = location
        )

        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = StandardDeviationBodyKinematics()
        for (i in 1..calibrator.requiredMeasurements) {
            calibrator.accelerometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = true,
            location = location
        )

        assertEquals(
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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

            val measurement =
                StandardDeviationBodyKinematics(measuredKinematics, ACCUMULATED_STD, 0.0)
            calibrator.accelerometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            StandardDeviationBodyKinematics(calibrator.accelerometerMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

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
        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabledAndListenersAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )
        val errorListener =
            mockk<StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerCalibrator>>(
                relaxUnitFun = true
            )

        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
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
            StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL,
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

            val measurement =
                StandardDeviationBodyKinematics(measuredKinematics, ACCUMULATED_STD, 0.0)
            calibrator.accelerometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val gravityNormEstimator: GravityNormEstimator? =
            calibrator.getPrivateProperty("gravityNormEstimator")
        requireNotNull(gravityNormEstimator)
        val gravityNormEstimatorSpy = spyk(gravityNormEstimator)
        every { gravityNormEstimatorSpy.running }.returns(true)
        calibrator.setPrivateProperty("gravityNormEstimator", gravityNormEstimatorSpy)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNull(accelerometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement =
            StandardDeviationBodyKinematics(calibrator.accelerometerMeasurements.last())
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        verify(exactly = 1) { gravityNormEstimatorSpy.stop() }
        verify(exactly = 1) { generatorSpy.stop() }

        accelerometerInternalCalibrator =
            calibrator.getPrivateProperty("accelerometerInternalCalibrator")
        assertNotNull(accelerometerInternalCalibrator)

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
        val triad = AccelerationTriad()
        assertTrue(calibrator.getEstimatedAccelerometerBiasAsTriad(triad))
        assertNotNull(calibrator.estimatedAccelerometerBiasStandardDeviationNorm)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
    fun onAccelerometerMeasurement_whenFirstMeasurementAndListener_updatesInitialBiasesAndNotifies() {
        val initialAccelerometerBiasAvailableListener =
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(0)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
            mockk<StaticIntervalAccelerometerCalibrator.OnInitialAccelerometerBiasAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
            context,
            initialAccelerometerBiasAvailableListener = initialAccelerometerBiasAvailableListener
        )

        assertNull(calibrator.accelerometerInitialBiasX)
        assertNull(calibrator.accelerometerInitialBiasY)
        assertNull(calibrator.accelerometerInitialBiasZ)

        val generator: AccelerometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        every { generatorSpy.numberOfProcessedAccelerometerMeasurements }.returns(2)
        calibrator.setPrivateProperty("generator", generatorSpy)

        val generatorDetectorMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("generatorDetectorMeasurementListener")
        requireNotNull(generatorDetectorMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        generatorDetectorMeasurementListener.onMeasurement(
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
    fun onGravityNormCompletedListener_setsGravityNorm() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
        val calibrator = StaticIntervalAccelerometerCalibrator(context)

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
            mockk<StaticIntervalAccelerometerCalibrator.OnUnreliableGravityEstimationListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalAccelerometerCalibrator(
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