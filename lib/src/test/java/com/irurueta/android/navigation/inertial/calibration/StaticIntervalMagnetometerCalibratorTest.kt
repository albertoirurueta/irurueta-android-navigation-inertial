/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import android.util.Log
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.WrongSizeException
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.MagnetometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.android.testutils.callPrivateFuncWithResult
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator
import com.irurueta.navigation.inertial.calibration.CalibrationException
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronPositionAndInstantMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import java.lang.reflect.InvocationTargetException
import java.util.Date
import java.util.GregorianCalendar

class StaticIntervalMagnetometerCalibratorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var initializationStartedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var initializationCompletedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var errorListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalDetectedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalDetectedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var staticIntervalSkippedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var dynamicIntervalSkippedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var generatedMagnetometerMeasurementListener:
            StaticIntervalMagnetometerCalibrator.OnGeneratedMagnetometerMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var readyToSolveCalibrationListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var calibrationSolvingStartedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var calibrationCompletedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>

    @MockK(relaxUnitFun = true)
    private lateinit var stoppedListener:
            StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>

    @MockK
    private lateinit var magnetometerQualityScoreMapper:
            QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>

    @MockK
    private lateinit var generator: MagnetometerMeasurementGenerator

    @MockK
    private lateinit var internalCalibrator: MagnetometerNonLinearCalibrator

    @MockK
    private lateinit var measurement: StandardDeviationBodyMagneticFluxDensity

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

    @Test
    fun constructor_whenContext_returnsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)

        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default values
        assertSame(context, calibrator.context)
        assertNull(calibrator.location)
        assertNotNull(calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
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
        assertNotNull(calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.initialMagneticFluxDensityNorm)
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
        assertSame(accelerometerSensor, calibrator.accelerometerSensor)
        assertSame(magnetometerSensor, calibrator.magnetometerSensor)
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
        assertNull(calibrator.estimatedMagnetometerChiSqDegreesOfFreedom)
        assertNull(calibrator.estimatedMagnetometerReducedChiSq)
        assertNull(calibrator.estimatedMagnetometerP)
        assertNull(calibrator.estimatedMagnetometerQ)
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
    fun constructor_whenAllParameters_returnsDefaultValues() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER.value
            )
        }.returns(magnetometerSensor)

        val location = getLocation()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            worldMagneticModel,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
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
            magnetometerQualityScoreMapper
        )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            calibrator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
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
        assertSame(magnetometerQualityScoreMapper, calibrator.magnetometerQualityScoreMapper)
        assertNull(calibrator.initialMagneticFluxDensityNorm)
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
        assertSame(accelerometerSensor, calibrator.accelerometerSensor)
        assertSame(magnetometerSensor, calibrator.magnetometerSensor)
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
        assertNull(calibrator.estimatedMagnetometerChiSqDegreesOfFreedom)
        assertNull(calibrator.estimatedMagnetometerReducedChiSq)
        assertNull(calibrator.estimatedMagnetometerP)
        assertNull(calibrator.estimatedMagnetometerQ)
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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun staticIntervalDetectedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalDetectedListener)

        // set new value
        calibrator.staticIntervalDetectedListener = staticIntervalDetectedListener

        // check
        assertSame(staticIntervalDetectedListener, calibrator.staticIntervalDetectedListener)
    }

    @Test
    fun dynamicIntervalDetectedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalDetectedListener)

        // set new value
        calibrator.dynamicIntervalDetectedListener = dynamicIntervalDetectedListener

        // check
        assertSame(dynamicIntervalDetectedListener, calibrator.dynamicIntervalDetectedListener)
    }

    @Test
    fun staticIntervalSkippedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.staticIntervalSkippedListener)

        // set new value
        calibrator.staticIntervalSkippedListener = staticIntervalSkippedListener

        // check
        assertSame(staticIntervalSkippedListener, calibrator.staticIntervalSkippedListener)
    }

    @Test
    fun dynamicIntervalSkippedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.dynamicIntervalSkippedListener)

        // set new value
        calibrator.dynamicIntervalSkippedListener = dynamicIntervalSkippedListener

        // check
        assertSame(dynamicIntervalSkippedListener, calibrator.dynamicIntervalSkippedListener)
    }

    @Test
    fun generatedMagnetometerMeasurementListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.generatedMagnetometerMeasurementListener)

        // set new value
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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun isMagnetometerGroundTruthInitialHardIron_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)

        // set new value
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
    }

    @Test
    fun isMagnetometerGroundTruthInitialHardIron_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.isMagnetometerGroundTruthInitialHardIron = true
        }
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.location)

        // set new value
        val location = getLocation()
        calibrator.location = location

        // check
        assertSame(location, calibrator.location)
    }

    @Test
    fun location_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator, "running", true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.location = getLocation()
        }
    }

    @Test
    fun timestamp_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNotNull(calibrator.timestamp)

        // set new value
        val timestamp = Date()
        calibrator.timestamp = timestamp

        // check
        assertSame(timestamp, calibrator.timestamp)
    }

    @Test
    fun timestamp_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator, "running", true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.timestamp = Date()
        }
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.worldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
    }

    @Test
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator, "running", true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.worldMagneticModel = WorldMagneticModel()
        }
    }

    @Test
    fun isInitialMagneticFluxDensityNormMeasured_returnsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default values
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertNull(calibrator.location)
        assertNotNull(calibrator.timestamp)

        // set location
        val location = getLocation()
        calibrator.location = location

        // check
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertSame(location, calibrator.location)
        assertNotNull(calibrator.timestamp)

        // unset timestamp
        calibrator.timestamp = null

        // check
        assertTrue(calibrator.isInitialMagneticFluxDensityNormMeasured)
        assertSame(location, calibrator.location)
        assertNull(calibrator.timestamp)
    }

    @Test
    fun magnetometerInitialMm_whenValid_setsExpectedValues() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(
            Matrix(MM_SIZE, MM_SIZE),
            calibrator.magnetometerInitialMm
        )
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

    @Test
    fun magnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val mm = Matrix(1, MM_SIZE)
        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerInitialMm = mm
        }
    }

    @Test
    fun magnetometerInitialMm_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val mm = Matrix(MM_SIZE, 1)
        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerInitialMm = mm
        }
    }

    @Test
    fun magnetometerInitialMm_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMm = Matrix(MM_SIZE, MM_SIZE)
        }
    }

    @Test
    fun getMagnetometerInitialMm_returnsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun getMagnetometerInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val mm = Matrix(1, MM_SIZE)
        assertThrows(IllegalArgumentException::class.java) {
            calibrator.getMagnetometerInitialMm(mm)
        }
    }

    @Test
    fun getMagnetometerInitialMm_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val mm = Matrix(MM_SIZE, 1)
        assertThrows(IllegalArgumentException::class.java) {
            calibrator.getMagnetometerInitialMm(mm)
        }
    }

    @Test
    fun magnetometerInitialSx_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.magnetometerInitialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.magnetometerInitialSx, 0.0)
    }

    @Test
    fun magnetometerInitialSx_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialSx = 0.0
        }
    }

    @Test
    fun magnetometerInitialSy_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.magnetometerInitialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.magnetometerInitialSy, 0.0)
    }

    @Test
    fun magnetometerInitialSy_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialSy = 0.0
        }
    }

    @Test
    fun magnetometerInitialSz_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.magnetometerInitialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.magnetometerInitialSz, 0.0)
    }

    @Test
    fun magnetometerInitialSz_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialSz = 0.0
        }
    }

    @Test
    fun magnetometerInitialMxy_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.magnetometerInitialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.magnetometerInitialMxy, 0.0)
    }

    @Test
    fun magnetometerInitialMxy_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMxy = 0.0
        }
    }

    @Test
    fun magnetometerInitialMxz_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.magnetometerInitialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.magnetometerInitialMxz, 0.0)
    }

    @Test
    fun magnetometerInitialMxz_whenRunning_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMxz = 0.0
        }
    }

    @Test
    fun magnetometerInitialMyx_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.magnetometerInitialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.magnetometerInitialMyx, 0.0)
    }

    @Test
    fun magnetometerInitialMyx_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMyx = 0.0
        }
    }

    @Test
    fun magnetometerInitialMyz_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.magnetometerInitialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.magnetometerInitialMyz, 0.0)
    }

    @Test
    fun magnetometerInitialMyz_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMyz = 0.0
        }
    }

    @Test
    fun magnetometerInitialMzx_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.magnetometerInitialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.magnetometerInitialMzx, 0.0)
    }

    @Test
    fun magnetometerInitialMzx_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMzx = 0.0
        }
    }

    @Test
    fun magnetometerInitialMzy_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertEquals(0.0, calibrator.magnetometerInitialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.magnetometerInitialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.magnetometerInitialMzy, 0.0)
    }

    @Test
    fun magnetometerInitialMzy_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerInitialMzy = 0.0
        }
    }

    @Test
    fun setMagnetometerInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun setMagnetometerInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            calibrator.setMagnetometerInitialScalingFactors(initialSx, initialSy, initialSz)
        }
    }

    @Test
    fun setMagnetometerInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun setMagnetometerInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        assertThrows(IllegalStateException::class.java) {
            calibrator.setMagnetometerInitialCrossCouplingErrors(
                initialMxy,
                initialMxz,
                initialMyx,
                initialMyz,
                initialMzx,
                initialMzy
            )
        }
    }

    @Test
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        assertThrows(IllegalStateException::class.java) {
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
    }

    @Test
    fun isMagnetometerCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertFalse(calibrator.isMagnetometerCommonAxisUsed)

        // set new value
        calibrator.isMagnetometerCommonAxisUsed = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
    }

    @Test
    fun isMagnetometerCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.isMagnetometerCommonAxisUsed = true
        }
    }

    @Test
    fun magnetometerRobustMethod_whenNotRunning_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        // check default value
        assertNull(calibrator.magnetometerRobustMethod)

        // set new value
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.magnetometerRobustMethod)
    }

    @Test
    fun magnetometerRobustMethod_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustMethod = RobustEstimatorMethod.RANSAC
        }
    }

    @Test
    fun magnetometerRobustConfidence_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustConfidence = -1.0
        }
    }

    @Test
    fun magnetometerRobustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustConfidence = 2.0
        }
    }

    @Test
    fun magnetometerRobustConfidence_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        }
    }

    @Test
    fun magnetometerRobustMaxIterations_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustMaxIterations = 0
        }
    }

    @Test
    fun magnetometerRobustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        }
    }

    @Test
    fun magnetometerRobustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustPreliminarySubsetSize = 12
        }
    }

    @Test
    fun magnetometerRobustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        }
    }

    @Test
    fun magnetometerRobustThreshold_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustThreshold = 0.0
        }
    }

    @Test
    fun magnetometerRobustThreshold_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD
        }
    }

    @Test
    fun magnetometerRobustThresholdFactor_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustThresholdFactor = 0.0
        }
    }

    @Test
    fun magnetometerRobustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        }
    }

    @Test
    fun magnetometerRobustStopThresholdFactor_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun magnetometerRobustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.magnetometerRobustStopThresholdFactor = 0.0
        }
    }

    @Test
    fun magnetometerRobustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.magnetometerRobustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
        }
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.windowSize = 0
        }
    }

    @Test
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.windowSize = WINDOW_SIZE
        }
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun initialStaticSamples_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.initialStaticSamples = 0
        }
    }

    @Test
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
        }
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun thresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.thresholdFactor = 0.0
        }
    }

    @Test
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.thresholdFactor = THRESHOLD_FACTOR
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun instantaneousNoiseLevelFactor_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.instantaneousNoiseLevelFactor = 0.0
        }
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenValid_setsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMeasurement_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val value = Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        assertThrows(IllegalArgumentException::class.java) {
            calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
        }
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdMeasurement_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        assertThrows(IllegalStateException::class.java) {
            calibrator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
        }
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement_getsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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

    @Test
    fun requiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalArgumentException::class.java) {
            calibrator.requiredMeasurements = 0
        }
    }

    @Test
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
        }
    }

    @Test
    fun onInitializationStarted_whenNoListenerAvailable_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        generatorInitializationStartedListener.onInitializationStarted(generator)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            initializationStartedListener = initializationStartedListener
        )

        val generatorInitializationStartedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationStartedListener")
        requireNotNull(generatorInitializationStartedListener)

        generatorInitializationStartedListener.onInitializationStarted(generator)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            initializationCompletedListener = initializationCompletedListener
        )

        val generatorInitializationCompletedListener: SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorInitializationCompletedListener")
        requireNotNull(generatorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        generatorInitializationCompletedListener.onInitializationCompleted(
            generator,
            baseNoiseLevel
        )

        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsGenerator() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
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
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val calibrator = StaticIntervalMagnetometerCalibrator(
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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)
    }

    @Test
    fun onStaticIntervalDetected_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            staticIntervalDetectedListener = staticIntervalDetectedListener
        )

        val generatorStaticIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalDetectedListener")
        requireNotNull(generatorStaticIntervalDetectedListener)

        generatorStaticIntervalDetectedListener.onStaticIntervalDetected(generator)

        verify(exactly = 1) { staticIntervalDetectedListener.onStaticIntervalDetected(calibrator) }
    }

    @Test
    fun onDynamicIntervalDetected_whenNoListenerAvailable_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            dynamicIntervalDetectedListener = dynamicIntervalDetectedListener
        )

        val generatorDynamicIntervalDetectedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalDetectedListener")
        requireNotNull(generatorDynamicIntervalDetectedListener)

        generatorDynamicIntervalDetectedListener.onDynamicIntervalDetected(generator)

        verify(exactly = 1) { dynamicIntervalDetectedListener.onDynamicIntervalDetected(calibrator) }
    }

    @Test
    fun onStaticIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)
    }

    @Test
    fun onStaticIntervalSkipped_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            staticIntervalSkippedListener = staticIntervalSkippedListener
        )

        val generatorStaticIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorStaticIntervalSkippedListener")
        requireNotNull(generatorStaticIntervalSkippedListener)

        generatorStaticIntervalSkippedListener.onStaticIntervalSkipped(generator)

        verify(exactly = 1) { staticIntervalSkippedListener.onStaticIntervalSkipped(calibrator) }
    }

    @Test
    fun onDynamicIntervalSkipped_whenNoListenerAvailable_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)
    }

    @Test
    fun onDynamicIntervalSkipped_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            dynamicIntervalSkippedListener = dynamicIntervalSkippedListener
        )

        val generatorDynamicIntervalSkippedListener: SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator>? =
            calibrator.getPrivateProperty("generatorDynamicIntervalSkippedListener")
        requireNotNull(generatorDynamicIntervalSkippedListener)

        generatorDynamicIntervalSkippedListener.onDynamicIntervalSkipped(generator)

        verify(exactly = 1) { dynamicIntervalSkippedListener.onDynamicIntervalSkipped(calibrator) }
    }

    @Test
    fun onGeneratedMeasurement_addsMeasurement() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val norm1 = randomizer.nextDouble()
        every { generator.initialMagneticFluxDensityNorm }.returns(norm1)
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])

        val norm2 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm2)
        assertEquals(norm1, norm2, 0.0)
    }

    @Test
    fun onGeneratedMeasurement_whenInitialMagneticFluxDensityNormAndNoLocation_keepsInitialMagneticFluxDensityNorm() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val norm1 = randomizer.nextDouble()
        every { generator.initialMagneticFluxDensityNorm }.returns(norm1)
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])

        val norm2 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm2)
        assertEquals(norm1, norm2, 0.0)

        // call again with a different value
        measurement.magneticFluxDensity.bx = 0.0
        measurement.magneticFluxDensity.by = 0.0
        measurement.magneticFluxDensity.bz = 0.0

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        // check that norm is preserved
        val norm3 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm3)
        assertEquals(norm1, norm3, 0.0)
    }

    @Test
    fun onGeneratedMeasurement_whenInitialMagneticFluxDensityNormAndLocation_doesNotSetInitialMagneticFluxDensityNorm() {
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialMagneticFluxDensityNorm)
        assertSame(location, calibrator.location)
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertNull(calibrator.initialMagneticFluxDensityNorm)
    }

    @Test
    fun onGeneratedMeasurement_whenListenerAvailable_notifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            generatedMagnetometerMeasurementListener = generatedMagnetometerMeasurementListener
        )

        assertTrue(calibrator.magnetometerMeasurements.isEmpty())

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        val norm1 = randomizer.nextDouble()
        every { generator.initialMagneticFluxDensityNorm }.returns(norm1)
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generator, measurement)

        assertEquals(1, calibrator.magnetometerMeasurements.size)
        assertSame(measurement, calibrator.magnetometerMeasurements[0])
        val norm2 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm2)
        assertEquals(norm1, norm2, 0.0)

        verify(exactly = 1) {
            generatedMagnetometerMeasurementListener.onGeneratedMagnetometerMeasurement(
                calibrator,
                measurement,
                1,
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            )
        }
        verify(exactly = 1) { generator.initialMagneticFluxDensityNorm }
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToCalibrate_stopsAndBuildsCalibrator() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            solveCalibrationWhenEnoughMeasurements = false
        )

        assertEquals(
            StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertFalse(calibrator.isReadyToSolveCalibration)

        // add enough measurements
        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        (1..calibrator.requiredMeasurements).forEach { _ ->
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val norm1 = randomizer.nextDouble()
        every { generatorSpy.initialMagneticFluxDensityNorm }.returns(norm1)
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

        val norm2 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm2)
        assertEquals(norm1, norm2, 0.0)
    }

    @Test
    fun onGeneratedMeasurement_whenReadyToSolveCalibrationListenerAvailable_notifies() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
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
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextDouble()
        val by = randomizer.nextDouble()
        val bz = randomizer.nextDouble()
        measurement.magneticFluxDensity = BodyMagneticFluxDensity(bx, by, bz)
        (1..calibrator.requiredMeasurements).forEach { _ ->
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val location = getLocation()
        val nedPosition = location.toNEDPosition()
        val timestamp = Date()
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()
        val earthB = wmmEstimator.estimate(nedPosition, timestamp)
        val norm1 = BodyMagneticFluxDensityEstimator.estimate(earthB, 0.0, 0.0, 0.0).norm
        every { generatorSpy.initialMagneticFluxDensityNorm }.returns(norm1)
        calibrator.setPrivateProperty("generator", generatorSpy)

        var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNull(magnetometerInternalCalibrator)

        val generatorGeneratedMeasurementListener: SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? =
            calibrator.getPrivateProperty("generatorGeneratedMeasurementListener")
        requireNotNull(generatorGeneratedMeasurementListener)

        generatorGeneratedMeasurementListener.onGeneratedMeasurement(generatorSpy, measurement)

        val norm2 = calibrator.initialMagneticFluxDensityNorm
        requireNotNull(norm2)
        assertEquals(norm1, norm2, 0.0)

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { generatorSpy.stop() }

        magnetometerInternalCalibrator =
            calibrator.getPrivateProperty("magnetometerInternalCalibrator")
        assertNotNull(magnetometerInternalCalibrator)
    }

    @Test
    fun onGeneratedMeasurement_whenSolveCalibrationEnabled_solvesCalibration() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
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

        (1..reqMeasurements).forEach { _ ->
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
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic,
                    MAGNETOMETER_NOISE_STD
                )
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val earthB = wmmEstimator.estimate(nedPosition, timestamp)
        val norm1 = BodyMagneticFluxDensityEstimator.estimate(earthB, 0.0, 0.0, 0.0).norm
        every { generatorSpy.initialMagneticFluxDensityNorm }.returns(norm1)
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
        assertNotNull(calibrator.estimatedMagnetometerChiSqDegreesOfFreedom)
        assertNotNull(calibrator.estimatedMagnetometerReducedChiSq)
        assertNotNull(calibrator.estimatedMagnetometerP)
        assertNotNull(calibrator.estimatedMagnetometerQ)
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
        every { context.getSystemService(Context.SENSOR_SERVICE) }
            .returns(sensorManager)
        every {
            sensorManager.getDefaultSensor(
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
            )
        }.returns(accelerometerSensor)
        every {
            sensorManager.getDefaultSensor(
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
            )
        }.returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(
            any<SensorEventListener>(), any<Sensor>())
        }

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()

        val location = getLocation()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
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

        (1..reqMeasurements).forEach { _ ->
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
                StandardDeviationBodyMagneticFluxDensity(measuredMagnetic,
                    MAGNETOMETER_NOISE_STD
                )
            calibrator.magnetometerMeasurements.add(measurement)
        }
        assertTrue(calibrator.isReadyToSolveCalibration)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        val earthB = wmmEstimator.estimate(nedPosition, timestamp)
        val norm1 = BodyMagneticFluxDensityEstimator.estimate(earthB, 0.0, 0.0, 0.0).norm
        every { generatorSpy.initialMagneticFluxDensityNorm }.returns(norm1)
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
    fun magnetometerBaseNoiseLevel_getsGeneratorBaseNoiseLevel() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(7, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        calibrator.isMagnetometerCommonAxisUsed = true
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertTrue(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = true

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun minimumRequiredMagnetometerMeasurements_whenNotCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        calibrator.isMagnetometerCommonAxisUsed = false
        calibrator.isMagnetometerGroundTruthInitialHardIron = false

        // check
        assertFalse(calibrator.isMagnetometerCommonAxisUsed)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(13, calibrator.minimumRequiredMagnetometerMeasurements)
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsGenerator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertFalse(calibrator.running)

        calibrator.setPrivateProperty("magnetometerInitialHardIronX", 0.0)
        calibrator.setPrivateProperty("magnetometerInitialHardIronY", 0.0)
        calibrator.setPrivateProperty("magnetometerInitialHardIronZ", 0.0)
        calibrator.setPrivateProperty("initialMagneticFluxDensityNorm", 0.0)

        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

        val generator: MagnetometerMeasurementGenerator? =
            calibrator.getPrivateProperty("generator")
        requireNotNull(generator)
        val generatorSpy = spyk(generator)
        justRun { generatorSpy.start() }
        calibrator.setPrivateProperty("generator", generatorSpy)

        calibrator.start()

        // check
        assertNull(calibrator.magnetometerInitialHardIronX)
        assertNull(calibrator.magnetometerInitialHardIronY)
        assertNull(calibrator.magnetometerInitialHardIronZ)
        assertNull(calibrator.initialMagneticFluxDensityNorm)
        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { generatorSpy.start() }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            every { context.getSystemService(Context.SENSOR_SERVICE) }
                .returns(sensorManager)
            every {
                sensorManager.getDefaultSensor(
                    AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value
                )
            }.returns(accelerometerSensor)
            every {
                sensorManager.getDefaultSensor(
                    MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value
                )
            }.returns(magnetometerSensor)
            every {
                sensorManager.registerListener(
                    any<SensorEventListener>(), any<Sensor>(), any()
                )
            }.returns(true)

            val calibrator = StaticIntervalMagnetometerCalibrator(context)

            assertFalse(calibrator.running)

            calibrator.start()

            assertTrue(calibrator.running)

            assertThrows(IllegalStateException::class.java) {
                calibrator.start()
            }
        }
    }

    @Test
    fun stop_whenNoListenerAvailable_stopsGenerator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val generator: MagnetometerMeasurementGenerator? =
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            stoppedListener = stoppedListener
        )

        val generator: MagnetometerMeasurementGenerator? =
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

    @Test
    fun calibrate_whenNotReadyToSolveCalibration_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertThrows(IllegalStateException::class.java) {
            calibrator.calibrate()
        }
    }

    @Test
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        setPrivateProperty(
            BaseStaticIntervalWithMeasurementGeneratorCalibrator::class,
            calibrator,
            "running",
            true
        )

        assertThrows(IllegalStateException::class.java) {
            calibrator.calibrate()
        }
    }

    @Test
    fun calibrate_whenReadyNotRunningAndNoInternalCalibrator_makesNoAction() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
            calibrator.magnetometerMeasurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

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
        mockkStatic(Log::class) {
            every { Log.e(any(), any(), any()) }.returns(1)

            val calibrator = StaticIntervalMagnetometerCalibrator(context)

            val measurement = StandardDeviationBodyMagneticFluxDensity()
            (1..13).forEach { _ ->
                calibrator.magnetometerMeasurements.add(measurement)
            }

            assertTrue(calibrator.isReadyToSolveCalibration)
            assertFalse(calibrator.running)

            every { internalCalibrator.calibrate() }.throws(CalibrationException())
            calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibrator)

            assertFalse(calibrator.calibrate())

            assertFalse(calibrator.running)
        }
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        mockkStatic(Log::class) {
            every { Log.e(any(), any(), any()) }.returns(1)

            val calibrator = StaticIntervalMagnetometerCalibrator(
                context,
                errorListener = errorListener
            )

            val measurement = StandardDeviationBodyMagneticFluxDensity()
            (1..13).forEach { _ ->
                calibrator.magnetometerMeasurements.add(measurement)
            }

            assertTrue(calibrator.isReadyToSolveCalibration)
            assertFalse(calibrator.running)

            every { internalCalibrator.calibrate() }.throws(CalibrationException())
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
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenNoInternalCalibrator_returnsNull() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronX_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedMagnetometerHardIronX)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenNoInternalCalibrator_returnsNull() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronY_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedMagnetometerHardIronY)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenNoInternalCalibrator_returnsNull() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronZ_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("magnetometerInternalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedMagnetometerHardIronZ)
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_whenNoInternalCalibrator_returnsNull() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronXAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronXAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronXAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronXAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronYAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronYAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronYAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronYAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronZAsMeasurement)
    }

    @Test
    fun estimatedMagnetometerHardIronZAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedMagnetometerHardIronZAsMeasurement(b))
    }

    @Test
    fun getEstimatedMagnetometerHardIronZAsMeasurement_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        assertNull(calibrator.estimatedMagnetometerHardIronAsTriad)
    }

    @Test
    fun estimatedMagnetometerHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.getPrivateProperty("magnetometerInternalCalibrator"))
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getEstimatedMagnetometerHardIronAsTriad(triad))
    }

    @Test
    fun getEstimatedMagnetometerHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

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
    fun buildMagnetometerInternalCalibrator_whenNoInitialMagneticFluxDensityNorm_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(context)

        assertNull(calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is java.lang.IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLocation_buildsExpectedCalibrator() {
        val location = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            worldMagneticModel = worldMagneticModel,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        assertNull(calibrator.initialMagneticFluxDensityNorm)
        assertSame(location, calibrator.location)
        assertNotNull(calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertFalse(calibrator.isInitialMagneticFluxDensityNormMeasured)

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
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

        assertNull(calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertNull(internalCalibrator2.groundTruthMagneticFluxDensityNorm)
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition,
            ABSOLUTE_ERROR
        ))
        val calendar = GregorianCalendar()
        val timestamp = calibrator.timestamp
        requireNotNull(timestamp)
        calendar.time = timestamp
        val year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar)
        assertEquals(year, internalCalibrator2.year, 0.0)
        assertSame(worldMagneticModel, internalCalibrator2.magneticModel)
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
    fun buildMagnetometerInternalCalibrator_whenNonRobustGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
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

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
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

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
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

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        (1..13).forEach { _ ->
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

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenRANSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenMSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()

        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
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

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
            ROBUST_THRESHOLD_FACTOR * StaticIntervalMagnetometerCalibratorTest.Companion.ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
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

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
            ROBUST_THRESHOLD_FACTOR * StaticIntervalMagnetometerCalibratorTest.Companion.ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenLMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.LMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = ROBUST_THRESHOLD

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
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

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
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

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertEquals(
            ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.magnetometerRobustStopThresholdFactor,
            0.0
        )
        assertEquals(baseNoiseLevel, calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            initialMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            0.0
        )
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
        assertEquals(
            calibrator.magnetometerMeasurements.size,
            internalCalibrator2.qualityScores.size
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = false
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertFalse(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    @Test
    fun buildMagnetometerInternalCalibrator_whenPROMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            isMagnetometerGroundTruthInitialHardIron = true
        )

        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        (1..13).forEach { _ ->
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
        calibrator.magnetometerRobustMethod = RobustEstimatorMethod.PROMEDS
        calibrator.magnetometerRobustConfidence = ROBUST_CONFIDENCE
        calibrator.magnetometerRobustMaxIterations = ROBUST_MAX_ITERATIONS
        calibrator.magnetometerRobustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        calibrator.magnetometerRobustThreshold = null
        calibrator.magnetometerRobustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.magnetometerRobustMethod)
        assertTrue(calibrator.isMagnetometerGroundTruthInitialHardIron)
        assertEquals(ROBUST_CONFIDENCE, calibrator.magnetometerRobustConfidence, 0.0)
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.magnetometerRobustMaxIterations)
        assertEquals(
            ROBUST_PRELIMINARY_SUBSET_SIZE,
            calibrator.magnetometerRobustPreliminarySubsetSize
        )
        val robustThreshold = calibrator.magnetometerRobustThreshold
        assertNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.magnetometerRobustThresholdFactor, 0.0)
        assertNull(calibrator.magnetometerBaseNoiseLevel)

        val initialMagneticFluxDensityNorm = randomizer.nextDouble()
        calibrator.setPrivateProperty(
            "initialMagneticFluxDensityNorm",
            initialMagneticFluxDensityNorm
        )

        assertEquals(initialMagneticFluxDensityNorm, calibrator.initialMagneticFluxDensityNorm)

        val ex = assertThrows(InvocationTargetException::class.java) {
            assertNull(calibrator.callPrivateFuncWithResult("buildMagnetometerInternalCalibrator"))
        }
        assertTrue(ex.cause is IllegalStateException)
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
        val longitudeDegrees =
            randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        return location
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
            } catch (_: WrongSizeException) {
                // never happens
                null
            }
        }
    }
}