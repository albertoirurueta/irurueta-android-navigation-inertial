package com.irurueta.android.navigation.inertial.calibration

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.WrongSizeException
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.MagnetometerIntervalDetector
import com.irurueta.android.navigation.inertial.callPrivateFuncWithResult
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
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
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronPositionAndInstantMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.GaussianRandomizer
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*
import kotlin.math.pow
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class StaticIntervalMagnetometerCalibratorTest {

    @Test
    fun constructor_whenContextLocationAndTimestamp_returnsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertNull(calibrator.worldMagneticModel)
        assertEquals(MagnetometerSensorCollector.SensorType.MAGNETOMETER, calibrator.sensorType)
        assertEquals(SensorDelay.FASTEST, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenWorldMagneticModel_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(context, location, timestamp, worldMagneticModel)

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(MagnetometerSensorCollector.SensorType.MAGNETOMETER, calibrator.sensorType)
        assertEquals(SensorDelay.FASTEST, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenSensorType_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.FASTEST, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenSensorDelay_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertTrue(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenSolveCalibrationWhenEnoughMeasurements_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                false
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenGroundTruthInitialHardIron_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertNull(calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenInitializationStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertNull(calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenInitializationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertNull(calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenErrorListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
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
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertNull(calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenInitialHardIronAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenNewCalibrationMeasurementAvailableListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertNull(calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenReadyToSolveCalibrationListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
                readyToSolveCalibrationListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertNull(calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenCalibrationSolvingStartedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertNull(calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenCalibrationCompletedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
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
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertNull(calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenStoppedListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
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
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertNull(calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenMagnetometerMeasurementListener_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener,
                magnetometerMeasurementListener
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(magnetometerMeasurementListener, calibrator.magnetometerMeasurementListener)
        assertNotNull(calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun constructor_whenQualityScoreMapper_returnsExpectedValues() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>()
        val stoppedListener = mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val qualityScoreMapper =
            mockk<QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val calibrator =
            StaticIntervalMagnetometerCalibrator(
                context,
                location,
                timestamp,
                worldMagneticModel,
                MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorDelay.NORMAL,
                solveCalibrationWhenEnoughMeasurements = false,
                isGroundTruthInitialHardIron = true,
                initializationStartedListener,
                initializationCompletedListener,
                errorListener,
                initialHardIronAvailableListener,
                newCalibrationMeasurementAvailableListener,
                readyToSolveCalibrationListener,
                calibrationSolvingStartedListener,
                calibrationCompletedListener,
                stoppedListener,
                magnetometerMeasurementListener,
                qualityScoreMapper
            )

        // check default values
        assertSame(context, calibrator.context)
        assertSame(location, calibrator.location)
        assertSame(timestamp, calibrator.timestamp)
        assertSame(worldMagneticModel, calibrator.worldMagneticModel)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            calibrator.sensorType
        )
        assertEquals(SensorDelay.NORMAL, calibrator.sensorDelay)
        assertFalse(calibrator.solveCalibrationWhenEnoughMeasurements)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
        assertSame(errorListener, calibrator.errorListener)
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
        assertSame(
            newCalibrationMeasurementAvailableListener,
            calibrator.newCalibrationMeasurementAvailableListener
        )
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
        assertSame(stoppedListener, calibrator.stoppedListener)
        assertSame(magnetometerMeasurementListener, calibrator.magnetometerMeasurementListener)
        assertSame(qualityScoreMapper, calibrator.qualityScoreMapper)
        assertTrue(calibrator.measurements.isEmpty())
        assertFalse(calibrator.isReadyToSolveCalibration)
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.initialHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))
        assertFalse(calibrator.running)
        assertNull(calibrator.magnetometerSensor)
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
        val baseNoiseLevel1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            calibrator.baseNoiseLevelAbsoluteThreshold,
            baseNoiseLevel1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, baseNoiseLevel1.unit)
        val baseNoiseLevel2 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(baseNoiseLevel2)
        assertEquals(baseNoiseLevel1, baseNoiseLevel2)
        assertNull(calibrator.baseNoiseLevel)
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))
        assertNull(calibrator.baseNoiseLevelPsd)
        assertNull(calibrator.baseNoiseLevelRootPsd)
        assertNull(calibrator.threshold)
        assertNull(calibrator.thresholdAsMagneticFluxDensity)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))
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
        val mm1 = Matrix(MM_SIZE, MM_SIZE)
        assertEquals(mm1, calibrator.initialMm)
        val mm2 = Matrix.identity(MM_SIZE, MM_SIZE)
        calibrator.getInitialMm(mm2)
        assertEquals(mm1, mm2)
        assertFalse(calibrator.isCommonAxisUsed)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.minimumRequiredMeasurements
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.requiredMeasurements
        )
        assertNull(calibrator.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )
        assertNull(calibrator.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )
        assertNull(calibrator.estimatedMm)
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
        assertNull(calibrator.estimatedHardIronX)
        assertNull(calibrator.estimatedHardIronY)
        assertNull(calibrator.estimatedHardIronZ)
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNull(calibrator.estimatedHardIronAsTriad)
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun initializationStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.initializationStartedListener)

        // set new value
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>()
        calibrator.initializationStartedListener = initializationStartedListener

        // check
        assertSame(initializationStartedListener, calibrator.initializationStartedListener)
    }

    @Test
    fun initializationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.initializationCompletedListener)

        // set new value
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>()
        calibrator.initializationCompletedListener = initializationCompletedListener

        // check
        assertSame(initializationCompletedListener, calibrator.initializationCompletedListener)
    }

    @Test
    fun errorListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.errorListener)

        // set new value
        val errorListener = mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>()
        calibrator.errorListener = errorListener

        // check
        assertSame(errorListener, calibrator.errorListener)
    }

    @Test
    fun initialHardIronAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.initialHardIronAvailableListener)

        // set new value
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>()
        calibrator.initialHardIronAvailableListener = initialHardIronAvailableListener

        // check
        assertSame(initialHardIronAvailableListener, calibrator.initialHardIronAvailableListener)
    }

    @Test
    fun newCalibrationMeasurementAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.newCalibrationMeasurementAvailableListener)

        // set new value
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>()
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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.readyToSolveCalibrationListener)

        // set new value
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>()
        calibrator.readyToSolveCalibrationListener = readyToSolveCalibrationListener

        // check
        assertSame(readyToSolveCalibrationListener, calibrator.readyToSolveCalibrationListener)
    }

    @Test
    fun calibrationSolvingStartedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.calibrationSolvingStartedListener)

        // set new value
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>()
        calibrator.calibrationSolvingStartedListener = calibrationSolvingStartedListener

        // check
        assertSame(calibrationSolvingStartedListener, calibrator.calibrationSolvingStartedListener)
    }

    @Test
    fun calibrationCompletedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.calibrationCompletedListener)

        // set new value
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>()
        calibrator.calibrationCompletedListener = calibrationCompletedListener

        // check
        assertSame(calibrationCompletedListener, calibrator.calibrationCompletedListener)
    }

    @Test
    fun stoppedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.stoppedListener)

        // set new value
        val stoppedListener = mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>()
        calibrator.stoppedListener = stoppedListener

        // check
        assertSame(stoppedListener, calibrator.stoppedListener)
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.magnetometerMeasurementListener)

        // set new value
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        calibrator.magnetometerMeasurementListener = magnetometerMeasurementListener

        // check
        assertSame(magnetometerMeasurementListener, calibrator.magnetometerMeasurementListener)
    }

    @Test
    fun qualityScoreMapper_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNotNull(calibrator.qualityScoreMapper)

        // set new value
        val qualityScoreMapper =
            mockk<QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>>()
        calibrator.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, calibrator.qualityScoreMapper)
    }

    @Test
    fun isGroundTruthInitialHardIron_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        // set new value
        calibrator.isGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isGroundTruthInitialHardIron)
    }

    @Test(expected = IllegalStateException::class)
    fun isGroundTruthInitialHardIron_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.isGroundTruthInitialHardIron = true
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location1 = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location1, timestamp)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location1 = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location1, timestamp)

        calibrator.setPrivateProperty("running", true)

        // check
        assertSame(location1, calibrator.location)
        assertTrue(calibrator.running)

        // set new value
        val location2 = mockk<Location>()
        calibrator.location = location2
    }

    @Test
    fun timestamp_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp1 = Date()
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp1 = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp1)

        calibrator.setPrivateProperty("running", true)

        // check
        assertEquals(timestamp1, calibrator.timestamp)
        assertTrue(calibrator.running)

        // set new value
        val timestamp2 = Date()
        calibrator.timestamp = timestamp2
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        // check default value
        assertNull(calibrator.worldMagneticModel)
        assertTrue(calibrator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        calibrator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun windowSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, calibrator.windowSize)

        // set new value
        calibrator.windowSize = WINDOW_SIZE

        // check
        assertEquals(WINDOW_SIZE, calibrator.windowSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun windowSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.windowSize = 0
    }

    @Test(expected = IllegalStateException::class)
    fun windowSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.windowSize = WINDOW_SIZE
    }

    @Test
    fun initialStaticSamples_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.initialStaticSamples = 0
    }

    @Test(expected = IllegalStateException::class)
    fun initialStaticSamples_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialStaticSamples = INITIAL_STATIC_SAMPLES
    }

    @Test
    fun thresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.thresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun thresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.thresholdFactor = THRESHOLD_FACTOR
    }

    @Test
    fun instantaneousNoiseLevelFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.instantaneousNoiseLevelFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun instantaneousNoiseLevelFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.instantaneousNoiseLevelFactor = INSTANTANEOUS_NOISE_LEVEL_FACTOR
    }

    @Test
    fun baseNoiseLevelAbsoluteThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.baseNoiseLevelAbsoluteThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.baseNoiseLevelAbsoluteThreshold = BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD
    }

    @Test
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        val value1 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, value1.unit)

        // set new value
        val value2 =
            MagneticFluxDensity(BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity = value2

        // check
        val value3 = calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
        assertEquals(value2, value3)
    }

    @Test(expected = IllegalArgumentException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val value = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity = value
    }

    @Test(expected = IllegalStateException::class)
    fun baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        val value =
            MagneticFluxDensity(BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity = value
    }

    @Test
    fun getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        val value1 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(value1)

        assertEquals(
            TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
            value1.value.toDouble(),
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, value1.unit)

        // set new value
        val value2 =
            MagneticFluxDensity(BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, MagneticFluxDensityUnit.TESLA)
        calibrator.baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity = value2

        // check
        val value3 = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        calibrator.getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(value3)
        assertEquals(value2, value3)
    }

    @Test
    fun initialSx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialSx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        calibrator.initialSx = initialSx

        // check
        assertEquals(initialSx, calibrator.initialSx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialSx = 0.0
    }

    @Test
    fun initialSy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialSy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSy = randomizer.nextDouble()
        calibrator.initialSy = initialSy

        // check
        assertEquals(initialSy, calibrator.initialSy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialSy = 0.0
    }

    @Test
    fun initialSz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSz = randomizer.nextDouble()
        calibrator.initialSz = initialSz

        // check
        assertEquals(initialSz, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialSz = 0.0
    }

    @Test
    fun initialMxy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMxy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        calibrator.initialMxy = initialMxy

        // check
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMxy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMxy = 0.0
    }

    @Test
    fun initialMxz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMxz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxz = randomizer.nextDouble()
        calibrator.initialMxz = initialMxz

        // check
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMxz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMxz = 0.0
    }

    @Test
    fun initialMyx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMyx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyx = randomizer.nextDouble()
        calibrator.initialMyx = initialMyx

        // check
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMyx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMyx = 0.0
    }

    @Test
    fun initialMyz_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMyz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMyz = randomizer.nextDouble()
        calibrator.initialMyz = initialMyz

        // check
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMyz_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMyz = 0.0
    }

    @Test
    fun initialMzx_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMzx, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzx = randomizer.nextDouble()
        calibrator.initialMzx = initialMzx

        // check
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMzx_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMzx = 0.0
    }

    @Test
    fun initialMzy_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMzy = randomizer.nextDouble()
        calibrator.initialMzy = initialMzy

        // check
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun initialMzy_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMzy = 0.0
    }

    @Test
    fun setInitialScalingFactors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default values
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz)

        // check
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialScalingFactors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz)
    }

    @Test
    fun setInitialCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default values
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

        // set new value
        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )

        // check
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)

        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        val randomizer = UniformRandomizer()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    @Test
    fun setInitialScalingFactorsAndCrossCouplingErrors_whenNotRunning_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default values
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun setInitialScalingFactorAndCrossCouplingErrors_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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
    fun initialMm_whenValid_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(Matrix(MM_SIZE, MM_SIZE), calibrator.initialMm)
        assertEquals(0.0, calibrator.initialSx, 0.0)
        assertEquals(0.0, calibrator.initialSy, 0.0)
        assertEquals(0.0, calibrator.initialSz, 0.0)
        assertEquals(0.0, calibrator.initialMxy, 0.0)
        assertEquals(0.0, calibrator.initialMxz, 0.0)
        assertEquals(0.0, calibrator.initialMyx, 0.0)
        assertEquals(0.0, calibrator.initialMyz, 0.0)
        assertEquals(0.0, calibrator.initialMzx, 0.0)
        assertEquals(0.0, calibrator.initialMzy, 0.0)

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

        calibrator.initialMm = mm

        // check
        assertEquals(mm, calibrator.initialMm)
        assertEquals(initialSx, calibrator.initialSx, 0.0)
        assertEquals(initialSy, calibrator.initialSy, 0.0)
        assertEquals(initialSz, calibrator.initialSz, 0.0)
        assertEquals(initialMxy, calibrator.initialMxy, 0.0)
        assertEquals(initialMxz, calibrator.initialMxz, 0.0)
        assertEquals(initialMyx, calibrator.initialMyx, 0.0)
        assertEquals(initialMyz, calibrator.initialMyz, 0.0)
        assertEquals(initialMzx, calibrator.initialMzx, 0.0)
        assertEquals(initialMzy, calibrator.initialMzy, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val mm = Matrix(1, MM_SIZE)
        calibrator.initialMm = mm
    }

    @Test(expected = IllegalArgumentException::class)
    fun initialMm_whenInvalidColumnsSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.initialMm = mm
    }

    @Test(expected = IllegalStateException::class)
    fun initialMm_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.initialMm = Matrix(MM_SIZE, MM_SIZE)
    }

    @Test
    fun getInitialMm_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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
        calibrator.getInitialMm(mm)

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
    fun getInitialMm_whenInvalidRowSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val mm = Matrix(1, MM_SIZE)
        calibrator.getInitialMm(mm)
    }

    @Test(expected = IllegalArgumentException::class)
    fun getInitialMm_whenInvalidColumnSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val mm = Matrix(MM_SIZE, 1)
        calibrator.getInitialMm(mm)
    }

    @Test
    fun isCommonAxisUsed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        assertFalse(calibrator.isCommonAxisUsed)

        // set new value
        calibrator.isCommonAxisUsed = true

        // check
        assertTrue(calibrator.isCommonAxisUsed)
    }

    @Test(expected = IllegalStateException::class)
    fun isCommonAxisUsed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.isCommonAxisUsed = true
    }

    @Test
    fun requiredMeasurements_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
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
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.requiredMeasurements = 0
    }

    @Test(expected = IllegalStateException::class)
    fun requiredMeasurements_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.requiredMeasurements = REQUIRED_MEASUREMENTS
    }

    @Test
    fun robustMethod_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.robustMethod)

        // set new value
        calibrator.robustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.robustMethod)
    }

    @Test(expected = IllegalStateException::class)
    fun robustMethod_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustMethod = RobustEstimatorMethod.RANSAC
    }

    @Test
    fun robustConfidence_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            calibrator.robustConfidence,
            0.0
        )

        // set new value
        calibrator.robustConfidence = ROBUST_CONFIDENCE

        // check
        assertEquals(ROBUST_CONFIDENCE, calibrator.robustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenInvalidLowerBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenInvalidUpperBound_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustConfidence = 2.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustConfidence_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustConfidence = ROBUST_CONFIDENCE
    }

    @Test
    fun robustMaxIterations_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            calibrator.robustMaxIterations
        )

        // set new value
        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS

        // check
        assertEquals(ROBUST_MAX_ITERATIONS, calibrator.robustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustMaxIterations = 0
    }

    @Test(expected = IllegalStateException::class)
    fun robustMaxIterations_whenRunning_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustMaxIterations = ROBUST_MAX_ITERATIONS
    }

    @Test
    fun robustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL,
            calibrator.robustPreliminarySubsetSize
        )

        // set new value
        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        // check
        assertEquals(ROBUST_PRELIMINARY_SUBSET_SIZE, calibrator.robustPreliminarySubsetSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustPreliminarySubsetSize = 12
    }

    @Test(expected = IllegalStateException::class)
    fun robustPreliminarySubsetSize_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
    }

    @Test
    fun robustThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertNull(calibrator.robustThreshold)

        // set new value
        calibrator.robustThreshold = ROBUST_THRESHOLD

        // check
        val robustThreshold = calibrator.robustThreshold
        requireNotNull(robustThreshold)
        assertEquals(ROBUST_THRESHOLD, robustThreshold, 0.0)

        // set new value
        calibrator.robustThreshold = null

        // check
        assertNull(calibrator.robustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustThreshold = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustThreshold = ROBUST_THRESHOLD
    }

    @Test
    fun robustThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            calibrator.robustThresholdFactor,
            0.0
        )

        // set new value
        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_THRESHOLD_FACTOR, calibrator.robustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
    }

    @Test
    fun robustStopThresholdFactor_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            calibrator.robustStopThresholdFactor,
            0.0
        )

        // set new value
        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR

        // check
        assertEquals(ROBUST_STOP_THRESHOLD_FACTOR, calibrator.robustStopThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.robustStopThresholdFactor = 0.0
    }

    @Test(expected = IllegalStateException::class)
    fun robustStopThresholdFactor_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)

        calibrator.robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
    }

    @Test
    fun onInitializationStarted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val intervalDetectorInitializationStartedListener: IntervalDetector.OnInitializationStartedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationStartedListener")
        requireNotNull(intervalDetectorInitializationStartedListener)

        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorInitializationStartedListener.onInitializationStarted(intervalDetector)
    }

    @Test
    fun onInitializationStarted_whenListenerAvailable_notifies() {
        val initializationStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationStartedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            initializationStartedListener = initializationStartedListener
        )

        val intervalDetectorInitializationStartedListener: IntervalDetector.OnInitializationStartedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationStartedListener")
        requireNotNull(intervalDetectorInitializationStartedListener)

        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorInitializationStartedListener.onInitializationStarted(intervalDetector)

        verify(exactly = 1) { initializationStartedListener.onInitializationStarted(calibrator) }
    }

    @Test
    fun onInitializationCompleted_whenNoListenerAvailable_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        val intervalDetectorInitializationCompletedListener: IntervalDetector.OnInitializationCompletedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationCompletedListener")
        requireNotNull(intervalDetectorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorInitializationCompletedListener.onInitializationCompleted(
            intervalDetector,
            baseNoiseLevel
        )
    }

    @Test
    fun onInitializationCompleted_whenListenerAvailable_notifies() {
        val initializationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitializationCompletedListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            initializationCompletedListener = initializationCompletedListener
        )

        val intervalDetectorInitializationCompletedListener: IntervalDetector.OnInitializationCompletedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorInitializationCompletedListener")
        requireNotNull(intervalDetectorInitializationCompletedListener)

        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorInitializationCompletedListener.onInitializationCompleted(
            intervalDetector,
            baseNoiseLevel
        )

        // check
        verify(exactly = 1) { initializationCompletedListener.onInitializationCompleted(calibrator) }
    }

    @Test
    fun onError_whenNoListeners_stopsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location, timestamp)

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorErrorListener: IntervalDetector.OnErrorListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorErrorListener")
        requireNotNull(intervalDetectorErrorListener)

        intervalDetectorErrorListener.onError(
            intervalDetectorSpy,
            IntervalDetector.ErrorReason.UNRELIABLE_SENSOR
        )

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
    }

    @Test
    fun onError_whenListenersAvailable_stopsAndNotifies() {
        val errorListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>(relaxUnitFun = true)
        val stoppedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            errorListener = errorListener,
            stoppedListener = stoppedListener
        )

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorErrorListener: IntervalDetector.OnErrorListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorErrorListener")
        requireNotNull(intervalDetectorErrorListener)

        intervalDetectorErrorListener.onError(
            intervalDetectorSpy,
            IntervalDetector.ErrorReason.UNRELIABLE_SENSOR
        )

        // check
        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                StaticIntervalMagnetometerCalibrator.ErrorReason.UNRELIABLE_SENSOR
            )
        }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }

    }

    @Test
    fun onDynamicIntervalDetector_addsOneMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp
        )

        assertTrue(calibrator.measurements.isEmpty())

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
            intervalDetector,
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0
        )

        assertEquals(1, calibrator.measurements.size)
        val measurement = calibrator.measurements[0]
        val b = measurement.magneticFluxDensity

        assertEquals(7.0, b.bx, 0.0)
        assertEquals(8.0, b.by, 0.0)
        assertEquals(9.0, b.bz, 0.0)

        val stdNorm = sqrt(10.0.pow(2.0) + 11.0.pow(2.0) + 12.0.pow(2.0))
        assertEquals(stdNorm, measurement.magneticFluxDensityStandardDeviation, 0.0)

        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenNewCalibrationMeasurementAvailable_notifies() {
        val newCalibrationMeasurementAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnNewCalibrationMeasurementAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = mockk<Location>()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            newCalibrationMeasurementAvailableListener = newCalibrationMeasurementAvailableListener
        )

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        val intervalDetector = mockk<MagnetometerIntervalDetector>()
        intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
            intervalDetector,
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0
        )

        assertEquals(1, calibrator.measurements.size)
        val measurement = calibrator.measurements[0]

        val measurementSize = calibrator.measurements.size
        assertEquals(1, measurementSize)

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        verify(exactly = 1) {
            newCalibrationMeasurementAvailableListener.onNewCalibrationMeasurementAvailable(
                calibrator,
                measurement,
                measurementSize,
                reqMeasurements
            )
        }

        assertFalse(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenEnoughMeasurementsAndNotSolveCalibrator_stopsCollectorAndBuildInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val timestamp = Date()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            solveCalibrationWhenEnoughMeasurements = false
        )

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val internalCalibrator1: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNull(internalCalibrator1)

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        for (i in 1..reqMeasurements) {
            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0
            )
        }

        assertEquals(reqMeasurements, calibrator.measurements.size)
        assertTrue(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }

        val internalCalibrator2: MagnetometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNotNull(internalCalibrator2)

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenListenersAvailable_notifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>(relaxUnitFun = true)
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            solveCalibrationWhenEnoughMeasurements = false,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            stoppedListener = stoppedListener
        )

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val internalCalibrator1: AccelerometerNonLinearCalibrator? =
            calibrator.getPrivateProperty("internalCalibrator")
        assertNull(internalCalibrator1)

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

        for (i in 1..reqMeasurements) {
            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0
            )
        }

        assertEquals(reqMeasurements, calibrator.measurements.size)
        assertTrue(calibrator.running)
        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenSolveCalibrationEnabled_solvesCalibration() {
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()
        val worldMagneticModel = wmmEstimator.model

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            worldMagneticModel,
            solveCalibrationWhenEnoughMeasurements = true
        )

        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        val nedPosition = location.toNEDPosition()

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

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

            val noiseRandomizer = GaussianRandomizer(Random(), 0.0, MAGNETOMETER_NOISE_STD)
            measuredMagnetic.bx = measuredMagnetic.bx + noiseRandomizer.nextDouble()
            measuredMagnetic.by = measuredMagnetic.by + noiseRandomizer.nextDouble()
            measuredMagnetic.bz = measuredMagnetic.bz + noiseRandomizer.nextDouble()

            val std = noiseRandomizer.standardDeviation

            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                measuredMagnetic.bx,
                measuredMagnetic.by,
                measuredMagnetic.bz,
                std,
                std,
                std
            )
        }

        assertNotNull(calibrator.estimatedMm)
        assertNotNull(calibrator.estimatedSx)
        assertNotNull(calibrator.estimatedSy)
        assertNotNull(calibrator.estimatedSz)
        assertNotNull(calibrator.estimatedMxy)
        assertNotNull(calibrator.estimatedMxz)
        assertNotNull(calibrator.estimatedMyx)
        assertNotNull(calibrator.estimatedMyz)
        assertNotNull(calibrator.estimatedMzx)
        assertNotNull(calibrator.estimatedMzy)
        assertNotNull(calibrator.estimatedCovariance)
        assertNotNull(calibrator.estimatedChiSq)
        assertNotNull(calibrator.estimatedMse)
        assertNotNull(calibrator.estimatedHardIronX)
        assertNotNull(calibrator.estimatedHardIronY)
        assertNotNull(calibrator.estimatedHardIronZ)
        assertNotNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertTrue(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertTrue(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedHardIronAsTriad(triad))

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onDynamicIntervalDetected_whenSolveCalibrationEnabledAndListenerAvailable_solvesCalibrationAndNotifies() {
        val readyToSolveCalibrationListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnReadyToSolveCalibrationListener>(
                relaxUnitFun = true
            )
        val stoppedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>(relaxUnitFun = true)
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>(relaxUnitFun = true)
        val errorListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>(relaxUnitFun = true)

        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()
        val worldMagneticModel = wmmEstimator.model

        val location = getLocation()
        val timestamp = Date()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            timestamp,
            worldMagneticModel,
            solveCalibrationWhenEnoughMeasurements = true,
            readyToSolveCalibrationListener = readyToSolveCalibrationListener,
            stoppedListener = stoppedListener,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener,
            errorListener = errorListener
        )

        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        val nedPosition = location.toNEDPosition()

        val requiredMeasurements = calibrator.requiredMeasurements
        assertEquals(13, requiredMeasurements)
        val minimumRequiredMeasurements = calibrator.minimumRequiredMeasurements
        assertEquals(13, minimumRequiredMeasurements)
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)

        val intervalDetectorDynamicIntervalDetectedListener: IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? =
            calibrator.getPrivateProperty("intervalDetectorDynamicIntervalDetectedListener")
        requireNotNull(intervalDetectorDynamicIntervalDetectedListener)

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

            val noiseRandomizer = GaussianRandomizer(Random(), 0.0, MAGNETOMETER_NOISE_STD)
            measuredMagnetic.bx = measuredMagnetic.bx + noiseRandomizer.nextDouble()
            measuredMagnetic.by = measuredMagnetic.by + noiseRandomizer.nextDouble()
            measuredMagnetic.bz = measuredMagnetic.bz + noiseRandomizer.nextDouble()

            val std = noiseRandomizer.standardDeviation

            intervalDetectorDynamicIntervalDetectedListener.onDynamicIntervalDetected(
                intervalDetector,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                measuredMagnetic.bx,
                measuredMagnetic.by,
                measuredMagnetic.bz,
                std,
                std,
                std
            )
        }

        assertNotNull(calibrator.estimatedMm)
        assertNotNull(calibrator.estimatedSx)
        assertNotNull(calibrator.estimatedSy)
        assertNotNull(calibrator.estimatedSz)
        assertNotNull(calibrator.estimatedMxy)
        assertNotNull(calibrator.estimatedMxz)
        assertNotNull(calibrator.estimatedMyx)
        assertNotNull(calibrator.estimatedMyz)
        assertNotNull(calibrator.estimatedMzx)
        assertNotNull(calibrator.estimatedMzy)
        assertNotNull(calibrator.estimatedCovariance)
        assertNotNull(calibrator.estimatedChiSq)
        assertNotNull(calibrator.estimatedMse)
        assertNotNull(calibrator.estimatedHardIronX)
        assertNotNull(calibrator.estimatedHardIronY)
        assertNotNull(calibrator.estimatedHardIronZ)
        assertNotNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
        assertTrue(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
        assertTrue(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
        assertNotNull(calibrator.estimatedHardIronAsTriad)
        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedHardIronAsTriad(triad))

        verify(exactly = 1) { readyToSolveCalibrationListener.onReadyToSolveCalibration(calibrator) }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
        verify(exactly = 1) {
            calibrationSolvingStartedListener.onCalibrationSolvingStarted(
                calibrator
            )
        }
        verify(exactly = 1) { calibrationCompletedListener.onCalibrationCompleted(calibrator) }
        verify { errorListener wasNot Called }

        assertTrue(calibrator.isReadyToSolveCalibration)
    }

    @Test
    fun onMeasurement_whenFirstMeasurement_updatesInitialHardIrons() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialHardIronX = calibrator.initialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.initialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.initialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(4.0, initialHardIronX, 0.0)
        assertEquals(5.0, initialHardIronY, 0.0)
        assertEquals(6.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoHardIronX_updatesInitialHardIrons() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            null,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialHardIronX = calibrator.initialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.initialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.initialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoHardIronY_updatesInitialHardIrons() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            null,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialHardIronX = calibrator.initialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.initialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.initialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndNoHardIronZ_updatesInitialHardIrons() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            null,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialHardIronX = calibrator.initialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.initialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.initialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)
    }

    @Test
    fun onMeasurement_whenFirstMeasurementAndListener_updatesInitialBiasesAndNotifies() {
        val initialHardIronAvailableListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnInitialHardIronAvailableListener>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            initialHardIronAvailableListener = initialHardIronAvailableListener
        )

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(0)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            null,
            null,
            null,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        val initialHardIronX = calibrator.initialHardIronX
        requireNotNull(initialHardIronX)
        val initialHardIronY = calibrator.initialHardIronY
        requireNotNull(initialHardIronY)
        val initialHardIronZ = calibrator.initialHardIronZ
        requireNotNull(initialHardIronZ)
        assertEquals(0.0, initialHardIronX, 0.0)
        assertEquals(0.0, initialHardIronY, 0.0)
        assertEquals(0.0, initialHardIronZ, 0.0)

        verify(exactly = 1) {
            initialHardIronAvailableListener.onInitialHardIronAvailable(
                calibrator,
                0.0,
                0.0,
                0.0
            )
        }
    }

    @Test
    fun onMeasurement_whenNotFirstMeasurement_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        every { intervalDetectorSpy.numberOfProcessedMeasurements }.returns(2)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val intervalDetectorMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
            calibrator.getPrivateProperty("intervalDetectorMeasurementListener")
        requireNotNull(intervalDetectorMeasurementListener)

        intervalDetectorMeasurementListener.onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            6.0f,
            SystemClock.elapsedRealtimeNanos(),
            SensorAccuracy.HIGH
        )

        // check
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
    }

    @Test
    fun initialHardIronX_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronX)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)

        assertEquals(initialHardIronX, calibrator.initialHardIronX)
    }

    @Test
    fun initialHardIronY_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronY)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)

        assertEquals(initialHardIronY, calibrator.initialHardIronY)
    }

    @Test
    fun initialHardIronZ_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronZ)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)

        assertEquals(initialHardIronZ, calibrator.initialHardIronZ)
    }

    @Test
    fun initialHardIronXAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronXAsMagneticFluxDensity)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)

        val hardIron = calibrator.initialHardIronXAsMagneticFluxDensity
        requireNotNull(hardIron)
        assertEquals(initialHardIronX, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getInitialHardIronXAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronXAsMagneticFluxDensity(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)

        // check
        assertTrue(calibrator.getInitialHardIronXAsMagneticFluxDensity(hardIron))
        assertEquals(initialHardIronX, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun initialHardIronYAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronYAsMagneticFluxDensity)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)

        val hardIron = calibrator.initialHardIronYAsMagneticFluxDensity
        requireNotNull(hardIron)
        assertEquals(initialHardIronY, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getInitialHardIronYAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronYAsMagneticFluxDensity(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)

        // check
        assertTrue(calibrator.getInitialHardIronYAsMagneticFluxDensity(hardIron))
        assertEquals(initialHardIronY, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun initialHardIronZAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronZAsMagneticFluxDensity)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)

        val hardIron = calibrator.initialHardIronZAsMagneticFluxDensity
        requireNotNull(hardIron)
        assertEquals(initialHardIronZ, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun getInitialHardIronZAsMagneticFluxDensity_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val hardIron = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getInitialHardIronZAsMagneticFluxDensity(hardIron))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)

        // check
        assertTrue(calibrator.getInitialHardIronZAsMagneticFluxDensity(hardIron))
        assertEquals(initialHardIronZ, hardIron.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIron.unit)
    }

    @Test
    fun initialHardIronAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.initialHardIronAsTriad)

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)

        assertNull(calibrator.initialHardIronAsTriad)

        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)

        assertNull(calibrator.initialHardIronAsTriad)

        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)

        // check
        val triad = calibrator.initialHardIronAsTriad
        requireNotNull(triad)
        assertEquals(initialHardIronX, triad.valueX, 0.0)
        assertEquals(initialHardIronY, triad.valueY, 0.0)
        assertEquals(initialHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun getInitialHardIronAsTriad_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getInitialHardIronAsTriad(triad))

        // set new value
        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)

        assertFalse(calibrator.getInitialHardIronAsTriad(triad))

        val initialHardIronY = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)

        assertFalse(calibrator.getInitialHardIronAsTriad(triad))

        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)

        // check
        assertTrue(calibrator.getInitialHardIronAsTriad(triad))
        assertEquals(initialHardIronX, triad.valueX, 0.0)
        assertEquals(initialHardIronY, triad.valueY, 0.0)
        assertEquals(initialHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun magnetometerSensor_getsIntervalDetectorSensor() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val sensor = mockk<Sensor>()
        every { intervalDetectorSpy.sensor }.returns(sensor)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(sensor, calibrator.magnetometerSensor)

        verify(exactly = 1) { intervalDetectorSpy.sensor }
    }

    @Test
    fun baseNoiseLevel_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevel }.returns(baseNoiseLevel)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevel, calibrator.baseNoiseLevel)

        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevel }
    }

    @Test
    fun baseNoiseLevelAsMagneticFluxDensity_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.baseNoiseLevelAsMagneticFluxDensity)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        val baseNoiseLevel1 = MagneticFluxDensity(baseNoiseLevel, MagneticFluxDensityUnit.TESLA)
        every { intervalDetectorSpy.baseNoiseLevelAsMeasurement }.returns(baseNoiseLevel1)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        val baseNoiseLevel2 = calibrator.baseNoiseLevelAsMagneticFluxDensity
        assertSame(baseNoiseLevel1, baseNoiseLevel2)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelAsMeasurement }
    }

    @Test
    fun getBaseNoiseLevelAsMagneticFluxDensity_getsIntervalDetectorBaseNoiseLevel() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevel = randomizer.nextDouble()
        every { intervalDetectorSpy.getBaseNoiseLevelAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = baseNoiseLevel
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getBaseNoiseLevelAsMagneticFluxDensity(b))

        // check
        assertEquals(baseNoiseLevel, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
        verify(exactly = 1) { intervalDetectorSpy.getBaseNoiseLevelAsMeasurement(b) }
    }

    @Test
    fun baseNoiseLevelPsd_getsIntervalDetectorBaseNoiseLevelPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.baseNoiseLevelPsd)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelPsd = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevelPsd }.returns(baseNoiseLevelPsd)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevelPsd, calibrator.baseNoiseLevelPsd)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelPsd }
    }

    @Test
    fun baseNoiseLevelRootPsd_getsIntervalDetectorBaseNoiseLevelRootPsd() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.baseNoiseLevelRootPsd)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val baseNoiseLevelRootPsd = randomizer.nextDouble()
        every { intervalDetectorSpy.baseNoiseLevelRootPsd }.returns(baseNoiseLevelRootPsd)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(baseNoiseLevelRootPsd, calibrator.baseNoiseLevelRootPsd)
        verify(exactly = 1) { intervalDetectorSpy.baseNoiseLevelRootPsd }
    }

    @Test
    fun threshold_getsIntervalDetectorThreshold() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.threshold)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { intervalDetectorSpy.threshold }.returns(threshold)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(threshold, calibrator.threshold)
        verify(exactly = 1) { intervalDetectorSpy.threshold }
    }

    @Test
    fun thresholdAsMagneticFluxDensity_getsIntervalDetectorThresholdAsMagneticFluxDensity() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.thresholdAsMagneticFluxDensity)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        val b = MagneticFluxDensity(threshold, MagneticFluxDensityUnit.TESLA)
        every { intervalDetectorSpy.thresholdAsMeasurement }.returns(b)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(b, calibrator.thresholdAsMagneticFluxDensity)
        verify(exactly = 1) { intervalDetectorSpy.thresholdAsMeasurement }
    }

    @Test
    fun getThresholdAsMagneticFluxDensity_getsIntervalDetectorThresholdAsMagneticFluxDensity() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getThresholdAsMagneticFluxDensity(b))

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val threshold = randomizer.nextDouble()
        every { intervalDetectorSpy.getThresholdAsMeasurement(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = threshold
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getThresholdAsMagneticFluxDensity(b))
        assertEquals(threshold, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
        verify(exactly = 1) { intervalDetectorSpy.getThresholdAsMeasurement(b) }
    }

    @Test
    fun averageTimeInterval_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.averageTimeInterval)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { intervalDetectorSpy.averageTimeInterval }.returns(averageTimeInterval)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(averageTimeInterval, calibrator.averageTimeInterval)
        verify(exactly = 1) { intervalDetectorSpy.averageTimeInterval }
    }

    @Test
    fun averageTimeIntervalAsTime_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.averageTimeIntervalAsTime)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        val time = Time(averageTimeInterval, TimeUnit.SECOND)
        every { intervalDetectorSpy.averageTimeIntervalAsTime }.returns(time)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(time, calibrator.averageTimeIntervalAsTime)
        verify(exactly = 1) { intervalDetectorSpy.averageTimeIntervalAsTime }
    }

    @Test
    fun getAverageTimeIntervalAsTime_getsIntervalDetectorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getAverageTimeIntervalAsTime(time))

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val averageTimeInterval = randomizer.nextDouble()
        every { intervalDetectorSpy.getAverageTimeIntervalAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = averageTimeInterval
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getAverageTimeIntervalAsTime(time))
        assertEquals(averageTimeInterval, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { intervalDetectorSpy.getAverageTimeIntervalAsTime(time) }
    }

    @Test
    fun timeIntervalVariance_getsIntervalDetectorTimeIntervalVariance() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.timeIntervalVariance)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val timeIntervalVariance = randomizer.nextDouble()
        every { intervalDetectorSpy.timeIntervalVariance }.returns(timeIntervalVariance)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(timeIntervalVariance, calibrator.timeIntervalVariance)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalVariance }
    }

    @Test
    fun timeIntervalStandardDeviation_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.timeIntervalStandardDeviation)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val timeIntervalStandardDeviation = randomizer.nextDouble()
        every { intervalDetectorSpy.timeIntervalStandardDeviation }.returns(
            timeIntervalStandardDeviation
        )
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertEquals(timeIntervalStandardDeviation, calibrator.timeIntervalStandardDeviation)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalStandardDeviation }
    }

    @Test
    fun timeIntervalStandardDeviationAsTime_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        assertNull(calibrator.timeIntervalStandardDeviationAsTime)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        val time = Time(value, TimeUnit.SECOND)
        every { intervalDetectorSpy.timeIntervalStandardDeviationAsTime }.returns(time)
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertSame(time, calibrator.timeIntervalStandardDeviationAsTime)
        verify(exactly = 1) { intervalDetectorSpy.timeIntervalStandardDeviationAsTime }
    }

    @Test
    fun getTimeIntervalStandardDeviationAsTime_getsIntervalDetectorTimeIntervalStandardDeviation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        // check default value
        val time = Time(0.0, TimeUnit.SECOND)
        assertFalse(calibrator.getTimeIntervalStandardDeviationAsTime(time))

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        every { intervalDetectorSpy.getTimeIntervalStandardDeviationAsTime(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as Time
            result.value = value
            result.unit = TimeUnit.SECOND
            return@answers true
        }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        assertTrue(calibrator.getTimeIntervalStandardDeviationAsTime(time))
        assertEquals(value, time.value.toDouble(), 0.0)
        assertEquals(TimeUnit.SECOND, time.unit)
        verify(exactly = 1) { intervalDetectorSpy.getTimeIntervalStandardDeviationAsTime(time) }
    }

    @Test
    fun minimumRequiredMeasurements_whenCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isCommonAxisUsed = true
        calibrator.isGroundTruthInitialHardIron = true

        // check
        assertTrue(calibrator.isCommonAxisUsed)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertEquals(7, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isCommonAxisUsed = true
        calibrator.isGroundTruthInitialHardIron = false

        // check
        assertTrue(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenNotCommonAxisAndKnownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isCommonAxisUsed = false
        calibrator.isGroundTruthInitialHardIron = true

        // check
        assertFalse(calibrator.isCommonAxisUsed)
        assertTrue(calibrator.isGroundTruthInitialHardIron)
        assertEquals(10, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun minimumRequiredMeasurements_whenNotCommonAxisAndUnknownHardIron_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        calibrator.isCommonAxisUsed = false
        calibrator.isGroundTruthInitialHardIron = false

        // check
        assertFalse(calibrator.isCommonAxisUsed)
        assertFalse(calibrator.isGroundTruthInitialHardIron)
        assertEquals(13, calibrator.minimumRequiredMeasurements)
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsIntervalDetector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertFalse(calibrator.running)

        val measurements = calibrator.measurements
        val measurementsSpy = spyk(measurements)
        calibrator.setPrivateProperty("measurements", measurementsSpy)

        calibrator.setPrivateProperty("resultUnreliable", true)
        calibrator.setPrivateProperty("initialHardIronX", 0.0)
        calibrator.setPrivateProperty("initialHardIronY", 0.0)
        calibrator.setPrivateProperty("initialHardIronZ", 0.0)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.start() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        calibrator.start()

        // check
        verify(exactly = 1) { measurementsSpy.clear() }
        assertNull(calibrator.initialHardIronX)
        assertNull(calibrator.initialHardIronY)
        assertNull(calibrator.initialHardIronZ)
        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        assertTrue(calibrator.running)

        verify(exactly = 1) { intervalDetectorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        assertFalse(calibrator.running)

        calibrator.start()

        assertTrue(calibrator.running)

        calibrator.start()
    }

    @Test
    fun stop_stopsIntervalDetector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(context, location)

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.stop() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
    }

    @Test
    fun stop_whenListenerAvailable_notifies() {
        val stoppedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnStoppedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            stoppedListener = stoppedListener
        )

        val intervalDetector: MagnetometerIntervalDetector? =
            calibrator.getPrivateProperty("intervalDetector")
        requireNotNull(intervalDetector)
        val intervalDetectorSpy = spyk(intervalDetector)
        justRun { intervalDetectorSpy.stop() }
        calibrator.setPrivateProperty("intervalDetector", intervalDetectorSpy)

        calibrator.setPrivateProperty("running", true)
        assertTrue(calibrator.running)

        calibrator.stop()

        assertFalse(calibrator.running)
        verify(exactly = 1) { intervalDetectorSpy.stop() }
        verify(exactly = 1) { stoppedListener.onStopped(calibrator) }
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenNotReadyToSolveCalibration_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        calibrator.calibrate()
    }

    @Test(expected = IllegalStateException::class)
    fun calibrate_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        calibrator.setPrivateProperty("running", true)

        calibrator.calibrate()
    }

    @Test
    fun calibrate_whenReadyNotRunningAndNoInternalCalibrator_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        assertTrue(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenReadyNotRunningAndInternalCalibratorAndListeners_callsInternalCalibratorAndNotifies() {
        val calibrationSolvingStartedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationSolvingStartedListener>(
                relaxUnitFun = true
            )
        val calibrationCompletedListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnCalibrationCompletedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            calibrationSolvingStartedListener = calibrationSolvingStartedListener,
            calibrationCompletedListener = calibrationCompletedListener
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        justRun { internalCalibrator.calibrate() }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

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
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
    }

    @Test
    fun calibrate_whenFailureAndErrorListener_setsAsNotRunning() {
        val errorListener =
            mockk<StaticIntervalMagnetometerCalibrator.OnErrorListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            errorListener = errorListener
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        assertTrue(calibrator.isReadyToSolveCalibration)
        assertFalse(calibrator.running)

        val internalCalibrator = mockk<MagnetometerNonLinearCalibrator>()
        every { internalCalibrator.calibrate() }.throws(NavigationException())
        calibrator.setPrivateProperty("internalCalibrator", internalCalibrator)

        assertFalse(calibrator.calibrate())

        assertFalse(calibrator.running)
        verify(exactly = 1) {
            errorListener.onError(
                calibrator,
                StaticIntervalMagnetometerCalibrator.ErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION
            )
        }
    }

    @Test
    fun estimatedHardIronX_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronX)
    }

    @Test
    fun estimatedHardIronX_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedHardIronX)
    }

    @Test
    fun estimatedHardIronX_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronX }.returns(estimatedHardIronX)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronX, calibrator.estimatedHardIronX)
    }

    @Test
    fun estimatedHardIronY_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronY)
    }

    @Test
    fun estimatedHardIronY_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedHardIronY)
    }

    @Test
    fun estimatedHardIronY_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronY }.returns(estimatedHardIronY)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronY, calibrator.estimatedHardIronY)
    }

    @Test
    fun estimatedHardIronZ_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronZ)
    }

    @Test
    fun estimatedHardIronZ_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.estimatedHardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedHardIronZ)
    }

    @Test
    fun estimatedHardIronZ_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.hardIronZ }.returns(estimatedHardIronZ)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertEquals(estimatedHardIronZ, calibrator.estimatedHardIronZ)
    }

    @Test
    fun estimatedHardIronXAsMagneticFluxDensity_whenNoIntervalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronXAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronXAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronXAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronXAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronXAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronX, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronXAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronXAsMagneticFluxDensity)
    }

    @Test
    fun getEstimatedHardIronXAsMagneticFluxDensity_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))
    }

    @Test
    fun getEstimatedHardIronXAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronXAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronX
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))

        assertEquals(estimatedHardIronX, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun getEstimatedHardIronXAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronX = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronXAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronX
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(b))

        assertEquals(estimatedHardIronX, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedHardIronYAsMagneticFluxDensity_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronYAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronYAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronYAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronYAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronYAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronY, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronYAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronYAsMagneticFluxDensity)
    }

    @Test
    fun getEstimatedHardIronYAsMagneticFluxDensity_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))
    }

    @Test
    fun getEstimatedHardIronYAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronYAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronY
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))

        assertEquals(estimatedHardIronY, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun getEstimatedHardIronYAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronY = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronYAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronY
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(b))

        assertEquals(estimatedHardIronY, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedHardIronZAsMagneticFluxDensity_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronZAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronZAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.estimatedHardIronZAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronZAsMagneticFluxDensity)
    }

    @Test
    fun estimatedHardIronZAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        val b = MagneticFluxDensity(estimatedHardIronZ, MagneticFluxDensityUnit.TESLA)
        every { internalCalibratorSpy.hardIronZAsMagneticFluxDensity }.returns(b)
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(b, calibrator.estimatedHardIronZAsMagneticFluxDensity)
    }

    @Test
    fun getEstimatedHardIronZAsMagneticFluxDensity_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
    }

    @Test
    fun getEstimatedHardIronZAsMagneticFluxDensity_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getEstimatedHardIronZAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
            return@answers true
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))
    }

    @Test
    fun getEstimatedHardIronZAsMagneticFluxDensity_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        val internalCalibratorSpy = spyk(KnownHardIronPositionAndInstantMagnetometerCalibrator())
        val randomizer = UniformRandomizer()
        val estimatedHardIronZ = randomizer.nextDouble()
        every { internalCalibratorSpy.getHardIronZAsMagneticFluxDensity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as MagneticFluxDensity
            result.value = estimatedHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
        }
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val b = MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)
        assertTrue(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(b))

        assertEquals(estimatedHardIronZ, b.value.toDouble(), 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, b.unit)
    }

    @Test
    fun estimatedHardIronAsTriad_whenNoInternalCalibrator_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))
        assertNull(calibrator.estimatedHardIronAsTriad)
    }

    @Test
    fun estimatedHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

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
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedHardIronAsTriad)
    }

    @Test
    fun estimatedHardIronAsTriad_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

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
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        assertSame(triad, calibrator.estimatedHardIronAsTriad)
    }

    @Test
    fun getEstimatedHardIronAsTriad_whenNoInternalCalibrator_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

        assertNull(calibrator.getPrivateProperty("internalCalibrator"))

        val triad = MagneticFluxDensityTriad()
        assertFalse(calibrator.getEstimatedHardIronAsTriad(triad))
    }

    @Test
    fun getEstimatedHardIronAsTriad_whenUnknownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

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
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedHardIronAsTriad(triad))

        assertEquals(estimatedHardIronX, triad.valueX, 0.0)
        assertEquals(estimatedHardIronY, triad.valueY, 0.0)
        assertEquals(estimatedHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun getEstimatedHardIronAsTriad_whenKnownHardIronInternalCalibrator_callsInternalCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location
        )

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
        calibrator.setPrivateProperty("internalCalibrator", internalCalibratorSpy)

        val triad = MagneticFluxDensityTriad()
        assertTrue(calibrator.getEstimatedHardIronAsTriad(triad))

        assertEquals(estimatedHardIronX, triad.valueX, 0.0)
        assertEquals(estimatedHardIronY, triad.valueY, 0.0)
        assertEquals(estimatedHardIronZ, triad.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = true
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertTrue(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownHardIronPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = false

        val randomizer = UniformRandomizer()
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        calibrator.setPrivateProperty("initialHardIronX", initialHardIronX)
        calibrator.setPrivateProperty("initialHardIronY", initialHardIronY)
        calibrator.setPrivateProperty("initialHardIronZ", initialHardIronZ)
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
    }

    @Test
    fun buildInternalCalibrator_whenNonRobustNoGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val calibrator = StaticIntervalMagnetometerCalibrator(
            context,
            location,
            isGroundTruthInitialHardIron = false
        )

        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            calibrator.measurements.add(measurement)
        }

        calibrator.isCommonAxisUsed = true

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
        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
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

        assertNull(calibrator.robustMethod)
        assertFalse(calibrator.isGroundTruthInitialHardIron)

        val internalCalibrator: MagnetometerNonLinearCalibrator? =
            calibrator.callPrivateFuncWithResult("buildInternalCalibrator")
        requireNotNull(internalCalibrator)

        // check
        val internalCalibrator2 =
            internalCalibrator as KnownPositionAndInstantMagnetometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(calibrator.measurements, internalCalibrator2.measurements)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
        assertEquals(
            calibrator.minimumRequiredMeasurements,
            internalCalibrator2.minimumRequiredMeasurements
        )
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
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON
                )
            } catch (ignore: WrongSizeException) {
                // never happens
                null
            }
        }
    }
}