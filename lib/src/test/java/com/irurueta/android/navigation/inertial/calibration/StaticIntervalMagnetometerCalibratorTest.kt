package com.irurueta.android.navigation.inertial.calibration

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import com.irurueta.units.TimeUnit
import io.mockk.mockk
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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
        assertNull(calibrator.estimatedMa)
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

    private companion object {
        const val MM_SIZE = 3

        const val INITIAL_STATIC_SAMPLES = 2500

        const val THRESHOLD_FACTOR = 3.0

        const val INSTANTANEOUS_NOISE_LEVEL_FACTOR = 3.0

        const val BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD = 1e-5

        const val WINDOW_SIZE = 51
    }
}