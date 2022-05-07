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
package com.irurueta.android.navigation.inertial.calibration.builder

import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.*
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.mockk
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class MagnetometerInternalCalibratorBuilderTest {

    @Test
    fun constructor_whenRequiredValues_setsExpectedValues() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check
        assertSame(measurements, builder.measurements)
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            builder.groundTruthMagneticFluxDensityNorm,
            0.0
        )
        assertEquals(robustPreliminarySubsetSize, builder.robustPreliminarySubsetSize)
        assertEquals(minimumRequiredMeasurements, builder.minimumRequiredMeasurements)
        assertNull(builder.robustMethod)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )
        assertNull(builder.robustThreshold)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )
        assertFalse(builder.isGroundTruthInitialHardIron)
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )
        assertNull(builder.initialHardIronX)
        assertNull(builder.initialHardIronY)
        assertNull(builder.initialHardIronZ)
        assertEquals(0.0, builder.initialSx, 0.0)
        assertEquals(0.0, builder.initialSy, 0.0)
        assertEquals(0.0, builder.initialSz, 0.0)
        assertEquals(0.0, builder.initialMxy, 0.0)
        assertEquals(0.0, builder.initialMxz, 0.0)
        assertEquals(0.0, builder.initialMyx, 0.0)
        assertEquals(0.0, builder.initialMyz, 0.0)
        assertEquals(0.0, builder.initialMzx, 0.0)
        assertEquals(0.0, builder.initialMzy, 0.0)
        assertNull(builder.baseNoiseLevel)
        assertNotNull(builder.qualityScoreMapper)
    }

    @Test
    fun constructor_whenAllValues_setsExpectedValues() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val robustMethod = RobustEstimatorMethod.RANSAC
        val robustConfidence = ROBUST_CONFIDENCE
        val robustMaxIterations = ROBUST_MAX_ITERATIONS
        val robustThreshold = ROBUST_THRESHOLD
        val robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        val robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
        val isGroundTruthInitialHardIron = true
        val isCommonAxisUsed = true
        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()
        val qualityScoreMapper = DefaultMagnetometerQualityScoreMapper()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod,
            robustConfidence,
            robustMaxIterations,
            robustThreshold,
            robustThresholdFactor,
            robustStopThresholdFactor,
            isGroundTruthInitialHardIron,
            isCommonAxisUsed,
            initialHardIronX,
            initialHardIronY,
            initialHardIronZ,
            initialSx,
            initialSy,
            initialSz,
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy,
            baseNoiseLevel,
            qualityScoreMapper
        )

        // check
        assertSame(measurements, builder.measurements)
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            builder.groundTruthMagneticFluxDensityNorm,
            0.0
        )
        assertEquals(robustPreliminarySubsetSize, builder.robustPreliminarySubsetSize)
        assertEquals(minimumRequiredMeasurements, builder.minimumRequiredMeasurements)
        assertEquals(robustMethod, builder.robustMethod)
        assertEquals(robustConfidence, builder.robustConfidence, 0.0)
        assertEquals(robustMaxIterations, builder.robustMaxIterations)
        assertEquals(robustThreshold, builder.robustThreshold)
        assertEquals(robustThresholdFactor, builder.robustThresholdFactor, 0.0)
        assertEquals(robustStopThresholdFactor, builder.robustStopThresholdFactor, 0.0)
        assertTrue(builder.isGroundTruthInitialHardIron)
        assertTrue(builder.isCommonAxisUsed)
        assertEquals(initialHardIronX, builder.initialHardIronX)
        assertEquals(initialHardIronY, builder.initialHardIronY)
        assertEquals(initialHardIronZ, builder.initialHardIronZ)
        assertEquals(initialSx, builder.initialSx, 0.0)
        assertEquals(initialSy, builder.initialSy, 0.0)
        assertEquals(initialSz, builder.initialSz, 0.0)
        assertEquals(initialMxy, builder.initialMxy, 0.0)
        assertEquals(initialMxz, builder.initialMxz, 0.0)
        assertEquals(initialMyx, builder.initialMyx, 0.0)
        assertEquals(initialMyz, builder.initialMyz, 0.0)
        assertEquals(initialMzx, builder.initialMzx, 0.0)
        assertEquals(initialMzy, builder.initialMzy, 0.0)
        assertEquals(baseNoiseLevel, builder.baseNoiseLevel)
        assertSame(qualityScoreMapper, builder.qualityScoreMapper)
    }

    @Test
    fun measurements_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements1 = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements1,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertSame(measurements1, builder.measurements)

        // set new value
        val measurements2 = emptyList<StandardDeviationBodyMagneticFluxDensity>()

        // check
        assertSame(measurements2, builder.measurements)
    }

    @Test
    fun groundTruthMagneticFluxDensityNorm_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm1 = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm1,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            groundTruthMagneticFluxDensityNorm1,
            builder.groundTruthMagneticFluxDensityNorm,
            0.0)

        // set new value
        val groundTruthMagneticFluxDensityNorm2 = randomizer.nextDouble()
        builder.groundTruthMagneticFluxDensityNorm = groundTruthMagneticFluxDensityNorm2

        // check
        assertEquals(
            groundTruthMagneticFluxDensityNorm2,
            builder.groundTruthMagneticFluxDensityNorm,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun groundTruthMagneticFluxDensityNorm_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = -randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)

        MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun robustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize1 = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize1, 2 * robustPreliminarySubsetSize1)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize1,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(robustPreliminarySubsetSize1, builder.robustPreliminarySubsetSize)

        // set new value
        val robustPreliminarySubsetSize2 = robustPreliminarySubsetSize1 + 1
        builder.robustPreliminarySubsetSize = robustPreliminarySubsetSize2

        // check
        assertEquals(robustPreliminarySubsetSize2, builder.robustPreliminarySubsetSize)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustPreliminarySubsetSize_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = 0
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)

        MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun minimumRequiredMeasurements_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements1 =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements1
        )

        // check default values
        assertEquals(minimumRequiredMeasurements1, builder.minimumRequiredMeasurements)

        // set new value
        val minimumRequiredMeasurements2 = minimumRequiredMeasurements1 + 1
        builder.minimumRequiredMeasurements = minimumRequiredMeasurements2

        // check
        assertEquals(minimumRequiredMeasurements2, builder.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalArgumentException::class)
    fun minimumRequiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = -1

        MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun robustMethod_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.robustThreshold)

        // set new value
        builder.robustMethod = RobustEstimatorMethod.RANSAC

        // check
        assertEquals(RobustEstimatorMethod.RANSAC, builder.robustMethod)

        // set new value
        builder.robustMethod = null

        // check
        assertNull(builder.robustMethod)
    }

    @Test
    fun robustConfidence_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        val robustConfidence = ROBUST_CONFIDENCE
        builder.robustConfidence = robustConfidence

        // check
        assertEquals(robustConfidence, builder.robustConfidence, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenLowerBoundExceeded_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenUpperBoundExceeded_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = 2.0
    }

    @Test
    fun robustMaxIterations_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )

        // set new value
        val robustMaxIterations = ROBUST_MAX_ITERATIONS
        builder.robustMaxIterations = robustMaxIterations

        // check
        assertEquals(robustMaxIterations, builder.robustMaxIterations)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustMaxIterations_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )

        // set new value
        builder.robustMaxIterations = 0
    }

    @Test
    fun robustThreshold_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.robustThreshold)

        // set new value
        val robustThreshold = ROBUST_THRESHOLD
        builder.robustThreshold = robustThreshold

        // check
        assertEquals(robustThreshold, builder.robustThreshold)

        // set null value
        builder.robustThreshold = null

        // check
        assertNull(builder.robustThreshold)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThreshold_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.robustThreshold)

        // set new value
        val robustThreshold = 0.0
        builder.robustThreshold = robustThreshold
    }

    @Test
    fun robustThresholdFactor_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )

        // set new value
        val robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        builder.robustThresholdFactor = robustThresholdFactor

        // check
        assertEquals(robustThresholdFactor, builder.robustThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )

        // set new value
        builder.robustThresholdFactor = 0.0
    }

    @Test
    fun robustStopThresholdFactor_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )

        // set new value
        val robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
        builder.robustStopThresholdFactor = robustStopThresholdFactor

        // check
        assertEquals(robustStopThresholdFactor, builder.robustStopThresholdFactor, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustStopThresholdFactor_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )

        // set new value
        builder.robustStopThresholdFactor = 0.0
    }

    @Test
    fun isGroundTruthInitialHardIron_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertFalse(builder.isGroundTruthInitialHardIron)

        // set new value
        builder.isGroundTruthInitialHardIron = true

        // check
        assertTrue(builder.isGroundTruthInitialHardIron)
    }

    @Test
    fun isCommonAxisUsed_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )

        // set new value
        builder.isCommonAxisUsed = !StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS

        // check
        assertEquals(
            !StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )
    }

    @Test
    fun initialHardIronX_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialHardIronX)

        // set new value
        val initialHardIronX = randomizer.nextDouble()
        builder.initialHardIronX = initialHardIronX

        // check
        assertEquals(initialHardIronX, builder.initialHardIronX)

        // set new value
        builder.initialHardIronX = null

        // check
        assertNull(builder.initialHardIronX)
    }

    @Test
    fun initialHardIronY_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialHardIronY)

        // set new value
        val initialHardIronY = randomizer.nextDouble()
        builder.initialHardIronY = initialHardIronY

        // check
        assertEquals(initialHardIronY, builder.initialHardIronY)

        // set new value
        builder.initialHardIronY = null

        // check
        assertNull(builder.initialHardIronY)
    }

    @Test
    fun initialHardIronZ_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialHardIronZ)

        // set new value
        val initialHardIronZ = randomizer.nextDouble()
        builder.initialHardIronZ = initialHardIronZ

        // check
        assertEquals(initialHardIronZ, builder.initialHardIronZ)

        // set new value
        builder.initialHardIronZ = null

        // check
        assertNull(builder.initialHardIronZ)
    }

    @Test
    fun initialSx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialSx, 0.0)

        // set new value
        val initialSx = randomizer.nextDouble()
        builder.initialSx = initialSx

        // check
        assertEquals(initialSx, builder.initialSx, 0.0)
    }

    @Test
    fun initialSy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialSy, 0.0)

        // set new value
        val initialSy = randomizer.nextDouble()
        builder.initialSy = initialSy

        // check
        assertEquals(initialSy, builder.initialSy, 0.0)
    }

    @Test
    fun initialSz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialSz, 0.0)

        // set new value
        val initialSz = randomizer.nextDouble()
        builder.initialSz = initialSz

        // check
        assertEquals(initialSz, builder.initialSz, 0.0)
    }

    @Test
    fun initialMxy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMxy, 0.0)

        // set new value
        val initialMxy = randomizer.nextDouble()
        builder.initialMxy = initialMxy

        // check
        assertEquals(initialMxy, builder.initialMxy, 0.0)
    }

    @Test
    fun initialMxz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMxz, 0.0)

        // set new value
        val initialMxz = randomizer.nextDouble()
        builder.initialMxz = initialMxz

        // check
        assertEquals(initialMxz, builder.initialMxz, 0.0)
    }

    @Test
    fun initialMyx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMyx, 0.0)

        // set new value
        val initialMyx = randomizer.nextDouble()
        builder.initialMyx = initialMyx

        // check
        assertEquals(initialMyx, builder.initialMyx, 0.0)
    }

    @Test
    fun initialMyz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMyz, 0.0)

        // set new value
        val initialMyz = randomizer.nextDouble()
        builder.initialMyz = initialMyz

        // check
        assertEquals(initialMyz, builder.initialMyz, 0.0)
    }

    @Test
    fun initialMzx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMzx, 0.0)

        // set new value
        val initialMzx = randomizer.nextDouble()
        builder.initialMzx = initialMzx

        // check
        assertEquals(initialMzx, builder.initialMzx, 0.0)
    }

    @Test
    fun initialMzy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.initialMzy, 0.0)

        // set new value
        val initialMzy = randomizer.nextDouble()
        builder.initialMzy = initialMzy

        // check
        assertEquals(initialMzy, builder.initialMzy, 0.0)
    }

    @Test
    fun baseNoiseLevel_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.baseNoiseLevel)

        // set new value
        val baseNoiseLevel = randomizer.nextDouble()
        builder.baseNoiseLevel = baseNoiseLevel

        // check
        assertEquals(baseNoiseLevel, builder.baseNoiseLevel)

        // set new value
        builder.baseNoiseLevel = null

        // check
        assertNull(builder.baseNoiseLevel)
    }

    @Test
    fun qualityScoreMapper_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyMagneticFluxDensity>()
        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNotNull(builder.qualityScoreMapper)

        // set new value
        val qualityScoreMapper = DefaultMagnetometerQualityScoreMapper()
        builder.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, builder.qualityScoreMapper)
    }

    @Test
    fun build_whenNonRobustGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenNonRobustGroundTruthHardIronSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenNonRobustGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as KnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenNonRobustAndGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as KnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenRANSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test
    fun build_whenMSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenMSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenMSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenMSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenMSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenMSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenLMedSGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = StandardDeviationBodyMagneticFluxDensity()
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronNotSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronNotSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = true,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronSetAndNoCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronSetAndCommonAxis_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialHardIronX = randomizer.nextDouble()
        val initialHardIronY = randomizer.nextDouble()
        val initialHardIronZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = true,
            initialHardIronX = initialHardIronX,
            initialHardIronY = initialHardIronY,
            initialHardIronZ = initialHardIronZ,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthHardIronSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy,
            baseNoiseLevel = baseNoiseLevel
        )

        val internalCalibrator = builder.build()

        val internalCalibrator2 =
            internalCalibrator as PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator
        assertEquals(
            groundTruthMagneticFluxDensityNorm,
            internalCalibrator2.groundTruthMagneticFluxDensityNorm,
            ABSOLUTE_ERROR
        )
        assertSame(measurements, internalCalibrator2.measurements)
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
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROMedSGroundTruthHardIronNotSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = false,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROMedSGroundTruthHardIronSetNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val magneticFluxDensityStandardDeviation = randomizer.nextDouble()
        val measurement = mockk<StandardDeviationBodyMagneticFluxDensity>()
        every { measurement.magneticFluxDensityStandardDeviation }.returns(
            magneticFluxDensityStandardDeviation
        )
        val measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val groundTruthMagneticFluxDensityNorm = randomizer.nextDouble()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = MagnetometerInternalCalibratorBuilder(
            measurements,
            groundTruthMagneticFluxDensityNorm,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialHardIron = true,
            isCommonAxisUsed = false,
            initialSx = initialSx,
            initialSy = initialSy,
            initialSz = initialSz,
            initialMxy = initialMxy,
            initialMxz = initialMxz,
            initialMyx = initialMyx,
            initialMyz = initialMyz,
            initialMzx = initialMzx,
            initialMzy = initialMzy
        )

        builder.build()
    }

    private companion object {
        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 15

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

        const val ABSOLUTE_ERROR = 1e-6
    }
}