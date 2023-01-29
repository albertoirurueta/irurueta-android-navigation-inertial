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

import android.location.Location
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.every
import io.mockk.mockk
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.max

@RunWith(RobolectricTestRunner::class)
class AccelerometerInternalCalibratorBuilderTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredValues_setsExpectedValues() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check
        assertSame(measurements, builder.measurements)
        assertEquals(robustPreliminarySubsetSize, builder.robustPreliminarySubsetSize)
        assertEquals(minimumRequiredMeasurements, builder.minimumRequiredMeasurements)
        assertNull(builder.robustMethod)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )
        assertNull(builder.robustThreshold)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )
        assertNull(builder.location)
        assertNull(builder.gravityNorm)
        assertFalse(builder.isGroundTruthInitialBias)
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )
        assertNull(builder.initialBiasX)
        assertNull(builder.initialBiasY)
        assertNull(builder.initialBiasZ)
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val robustMethod = RobustEstimatorMethod.RANSAC
        val robustConfidence = ROBUST_CONFIDENCE
        val robustMaxIterations = ROBUST_MAX_ITERATIONS
        val robustThreshold = ROBUST_THRESHOLD
        val robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        val robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
        val location = getLocation()
        val gravityNorm = randomizer.nextDouble()
        val isGroundTruthInitialBias = true
        val isCommonAxisUsed = true
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
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
        val qualityScoreMapper = DefaultAccelerometerQualityScoreMapper()
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod,
            robustConfidence,
            robustMaxIterations,
            robustThreshold,
            robustThresholdFactor,
            robustStopThresholdFactor,
            location,
            gravityNorm,
            isGroundTruthInitialBias,
            isCommonAxisUsed,
            initialBiasX,
            initialBiasY,
            initialBiasZ,
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
        assertEquals(robustPreliminarySubsetSize, builder.robustPreliminarySubsetSize)
        assertEquals(minimumRequiredMeasurements, builder.minimumRequiredMeasurements)
        assertEquals(robustMethod, builder.robustMethod)
        assertEquals(robustConfidence, builder.robustConfidence, 0.0)
        assertEquals(robustMaxIterations, builder.robustMaxIterations)
        assertEquals(robustThreshold, builder.robustThreshold)
        assertEquals(robustThresholdFactor, builder.robustThresholdFactor, 0.0)
        assertEquals(robustStopThresholdFactor, builder.robustStopThresholdFactor, 0.0)
        assertSame(location, builder.location)
        assertEquals(gravityNorm, builder.gravityNorm)
        assertTrue(builder.isGroundTruthInitialBias)
        assertTrue(builder.isCommonAxisUsed)
        assertEquals(initialBiasX, builder.initialBiasX)
        assertEquals(initialBiasY, builder.initialBiasY)
        assertEquals(initialBiasZ, builder.initialBiasZ)
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

        val measurements1 = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements1,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertSame(measurements1, builder.measurements)

        // set new value
        val measurements2 = emptyList<StandardDeviationBodyKinematics>()

        // check
        assertSame(measurements2, builder.measurements)
    }

    @Test
    fun robustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize1 = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize1, 2 * robustPreliminarySubsetSize1)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = 0
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)

        AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun minimumRequiredMeasurements_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements1 =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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
        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = -1

        AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun robustMethod_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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
        @Suppress("KotlinConstantConditions")
        assertNull(builder.robustMethod)
    }

    @Test
    fun robustConfidence_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenUpperBoundExceeded_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = 2.0
    }

    @Test
    fun robustMaxIterations_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )

        // set new value
        builder.robustMaxIterations = 0
    }

    @Test
    fun robustThreshold_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )

        // set new value
        builder.robustThresholdFactor = 0.0
    }

    @Test
    fun robustStopThresholdFactor_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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
        builder.robustStopThresholdFactor = 0.0
    }

    @Test
    fun location_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.location)

        // set new value
        val location = getLocation()
        builder.location = location

        // check
        assertSame(location, builder.location)

        // set new value
        builder.location = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.location)
    }

    @Test
    fun gravityNorm_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.gravityNorm)

        // set new value
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        builder.gravityNorm = gravityNorm

        // check
        assertEquals(gravityNorm, builder.gravityNorm)

        // set new value
        builder.gravityNorm = null

        // check
        assertNull(builder.gravityNorm)
    }

    @Test(expected = IllegalArgumentException::class)
    fun gravityNorm_whenInvalid_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.gravityNorm)

        // set new value
        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        builder.gravityNorm = -gravityNorm
    }

    @Test
    fun isGroundTruthInitialBias_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertFalse(builder.isGroundTruthInitialBias)

        // set new value
        builder.isGroundTruthInitialBias = true

        // check
        @Suppress("KotlinConstantConditions")
        assertTrue(builder.isGroundTruthInitialBias)
    }

    @Test
    fun isCommonAxisUsed_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )

        // set new value
        builder.isCommonAxisUsed = !StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS

        // check
        assertEquals(
            !StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )
    }

    @Test
    fun initialBiasX_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialBiasX)

        // set new value
        val initialBiasX = randomizer.nextDouble()
        builder.initialBiasX = initialBiasX

        // check
        assertEquals(initialBiasX, builder.initialBiasX)

        // set new value
        builder.initialBiasX = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.initialBiasX)
    }

    @Test
    fun initialBiasY_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialBiasY)

        // set new value
        val initialBiasY = randomizer.nextDouble()
        builder.initialBiasY = initialBiasY

        // check
        assertEquals(initialBiasY, builder.initialBiasY)

        // set new value
        builder.initialBiasY = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.initialBiasY)
    }

    @Test
    fun initialBiasZ_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.initialBiasZ)

        // set new value
        val initialBiasZ = randomizer.nextDouble()
        builder.initialBiasZ = initialBiasZ

        // check
        assertEquals(initialBiasZ, builder.initialBiasZ)

        // set new value
        builder.initialBiasZ = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.initialBiasZ)
    }

    @Test
    fun initialSx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
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
        @Suppress("KotlinConstantConditions")
        assertNull(builder.baseNoiseLevel)
    }

    @Test
    fun qualityScoreMapper_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<StandardDeviationBodyKinematics>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNotNull(builder.qualityScoreMapper)

        // set new value
        val qualityScoreMapper = DefaultAccelerometerQualityScoreMapper()
        builder.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, builder.qualityScoreMapper)
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndPositionBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = true,
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
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndPositionBiasSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
    fun build_whenNonRobustGroundTruthBiasAndPositionBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = true,
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
        val internalCalibrator2 = internalCalibrator as KnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndGravityBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndGravityBiasSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
    fun build_whenNonRobustGroundTruthBiasAndGravityBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as KnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(7, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenNonRobustGroundTruthBiasAndGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test
    fun build_whenNonRobustNoGroundTruthBiasAndPositionBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = false,
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
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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

        assertTrue(internalCalibrator2.isReady)
        assertEquals(13, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenNonRobustNoGroundTruthBiasAndPositionBiasNotAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.initialBiasZ, 0.0)
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
    fun build_whenNonRobustNoGroundTruthBiasAndPositionBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            location = location,
            isGroundTruthInitialBias = false,
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
        val internalCalibrator2 = internalCalibrator as KnownPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
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
    fun build_whenNonRobustNoGroundTruthBiasAndGravityBiasNotSetAndCommonAxisNotUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = false,
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
        val internalCalibrator2 = internalCalibrator as KnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
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
    fun build_whenNonRobustNoGroundTruthBiasAndGravityBiasNotSetAndCommonAxisUsed_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = false,
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
        val internalCalibrator2 = internalCalibrator as KnownGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertTrue(internalCalibrator2.isCommonAxisUsed)
        assertEquals(0.0, internalCalibrator2.initialBiasX, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasY, 0.0)
        assertEquals(0.0, internalCalibrator2.initialBiasZ, 0.0)
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

    @Test(expected = IllegalStateException::class)
    fun build_whenNonRobustNoGroundTruthBiasAndGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = false,
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

        builder.build()
    }

    @Test
    fun build_whenRANSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as RANSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenMSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenMSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenMSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as MSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenPROSACGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROSACRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenLMedSGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as LMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasLocationBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasLocationBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            location = location,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasLocationBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator
        assertTrue(location.toNEDPosition().equals(internalCalibrator2.nedPosition, ABSOLUTE_ERROR))
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
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
    fun build_whenPROMedSGroundTruthBiasLocationMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val location = getLocation()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            location = location,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenRANSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenMSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenMSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenMSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenPROSACGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenLMedSGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(
            ROBUST_THRESHOLD_FACTOR * ROBUST_STOP_THRESHOLD_FACTOR * baseNoiseLevel,
            internalCalibrator2.stopThreshold,
            0.0
        )

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSCGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasGravityBiasNotSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasGravityBiasSetAndRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialBiasX = randomizer.nextDouble()
        val initialBiasY = randomizer.nextDouble()
        val initialBiasZ = randomizer.nextDouble()
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = false,
            initialBiasX = initialBiasX,
            initialBiasY = initialBiasY,
            initialBiasZ = initialBiasZ,
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
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
        assertEquals(initialBiasX, internalCalibrator2.biasX, 0.0)
        assertEquals(initialBiasY, internalCalibrator2.biasY, 0.0)
        assertEquals(initialBiasZ, internalCalibrator2.biasZ, 0.0)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(10, internalCalibrator2.minimumRequiredMeasurements)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasGravityBiasNotSetAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
            internalCalibrator as PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator
        assertEquals(gravityNorm, internalCalibrator2.groundTruthGravityNorm, 0.0)
        assertSame(measurements, internalCalibrator2.measurements)
        assertFalse(internalCalibrator2.isCommonAxisUsed)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
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
    fun build_whenPROMedSCGroundTruthBiasGravityMissingGravityNorm_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
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

        builder.build()
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROMedSGroundTruthBiasGravityMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()
        val specificForceStandardDeviation = randomizer.nextDouble()

        val measurement = mockk<StandardDeviationBodyKinematics>()
        every { measurement.specificForceStandardDeviation }.returns(specificForceStandardDeviation)
        every { measurement.angularRateStandardDeviation }.returns(0.0)
        val measurements = mutableListOf<StandardDeviationBodyKinematics>()
        for (i in 1..13) {
            measurements.add(measurement)
        }

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

        val gravityNorm = GravityHelper.getGravityNormForLocation(getLocation())
        val initialSx = randomizer.nextDouble()
        val initialSy = randomizer.nextDouble()
        val initialSz = randomizer.nextDouble()
        val initialMxy = randomizer.nextDouble()
        val initialMxz = randomizer.nextDouble()
        val initialMyx = randomizer.nextDouble()
        val initialMyz = randomizer.nextDouble()
        val initialMzx = randomizer.nextDouble()
        val initialMzy = randomizer.nextDouble()

        val builder = AccelerometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            gravityNorm = gravityNorm,
            isGroundTruthInitialBias = true,
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
        )

        builder.build()
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 15

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3

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
    }
}