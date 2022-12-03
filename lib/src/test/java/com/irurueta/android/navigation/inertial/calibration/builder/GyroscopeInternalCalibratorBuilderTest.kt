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

import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalGyroscopeCalibrator
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.gyroscope.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.max

@RunWith(RobolectricTestRunner::class)
class GyroscopeInternalCalibratorBuilderTest {

    @Test
    fun constructor_whenRequiredValues_setsExpectedValues() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )
        assertNull(builder.robustThreshold)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )
        assertFalse(builder.isGroundTruthInitialBias)
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES,
            builder.isGDependentCrossBiasesEstimated
        )
        assertEquals(
            Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS),
            builder.gyroscopeInitialGg
        )
        assertNull(builder.accelerometerBiasX)
        assertNull(builder.accelerometerBiasY)
        assertNull(builder.accelerometerBiasZ)
        assertEquals(0.0, builder.accelerometerSx, 0.0)
        assertEquals(0.0, builder.accelerometerSy, 0.0)
        assertEquals(0.0, builder.accelerometerSz, 0.0)
        assertEquals(0.0, builder.accelerometerMxy, 0.0)
        assertEquals(0.0, builder.accelerometerMxz, 0.0)
        assertEquals(0.0, builder.accelerometerMyx, 0.0)
        assertEquals(0.0, builder.accelerometerMyz, 0.0)
        assertEquals(0.0, builder.accelerometerMzx, 0.0)
        assertEquals(0.0, builder.accelerometerMzy, 0.0)
        assertNull(builder.baseNoiseLevel)
        assertNotNull(builder.qualityScoreMapper)
    }

    @Test
    fun constructor_whenAllValues_setsExpectedValues() {
        val randomizer = UniformRandomizer()

        val measurements = emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val robustMethod = RobustEstimatorMethod.RANSAC
        val robustConfidence = ROBUST_CONFIDENCE
        val robustMaxIterations = ROBUST_MAX_ITERATIONS
        val robustThreshold = ROBUST_THRESHOLD
        val robustThresholdFactor = ROBUST_THRESHOLD_FACTOR
        val robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR
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
        val isGDependentCrossBiasesEstimated = true
        val gyroscopeInitialGg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gyroscopeInitialGg)
        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()
        val accelerometerSx = randomizer.nextDouble()
        val accelerometerSy = randomizer.nextDouble()
        val accelerometerSz = randomizer.nextDouble()
        val accelerometerMxy = randomizer.nextDouble()
        val accelerometerMxz = randomizer.nextDouble()
        val accelerometerMyx = randomizer.nextDouble()
        val accelerometerMyz = randomizer.nextDouble()
        val accelerometerMzx = randomizer.nextDouble()
        val accelerometerMzy = randomizer.nextDouble()
        val baseNoiseLevel = randomizer.nextDouble()
        val qualityScoreMapper = DefaultGyroscopeQualityScoreMapper()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod,
            robustConfidence,
            robustMaxIterations,
            robustThreshold,
            robustThresholdFactor,
            robustStopThresholdFactor,
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
            isGDependentCrossBiasesEstimated,
            gyroscopeInitialGg,
            accelerometerBiasX,
            accelerometerBiasY,
            accelerometerBiasZ,
            accelerometerSx,
            accelerometerSy,
            accelerometerSz,
            accelerometerMxy,
            accelerometerMxz,
            accelerometerMyx,
            accelerometerMyz,
            accelerometerMzx,
            accelerometerMzy,
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
        assertEquals(isGroundTruthInitialBias, builder.isGroundTruthInitialBias)
        assertEquals(isCommonAxisUsed, builder.isCommonAxisUsed)
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
        assertEquals(isGDependentCrossBiasesEstimated, builder.isGDependentCrossBiasesEstimated)
        assertEquals(gyroscopeInitialGg, builder.gyroscopeInitialGg)
        assertEquals(accelerometerBiasX, builder.accelerometerBiasX)
        assertEquals(accelerometerBiasY, builder.accelerometerBiasY)
        assertEquals(accelerometerBiasZ, builder.accelerometerBiasZ)
        assertEquals(accelerometerSx, builder.accelerometerSx, 0.0)
        assertEquals(accelerometerSy, builder.accelerometerSy, 0.0)
        assertEquals(accelerometerSz, builder.accelerometerSz, 0.0)
        assertEquals(accelerometerMxy, builder.accelerometerMxy, 0.0)
        assertEquals(accelerometerMxz, builder.accelerometerMxz, 0.0)
        assertEquals(accelerometerMyx, builder.accelerometerMyx, 0.0)
        assertEquals(accelerometerMyz, builder.accelerometerMyz, 0.0)
        assertEquals(accelerometerMzx, builder.accelerometerMzx, 0.0)
        assertEquals(accelerometerMzy, builder.accelerometerMzy, 0.0)
        assertEquals(baseNoiseLevel, builder.baseNoiseLevel)
        assertSame(qualityScoreMapper, builder.qualityScoreMapper)
    }

    @Test
    fun measurements_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements1 =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements1,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertSame(measurements1, builder.measurements)

        // set new value
        val measurements2 =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()

        // check
        assertSame(measurements2, builder.measurements)
    }

    @Test
    fun robustPreliminarySubsetSize_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize1 = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize1, 2 * robustPreliminarySubsetSize1)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = 0
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)

        GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun minimumRequiredMeasurements_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements1 =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements1
        )

        // check default value
        assertEquals(minimumRequiredMeasurements1, builder.minimumRequiredMeasurements)

        // set new value
        val minimumRequiredMeasurements2 = minimumRequiredMeasurements1 + 1
        builder.minimumRequiredMeasurements = minimumRequiredMeasurements2

        // check
        assertEquals(minimumRequiredMeasurements2, builder.minimumRequiredMeasurements)
    }

    @Test(expected = IllegalArgumentException::class)
    fun minimumRequiredMeasurements_whenInvalid_throwsIllegalArgumentException() {
        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = -1

        GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )
    }

    @Test
    fun robustMethod_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = -1.0
    }

    @Test(expected = IllegalArgumentException::class)
    fun robustConfidence_whenUpperBoundExceeded_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
            builder.robustConfidence,
            0.0
        )

        // set new value
        builder.robustConfidence = 2.0
    }

    @Test
    fun robustMaxIterations_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
            builder.robustMaxIterations
        )

        // set new value
        builder.robustMaxIterations = 0
    }

    @Test
    fun robustThreshold_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
            builder.robustThresholdFactor,
            0.0
        )

        // set new value
        builder.robustThresholdFactor = 0.0
    }

    @Test
    fun robustStopThresholdFactor_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
            builder.robustStopThresholdFactor,
            0.0
        )

        // set new value
        builder.robustStopThresholdFactor = 0.0
    }

    @Test
    fun isGroundTruthInitialBias_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )

        // set new value
        builder.isCommonAxisUsed = !StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS

        // check
        assertEquals(
            !StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
            builder.isCommonAxisUsed
        )
    }

    @Test
    fun initialBiasX_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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
    fun isGDependentCrossBiasesEstimated_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES,
            builder.isGDependentCrossBiasesEstimated
        )

        // set new value
        builder.isGDependentCrossBiasesEstimated =
            !StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES

        // check
        assertEquals(
            !StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES,
            builder.isGDependentCrossBiasesEstimated
        )
    }

    @Test
    fun gyroscopeInitialGg_whenValid_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(
            Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS),
            builder.gyroscopeInitialGg
        )

        // set new value
        val gyroscopeInitialGg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gyroscopeInitialGg)
        builder.gyroscopeInitialGg = gyroscopeInitialGg

        // check
        assertSame(gyroscopeInitialGg, builder.gyroscopeInitialGg)
    }

    @Test
    fun accelerometerBiasX_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.accelerometerBiasX)

        // set new value
        val accelerometerBiasX = randomizer.nextDouble()
        builder.accelerometerBiasX = accelerometerBiasX

        // check
        assertEquals(accelerometerBiasX, builder.accelerometerBiasX)

        // set new value
        builder.accelerometerBiasX = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.accelerometerBiasX)
    }

    @Test
    fun accelerometerBiasY_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.accelerometerBiasY)

        // set new value
        val accelerometerBiasY = randomizer.nextDouble()
        builder.accelerometerBiasY = accelerometerBiasY

        // check
        assertEquals(accelerometerBiasY, builder.accelerometerBiasY)

        // set new value
        builder.accelerometerBiasY = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.accelerometerBiasY)
    }

    @Test
    fun accelerometerBiasZ_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNull(builder.accelerometerBiasZ)

        // set new value
        val accelerometerBiasZ = randomizer.nextDouble()
        builder.accelerometerBiasZ = accelerometerBiasZ

        // check
        assertEquals(accelerometerBiasZ, builder.accelerometerBiasZ)

        // set new value
        builder.accelerometerBiasZ = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(builder.accelerometerBiasZ)
    }

    @Test
    fun accelerometerSx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerSx, 0.0)

        // set new value
        val accelerometerSx = randomizer.nextDouble()
        builder.accelerometerSx = accelerometerSx

        // check
        assertEquals(accelerometerSx, builder.accelerometerSx, 0.0)
    }

    @Test
    fun accelerometerSy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerSy, 0.0)

        // set new value
        val accelerometerSy = randomizer.nextDouble()
        builder.accelerometerSy = accelerometerSy

        // check
        assertEquals(accelerometerSy, builder.accelerometerSy, 0.0)
    }

    @Test
    fun accelerometerSz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerSz, 0.0)

        // set new value
        val accelerometerSz = randomizer.nextDouble()
        builder.accelerometerSz = accelerometerSz

        // check
        assertEquals(accelerometerSz, builder.accelerometerSz, 0.0)
    }

    @Test
    fun accelerometerMxy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMxy, 0.0)

        // set new value
        val accelerometerMxy = randomizer.nextDouble()
        builder.accelerometerMxy = accelerometerMxy

        // check
        assertEquals(accelerometerMxy, builder.accelerometerMxy, 0.0)
    }

    @Test
    fun accelerometerMxz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMxz, 0.0)

        // set new value
        val accelerometerMxz = randomizer.nextDouble()
        builder.accelerometerMxz = accelerometerMxz

        // check
        assertEquals(accelerometerMxz, builder.accelerometerMxz, 0.0)
    }

    @Test
    fun accelerometerMyx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMyx, 0.0)

        // set new value
        val accelerometerMyx = randomizer.nextDouble()
        builder.accelerometerMyx = accelerometerMyx

        // check
        assertEquals(accelerometerMyx, builder.accelerometerMyx, 0.0)
    }

    @Test
    fun accelerometerMyz_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMyz, 0.0)

        // set new value
        val accelerometerMyz = randomizer.nextDouble()
        builder.accelerometerMyz = accelerometerMyz

        // check
        assertEquals(accelerometerMyz, builder.accelerometerMyz, 0.0)
    }

    @Test
    fun accelerometerMzx_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMzx, 0.0)

        // set new value
        val accelerometerMzx = randomizer.nextDouble()
        builder.accelerometerMzx = accelerometerMzx

        // check
        assertEquals(accelerometerMzx, builder.accelerometerMzx, 0.0)
    }

    @Test
    fun accelerometerMzy_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertEquals(0.0, builder.accelerometerMzy, 0.0)

        // set new value
        val accelerometerMzy = randomizer.nextDouble()
        builder.accelerometerMzy = accelerometerMzy

        // check
        assertEquals(accelerometerMzy, builder.accelerometerMzy, 0.0)
    }

    @Test
    fun baseNoiseLevel_setsExpectedValue() {
        val randomizer = UniformRandomizer()

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
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

        val measurements =
            emptyList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements =
            randomizer.nextInt(robustPreliminarySubsetSize, 2 * robustPreliminarySubsetSize)
        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements
        )

        // check default value
        assertNotNull(builder.qualityScoreMapper)

        // set new value
        val qualityScoreMapper = DefaultGyroscopeQualityScoreMapper()
        builder.qualityScoreMapper = qualityScoreMapper

        // check
        assertSame(qualityScoreMapper, builder.qualityScoreMapper)
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as KnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
    }

    @Test
    fun build_whenNonRobustGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as KnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
    }

    @Test
    fun build_whenNonRobustNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as EasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
    }

    @Test
    fun build_whenNonRobustNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            robustPreliminarySubsetSize,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as EasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
    }

    @Test
    fun build_whenRANSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenRANSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenRANSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenRANSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as RANSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenRANSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as RANSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenRANSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.RANSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
    fun build_whenMSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenMSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenMSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenMSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenMSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as MSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenMSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as MSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenMSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.MSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
    fun build_whenPROSACGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROSACNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROSACNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROSACGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROSACRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROSACNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROSACRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD_FACTOR * baseNoiseLevel, internalCalibrator2.threshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROSACNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROSAC,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
    fun build_whenLMedSGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenLMedSNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenLMedSNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenLMedSGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as LMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenLMedSNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as LMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenLMedSNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.LMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
    fun build_whenPROMedSGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROMedSNoGroundTruthBiasAndNoInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROMedSNoGroundTruthBiasAndInitialBias_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

        val robustPreliminarySubsetSize = ROBUST_PRELIMINARY_SUBSET_SIZE
        val minimumRequiredMeasurements = ROBUST_PRELIMINARY_SUBSET_SIZE + 1

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

        val accelerometerBiasX = randomizer.nextDouble()
        val accelerometerBiasY = randomizer.nextDouble()
        val accelerometerBiasZ = randomizer.nextDouble()

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = ROBUST_THRESHOLD,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
            initialMzy = initialMzy,
            accelerometerBiasX = accelerometerBiasX,
            accelerometerBiasY = accelerometerBiasY,
            accelerometerBiasZ = accelerometerBiasZ
        )

        val internalCalibrator = builder.build()
        val internalCalibrator2 = internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(
            max(minimumRequiredMeasurements, robustPreliminarySubsetSize),
            internalCalibrator2.preliminarySubsetSize
        )
        assertEquals(ROBUST_THRESHOLD, internalCalibrator2.stopThreshold, 0.0)
        assertEquals(measurements.size, internalCalibrator2.qualityScores.size)

        assertTrue(internalCalibrator2.isReady)
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROMedSGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..16) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = true,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROMedSRobustKnownBiasEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(16, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test
    fun build_whenPROMedSNoGroundTruthBiasAndNoRobustThreshold_buildsExpectedCalibrator() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        val internalCalibrator2 = internalCalibrator as PROMedSRobustEasyGyroscopeCalibrator
        assertSame(measurements, internalCalibrator2.sequences)
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
        assertEquals(19, internalCalibrator2.minimumRequiredMeasurementsOrSequences)
    }

    @Test(expected = IllegalStateException::class)
    fun build_whenPROMedSNoGroundTruthBiasNoRobustThresholdAndMissingBaseNoiseLevel_throwsIllegalStateException() {
        val randomizer = UniformRandomizer()

        val measurement = BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>()
        val measurements =
            mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()
        for (i in 1..19) {
            measurements.add(measurement)
        }

        val gg = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        Matrix.fillWithUniformRandomValues(0.0, 1.0, gg)

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

        val builder = GyroscopeInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            minimumRequiredMeasurements,
            robustMethod = RobustEstimatorMethod.PROMedS,
            robustConfidence = ROBUST_CONFIDENCE,
            robustMaxIterations = ROBUST_MAX_ITERATIONS,
            robustThreshold = null,
            robustThresholdFactor = ROBUST_THRESHOLD_FACTOR,
            robustStopThresholdFactor = ROBUST_STOP_THRESHOLD_FACTOR,
            isGroundTruthInitialBias = false,
            isCommonAxisUsed = true,
            isGDependentCrossBiasesEstimated = true,
            gyroscopeInitialGg = gg,
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
        const val ROBUST_CONFIDENCE = 0.9

        const val ROBUST_MAX_ITERATIONS = 1000

        const val ROBUST_PRELIMINARY_SUBSET_SIZE = 19

        const val ROBUST_THRESHOLD = 1e-5

        const val ROBUST_THRESHOLD_FACTOR = 2.0

        const val ROBUST_STOP_THRESHOLD_FACTOR = 1e-3
    }
}