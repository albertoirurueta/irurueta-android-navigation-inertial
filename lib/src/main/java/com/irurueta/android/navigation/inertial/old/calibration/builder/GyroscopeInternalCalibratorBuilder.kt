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
package com.irurueta.android.navigation.inertial.old.calibration.builder

import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.old.calibration.StaticIntervalGyroscopeCalibrator
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.gyroscope.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod

/**
 * Builds a gyroscope calibrator to be used internally by other calibrators.
 *
 * @property measurements List of gyroscope measurements.
 * @property robustMethod Indicates robust method used to solve gyroscope calibration.
 * @property isGroundTruthInitialBias Indicates whether gyroscope initial bias is considered a
 * ground-truth known bias.
 * @property isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
 * magnetometer and gyroscope.
 * @property initialBiasX x-coordinate of gyroscope bias used as an initial guess and expressed in
 * radians per second (rad/s).
 * @property initialBiasY y-coordinate of gyroscope bias used as an initial guess and expressed in
 * radians per second (rad/s).
 * @property initialBiasZ z-coordinate of gyroscope bias used as an initial guess and expressed in
 * radians per second (rad/s).
 * @property initialSx initial x scaling factor for gyroscope calibration.
 * @property initialSy initial y scaling factor for gyroscope calibration.
 * @property initialSz initial z scaling factor for gyroscope calibration.
 * @property initialMxy initial x-y cross coupling error for gyroscope calibration.
 * @property initialMxz initial x-z cross coupling error for gyroscope calibration.
 * @property initialMyx initial y-x cross coupling error for gyroscope calibration.
 * @property initialMyz initial y-z cross coupling error for gyroscope calibration.
 * @property initialMzx initial z-x cross coupling error for gyroscope calibration.
 * @property initialMzy initial z-y cross coupling error for gyroscope calibration.
 * @property isGDependentCrossBiasesEstimated Indicates whether G-dependent cross biases are being
 * estimated or not.
 * @property accelerometerBiasX x-coordinate of estimated accelerometer bias expressed in meters per
 * squared second (m/s^2).
 * @property accelerometerBiasY y-coordinate of estimated accelerometer bias expressed in meters per
 * squared second (m/s^2).
 * @property accelerometerBiasZ z-coordinate of estimated accelerometer bias expressed in meters per
 * squared second (m/s^2).
 * @property accelerometerSx accelerometer initial x scaling factor.
 * @property accelerometerSy accelerometer initial y scaling factor.
 * @property accelerometerSz accelerometer initial z scaling factor.
 * @property accelerometerMxy accelerometer x-y cross coupling error.
 * @property accelerometerMxz accelerometer x-z cross coupling error.
 * @property accelerometerMyx accelerometer y-x cross coupling error.
 * @property accelerometerMyz accelerometer y-z cross coupling error.
 * @property accelerometerMzx accelerometer z-x cross coupling error.
 * @property accelerometerMzy accelerometer z-y cross coupling error.
 * @property baseNoiseLevel gyroscope measurement base noise level that has been detected during
 * initialization expressed in radians per second (rad/s).
 * @property qualityScoreMapper mapper to convert collected gyroscope measurements into quality
 * scores, based on the amount of standard deviation (the larger the variability, the worse the
 * score will be).
 */
class GyroscopeInternalCalibratorBuilder private constructor(
    var measurements: List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>,
    var robustMethod: RobustEstimatorMethod?,
    var isGroundTruthInitialBias: Boolean,
    var isCommonAxisUsed: Boolean,
    var initialBiasX: Double?,
    var initialBiasY: Double?,
    var initialBiasZ: Double?,
    var initialSx: Double,
    var initialSy: Double,
    var initialSz: Double,
    var initialMxy: Double,
    var initialMxz: Double,
    var initialMyx: Double,
    var initialMyz: Double,
    var initialMzx: Double,
    var initialMzy: Double,
    var isGDependentCrossBiasesEstimated: Boolean,
    var accelerometerBiasX: Double? = null,
    var accelerometerBiasY: Double? = null,
    var accelerometerBiasZ: Double? = null,
    var accelerometerSx: Double = 0.0,
    var accelerometerSy: Double = 0.0,
    var accelerometerSz: Double = 0.0,
    var accelerometerMxy: Double = 0.0,
    var accelerometerMxz: Double = 0.0,
    var accelerometerMyx: Double = 0.0,
    var accelerometerMyz: Double = 0.0,
    var accelerometerMzx: Double = 0.0,
    var accelerometerMzy: Double = 0.0,
    var baseNoiseLevel: Double? = null,
    var qualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
) {
    /**
     * Constructor.
     *
     * @param measurements List of gyroscope measurements.
     * @param robustPreliminarySubsetSize Size of preliminary subsets picked while
     * finding a robust gyroscope calibration solution.
     * @param minimumRequiredMeasurements Minimum number of required measurements to
     * start gyroscope calibration.
     * @param robustMethod Indicates robust method used to solve gyroscope calibration.
     * @param robustConfidence Confidence of estimated gyroscope calibration result
     * expressed as a value between 0.0 and 1.0.
     * @param robustMaxIterations Maximum number of iterations to attempt to find a
     * robust gyroscope calibration solution. By default this is 5000.
     * @param robustThreshold Threshold to be used to determine whether a measurement is
     * considered an outlier by robust gyroscope calibration algorithms or not.
     * @param robustThresholdFactor Factor to be used respect estimated gyroscope
     * base noise level to consider a measurement an outlier when using robust calibration methods.
     * @param robustStopThresholdFactor Additional factor to be taken into account for
     * robust methods based on LMedS or PROMedS, where factor is not directly related to LMSE, but
     * to a smaller value.
     * @param isGroundTruthInitialBias Indicates whether gyroscope initial bias is considered a
     * ground-truth known bias.
     * @param isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
     * magnetometer and gyroscope.
     * @param initialBiasX x-coordinate of gyroscope bias used as an initial guess and expressed
     * in radians per second (rad/s).
     * @param initialBiasY y-coordinate of gyroscope bias used as an initial guess and expressed
     * in radians per second (rad/s).
     * @param initialBiasZ z-coordinate of gyroscope bias used as an initial guess and expressed
     * in radians per second (rad/s).
     * @param initialSx initial x scaling factor for gyroscope calibration.
     * @param initialSy initial y scaling factor for gyroscope calibration.
     * @param initialSz initial z scaling factor for gyroscope calibration.
     * @param initialMxy initial x-y cross coupling error for gyroscope calibration.
     * @param initialMxz initial x-z cross coupling error for gyroscope calibration.
     * @param initialMyx initial y-x cross coupling error for gyroscope calibration.
     * @param initialMyz initial y-z cross coupling error for gyroscope calibration.
     * @param initialMzx initial z-x cross coupling error for gyroscope calibration.
     * @param initialMzy initial z-y cross coupling error for gyroscope calibration.
     * @param isGDependentCrossBiasesEstimated Indicates whether G-dependent cross biases are being
     * estimated or not.
     * @param gyroscopeInitialGg Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * @param accelerometerBiasX x-coordinate of estimated accelerometer bias expressed in meters
     * per squared second (m/s^2).
     * @param accelerometerBiasY y-coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     * @param accelerometerBiasZ z-coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     * @param accelerometerSx accelerometer initial x scaling factor.
     * @param accelerometerSy accelerometer initial y scaling factor.
     * @param accelerometerSz accelerometer initial z scaling factor.
     * @param accelerometerMxy accelerometer x-y cross coupling error.
     * @param accelerometerMxz accelerometer x-z cross coupling error.
     * @param accelerometerMyx accelerometer y-x cross coupling error.
     * @param accelerometerMyz accelerometer y-z cross coupling error.
     * @param accelerometerMzx accelerometer z-x cross coupling error.
     * @param accelerometerMzy accelerometer z-y cross coupling error.
     * @param baseNoiseLevel gyroscope measurement base noise level that has been detected during
     * initialization expressed in radians per second (rad/s).
     * @param qualityScoreMapper mapper to convert collected gyroscope measurements into quality
     * scores, based on the amount of standard deviation (the larger the variability, the worse the
     * score will be).
     *
     * @throws IllegalArgumentException if any of provided parameters is invalid.
     */
    @Throws(IllegalArgumentException::class)
    constructor(
        measurements: List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>,
        robustPreliminarySubsetSize: Int,
        minimumRequiredMeasurements: Int,
        robustMethod: RobustEstimatorMethod? = null,
        robustConfidence: Double = StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE,
        robustMaxIterations: Int = StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
        robustThreshold: Double? = null,
        robustThresholdFactor: Double = StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
        robustStopThresholdFactor: Double = StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
        isGroundTruthInitialBias: Boolean = false,
        isCommonAxisUsed: Boolean = false,
        initialBiasX: Double? = null,
        initialBiasY: Double? = null,
        initialBiasZ: Double? = null,
        initialSx: Double = 0.0,
        initialSy: Double = 0.0,
        initialSz: Double = 0.0,
        initialMxy: Double = 0.0,
        initialMxz: Double = 0.0,
        initialMyx: Double = 0.0,
        initialMyz: Double = 0.0,
        initialMzx: Double = 0.0,
        initialMzy: Double = 0.0,
        isGDependentCrossBiasesEstimated: Boolean = false,
        gyroscopeInitialGg: Matrix = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS),
        accelerometerBiasX: Double? = null,
        accelerometerBiasY: Double? = null,
        accelerometerBiasZ: Double? = null,
        accelerometerSx: Double = 0.0,
        accelerometerSy: Double = 0.0,
        accelerometerSz: Double = 0.0,
        accelerometerMxy: Double = 0.0,
        accelerometerMxz: Double = 0.0,
        accelerometerMyx: Double = 0.0,
        accelerometerMyz: Double = 0.0,
        accelerometerMzx: Double = 0.0,
        accelerometerMzy: Double = 0.0,
        baseNoiseLevel: Double? = null,
        qualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> = DefaultGyroscopeQualityScoreMapper()
    ) : this(
        measurements,
        robustMethod,
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
    ) {
        this.robustPreliminarySubsetSize = robustPreliminarySubsetSize
        this.minimumRequiredMeasurements = minimumRequiredMeasurements
        this.robustConfidence = robustConfidence
        this.robustMaxIterations = robustMaxIterations
        this.robustThreshold = robustThreshold
        this.robustThresholdFactor = robustThresholdFactor
        this.robustStopThresholdFactor = robustStopThresholdFactor
        this.gyroscopeInitialGg = gyroscopeInitialGg
    }

    /**
     * Size of preliminary subsets picked while finding a robust gyroscope calibration solution.
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is less than 0
     */
    var robustPreliminarySubsetSize: Int = 0
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0)
            field = value
        }

    /**
     * Minimum number of required measurements to start gyroscope calibration.
     *
     * @throws IllegalArgumentException if provided value is less than 0
     */
    var minimumRequiredMeasurements: Int = 0
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0)
            field = value
        }

    /**
     * Confidence of estimated gyroscope calibration result expressed as a value between 0.0
     * and 1.0.
     * By default 99% of confidence is used, which indicates that with a probability of 99%
     * estimation will be accurate because chosen sub-samples will be inliers (in other terms,
     * outliers will be correctly discarded).
     * This property is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0 (both
     * included).
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustConfidence: Double =
        StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust gyroscope calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [robustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustMaxIterations: Int =
        StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * gyroscope calibration algorithms or not.
     * Threshold varies depending on chosen [robustMethod].
     * By default, if null is provided, the estimated [baseNoiseLevel] will be used to
     * determine a suitable threshold. Otherwise, if a value is provided, such value will be used
     * instead.
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustThreshold: Double? = null
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value == null || value > 0.0)
            field = value
        }

    /**
     * Factor to be used respect estimated gyroscope base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [baseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustThresholdFactor: Double =
        StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to gyroscope calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustStopThresholdFactor: Double =
        StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Gets or sets initial G-dependent cross biases introduced on the gyroscope by the specific
     * forces sensed by the accelerometer.
     *
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var gyroscopeInitialGg: Matrix = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            field = value
        }

    /**
     * Builds an internal gyroscope calibrator based on all provided parameters.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    fun build(): GyroscopeNonLinearCalibrator {
        return if (robustMethod == null) {
            buildNonRobustCalibrator()
        } else {
            buildRobustCalibrator()
        }
    }

    /**
     * Builds a non-robust gyroscope calibrator based on all provided parameters.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildNonRobustCalibrator(): GyroscopeNonLinearCalibrator {
        return if (isGroundTruthInitialBias) {
            val result = KnownBiasEasyGyroscopeCalibrator()
            result.sequences = measurements
            result.isCommonAxisUsed = isCommonAxisUsed
            result.setBiasCoordinates(
                initialBiasX ?: 0.0,
                initialBiasY ?: 0.0,
                initialBiasZ ?: 0.0
            )
            result.setInitialScalingFactorsAndCrossCouplingErrors(
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
            result.isGDependentCrossBiasesEstimated = isGDependentCrossBiasesEstimated
            result.initialGg = gyroscopeInitialGg
            result.setAccelerometerBias(
                accelerometerBiasX ?: 0.0,
                accelerometerBiasY ?: 0.0,
                accelerometerBiasZ ?: 0.0
            )
            result.accelerometerSx = accelerometerSx
            result.accelerometerSy = accelerometerSy
            result.accelerometerSz = accelerometerSz
            result.accelerometerMxy = accelerometerMxy
            result.accelerometerMxz = accelerometerMxz
            result.accelerometerMyx = accelerometerMyx
            result.accelerometerMyz = accelerometerMyz
            result.accelerometerMzx = accelerometerMzx
            result.accelerometerMzy = accelerometerMzy
            result
        } else {
            val result = EasyGyroscopeCalibrator()
            result.sequences = measurements
            result.isCommonAxisUsed = isCommonAxisUsed
            result.setInitialBias(
                initialBiasX ?: 0.0,
                initialBiasY ?: 0.0,
                initialBiasZ ?: 0.0
            )
            result.setInitialScalingFactorsAndCrossCouplingErrors(
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
            result.isGDependentCrossBiasesEstimated = isGDependentCrossBiasesEstimated
            result.initialGg = gyroscopeInitialGg
            result.setAccelerometerBias(
                accelerometerBiasX ?: 0.0,
                accelerometerBiasY ?: 0.0,
                accelerometerBiasZ ?: 0.0
            )
            result.accelerometerSx = accelerometerSx
            result.accelerometerSy = accelerometerSy
            result.accelerometerSz = accelerometerSz
            result.accelerometerMxy = accelerometerMxy
            result.accelerometerMxz = accelerometerMxz
            result.accelerometerMyx = accelerometerMyx
            result.accelerometerMyz = accelerometerMyz
            result.accelerometerMzx = accelerometerMzx
            result.accelerometerMzy = accelerometerMzy
            result
        }
    }

    /**
     * Internally builds a robust gyroscope calibrator based on all provided parameters.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustCalibrator(): GyroscopeNonLinearCalibrator {
        return if (isGroundTruthInitialBias) {
            buildKnownBiasRobustCalibrator()
        } else {
            buildUnknownBiasRobustCalibrator()
        }
    }

    /**
     * Internally builds a robust gyroscope calibrator when bias is known.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildKnownBiasRobustCalibrator(): GyroscopeNonLinearCalibrator {
        val baseNoiseLevel = this.baseNoiseLevel
        val robustThreshold = this.robustThreshold

        val result = RobustKnownBiasEasyGyroscopeCalibrator.create(robustMethod)
        result.sequences = measurements
        result.isCommonAxisUsed = isCommonAxisUsed
        result.setBiasCoordinates(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
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
        result.isGDependentCrossBiasesEstimated = isGDependentCrossBiasesEstimated
        result.initialGg = gyroscopeInitialGg
        result.setAccelerometerBias(
            accelerometerBiasX ?: 0.0,
            accelerometerBiasY ?: 0.0,
            accelerometerBiasZ ?: 0.0
        )
        result.accelerometerSx = accelerometerSx
        result.accelerometerSy = accelerometerSy
        result.accelerometerSz = accelerometerSz
        result.accelerometerMxy = accelerometerMxy
        result.accelerometerMxz = accelerometerMxz
        result.accelerometerMyx = accelerometerMyx
        result.accelerometerMyz = accelerometerMyz
        result.accelerometerMzx = accelerometerMzx
        result.accelerometerMzy = accelerometerMzy
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold and quality scores
        when (result) {
            is RANSACRobustKnownBiasEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownBiasEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownBiasEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildGyroscopeQualityScores()
            }
            is LMedSRobustKnownBiasEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownBiasEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildGyroscopeQualityScores()
            }
        }
        return result
    }

    /**
     * Internally builds a robust gyroscope calibrator when bias is unknown.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildUnknownBiasRobustCalibrator(): GyroscopeNonLinearCalibrator {
        val baseNoiseLevel = this.baseNoiseLevel
        val robustThreshold = this.robustThreshold

        val result = RobustEasyGyroscopeCalibrator.create(robustMethod)
        result.sequences = measurements
        result.isCommonAxisUsed = isCommonAxisUsed
        result.setInitialBias(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
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
        result.isGDependentCrossBiasesEstimated = isGDependentCrossBiasesEstimated
        result.initialGg = gyroscopeInitialGg
        result.setAccelerometerBias(
            accelerometerBiasX ?: 0.0,
            accelerometerBiasY ?: 0.0,
            accelerometerBiasZ ?: 0.0
        )
        result.accelerometerSx = accelerometerSx
        result.accelerometerSy = accelerometerSy
        result.accelerometerSz = accelerometerSz
        result.accelerometerMxy = accelerometerMxy
        result.accelerometerMxz = accelerometerMxz
        result.accelerometerMyx = accelerometerMyx
        result.accelerometerMyz = accelerometerMyz
        result.accelerometerMzx = accelerometerMzx
        result.accelerometerMzy = accelerometerMzy
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold and quality scores
        when (result) {
            is RANSACRobustEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildGyroscopeQualityScores()
            }
            is LMedSRobustEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustEasyGyroscopeCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildGyroscopeQualityScores()
            }
        }
        return result
    }

    /**
     * Builds required quality scores for PROSAC and PROMedS robust methods used for gyroscope
     * calibration.
     * Quality scores are build for each measurement. By default the standard deviation
     * of each measurement is taken into account, so that the larger the standard deviation
     * the poorer the measurement is considered (lower score).
     *
     * @return build quality score array.
     */
    private fun buildGyroscopeQualityScores(): DoubleArray {
        val size = measurements.size
        val qualityScores = DoubleArray(size)
        measurements.forEachIndexed { index, measurement ->
            qualityScores[index] = qualityScoreMapper.map(measurement)
        }
        return qualityScores
    }
}