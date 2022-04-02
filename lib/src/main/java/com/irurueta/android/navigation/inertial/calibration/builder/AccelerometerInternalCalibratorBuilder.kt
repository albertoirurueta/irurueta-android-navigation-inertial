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
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalAccelerometerCalibrator
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod

/**
 * Builds an accelerometer calibrator to be used internally by other calibrators.
 *
 * @property measurements List of accelerometer measurements.
 * @property robustMethod Indicates robust method used to solve accelerometer calibration.
 * @property location Location of device when running calibration.
 * @property isGroundTruthInitialBias Indicates whether accelerometer initial bias is considered a
 * ground-truth known bias.
 * @property isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
 * magnetometer and gyroscope.
 * @property initialBiasX x-coordinate of accelerometer bias used as an initial guess and expressed
 * in meters per squared second (m/s^2).
 * @property initialBiasY y-coordinate of accelerometer bias used as an initial guess and expressed
 * in meters per squared second (m/s^2).
 * @property initialBiasZ z-coordinate of accelerometer bias used as an initial guess and expressed
 * in meters per squared second (m/s^2).
 * @property initialSx initial x scaling factor for accelerometer calibration.
 * @property initialSy initial y scaling factor for accelerometer calibration.
 * @property initialSz initial z scaling factor for accelerometer calibration.
 * @property initialMxy initial x-y cross coupling error for accelerometer calibration.
 * @property initialMxz initial x-z cross coupling error for accelerometer calibration.
 * @property initialMyx initial y-x cross coupling error for accelerometer calibration.
 * @property initialMyz initial y-z cross coupling error for accelerometer calibration.
 * @property initialMzx initial z-x cross coupling error for accelerometer calibration.
 * @property initialMzy initial z-y cross coupling error for accelerometer calibration.
 * @property baseNoiseLevel accelerometer measurement base noise level that has been
 * detected during initialization expressed in meters per squared second (m/s^2).
 * @property qualityScoreMapper mapper to convert collected accelerometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class AccelerometerInternalCalibratorBuilder private constructor(
    var measurements: List<StandardDeviationBodyKinematics>,
    var robustMethod: RobustEstimatorMethod?,
    var location: Location?,
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
    var baseNoiseLevel: Double?,
    var qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>
) {

    /**
     * Constructor.
     *
     * @param measurements List of accelerometer measurements.
     * @param robustPreliminarySubsetSize Size of preliminary subsets picked while
     * finding a robust accelerometer calibration solution.
     * @param minimumRequiredMeasurements Minimum number of required measurements to
     * start accelerometer calibration.
     * @param robustMethod Indicates robust method used to solve accelerometer calibration.
     * @param robustConfidence Confidence of estimated accelerometer calibration result
     * expressed as a value between 0.0 and 1.0.
     * @param robustMaxIterations Maximum number of iterations to attempt to find a
     * robust accelerometer calibration solution. By default this is 5000.
     * @param robustThreshold Threshold to be used to determine whether a measurement is
     * considered an outlier by robust accelerometer calibration algorithms or not.
     * @param robustThresholdFactor Factor to be used respect estimated accelerometer
     * base noise level to consider a measurement an outlier when using robust calibration methods.
     * @param robustStopThresholdFactor Additional factor to be taken into account for
     * robust methods based on LMedS or PROMedS, where factor is not directly related to LMSE, but to a
     * smaller value.
     * @param location Location of device when running calibration.
     * @param gravityNorm Contains gravity norm (either obtained by the gravity sensor, or determined
     * by current location using WGS84 Earth model). Expressed in meters per squared second (m/s^2).
     * @param isGroundTruthInitialBias Indicates whether accelerometer initial bias is considered a
     * ground-truth known bias.
     * @param isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
     * magnetometer and gyroscope.
     * @param initialBiasX x-coordinate of accelerometer bias used as an initial guess and expressed
     * in meters per squared second (m/s^2).
     * @param initialBiasY y-coordinate of accelerometer bias used as an initial guess and expressed
     * in meters per squared second (m/s^2).
     * @param initialBiasZ z-coordinate of accelerometer bias used as an initial guess and expressed
     * in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor for accelerometer calibration.
     * @param initialSy initial y scaling factor for accelerometer calibration.
     * @param initialSz initial z scaling factor for accelerometer calibration.
     * @param initialMxy initial x-y cross coupling error for accelerometer calibration.
     * @param initialMxz initial x-z cross coupling error for accelerometer calibration.
     * @param initialMyx initial y-x cross coupling error for accelerometer calibration.
     * @param initialMyz initial y-z cross coupling error for accelerometer calibration.
     * @param initialMzx initial z-x cross coupling error for accelerometer calibration.
     * @param initialMzy initial z-y cross coupling error for accelerometer calibration.
     * @param baseNoiseLevel accelerometer measurement base noise level that has been
     * detected during initialization expressed in meters per squared second (m/s^2).
     * @param qualityScoreMapper mapper to convert collected accelerometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     *
     * @throws IllegalArgumentException if any of provided parameters is invalid.
     */
    @Throws(IllegalArgumentException::class)
    constructor(
        measurements: List<StandardDeviationBodyKinematics>,
        robustPreliminarySubsetSize: Int,
        minimumRequiredMeasurements: Int,
        robustMethod: RobustEstimatorMethod? = null,
        robustConfidence: Double = StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
        robustMaxIterations: Int =
            StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
        robustThreshold: Double? = null,
        robustThresholdFactor: Double =
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
        robustStopThresholdFactor: Double =
            StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
        location: Location? = null,
        gravityNorm: Double? = null,
        isGroundTruthInitialBias: Boolean = false,
        isCommonAxisUsed: Boolean = StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
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
        baseNoiseLevel: Double? = null,
        qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics> =
            DefaultAccelerometerQualityScoreMapper()
    ) : this(
        measurements,
        robustMethod,
        location,
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
    ) {
        this.robustPreliminarySubsetSize = robustPreliminarySubsetSize
        this.minimumRequiredMeasurements = minimumRequiredMeasurements
        this.robustConfidence = robustConfidence
        this.robustMaxIterations = robustMaxIterations
        this.robustThreshold = robustThreshold
        this.robustThresholdFactor = robustThresholdFactor
        this.robustStopThresholdFactor = robustStopThresholdFactor
        this.gravityNorm = gravityNorm
    }

    /**
     * Size of preliminary subsets picked while finding a robust accelerometer calibration solution.
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
     * Minimum number of required measurements to start accelerometer calibration.
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
     * Confidence of estimated accelerometer calibration result expressed as a value between 0.0
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
        StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust accelerometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [robustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustMaxIterations: Int =
        StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * accelerometer calibration algorithms or not.
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
     * Factor to be used respect estimated accelerometer base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [baseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustThresholdFactor: Double =
        StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to accelerometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustStopThresholdFactor: Double =
        StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Contains gravity norm (either obtained by the gravity sensor, or determined by current
     * location using WGS84 Earth model). Expressed in meters per squared second (m/s^2).
     */
    var gravityNorm: Double? = null
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value == null || value >= 0.0)
            field = value
        }

    /**
     * Builds an internal accelerometer calibrator based on all provided parameters.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    fun build(): AccelerometerNonLinearCalibrator {
        return if (robustMethod == null) {
            buildNonRobustCalibrator()
        } else {
            buildRobustCalibrator()
        }
    }

    /**
     * Builds a non-robust accelerometer calibrator based on all provided parameters.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildNonRobustCalibrator(): AccelerometerNonLinearCalibrator {
        val location = this.location
        val gravityNorm = this.gravityNorm
        if (isGroundTruthInitialBias) {
            if (location != null) {
                return KnownBiasAndPositionAccelerometerCalibrator(
                    location.toNEDPosition(),
                    measurements,
                    isCommonAxisUsed,
                    initialBiasX ?: 0.0,
                    initialBiasY ?: 0.0,
                    initialBiasZ ?: 0.0,
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
            } else {
                checkNotNull(gravityNorm)
                return KnownBiasAndGravityNormAccelerometerCalibrator(
                    gravityNorm,
                    measurements,
                    isCommonAxisUsed,
                    initialBiasX ?: 0.0,
                    initialBiasY ?: 0.0,
                    initialBiasZ ?: 0.0,
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
        } else {
            if (location != null) {
                return KnownPositionAccelerometerCalibrator(
                    location.toNEDPosition(),
                    measurements,
                    isCommonAxisUsed,
                    initialBiasX ?: 0.0,
                    initialBiasY ?: 0.0,
                    initialBiasZ ?: 0.0,
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
            } else {
                checkNotNull(gravityNorm)
                return KnownGravityNormAccelerometerCalibrator(
                    gravityNorm,
                    measurements,
                    isCommonAxisUsed,
                    initialBiasX ?: 0.0,
                    initialBiasY ?: 0.0,
                    initialBiasZ ?: 0.0,
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
    }

    /**
     * Internally builds a robust accelerometer calibrator based on all provided parameters.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustCalibrator(): AccelerometerNonLinearCalibrator {
        val location = this.location
        return if (isGroundTruthInitialBias) {
            if (location != null) {
                buildRobustKnownBiasAndPositionCalibrator(location)
            } else {
                buildRobustKnownBiasAndGravityCalibrator()
            }
        } else {
            if (location != null) {
                buildRobustKnownPositionCalibrator(location)
            } else {
                buildRobustKnownGravityCalibrator()
            }
        }
    }

    /**
     * Internally build a robust accelerometer calibrator when bias and position is known.
     *
     * @param location current device location.
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustKnownBiasAndPositionCalibrator(location: Location): RobustKnownBiasAndPositionAccelerometerCalibrator {
        val baseNoiseLevel = baseNoiseLevel
        val robustThreshold = robustThreshold

        val result = RobustKnownBiasAndPositionAccelerometerCalibrator.create(
            location.toNEDPosition(),
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setBiasCoordinates(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx, initialSy, initialSz,
            initialMxy, initialMxz, initialMyx,
            initialMyz, initialMzx, initialMzy
        )
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize = robustPreliminarySubsetSize.coerceAtLeast(
            minimumRequiredMeasurements
        )

        // set threshold and quality scores
        when (result) {
            is RANSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Internally builds a robust accelerometer calibrator when bias and gravity is known.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustKnownBiasAndGravityCalibrator(): RobustKnownBiasAndGravityNormAccelerometerCalibrator {
        val gravityNorm = this.gravityNorm
        checkNotNull(gravityNorm)

        val baseNoiseLevel = baseNoiseLevel
        val robustThreshold = robustThreshold

        val result = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(
            gravityNorm,
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setBiasCoordinates(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx, initialSy, initialSz,
            initialMxy, initialMxz, initialMyx,
            initialMyz, initialMzx, initialMzy
        )
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize = robustPreliminarySubsetSize.coerceAtLeast(
            minimumRequiredMeasurements
        )

        // set threshold
        when (result) {
            is RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Internally builds a robust accelerometer calibrator when position is known.
     *
     * @param location current device location.
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustKnownPositionCalibrator(location: Location): RobustKnownPositionAccelerometerCalibrator {
        val baseNoiseLevel = baseNoiseLevel
        val robustThreshold = robustThreshold

        val result = RobustKnownPositionAccelerometerCalibrator.create(
            location.toNEDPosition(),
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setInitialBias(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx, initialSy, initialSz,
            initialMxy, initialMxz, initialMyx,
            initialMyz, initialMzx, initialMzy
        )
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize = robustPreliminarySubsetSize.coerceAtLeast(
            minimumRequiredMeasurements
        )

        // set threshold
        when (result) {
            is RANSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Internally builds a robust accelerometer calibrator when gravity is known.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustKnownGravityCalibrator(): RobustKnownGravityNormAccelerometerCalibrator {
        val gravityNorm = this.gravityNorm
        checkNotNull(gravityNorm)

        val baseNoiseLevel = baseNoiseLevel
        val robustThreshold = robustThreshold

        val result = RobustKnownGravityNormAccelerometerCalibrator.create(
            gravityNorm,
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setInitialBias(
            initialBiasX ?: 0.0,
            initialBiasY ?: 0.0,
            initialBiasZ ?: 0.0
        )
        result.setInitialScalingFactorsAndCrossCouplingErrors(
            initialSx, initialSy, initialSz,
            initialMxy, initialMxz, initialMyx,
            initialMyz, initialMzx, initialMzy
        )
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize = robustPreliminarySubsetSize.coerceAtLeast(
            minimumRequiredMeasurements
        )

        // set threshold
        when (result) {
            is RANSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Builds required quality scores for PROSAC and PROMedS robust methods used for accelerometer
     * calibration.
     * Quality scores are build for each measurement. By default the standard deviation
     * of each measurement is taken into account, so that the larger the standard deviation
     * the poorer the measurement is considered (lower score).
     *
     * @return build quality score array.
     */
    private fun buildQualityScores(): DoubleArray {
        val size = measurements.size
        val qualityScores = DoubleArray(size)
        measurements.forEachIndexed { index, measurement ->
            qualityScores[index] = qualityScoreMapper.map(measurement)
        }
        return qualityScores
    }
}