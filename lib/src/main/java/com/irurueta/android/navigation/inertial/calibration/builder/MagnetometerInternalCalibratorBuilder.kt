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
import com.irurueta.android.navigation.inertial.calibration.StaticIntervalMagnetometerCalibrator
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.*
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import java.util.*

/**
 * Builds a magnetometer calibrator to be used internally by other calibrators.
 *
 * @property measurements List of magnetometer measurements.
 * @property location Location of device when running calibration.
 * @property robustMethod Indicates robust method used to solve magnetometer calibration.
 * @property timestamp Current timestamp.
 * @property isGroundTruthInitialHardIron Indicates whether magnetometer initial hard iron is
 * considered a ground-truth known hard iron.
 * @property isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
 * magnetometer, gyroscope and accelerometer.
 * @property initialHardIronX x-coordinate of magnetometer hard iron used as an initial guess and
 * expressed in Teslas (T).
 * @property initialHardIronY y-coordinate of magnetometer hard iron used as an initial guess and
 * expressed in Teslas (T).
 * @property initialHardIronZ z-coordinate of magnetometer hard iron used as an initial guess and
 * expressed in Teslas (T).
 * @property initialSx initial x scaling factor for magnetometer calibration.
 * @property initialSy initial y scaling factor for magnetometer calibration.
 * @property initialSz initial z scaling factor for magnetometer calibration.
 * @property initialMxy initial x-y cross coupling error for magnetometer calibration.
 * @property initialMxz initial x-z cross coupling error for magnetometer calibration.
 * @property initialMyx initial y-x cross coupling error for magnetometer calibration.
 * @property initialMyz initial y-z cross coupling error for magnetometer calibration.
 * @property initialMzx initial z-x cross coupling error for magnetometer calibration.
 * @property initialMzy initial z-y cross coupling error for magnetometer calibration.
 * @property baseNoiseLevel magnetometer measurement base noise level that has been detected during
 * initialization expressed in Teslas (T).
 * @property worldMagneticModel Earth's magnetic model. Null indicates that default model is being
 * used.
 * @property qualityScoreMapper mapper to convert collected magnetometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class MagnetometerInternalCalibratorBuilder private constructor(
    var measurements: List<StandardDeviationBodyMagneticFluxDensity>,
    var location: Location,
    var robustMethod: RobustEstimatorMethod?,
    var timestamp: Date,
    var isGroundTruthInitialHardIron: Boolean,
    var isCommonAxisUsed: Boolean,
    var initialHardIronX: Double?,
    var initialHardIronY: Double?,
    var initialHardIronZ: Double?,
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
    var worldMagneticModel: WorldMagneticModel?,
    var qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) {
    /**
     * Constructor.
     *
     * @param measurements List of magnetometer measurements.
     * @param location Location of device when running calibration.
     * @param robustPreliminarySubsetSize Size of preliminary subsets picked while
     * finding a robust magnetometer calibration solution.
     * @param minimumRequiredMeasurements Minimum number of required measurements to
     * start magnetometer calibration.
     * @param robustMethod Indicates robust method used to solve magnetometer calibration.
     * @param timestamp Current timestamp.
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
     * @param isGroundTruthInitialHardIron Indicates whether magnetometer initial hard iron is
     * considered a ground-truth known hard iron.
     * @param isCommonAxisUsed Indicates or specifies whether z-axis is assumed to be common for
     * magnetometer, gyroscope and accelerometer.
     * @param initialHardIronX x-coordinate of magnetometer hard iron used as an initial guess and
     * expressed in Teslas (T).
     * @param initialHardIronY y-coordinate of magnetometer hard iron used as an initial guess and
     * expressed in Teslas (T).
     * @param initialHardIronZ z-coordinate of magnetometer hard iron used as an initial guess and
     * expressed in Teslas (T).
     * @param initialSx initial x scaling factor for magnetometer calibration.
     * @param initialSy initial y scaling factor for magnetometer calibration.
     * @param initialSz initial z scaling factor for magnetometer calibration.
     * @param initialMxy initial x-y cross coupling error for magnetometer calibration.
     * @param initialMxz initial x-z cross coupling error for magnetometer calibration.
     * @param initialMyx initial y-x cross coupling error for magnetometer calibration.
     * @param initialMyz initial y-z cross coupling error for magnetometer calibration.
     * @param initialMzx initial z-x cross coupling error for magnetometer calibration.
     * @param initialMzy initial z-y cross coupling error for magnetometer calibration.
     * @param baseNoiseLevel magnetometer measurement base noise level that has been detected during
     * initialization expressed in Teslas (T).
     * @param worldMagneticModel Earth's magnetic model.
     * @param qualityScoreMapper mapper to convert collected magnetometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     */
    @Throws(IllegalArgumentException::class)
    constructor(
        measurements: List<StandardDeviationBodyMagneticFluxDensity>,
        location: Location,
        robustPreliminarySubsetSize: Int,
        minimumRequiredMeasurements: Int,
        robustMethod: RobustEstimatorMethod? = null,
        timestamp: Date = Date(),
        robustConfidence: Double = StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE,
        robustMaxIterations: Int =
            StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS,
        robustThreshold: Double? = null,
        robustThresholdFactor: Double =
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR,
        robustStopThresholdFactor: Double =
            StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR,
        isGroundTruthInitialHardIron: Boolean = false,
        isCommonAxisUsed: Boolean =
            StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS,
        initialHardIronX: Double? = null,
        initialHardIronY: Double? = null,
        initialHardIronZ: Double? = null,
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
        worldMagneticModel: WorldMagneticModel? = null,
        qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> =
            DefaultMagnetometerQualityScoreMapper()
    ) : this(
        measurements,
        location,
        robustMethod,
        timestamp,
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
        worldMagneticModel,
        qualityScoreMapper
    ) {
        this.robustPreliminarySubsetSize = robustPreliminarySubsetSize
        this.minimumRequiredMeasurements = minimumRequiredMeasurements
        this.robustConfidence = robustConfidence
        this.robustMaxIterations = robustMaxIterations
        this.robustThreshold = robustThreshold
        this.robustThresholdFactor = robustThresholdFactor
        this.robustStopThresholdFactor = robustStopThresholdFactor
        this.worldMagneticModel = worldMagneticModel
    }

    /**
     * Size of preliminary subsets picked while finding a robust magnetometer calibration solution.
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
     * Minimum number of required measurements to start magnetometer calibration.
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
     * Confidence of estimated magnetometer calibration result expressed as a value between 0.0
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
        StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust magnetometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [robustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustMaxIterations: Int =
        StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * magnetometer calibration algorithms or not.
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
     * Factor to be used respect estimated magnetometer base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [baseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustThresholdFactor: Double =
        StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to magnetometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var robustStopThresholdFactor: Double =
        StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Builds an internal magnetometer calibrator based on all provided parameters.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    fun build(): MagnetometerNonLinearCalibrator {
        return if (robustMethod == null) {
            buildNonRobustCalibrator()
        } else {
            buildRobustCalibrator()
        }
    }

    /**
     * Builds a non-robust magnetometer calibrator based on all provided parameters.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildNonRobustCalibrator(): MagnetometerNonLinearCalibrator {
        return if (isGroundTruthInitialHardIron) {
            val result = KnownHardIronPositionAndInstantMagnetometerCalibrator(
                location.toNEDPosition(),
                measurements,
                isCommonAxisUsed
            )
            result.setTime(timestamp)
            result.setHardIronCoordinates(
                initialHardIronX ?: 0.0,
                initialHardIronY ?: 0.0,
                initialHardIronZ ?: 0.0
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
            result.magneticModel = worldMagneticModel
            result
        } else {
            val result = KnownPositionAndInstantMagnetometerCalibrator(
                location.toNEDPosition(),
                measurements,
                isCommonAxisUsed
            )
            result.setTime(timestamp)
            result.setInitialHardIron(
                initialHardIronX ?: 0.0,
                initialHardIronY ?: 0.0,
                initialHardIronZ ?: 0.0
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
            result.magneticModel = worldMagneticModel
            result
        }
    }

    /**
     * Internally builds a robust magnetometer calibrator based on all provided parameters.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildRobustCalibrator(): MagnetometerNonLinearCalibrator {
        return if (isGroundTruthInitialHardIron) {
            buildKnownHardIronPositionAndInstantRobustCalibrator()
        } else {
            buildKnownPositionAndInstantRobustCalibrator()
        }
    }

    /**
     * Internally builds a robust magnetometer calibrator when hard iron is known.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildKnownHardIronPositionAndInstantRobustCalibrator(): MagnetometerNonLinearCalibrator {
        val baseNoiseLevel = this.baseNoiseLevel
        val robustThreshold = this.robustThreshold

        val result = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(
            location.toNEDPosition(),
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setTime(timestamp)
        result.setHardIronCoordinates(
            initialHardIronX ?: 0.0,
            initialHardIronY ?: 0.0,
            initialHardIronZ ?: 0.0
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
        result.magneticModel = worldMagneticModel
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold and quality scores
        when (result) {
            is RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator -> {
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
     * Internally builds a robust magnetometer calibrator when hard iron is not known.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildKnownPositionAndInstantRobustCalibrator(): MagnetometerNonLinearCalibrator {
        val baseNoiseLevel = this.baseNoiseLevel
        val robustThreshold = this.robustThreshold

        val result = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
            location.toNEDPosition(),
            measurements,
            isCommonAxisUsed,
            robustMethod
        )
        result.setTime(timestamp)
        result.setInitialHardIron(
            initialHardIronX ?: 0.0,
            initialHardIronY ?: 0.0,
            initialHardIronZ ?: 0.0
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
        result.magneticModel = worldMagneticModel
        result.confidence = robustConfidence
        result.maxIterations = robustMaxIterations
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold and quality scores
        when (result) {
            is RANSACRobustKnownPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownPositionAndInstantMagnetometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    checkNotNull(baseNoiseLevel)
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator -> {
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
     * Builds required quality scores for PROSAC and PROMedS robust methods used for magnetometer
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