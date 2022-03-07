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
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.Measurement
import com.irurueta.units.Time

/**
 * Base class for static interval calibrators, which detects static periods of sensor measurements
 * to solve calibration.
 * Implementations of this class only use a single sensor (no combination of accelerometer and
 * another sensor is used).
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
 * enough measurements are available, false otherwise.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization completes.
 * @property errorListener listener to notify errors.
 * @property newCalibrationMeasurementAvailableListener listener to notify when a new calibration
 * measurement is obtained.
 * @property readyToSolveCalibrationListener listener to notify when calibrator is ready to be
 * solved.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration is successfully
 * completed.
 * @property stoppedListener listener to notify when measurement collection stops.
 * @property qualityScoreMapper mapper to convert collected measurements into quality scores,
 * based on the amount of standard deviation (the larger the variability, the worse the score
 * will be).
 * @param C an implementation of [SingleSensorStaticIntervalCalibrator].
 * @param R reading type of collected measurements during static period that is used for solving
 * calibration.
 * @param I an implementation of [IntervalDetector]
 * @param U type of unit.
 * @param M a type of measurement.
 * @param T a triad type.
 */
abstract class SingleSensorStaticIntervalCalibrator<C : SingleSensorStaticIntervalCalibrator<C, R, I, U, M, T>, R,
        I : IntervalDetector<I, *, U, M, T, *, *>, U : Enum<*>, M : Measurement<U>,
        T : Triad<U, M>>(
    val context: Context,
    val sensorDelay: SensorDelay,
    val solveCalibrationWhenEnoughMeasurements: Boolean,
    var initializationStartedListener: OnInitializationStartedListener<C>?,
    var initializationCompletedListener: OnInitializationCompletedListener<C>?,
    var errorListener: OnErrorListener<C>?,
    var newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener<C, R>?,
    var readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<C>?,
    var calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<C>?,
    var calibrationCompletedListener: OnCalibrationCompletedListener<C>?,
    var stoppedListener: OnStoppedListener<C>?,
    val qualityScoreMapper: QualityScoreMapper<R>
) {
    /**
     * Listener used by internal detector to handle events when initialization starts.
     */
    @Suppress("UNCHECKED_CAST")
    protected val intervalDetectorInitializationStartedListener =
        IntervalDetector.OnInitializationStartedListener<I> {
            initializationStartedListener?.onInitializationStarted(this@SingleSensorStaticIntervalCalibrator as C)
        }

    /**
     * Listener used by internal interval detector to handle events when initialization is
     * completed.
     */
    @Suppress("UNCHECKED_CAST")
    protected open val intervalDetectorInitializationCompletedListener =
        IntervalDetector.OnInitializationCompletedListener<I> { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(this@SingleSensorStaticIntervalCalibrator as C)
        }

    @Suppress("UNCHECKED_CAST")
    protected val intervalDetectorErrorListener =
        IntervalDetector.OnErrorListener<I> { _, reason ->
            stop()
            errorListener?.onError(
                this@SingleSensorStaticIntervalCalibrator as C,
                CalibratorErrorReason.mapErrorReason(reason)
            )
        }

    /**
     * Internal interval detector to detect periods when device remains static.
     */
    protected abstract val intervalDetector: I

    /**
     * Indicates whether the interval detector has picked the first magnetometer measurement.
     */
    protected val isFirstMeasurement
        get() = intervalDetector.numberOfProcessedMeasurements <= FIRST_MEASUREMENT

    /**
     * List of measurements that have been collected so far to be used for calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    var measurements = mutableListOf<R>()
        protected set

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    val isReadyToSolveCalibration
        get() = measurements.size >= requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

    /**
     * Indicates whether calibrator is running.
     * While calibrator is running, calibrator parameters cannot be changed.
     */
    var running: Boolean = false
        protected set

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level during initialization of the interval
     * detector. Window size must always be larger than allowed minimum value, which is 2 and
     * must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var windowSize
        get() = intervalDetector.windowSize
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.windowSize = value
        }

    /**
     * Gets or sets number of samples to be processed initially by the internal interval detector
     * while keeping the sensor static in order to find the base noise level when device is static.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [TriadStaticIntervalDetector.MINIMUM_INITIAL_STATIC_SAMPLES].
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialStaticSamples
        get() = intervalDetector.initialStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.initialStaticSamples = value
        }

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine a
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var thresholdFactor
        get() = intervalDetector.thresholdFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.thresholdFactor = value
        }

    /**
     * Gets or sets factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount. This factor
     * is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var instantaneousNoiseLevelFactor
        get() = intervalDetector.instantaneousNoiseLevelFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.instantaneousNoiseLevelFactor = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes. This threshold is expressed in meters
     * per squared second (m/s^2) for accelerometer or Teslas (T) for magnetometer.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var baseNoiseLevelAbsoluteThreshold
        get() = intervalDetector.baseNoiseLevelAbsoluteThreshold
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.baseNoiseLevelAbsoluteThreshold = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var baseNoiseLevelAbsoluteThresholdAsMeasurement
        get() = intervalDetector.baseNoiseLevelAbsoluteThresholdAsMeasurement
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
        }

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: M) {
        intervalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets sensor measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2) for accelerometer or Teslas (T) for
     * magnetometer.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevel
        get() = intervalDetector.baseNoiseLevel

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelAsMeasurement
        get() = intervalDetector.baseNoiseLevelAsMeasurement

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsMeasurement(result: M): Boolean {
        return intervalDetector.getBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3) for
     * accelerometer or (T^2 * s) for magnetometer.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelPsd
        get() = intervalDetector.baseNoiseLevelPsd

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed in
     * (m * s^-1.5) for accelerometer or (T * s^0.5) for magnetometer.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelRootPsd
        get() = intervalDetector.baseNoiseLevelRootPsd

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2) for accelerometer or Teslas (T) for magnetometer.
     * This is only available once initialization is completed.
     */
    val threshold
        get() = intervalDetector.threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     */
    val thresholdAsMeasurement
        get() = intervalDetector.thresholdAsMeasurement

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getThresholdAsMeasurement(result: M): Boolean {
        return intervalDetector.getThresholdAsMeasurement(result)
    }

    /**
     * Gets average time interval between magnetometer samples expressed in seconds (s).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val averageTimeInterval
        get() = intervalDetector.averageTimeInterval

    /**
     * Gets average time interval between magnetometer samples.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val averageTimeIntervalAsTime
        get() = intervalDetector.averageTimeIntervalAsTime

    /**
     * Gets average time interval between measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageTimeIntervalAsTime(result: Time): Boolean {
        return intervalDetector.getAverageTimeIntervalAsTime(result)
    }

    /**
     * Gets estimated variance of time interval between measurements expressed in squared
     * seconds (s^2).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val timeIntervalVariance
        get() = intervalDetector.timeIntervalVariance

    /**
     * Gets estimated standard deviation of time interval between measurements expressed in
     * seconds (s).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val timeIntervalStandardDeviation
        get() = intervalDetector.timeIntervalStandardDeviation

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val timeIntervalStandardDeviationAsTime
        get() = intervalDetector.timeIntervalStandardDeviationAsTime

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false othewise.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return intervalDetector.getTimeIntervalStandardDeviationAsTime(result)
    }

    /**
     * Gets or sets initial x scaling factor.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialSx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y scaling factor.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialSy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z scaling factor.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialSz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-y cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMxy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-z cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMxz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-x cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMyx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-z cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMyz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-x cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMzx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-y cross coupling error.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialMzy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Sets initial scaling factors.
     *
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setInitialScalingFactors(initialSx: Double, initialSy: Double, initialSz: Double) {
        check(!running)
        this.initialSx = initialSx
        this.initialSy = initialSy
        this.initialSz = initialSz
    }

    /**
     * Sets initial cross coupling errors.
     *
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setInitialCrossCouplingErrors(
        initialMxy: Double,
        initialMxz: Double,
        initialMyx: Double,
        initialMyz: Double,
        initialMzx: Double,
        initialMzy: Double
    ) {
        check(!running)
        this.initialMxy = initialMxy
        this.initialMxz = initialMxz
        this.initialMyx = initialMyx
        this.initialMyz = initialMyz
        this.initialMzx = initialMzx
        this.initialMzy = initialMzy
    }

    /**
     * Sets initial scaling factors and cross couping errors.
     *
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setInitialScalingFactorsAndCrossCouplingErrors(
        initialSx: Double,
        initialSy: Double,
        initialSz: Double,
        initialMxy: Double,
        initialMxz: Double,
        initialMyx: Double,
        initialMyz: Double,
        initialMzx: Double,
        initialMzy: Double
    ) {
        setInitialScalingFactors(initialSx, initialSy, initialSz)
        setInitialCrossCouplingErrors(
            initialMxy,
            initialMxz,
            initialMyx,
            initialMyz,
            initialMzx,
            initialMzy
        )
    }

    /**
     * Indicates or specifies whether z-axis is assumed to be common for magnetometer and
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isCommonAxisUsed: Boolean = DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements is collected, calibration can start.
     */
    abstract val minimumRequiredMeasurements: Int

    /**
     * Required number of measurements to be collected before calibration can start.
     * The number of required measurements must be greater than [minimumRequiredMeasurements],
     * otherwise at least [minimumRequiredMeasurements] will be collected before calibration can
     * start.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var requiredMeasurements: Int = 0
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0)
            check(!running)
            field = value
        }

    /**
     * Indicates robust method used to solve calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustMethod: RobustEstimatorMethod? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Confidence of estimated result expressed as a value between 0.0 and 1.0.
     * By default 99% of confidence is used, which indicates that with a probability of 99%
     * estimation will be accurate because chosen sub-samples will be inliers (in other terms,
     * outliers will be correctly discarded).
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0 (both
     * included).
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustConfidence: Double = ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value in 0.0..1.0)
            check(!running)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust calibration solution.
     * By default this is 5000.
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustMaxIterations: Int = ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0)
            check(!running)
            field = value
        }

    /**
     * Size of preliminary subsets picked while finding a robust calibration solution.
     * By default this is the [minimumRequiredMeasurements], which results in the smallest number
     * of iterations to complete robust algorithms.
     * Larger values can be used to ensure that error in each preliminary solution is minimized
     * among more measurements (thus, softening the effect of outliers), but this comes at the
     * expense of larger number of iterations.
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is less than [minimumRequiredMeasurements]
     * at the moment the setter is called.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustPreliminarySubsetSize: Int = 0
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value >= minimumRequiredMeasurements)
            check(!running)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * algorithms or not.
     * Threshold varies depending on chosen [robustMethod].
     * By default, if null is provided, the estimated [baseNoiseLevel] will be used to determine a
     * suitable threshold. Otherwise, if a value is provided, such value will be used instead.
     * This properly is only taken into account if a not-null [robustMethod] is specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustThreshold: Double? = null
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value == null || value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Factor to be used respect estimated base noise level to consider a measurement an outlier
     * when using robust calibration methods.
     * By default this is 3.0 times [baseNoiseLevel], which considering the noise level as
     * the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustThresholdFactor: Double = DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var robustStopThresholdFactor: Double = DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Gets estimated x-axis scale factor or null if not available.
     */
    abstract val estimatedSx: Double?

    /**
     * Gets estimated y-axis scale factor or null if not available.
     */
    abstract val estimatedSy: Double?

    /**
     * Gets estimated z-axis scale factor or null if not available.
     */
    abstract val estimatedSz: Double?

    /**
     * Gets estimated x-y cross-coupling error or null if not available.
     */
    abstract val estimatedMxy: Double?

    /**
     * Gets estimated x-z cross-coupling error or null if not available.
     */
    abstract val estimatedMxz: Double?

    /**
     * Gets estimated y-x cross-coupling error or null if not available.
     */
    abstract val estimatedMyx: Double?

    /**
     * Gets estimated y-z cross-coupling error or null if not available.
     */
    abstract val estimatedMyz: Double?

    /**
     * Gets estimated z-x cross-coupling error or null if not available.
     */
    abstract val estimatedMzx: Double?

    /**
     * Gets estimated z-y cross-coupling error or null if not available.
     */
    abstract val estimatedMzy: Double?

    /**
     * Gets estimated covariance matrix for estimated sensor parameters or null if not
     * available.
     * When bias or hard iron is known, diagonal elements of the covariance matrix contains variance
     * for the following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx,
     * mzy.
     * When bias or hard iron is not known, diagonal elements of the covariance matrix contains
     * variance for the following parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, where bx, by, bz corresponds to bias or hard iron coordinates.
     */
    abstract val estimatedCovariance: Matrix?

    /**
     * Gets estimated chi square value or null if not available.
     */
    abstract val estimatedChiSq: Double?

    /**
     * Gets estimated mean square error respect to provided measurements or null if not available.
     */
    abstract val estimatedMse: Double?

    /**
     * Starts calibrator.
     * This method starts collecting sensor measurements.
     * When calibrator is started, it begins with an initialization stage where sensor noise
     * is estimated while device remains static.
     * Once initialization is completed, calibrator determines intervals where device remains static
     * when device has different poses, so that measurements are collected to solve calibration.
     * If [solveCalibrationWhenEnoughMeasurements] is true, calibration is automatically solved
     * once enough measurements are collected, otherwise a call to [calibrate] must be done to solve
     * calibration.
     *
     * @throws IllegalStateException if calibrator is already running or magnetometer sensor is
     * missing.
     */
    @Throws(IllegalStateException::class)
    open fun start() {
        check(!running)

        reset()

        running = true
        intervalDetector.start()
    }

    /**
     * Stops calibrator.
     * When this is called, no more sensor measurements are collected.
     */
    fun stop() {
        internalStop(false)
    }

    /**
     * Solves calibration using collected measurements.
     * This must be explicitly called after enough measurements are collected if
     * [solveCalibrationWhenEnoughMeasurements] is false.
     *
     * @return true if calibration completes successfully, false if a numerical error occurs while
     * solving calibration (which is notified using provided [errorListener].
     * @throws IllegalStateException if calibrator is already running or not enough measurements
     * have already been collected.
     */
    @Throws(IllegalStateException::class)
    fun calibrate(): Boolean {
        check(isReadyToSolveCalibration)
        check(!running)
        return internalCalibrate()
    }

    /**
     * Stops calibrator.
     * When this is called, no more magnetometer measurements are collected.
     *
     * @param running specifies the running parameter to be set. This is true when stop occurs
     * internally during measurement collection to start solving calibration, otherwise is false
     * when calling public [stop] method.
     */
    protected open fun internalStop(running: Boolean) {
        intervalDetector.stop()
        this.running = running

        @Suppress("UNCHECKED_CAST")
        stoppedListener?.onStopped(this as C)
    }

    /**
     * Internally solves calibration using collected measurements without checking pre-requisites
     * (if either calibrator is already running or enough measurements are available).
     */
    @Throws(IllegalStateException::class)
    protected abstract fun internalCalibrate(): Boolean

    /**
     * Resets calibrator to its initial state.
     */
    protected open fun reset() {
        measurements.clear()
    }

    /**
     * Builds required quality scores for PROSAC and PROMedS robust methods.
     * Quality scores are build for each measurement. By default the standard deviation
     * of each measurement is taken into account, so that the larger the standard deviation
     * the poorer the measurement is considered (lower score).
     *
     * @return build quality score array.
     */
    protected fun buildQualityScores(): DoubleArray {
        val size = measurements.size
        val qualityScores = DoubleArray(size)
        measurements.forEachIndexed { index, measurement ->
            qualityScores[index] = qualityScoreMapper.map(measurement)
        }
        return qualityScores
    }

    companion object {
        /**
         * Indicates when first sensor measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1

        /**
         * Indicates whether by default a common z-axis is assumed for both accelerometer,
         * gyroscope and magnetometer.
         */
        const val DEFAULT_USE_COMMON_Z_AXIS = false

        /**
         * Default confidence to find a robust calibration. By default this is 99%.
         */
        const val ROBUST_DEFAULT_CONFIDENCE = 0.99

        /**
         * Default maximum allowed number of iterations to find a robust calibration solution.
         */
        const val ROBUST_DEFAULT_MAX_ITERATIONS = 5000

        /**
         * Default factor to be used respect estimated base noise level to consider a measurement
         * an outlier when using robust calibration methods.
         * By default this is 3.0 times [baseNoiseLevel], which considering the noise level as
         * the standard deviation of a Gaussian distribution, should account for 99% of the cases.
         * Any measurement having an error greater than that in the estimated solution, will be
         * considered an outlier and be discarded.
         */
        const val DEFAULT_ROBUST_THRESHOLD_FACTOR = 3.0

        /**
         * Additional factor for robust methods based on LMedS or PROMedS, where factor is not
         * directly related to LMSE, but to a smaller value.
         */
        const val DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR = 1e-2
    }

    /**
     * Interface to notify when calibrator starts initialization.
     */
    fun interface OnInitializationStartedListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when calibrator starts initialization to determine base noise level when device
         * remains static.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onInitializationStarted(calibrator: C)
    }

    /**
     * Interface to notify when calibrator successfully completes initialization.
     */
    fun interface OnInitializationCompletedListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when calibrator successfully completes initialization to determine base noise
         * level when device remains static.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onInitializationCompleted(calibrator: C)
    }

    /**
     * Interface to notify when an error occurs.
     */
    fun interface OnErrorListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when an error is detected, either at initialization because excessive noise
         * is detected, because a sensor becomes unreliable or because obtained
         * measurements produce a numerically unstable calibration solution.
         *
         * @param calibrator calibrator that raised the event.
         * @param errorReason reason why error was detected.
         */
        fun onError(calibrator: C, errorReason: CalibratorErrorReason)
    }

    /**
     * Interface to notify when a new measurement is obtained for calibration purposes.
     * Such measurements are determined each time a static period finishes (when sensor
     * measurements stop being static).
     * When enough of these measurements are obtained, calibration can actually be solved.
     */
    fun interface OnNewCalibrationMeasurementAvailableListener<C : SingleSensorStaticIntervalCalibrator<C, R, *, *, *, *>, R> {
        /**
         * Called when a new measurement for calibration is found.
         * A new measurement each time a static period finishes (when sensor measurements
         * stop being static).
         * When enough of these measurements are obtained, calibration can actually be solved.
         * This listener can be used to modify each measurement as it is being collected.
         * Notice that changes to new measurements should be done carefully as they might affect
         * result of solved calibration.
         *
         * @param calibrator calibrator that raised the event.
         * @param newMeasurement new measurement that has been found.
         * @param measurementsFoundSoFar number of measurements that have been found so far.
         * @param requiredMeasurements required number of measurements to solve calibration.
         */
        fun onNewCalibrationMeasurementAvailable(
            calibrator: C,
            newMeasurement: R,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    /**
     * Interface to notify when enough measurements are obtained to start solving calibration.
     */
    fun interface OnReadyToSolveCalibrationListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when enough measurements are obtained to start solving calibration.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onReadyToSolveCalibration(calibrator: C)
    }

    /**
     * Interface to notify when calibration starts being solved.
     */
    fun interface OnCalibrationSolvingStartedListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when calibration starts being solved after enough measurements are found.
         * Calibration can automatically started when enough measurements are available if
         * [solveCalibrationWhenEnoughMeasurements] is true, otherwise [calibrate] must be called
         * after enough measurements are found, which raises this event.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onCalibrationSolvingStarted(calibrator: C)
    }

    /**
     * Interface to notify when calibration is solved and completed.
     */
    fun interface OnCalibrationCompletedListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when calibration successfully completes.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onCalibrationCompleted(calibrator: C)
    }

    /**
     * Interface to notify when measurement collection stops.
     * This happens automatically when enough measurements are found after periods when
     * device stops being static, or if an error occurs.
     */
    fun interface OnStoppedListener<C : SingleSensorStaticIntervalCalibrator<C, *, *, *, *, *>> {
        /**
         * Called when measurement collection stops.
         * This happens automatically when enough measurements are found after periods when
         * device stops being static, or if an error occurs.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onStopped(calibrator: C)
    }
}