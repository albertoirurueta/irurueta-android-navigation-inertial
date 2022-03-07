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
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.CalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.Acceleration
import com.irurueta.units.Time

/**
 * Base class for static interval calibrators, which detects static period of accelerometer
 * measurements along with other sensors (if needed) to solve calibration.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerSensorDelay Delay of sensor between samples.
 * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
 * enough measurements are available, false otherwise.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization completes.
 * @property errorListener listener to notify errors.
 * @property staticIntervalDetectedListener listener to notify when a static interval is detected.
 * @property dynamicIntervalDetectedListener listener to notify when a dynamic interval is detected.
 * @property staticIntervalSkippedListener listener to notify when a static interval is skipped if
 * its duration is too short.
 * @property dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped if
 * its duration is too long.
 * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
 * collected and calibrator is ready to solve calibration.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration solving completes.
 * @property stoppedListener listener to notify when calibrator is stopped.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
 * @param I type of input data to be processed by internal generator.
 */
abstract class StaticIntervalWithMeasurementGeneratorCalibrator<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, I>, I>(
    val context: Context,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val solveCalibrationWhenEnoughMeasurements: Boolean = true,
    var initializationStartedListener: OnInitializationStartedListener<C>? = null,
    var initializationCompletedListener: OnInitializationCompletedListener<C>? = null,
    var errorListener: OnErrorListener<C>? = null,
    var staticIntervalDetectedListener: OnStaticIntervalDetectedListener<C>? = null,
    var dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<C>? = null,
    var staticIntervalSkippedListener: OnStaticIntervalSkippedListener<C>? = null,
    var dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<C>? = null,
    var readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<C>? = null,
    var calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<C>? = null,
    var calibrationCompletedListener: OnCalibrationCompletedListener<C>? = null,
    var stoppedListener: OnStoppedListener<C>? = null,
    var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) {
    /**
     * Internal generator to generate measurements for calibration.
     */
    protected abstract val generator: CalibrationMeasurementGenerator<I>

    /**
     * Indicates whether the interval detector has picked the first magnetometer measurement.
     */
    protected val isFirstMeasurement
        get() = generator.numberOfProcessedAccelerometerMeasurements <= FIRST_MEASUREMENT

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    abstract val isReadyToSolveCalibration: Boolean

    /**
     * Indicates whether calibrator is running.
     * While calibrator is running, calibrator parameters cannot be changed.
     */
    var running: Boolean = false
        protected set

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level during initialization of the internal calibrator
     * measurement generator. Window size must always be larger than allowed minimum value, which
     * is 2 and must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var windowSize
        get() = generator.windowSize
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.windowSize = value
        }

    /**
     * Gets or sets number of samples to be processed initially by the internal
     * measurement generator, while keeping the sensor static in order to find the base noise level
     * when device is static.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [TriadStaticIntervalDetector.MINIMUM_INITIAL_STATIC_SAMPLES].
     * @throws IllegalStateException if calibrator is currently running.
     */
    var initialStaticSamples
        get() = generator.initialStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.initialStaticSamples = value
        }

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine a
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var thresholdFactor
        get() = generator.thresholdFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.thresholdFactor = value
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
        get() = generator.instantaneousNoiseLevelFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.instantaneousNoiseLevelFactor = value
        }

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes. This threshold is expressed in meters
     * per squared second (m/s^2).
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var baseNoiseLevelAbsoluteThreshold
        get() = generator.baseNoiseLevelAbsoluteThreshold
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.baseNoiseLevelAbsoluteThreshold = value
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
        get() = generator.baseNoiseLevelAbsoluteThresholdAsMeasurement
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            generator.baseNoiseLevelAbsoluteThresholdAsMeasurement = value
        }

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: Acceleration) {
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once internal generator completes initialization.
     */
    val accelerometerBaseNoiseLevel
        get() = generator.accelerometerBaseNoiseLevel

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once internal generator completes initialization.
     */
    val accelerometerBaseNoiseLevelAsMeasurement
        get() = generator.accelerometerBaseNoiseLevelAsMeasurement

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAccelerometerBaseNoiseLevelAsMeasurement(result: Acceleration): Boolean {
        return generator.getAccelerometerBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     * This is only available once internal generator completes initialization.
     */
    val accelerometerBaseNoiseLevelPsd
        get() = generator.accelerometerBaseNoiseLevelPsd

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once internal generator completes initialization.
     */
    val accelerometerBaseNoiseLevelRootPsd
        get() = generator.accelerometerBaseNoiseLevelRootPsd

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
     * This is only available once internal generator completes initialization.
     */
    val threshold
        get() = generator.threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once internal generator completes initialization.
     */
    val thresholdAsMeasurement
        get() = generator.thresholdAsMeasurement

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getThresholdAsMeasurement(result: Acceleration): Boolean {
        return generator.getThresholdAsMeasurement(result)
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     */
    val processedStaticSamples
        get() = generator.processedStaticSamples

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     */
    val processedDynamicSamples
        get() = generator.processedDynamicSamples

    /**
     * Indicates whether last static interval must be skipped.
     */
    val isStaticIntervalSkipped
        get() = generator.isStaticIntervalSkipped

    /**
     * Indicates whether last dynamic interval must be skipped.
     */
    val isDynamicIntervalSkipped
        get() = generator.isDynamicIntervalSkipped

    /**
     * Gets average time interval between accelerometer samples expressed in seconds (s).
     * This is only available once the internal generator completes initialization.
     */
    val accelerometerAverageTimeInterval
        get() = generator.accelerometerAverageTimeInterval

    /**
     * Gets average time interval between accelerometer samples.
     * This is only available once the internal generator completes initialization.
     */
    val accelerometerAverageTimeIntervalAsTime
        get() = generator.accelerometerAverageTimeIntervalAsTime

    /**
     * Gets average time interval between accelerometer measurements.
     * This is only available once the internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAccelerometerAverageTimeIntervalAsTime(result: Time): Boolean {
        return generator.getAccelerometerAverageTimeIntervalAsTime(result)
    }

    /**
     * Gets estimated variance of time interval between accelerometer measurements expressed in
     * squared seconds (s^2).
     * This is only available once internal generator completes initialization.
     */
    val accelerometerTimeIntervalVariance
        get() = generator.accelerometerTimeIntervalVariance

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements
     * expressed in seconds (s).
     * This is only available once internal generator completes initialization.
     */
    val accelerometerTimeIntervalStandardDeviation
        get() = generator.accelerometerTimeIntervalStandardDeviation

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once internal generator completes initialization.
     */
    val accelerometerTimeIntervalStandardDeviationAsTime
        get() = generator.accelerometerTimeIntervalStandardDeviationAsTime

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return generator.getTimeIntervalStandardDeviationAsTime(result)
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
     * Number of accelerometer measurements that have been processed.
     */
    val numberOfProcessedAccelerometerMeasurements
        get() = generator.numberOfProcessedAccelerometerMeasurements

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
        generator.start()
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
        generator.stop()
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
    protected abstract fun reset()

    companion object {
        /**
         * Indicates when first sensor measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1
    }

    /**
     * Interface to notify when calibrator starts initialization.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnInitializationStartedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnInitializationCompletedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnErrorListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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
     * Interface to notify that a new static interval is detected.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnStaticIntervalDetectedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
        /**
         * Called when a static interval has been detected after initialization.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onStaticIntervalDetected(calibrator: C)
    }

    /**
     * Interface to notify that a new dynamic interval is detected.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnDynamicIntervalDetectedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
        /**
         * Called when a dynamic interval has been detected after initialization.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onDynamicIntervalDetected(calibrator: C)
    }

    /**
     * Interface to notify when a static interval is skipped.
     * This happens when interval is too short.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnStaticIntervalSkippedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
        /**
         * Called when a detected static interval is skipped because there are not enough samples to
         * be processed.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onStaticIntervalSkipped(calibrator: C)
    }

    /**
     * Interface to notify when a dynamic interval is skipped.
     * This happens when interval is too long.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnDynamicIntervalSkippedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
        /**
         * Called when a detected dynamic interval is skipped because it has too many samples in it.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onDynamicIntervalSkipped(calibrator: C)
    }

    /**
     * Interface to notify when enough measurements are obtained to start solving calibration.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnReadyToSolveCalibrationListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
        /**
         * Called when enough measurements are obtained to start solving calibration.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onReadyToSolveCalibration(calibrator: C)
    }

    /**
     * Interface to notify when calibration starts being solved.
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator].
     */
    fun interface OnCalibrationSolvingStartedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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
     *
     * @param C an implementation of [StaticIntervalWithMeasurementGeneratorCalibrator]
     */
    fun interface OnCalibrationCompletedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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
    fun interface OnStoppedListener<C : StaticIntervalWithMeasurementGeneratorCalibrator<C, *>> {
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