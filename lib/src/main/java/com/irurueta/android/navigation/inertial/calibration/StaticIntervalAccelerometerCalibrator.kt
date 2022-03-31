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
import android.location.Location
import android.util.Log
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.GravityHelper
import com.irurueta.android.navigation.inertial.calibration.builder.AccelerometerInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.AccelerometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.StopMode
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AccelerometerBiasUncertaintySource
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Collects accelerometer measurements by detecting periods when device remains static,
 * and using such static periods, measurements are obtained to solve calibration parameters.
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
 * @property generatedAccelerometerMeasurementListener listener to notify when a new accelerometer
 * calibration measurement is generated.
 * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
 * collected and calibrator is ready to solve calibration.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration solving completes.
 * @property stoppedListener listener to notify when calibrator is stopped.
 * @property unreliableGravityNormEstimationListener listener to notify when gravity norm
 * estimation becomes unreliable. This is only used if no location is provided.
 * @property initialAccelerometerBiasAvailableListener listener to notify when a guess of bias values is
 * obtained.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 * @property accelerometerQualityScoreMapper mapper to convert collected accelerometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class StaticIntervalAccelerometerCalibrator private constructor(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    accelerometerSensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>?,
    initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>?,
    errorListener: OnErrorListener<StaticIntervalAccelerometerCalibrator>?,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>?,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>?,
    staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>?,
    dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>?,
    var generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener?,
    readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>?,
    calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>?,
    calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>?,
    stoppedListener: OnStoppedListener<StaticIntervalAccelerometerCalibrator>?,
    var unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener?,
    var initialAccelerometerBiasAvailableListener: OnInitialAccelerometerBiasAvailableListener?,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener?,
    val accelerometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>
) : StaticIntervalWithMeasurementGeneratorCalibrator<StaticIntervalAccelerometerCalibrator, BodyKinematics>(
    context,
    accelerometerSensorType,
    accelerometerSensorDelay,
    solveCalibrationWhenEnoughMeasurements,
    initializationStartedListener,
    initializationCompletedListener,
    errorListener,
    staticIntervalDetectedListener,
    dynamicIntervalDetectedListener,
    staticIntervalSkippedListener,
    dynamicIntervalSkippedListener,
    readyToSolveCalibrationListener,
    calibrationSolvingStartedListener,
    calibrationCompletedListener,
    stoppedListener,
    accuracyChangedListener
) {
    /**
     * Constructor.
     *
     * @param context Android context.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * @param accelerometerSensorDelay Delay of sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param isAccelerometerGroundTruthInitialBias true if estimated accelerometer bias is assumed
     * to be the true value, false if estimated bias is assumed to be only an initial guess. When
     * [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], bias
     * guess is zero, otherwise when it is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias guess is the
     * device calibrated values.
     * @param location location where device is located at. When location is provided, gravity norm
     * is assumed to be the theoretical value determined by WGS84 Earth model, otherwise, if no
     * location is provided, gravity norm is estimated using a gravity sensor.
     * @param initializationStartedListener listener to notify when initialization starts.
     * @param initializationCompletedListener listener to notify when initialization completes.
     * @param errorListener listener to notify errors.
     * @param staticIntervalDetectedListener listener to notify when a static interval is detected.
     * @param dynamicIntervalDetectedListener listener to notify when a dynamic interval is
     * detected.
     * @param staticIntervalSkippedListener listener to notify when a static interval is skipped if
     * its duration is too short.
     * @param dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped
     * if its duration is too long.
     * @param generatedAccelerometerMeasurementListener listener to notify when a new accelerometer
     * calibration measurement is generated.
     * @param readyToSolveCalibrationListener listener to notify when enough measurements have been
     * collected and calibrator is ready to solve calibration.
     * @param calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @param calibrationCompletedListener listener to notify when calibration solving completes.
     * @para stoppedListener listener to notify when calibrator is stopped.
     * @param unreliableGravityNormEstimationListener listener to notify when gravity norm
     * estimation becomes unreliable. This is only used if no location is provided.
     * @param initialAccelerometerBiasAvailableListener listener to notify when a guess of bias
     * values is obtained.
     * @param accuracyChangedListener listener to notify when sensor accuracy changes.
     * @param accelerometerQualityScoreMapper mapper to convert collected accelerometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability,
     * the worse the score will be).
     */
    constructor(
        context: Context,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isAccelerometerGroundTruthInitialBias: Boolean = false,
        location: Location? = null,
        initializationStartedListener: OnInitializationStartedListener<StaticIntervalAccelerometerCalibrator>? = null,
        initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalAccelerometerCalibrator>? = null,
        errorListener: OnErrorListener<StaticIntervalAccelerometerCalibrator>? = null,
        staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>? = null,
        dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerCalibrator>? = null,
        staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>? = null,
        dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerCalibrator>? = null,
        generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerCalibrator>? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerCalibrator>? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalAccelerometerCalibrator>? = null,
        stoppedListener: OnStoppedListener<StaticIntervalAccelerometerCalibrator>? = null,
        unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener? = null,
        initialAccelerometerBiasAvailableListener: OnInitialAccelerometerBiasAvailableListener? = null,
        accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null,
        accelerometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics> =
            DefaultAccelerometerQualityScoreMapper()
    ) : this(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        staticIntervalDetectedListener,
        dynamicIntervalDetectedListener,
        staticIntervalSkippedListener,
        dynamicIntervalSkippedListener,
        generatedAccelerometerMeasurementListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        unreliableGravityNormEstimationListener,
        initialAccelerometerBiasAvailableListener,
        accuracyChangedListener,
        accelerometerQualityScoreMapper
    ) {
        this.isAccelerometerGroundTruthInitialBias = isAccelerometerGroundTruthInitialBias
        this.location = location
        requiredMeasurements = minimumRequiredMeasurements
        accelerometerRobustPreliminarySubsetSize = minimumRequiredMeasurements
    }

    /**
     * Listener used by internal generator to handle events when initialization is started.
     */
    private val generatorInitializationStartedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<AccelerometerMeasurementGenerator> {
            initializationStartedListener?.onInitializationStarted(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when initialization is completed.
     */
    private val generatorInitializationCompletedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<AccelerometerMeasurementGenerator> { _, _ ->
            gravityNorm = gravityNormEstimator.averageNorm
            initializationCompletedListener?.onInitializationCompleted(
                this@StaticIntervalAccelerometerCalibrator
            )
        }

    /**
     * Listener used by internal generator to handle events when an error occurs.
     */
    private val generatorErrorListener =
        SingleSensorCalibrationMeasurementGenerator.OnErrorListener<AccelerometerMeasurementGenerator> { _, reason ->
            stop()
            errorListener?.onError(
                this@StaticIntervalAccelerometerCalibrator,
                CalibratorErrorReason.mapErrorReason(reason)
            )
        }

    /**
     * Listener used by internal generator to handle events when a static interval is detected.
     */
    private val generatorStaticIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<AccelerometerMeasurementGenerator> {
            staticIntervalDetectedListener?.onStaticIntervalDetected(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is detected.
     */
    private val generatorDynamicIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<AccelerometerMeasurementGenerator> {
            dynamicIntervalDetectedListener?.onDynamicIntervalDetected(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a static interval is skipped.
     */
    private val generatorStaticIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<AccelerometerMeasurementGenerator> {
            staticIntervalSkippedListener?.onStaticIntervalSkipped(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is skipped.
     */
    private val generatorDynamicIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<AccelerometerMeasurementGenerator> {
            dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a new measurement is generated.
     */
    private val generatorGeneratedMeasurementListener =
        SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics> { _, measurement ->
            accelerometerMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = accelerometerMeasurements.size

            generatedAccelerometerMeasurementListener?.onGeneratedAccelerometerMeasurement(
                this@StaticIntervalAccelerometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@StaticIntervalAccelerometerCalibrator
                )

                // stop internal generator since no more measurements need to be collected
                internalStop(true)

                // build calibrator
                accelerometerInternalCalibrator = buildAccelerometerInternalCalibrator()

                if (solveCalibrationWhenEnoughMeasurements) {
                    // execute calibration
                    internalCalibrate()
                }
            }
        }

    /**
     * Listener for accelerometer sensor collector.
     * This is used to determine device calibration and obtain initial guesses
     * for accelerometer bias (only available if
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED] is used, otherwise zero
     * bias is assumed as an initial guess).
     */
    private val generatorAccelerometerMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { _, _, _, bx, by, bz, _, _ ->
            if (isFirstAccelerometerMeasurement) {
                updateAccelerometerInitialBiases(bx, by, bz)
            }
        }

    /**
     * Listener for gravity norm estimator.
     * This is used to approximately estimate gravity when no location is provided, by using the
     * gravity sensor provided by the device.
     */
    private val gravityNormCompletedListener =
        AccumulatedMeasurementEstimator.OnEstimationCompletedListener<GravityNormEstimator> { estimator ->
            gravityNorm = estimator.averageNorm
        }

    /**
     * Listener to handle events when gravity sensor becomes unreliable.
     * When this happens, result of calibration is marked as unreliable and should probably be
     * discarded.
     */
    private val gravityNormUnreliableListener =
        AccumulatedMeasurementEstimator.OnUnreliableListener<GravityNormEstimator> {
            accelerometerResultUnreliable = true
            unreliableGravityNormEstimationListener?.onUnreliableGravityEstimation(this@StaticIntervalAccelerometerCalibrator)
        }

    /**
     * Internal generator to generate measurements for calibration.
     */
    override val generator = AccelerometerMeasurementGenerator(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        generatorInitializationStartedListener,
        generatorInitializationCompletedListener,
        generatorErrorListener,
        generatorStaticIntervalDetectedListener,
        generatorDynamicIntervalDetectedListener,
        generatorStaticIntervalSkippedListener,
        generatorDynamicIntervalSkippedListener,
        generatorGeneratedMeasurementListener,
        accelerometerMeasurementListener = generatorAccelerometerMeasurementListener,
        accuracyChangedListener = accuracyChangedListener
    )

    /**
     * Gravity norm estimator. It is used to estimate gravity norm when no location is provided.
     */
    private val gravityNormEstimator: GravityNormEstimator =
        GravityNormEstimator(
            context,
            accelerometerSensorDelay,
            initialStaticSamples,
            stopMode = StopMode.MAX_SAMPLES_ONLY,
            completedListener = gravityNormCompletedListener,
            unreliableListener = gravityNormUnreliableListener
        )

    /**
     * Internal accelerometer calibrator used to solve the calibration parameters once enough
     * measurements are collected at static intervals.
     */
    private var accelerometerInternalCalibrator: AccelerometerNonLinearCalibrator? = null

    /**
     * Contains gravity norm (either obtained by the gravity sensor, or determined by current
     * location using WGS84 Earth model). Expressed in meters per squared second (m/s^2).
     */
    var gravityNorm: Double? = null
        private set

    /**
     * Indicates if accelerometer result is unreliable. This can happen if no location is provided
     * and gravity estimation becomes unreliable. When this happens result of calibration should
     * probably be discarded.
     */
    var accelerometerResultUnreliable = false
        private set

    /**
     * Gets x-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasX].
     */
    var accelerometerInitialBiasX: Double? = null
        private set

    /**
     * Gets y-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasY].
     */
    var accelerometerInitialBiasY: Double? = null
        private set

    /**
     * Gets z-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasZ].
     */
    var accelerometerInitialBiasZ: Double? = null
        private set

    /**
     * Gets accelerometer X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasX].
     */
    val accelerometerInitialBiasXAsMeasurement: Acceleration?
        get() {
            val initialBiasX = accelerometerInitialBiasX ?: return null
            return Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets accelerometer X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasX].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getAccelerometerInitialBiasXAsMeasurement(result: Acceleration): Boolean {
        val initialBiasX = accelerometerInitialBiasX
        return if (initialBiasX != null) {
            result.value = initialBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets accelerometer Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasY].
     */
    val accelerometerInitialBiasYAsMeasurement: Acceleration?
        get() {
            val initialBiasY = accelerometerInitialBiasY ?: return null
            return Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets accelerometer Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasY].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getAccelerometerInitialBiasYAsMeasurement(result: Acceleration): Boolean {
        val initialBiasY = accelerometerInitialBiasY
        return if (initialBiasY != null) {
            result.value = initialBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets accelerometer Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasZ].
     */
    val accelerometerInitialBiasZAsMeasurement: Acceleration?
        get() {
            val initialBiasZ = accelerometerInitialBiasZ ?: return null
            return Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets accelerometer Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasZ].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getAccelerometerInitialBiasZAsMeasurement(result: Acceleration): Boolean {
        val initialBiasZ = accelerometerInitialBiasZ
        return if (initialBiasZ != null) {
            result.value = initialBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets initial bias coordinate used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the values used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasAsTriad] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasAsTriad] will be the estimated bias after solving calibration,
     * which will differ from [estimatedAccelerometerBiasAsTriad].
     */
    val accelerometerInitialBiasAsTriad: AccelerationTriad?
        get() {
            val initialBiasX = accelerometerInitialBiasX
            val initialBiasY = accelerometerInitialBiasY
            val initialBiasZ = accelerometerInitialBiasZ
            return if (initialBiasX != null && initialBiasY != null && initialBiasZ != null) {
                AccelerationTriad(
                    AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    initialBiasX,
                    initialBiasY,
                    initialBiasZ
                )
            } else {
                null
            }
        }

    /**
     * Gets initial bias coordinates used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the values used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasAsTriad] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasAsTriad] will be the estimated bias after solving calibration,
     * which will differ from [estimatedAccelerometerBiasAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAccelerometerInitialBiasAsTriad(result: AccelerationTriad): Boolean {
        val initialBiasX = accelerometerInitialBiasX
        val initialBiasY = accelerometerInitialBiasY
        val initialBiasZ = accelerometerInitialBiasZ
        return if (initialBiasX != null && initialBiasY != null && initialBiasZ != null) {
            result.setValueCoordinatesAndUnit(
                initialBiasX,
                initialBiasY,
                initialBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            true
        } else {
            false
        }
    }

    /**
     * Indicates whether accelerometer initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isAccelerometerGroundTruthInitialBias: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Location of device when running calibration.
     * If location is provided, WGS84 Earth model is used to determine gravity norm
     * at such location, otherwise gravity norm is estimated during initialization by using the
     * gravity sensor of device.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
            if (value != null) {
                // set gravity norm based on provided location
                gravityNorm = GravityHelper.getGravityNormForLocation(value)
            }
        }

    /**
     * Indicates whether gravity norm is estimated during initialization.
     * If location is provided, gravity is not estimated and instead theoretical
     * gravity for provided location is used.
     */
    val isGravityNormEstimated
        get() = location == null

    /**
     * Gets accelerometer sensor being used for interval detection.
     * This can be used to obtain additional information about the sensor.
     */
    val accelerometerSensor
        get() = generator.accelerometerSensor

    /**
     * Gets gravity sensor being used for gravity estimation.
     * This can be used to obtain additional information about the sensor.
     */
    val gravitySensor
        get() = gravityNormEstimator.sensor

    /**
     * Gets or sets initial accelerometer scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var accelerometerInitialMa: Matrix
        get() {
            val result = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
            getAccelerometerInitialMa(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            accelerometerInitialSx = value.getElementAtIndex(0)
            accelerometerInitialMyx = value.getElementAtIndex(1)
            accelerometerInitialMzx = value.getElementAtIndex(2)

            accelerometerInitialMxy = value.getElementAtIndex(3)
            accelerometerInitialSy = value.getElementAtIndex(4)
            accelerometerInitialMzy = value.getElementAtIndex(5)

            accelerometerInitialMxz = value.getElementAtIndex(6)
            accelerometerInitialMyz = value.getElementAtIndex(7)
            accelerometerInitialSz = value.getElementAtIndex(8)
        }

    /**
     * Gets initial accelerometer scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getAccelerometerInitialMa(result: Matrix) {
        require(result.rows == BodyKinematics.COMPONENTS && result.columns == BodyKinematics.COMPONENTS)

        result.setElementAtIndex(0, accelerometerInitialSx)
        result.setElementAtIndex(1, accelerometerInitialMyx)
        result.setElementAtIndex(2, accelerometerInitialMzx)

        result.setElementAtIndex(3, accelerometerInitialMxy)
        result.setElementAtIndex(4, accelerometerInitialSy)
        result.setElementAtIndex(5, accelerometerInitialMzy)

        result.setElementAtIndex(6, accelerometerInitialMxz)
        result.setElementAtIndex(7, accelerometerInitialMyz)
        result.setElementAtIndex(8, accelerometerInitialSz)
    }

    /**
     * Gets or sets initial x scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-y cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMxy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-z cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMxz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-x cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMyx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-z cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMyz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-x cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMzx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-y cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMzy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Sets initial scaling factors for accelerometer calibration.
     *
     * @param accelerometerInitialSx initial x scaling factor.
     * @param accelerometerInitialSy initial y scaling factor.
     * @param accelerometerInitialSz initial z scaling factor.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setAccelerometerInitialScalingFactors(
        accelerometerInitialSx: Double,
        accelerometerInitialSy: Double,
        accelerometerInitialSz: Double
    ) {
        check(!running)
        this.accelerometerInitialSx = accelerometerInitialSx
        this.accelerometerInitialSy = accelerometerInitialSy
        this.accelerometerInitialSz = accelerometerInitialSz
    }

    /**
     * Sets initial cross coupling errors for accelerometer calibration.
     *
     * @param accelerometerInitialMxy initial x-y cross coupling error.
     * @param accelerometerInitialMxz initial x-z cross coupling error.
     * @param accelerometerInitialMyx initial y-x cross coupling error.
     * @param accelerometerInitialMyz initial y-z cross coupling error.
     * @param accelerometerInitialMzx initial z-x cross coupling error.
     * @param accelerometerInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setAccelerometerInitialCrossCouplingErrors(
        accelerometerInitialMxy: Double,
        accelerometerInitialMxz: Double,
        accelerometerInitialMyx: Double,
        accelerometerInitialMyz: Double,
        accelerometerInitialMzx: Double,
        accelerometerInitialMzy: Double
    ) {
        check(!running)
        this.accelerometerInitialMxy = accelerometerInitialMxy
        this.accelerometerInitialMxz = accelerometerInitialMxz
        this.accelerometerInitialMyx = accelerometerInitialMyx
        this.accelerometerInitialMyz = accelerometerInitialMyz
        this.accelerometerInitialMzx = accelerometerInitialMzx
        this.accelerometerInitialMzy = accelerometerInitialMzy
    }

    /**
     * Sets initial scaling factors and cross couping errors for accelerometer calibration.
     *
     * @param accelerometerInitialSx initial x scaling factor.
     * @param accelerometerInitialSy initial y scaling factor.
     * @param accelerometerInitialSz initial z scaling factor.
     * @param accelerometerInitialMxy initial x-y cross coupling error.
     * @param accelerometerInitialMxz initial x-z cross coupling error.
     * @param accelerometerInitialMyx initial y-x cross coupling error.
     * @param accelerometerInitialMyz initial y-z cross coupling error.
     * @param accelerometerInitialMzx initial z-x cross coupling error.
     * @param accelerometerInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
        accelerometerInitialSx: Double,
        accelerometerInitialSy: Double,
        accelerometerInitialSz: Double,
        accelerometerInitialMxy: Double,
        accelerometerInitialMxz: Double,
        accelerometerInitialMyx: Double,
        accelerometerInitialMyz: Double,
        accelerometerInitialMzx: Double,
        accelerometerInitialMzy: Double
    ) {
        setAccelerometerInitialScalingFactors(
            accelerometerInitialSx,
            accelerometerInitialSy,
            accelerometerInitialSz
        )
        setAccelerometerInitialCrossCouplingErrors(
            accelerometerInitialMxy,
            accelerometerInitialMxz,
            accelerometerInitialMyx,
            accelerometerInitialMyz,
            accelerometerInitialMzx,
            accelerometerInitialMzy
        )
    }

    /**
     * Indicates or specifies whether z-axis is assumed to be common for magnetometer and
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix during accelerometer
     * calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isAccelerometerCommonAxisUsed: Boolean = DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets minimum number of required measurements to start accelerometer calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredAccelerometerMeasurements: Int
        get() = if (isAccelerometerCommonAxisUsed) {
            if (isAccelerometerGroundTruthInitialBias) {
                ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isAccelerometerGroundTruthInitialBias) {
                ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            }
        }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = minimumRequiredAccelerometerMeasurements

    /**
     * Gets estimated average of gravity norm expressed in meters per squared second (m/s^2).
     * This is only available if no location is provided and initialization has completed.
     */
    val averageGravityNorm
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.averageNorm
        } else {
            null
        }

    /**
     * Gets estimated average gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    val averageGravityNormAsMeasurement
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.averageNormAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated average gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    fun getAverageGravityNormAsMeasurement(result: Acceleration): Boolean {
        return if (isGravityNormEstimated) {
            gravityNormEstimator.getAverageNormAsMeasurement(result)
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of gravity norm expressed in (m^2/s^4).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormVariance
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.normVariance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm expressed in meters per squared second
     * (m/s^2).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormStandardDeviation
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.normStandardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormStandardDeviationAsMeasurement
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.normStandardDeviationAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     *
     * @param result instance where result will be stored.
     * @return true i result is available, false otherwise.
     */
    fun getGravityNormStandardDeviationAsMeasurement(result: Acceleration): Boolean {
        return if (isGravityNormEstimated) {
            gravityNormEstimator.getNormStandardDeviationAsMeasurement(result)
        } else {
            false
        }
    }

    /**
     * Gets PSD (Power Spectral Density) of gravity norm expressed in (m^2 * s^-3).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityPsd
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.psd
        } else {
            null
        }

    /**
     * Gets root PSD (Power Spectral Density) of gravity norm expressed in (m * s^-1.5).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityRootPsd
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.rootPsd
        } else {
            null
        }

    /**
     * Indicates robust method used to solve accelerometer calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustMethod: RobustEstimatorMethod? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Confidence of estimated accelerometer calibration result expressed as a value between 0.0
     * and 1.0.
     * By default 99% of confidence is used, which indicates that with a probability of 99%
     * estimation will be accurate because chosen sub-samples will be inliers (in other terms,
     * outliers will be correctly discarded).
     * This property is only taken into account if a not-null [accelerometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0 (both
     * included).
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustConfidence: Double = ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value in 0.0..1.0)
            check(!running)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust accelerometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [accelerometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustMaxIterations: Int = ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0)
            check(!running)
            field = value
        }

    /**
     * Size of preliminary subsets picked while finding a robust accelerometer calibration solution.
     * By default this is the [minimumRequiredAccelerometerMeasurements], which results in the
     * smallest number of iterations to complete robust algorithms.
     * Larger values can be used to ensure that error in each preliminary solution is minimized
     * among more measurements (thus, softening the effect of outliers), but this comes at the
     * expense of larger number of iterations.
     * This properly is only taken into account if a not-null [accelerometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [minimumRequiredAccelerometerMeasurements] at the moment the setter is called.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustPreliminarySubsetSize: Int = 0
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value >= minimumRequiredAccelerometerMeasurements)
            check(!running)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * accelerometer calibration algorithms or not.
     * Threshold varies depending on chosen [accelerometerRobustMethod].
     * By default, if null is provided, the estimated [accelerometerBaseNoiseLevel] will be used to
     * determine a suitable threshold. Otherwise, if a value is provided, such value will be used
     * instead.
     * This properly is only taken into account if a not-null [accelerometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustThreshold: Double? = null
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value == null || value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Factor to be used respect estimated accelerometer base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [accelerometerBaseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustThresholdFactor: Double = DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to accelerometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustStopThresholdFactor: Double = DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Gets estimated accelerometer scale factors and cross coupling errors, or null if not
     * available.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    val estimatedAccelerometerMa
        get() = accelerometerInternalCalibrator?.estimatedMa

    /**
     * Gets estimated accelerometer x-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSx: Double?
        get() = accelerometerInternalCalibrator?.estimatedSx

    /**
     * Gets estimated accelerometer y-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSy: Double?
        get() = accelerometerInternalCalibrator?.estimatedSy

    /**
     * Gets estimated accelerometer z-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSz: Double?
        get() = accelerometerInternalCalibrator?.estimatedSz

    /**
     * Gets estimated accelerometer x-y cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMxy: Double?
        get() = accelerometerInternalCalibrator?.estimatedMxy

    /**
     * Gets estimated accelerometer x-z cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMxz: Double?
        get() = accelerometerInternalCalibrator?.estimatedMxz

    /**
     * Gets estimated accelerometer y-x cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMyx: Double?
        get() = accelerometerInternalCalibrator?.estimatedMyx

    /**
     * Gets estimated accelerometer y-z cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMyz: Double?
        get() = accelerometerInternalCalibrator?.estimatedMyz

    /**
     * Gets estimated accelerometer z-x cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMzx: Double?
        get() = accelerometerInternalCalibrator?.estimatedMzx

    /**
     * Gets estimated accelerometer z-y cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMzy: Double?
        get() = accelerometerInternalCalibrator?.estimatedMzy

    /**
     * Gets estimated covariance matrix for estimated accelerometer parameters or null if not
     * available.
     * When bias is known, diagonal elements of the covariance matrix contains variance
     * for the following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx,
     * mzy.
     * When bias is not known, diagonal elements of the covariance matrix contains
     * variance for the following parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, where bx, by, bz corresponds to bias or hard iron coordinates.
     */
    val estimatedAccelerometerCovariance: Matrix?
        get() = accelerometerInternalCalibrator?.estimatedCovariance

    /**
     * Gets estimated chi square value for accelerometer or null if not available.
     */
    val estimatedAccelerometerChiSq: Double?
        get() = accelerometerInternalCalibrator?.estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided accelerometer measurements or null if
     * not available.
     */
    val estimatedAccelerometerMse: Double?
        get() = accelerometerInternalCalibrator?.estimatedMse

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasX].
     */
    val estimatedAccelerometerBiasX: Double?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFx
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasX
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasY].
     */
    val estimatedAccelerometerBiasY: Double?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFy
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasY
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasZ].
     */
    val estimatedAccelerometerBiasZ: Double?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFz
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasZ
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasX].
     */
    val estimatedAccelerometerBiasXAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFxAsAcceleration
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasXAsAcceleration
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasX].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasXAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = accelerometerInternalCalibrator) {
            is UnknownBiasAccelerometerCalibrator -> {
                internalCalibrator.getEstimatedBiasFxAsAcceleration(result)
            }
            is KnownBiasAccelerometerCalibrator -> {
                internalCalibrator.getBiasXAsAcceleration(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasY].
     */
    val estimatedAccelerometerBiasYAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFyAsAcceleration
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasYAsAcceleration
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasY].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasYAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = accelerometerInternalCalibrator) {
            is UnknownBiasAccelerometerCalibrator -> {
                internalCalibrator.getEstimatedBiasFyAsAcceleration(result)
            }
            is KnownBiasAccelerometerCalibrator -> {
                internalCalibrator.getBiasYAsAcceleration(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasZ].
     */
    val estimatedAccelerometerBiasZAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasFzAsAcceleration
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasZAsAcceleration
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasZ].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasZAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = accelerometerInternalCalibrator) {
            is UnknownBiasAccelerometerCalibrator -> {
                internalCalibrator.getEstimatedBiasFzAsAcceleration(result)
            }
            is KnownBiasAccelerometerCalibrator -> {
                internalCalibrator.getBiasZAsAcceleration(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [accelerometerInitialBiasAsTriad].
     */
    val estimatedAccelerometerBiasAsTriad: AccelerationTriad?
        get() {
            return when (val internalCalibrator = accelerometerInternalCalibrator) {
                is UnknownBiasAccelerometerCalibrator -> {
                    internalCalibrator.estimatedBiasAsTriad
                }
                is KnownBiasAccelerometerCalibrator -> {
                    internalCalibrator.biasAsTriad
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [accelerometerInitialBiasAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasAsTriad(result: AccelerationTriad): Boolean {
        return when (val internalCalibrator = accelerometerInternalCalibrator) {
            is UnknownBiasAccelerometerCalibrator -> {
                internalCalibrator.getEstimatedBiasAsTriad(result)
            }
            is KnownBiasAccelerometerCalibrator -> {
                internalCalibrator.getBiasAsTriad(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in meters per
     * squared second (m/s^2), or null if not available.
     */
    val estimatedAccelerometerBiasStandardDeviationNorm: Double?
        get() {
            val internalCalibrator = accelerometerInternalCalibrator ?: return null
            return if (internalCalibrator is AccelerometerBiasUncertaintySource) {
                internalCalibrator.estimatedBiasStandardDeviationNorm
            } else {
                null
            }
        }

    /**
     * List of accelerometer measurements that have been collected so far to be used for
     * accelerometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val accelerometerMeasurements = mutableListOf<StandardDeviationBodyKinematics>()

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    override val isReadyToSolveCalibration
        get() = accelerometerMeasurements.size >= requiredMeasurements.coerceAtLeast(
            minimumRequiredMeasurements
        )

    /**
     * Starts calibrator.
     * This method starts collecting accelerometer measurements.
     * When calibrator is started, it begins with an initialization stage where accelerometer noise
     * is estimated while device remains static. If no location is provided, during initialization
     * gravity norm is also estimated.
     * Once initialization is completed, calibrator determines intervals where device remains static
     * when device has different poses, so that measurements are collected to solve calibration.
     * If [solveCalibrationWhenEnoughMeasurements] is true, calibration is automatically solved
     * once enough measurements are collected, otherwise a call to [calibrate] must be done to solve
     * calibration.
     *
     * @throws IllegalStateException if calibrator is already running or a sensor is missing
     * (either accelerometer or gravity if it is being used when no location is provided).
     */
    @Throws(IllegalStateException::class)
    override fun start() {
        super.start()
        if (isGravityNormEstimated) {
            gravityNormEstimator.start()
        }
    }

    /**
     * Stops calibrator.
     * When this is called, no more accelerometer or gravity measurements are collected.
     *
     * @param running specifies the running parameter to be set. This is true when stop occurs
     * internally during measurement collection to start solving calibration, otherwise is false
     * when calling public [stop] method.
     */
    override fun internalStop(running: Boolean) {
        if (gravityNormEstimator.running) {
            gravityNormEstimator.stop()
        }
        super.internalStop(running)
    }

    /**
     * Internally solves calibration using collected measurements without checking pre-requisites
     * (if either calibrator is already running or enough measurements are available).
     * This is called when calibration occurs automatically when enough measurements are collected.
     * When calling [calibrate] method, pre-requisites are checked before calling this method.
     */
    @Throws(IllegalStateException::class)
    override fun internalCalibrate(): Boolean {
        return try {
            calibrationSolvingStartedListener?.onCalibrationSolvingStarted(this)
            accelerometerInternalCalibrator?.calibrate()
            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(
                StaticIntervalAccelerometerCalibrator::class.qualifiedName,
                "Calibration estimation failed",
                e
            )
            errorListener?.onError(
                this,
                CalibratorErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION
            )
            running = false
            false
        }
    }

    /**
     * Resets calibrator to its initial state.
     */
    override fun reset() {
        accelerometerMeasurements.clear()

        gravityNorm = null
        accelerometerResultUnreliable = false
        accelerometerInitialBiasX = null
        accelerometerInitialBiasY = null
        accelerometerInitialBiasZ = null

        accelerometerInternalCalibrator = null
    }

    /**
     * Updates initial biases values when first accelerometer measurement is received, so
     * that hardware calibrated biases are retrieved if
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED] is used.
     *
     * @param bx x-coordinate of initial bias to be set expressed in meters per squared second
     * (m/s^2).
     * @param by y-coordinate of initial bias to be set expressed in meters per squared second
     * (m/s^2).
     * @param bz z-coordinate of initial bias to be set expressed in meters per squared second
     * (m/s^2).
     */
    private fun updateAccelerometerInitialBiases(bx: Float?, by: Float?, bz: Float?) {
        val initialBiasX: Double
        val initialBiasY: Double
        val initialBiasZ: Double
        if (bx != null && by != null && bz != null) {
            initialBiasX = bx.toDouble()
            initialBiasY = by.toDouble()
            initialBiasZ = bz.toDouble()
        } else {
            initialBiasX = 0.0
            initialBiasY = 0.0
            initialBiasZ = 0.0
        }

        accelerometerInitialBiasX = initialBiasX
        accelerometerInitialBiasY = initialBiasY
        accelerometerInitialBiasZ = initialBiasZ

        initialAccelerometerBiasAvailableListener?.onInitialBiasAvailable(
            this,
            initialBiasX,
            initialBiasY,
            initialBiasZ
        )
    }

    /**
     * Builds an internal accelerometer calibrator based on all provided parameters.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildAccelerometerInternalCalibrator(): AccelerometerNonLinearCalibrator {
        return AccelerometerInternalCalibratorBuilder(
            accelerometerMeasurements,
            accelerometerRobustPreliminarySubsetSize,
            minimumRequiredAccelerometerMeasurements,
            accelerometerRobustMethod,
            accelerometerRobustConfidence,
            accelerometerRobustMaxIterations,
            accelerometerRobustThreshold,
            accelerometerRobustThresholdFactor,
            accelerometerRobustStopThresholdFactor,
            location,
            gravityNorm,
            isAccelerometerGroundTruthInitialBias,
            isAccelerometerCommonAxisUsed,
            accelerometerInitialBiasX,
            accelerometerInitialBiasY,
            accelerometerInitialBiasZ,
            accelerometerInitialSx,
            accelerometerInitialSy,
            accelerometerInitialSz,
            accelerometerInitialMxy,
            accelerometerInitialMxz,
            accelerometerInitialMyx,
            accelerometerInitialMyz,
            accelerometerInitialMzx,
            accelerometerInitialMzy,
            accelerometerBaseNoiseLevel,
            accelerometerQualityScoreMapper
        ).build()
    }

    companion object {
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
         * By default this is 3.0 times [accelerometerBaseNoiseLevel], which considering the noise
         * level as the standard deviation of a Gaussian distribution, should account for 99% of
         * the cases.
         * Any measurement having an error greater than that in the estimated solution, will be
         * considered an outlier and be discarded.
         */
        const val DEFAULT_ROBUST_THRESHOLD_FACTOR = 3.0

        /**
         * Additional factor for robust methods based on LMedS or PROMedS, where factor is not
         * directly related to LMSE, but to a smaller value.
         */
        const val DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR = 1e-2

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope and bias is unknown.
         */
        private const val ACCELEROMETER_UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 9

        /**
         * Number of unknowns for the general calibration case when bias is unknown.
         */
        private const val ACCELEROMETER_UNKNOWN_BIAS_GENERAL_UNKNOWNS = 12

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope and bias is known.
         */
        private const val ACCELEROMETER_KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 6

        /**
         * Number of unknowns for the general calibration case when bias is known.
         */
        private const val ACCELEROMETER_KNOWN_BIAS_GENERAL_UNKNOWNS = 9

        /**
         * Required minimum number of measurements when common z-axis is assumed and bias is
         * unknown.
         */
        const val ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            ACCELEROMETER_UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when bias is unknown.
         */
        const val ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL =
            ACCELEROMETER_UNKNOWN_BIAS_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of measurements when common z-axis is assumed and bias is known.
         */
        const val ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            ACCELEROMETER_KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when bias is known.
         */
        const val ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL =
            ACCELEROMETER_KNOWN_BIAS_GENERAL_UNKNOWNS + 1
    }

    /**
     * Interface to notify when a new accelerometer calibration measurement is generated.
     */
    fun interface OnGeneratedAccelerometerMeasurementListener {
        /**
         * Called when a new accelerometer calibration measurement is generated.
         *
         * @param calibrator calibrator that raised the event.
         * @param measurement generated accelerometer calibration measurement.
         * @param measurementsFoundSoFar number of measurements that have been found so far.
         * @param requiredMeasurements required number of measurements to solve calibration.
         */
        fun onGeneratedAccelerometerMeasurement(
            calibrator: StaticIntervalAccelerometerCalibrator,
            measurement: StandardDeviationBodyKinematics,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    /**
     * Interface to notify when gravity norm estimation is unreliable.
     * This only happens if no location is provided and gravity sensor becomes unreliable.
     */
    fun interface OnUnreliableGravityEstimationListener {
        /**
         * Called when gravity norm estimation becomes unreliable.
         * This only happens if no location is provided and gravity sensor becomes unreliable.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onUnreliableGravityEstimation(calibrator: StaticIntervalAccelerometerCalibrator)
    }

    /**
     * Interface to notify when initial accelerometer bias guess is available.
     * If [isAccelerometerGroundTruthInitialBias] is true, then initial bias is considered the true
     * value after solving calibration, otherwise, initial bias is considered only an initial guess.
     */
    fun interface OnInitialAccelerometerBiasAvailableListener {
        /**
         * Called when initial accelerometer bias is available.
         * If [isAccelerometerGroundTruthInitialBias] is true, then initial bias is considered the
         * true value after solving calibration, otherwise, initial bias is considered only an
         * initial guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param biasX x-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasY y-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
         */
        fun onInitialBiasAvailable(
            calibrator: StaticIntervalAccelerometerCalibrator,
            biasX: Double,
            biasY: Double,
            biasZ: Double
        )
    }
}