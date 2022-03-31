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
import com.irurueta.android.navigation.inertial.calibration.intervals.AccelerometerIntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.StopMode
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AccelerometerBiasUncertaintySource
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Collects accelerometer measurements by detecting periods when device remains static,
 * and using such static periods, measurements are obtained to solve calibration parameters.
 *
 * @property context Android context.
 * @property sensorType One of the supported accelerometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
 * enough measurements are available, false otherwise.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization completes.
 * @property errorListener listener to notify errors.
 * @property unreliableGravityNormEstimationListener listener to notify when gravity norm estimation
 * becomes unreliable. This is only used if no location is provided.
 * @property initialBiasAvailableListener listener to notify when a guess of bias values is
 * obtained.
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
 */
class SingleSensorStaticIntervalAccelerometerCalibrator private constructor(
    context: Context,
    val sensorType: AccelerometerSensorCollector.SensorType,
    sensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: OnInitializationStartedListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    initializationCompletedListener: OnInitializationCompletedListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    errorListener: OnErrorListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    var unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener?,
    var initialBiasAvailableListener: OnInitialBiasAvailableListener?,
    newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener<SingleSensorStaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>?,
    readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    calibrationCompletedListener: OnCalibrationCompletedListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    stoppedListener: OnStoppedListener<SingleSensorStaticIntervalAccelerometerCalibrator>?,
    qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>
) : SingleSensorStaticIntervalCalibrator<SingleSensorStaticIntervalAccelerometerCalibrator,
        StandardDeviationBodyKinematics, AccelerometerIntervalDetector, AccelerationUnit,
        Acceleration, AccelerationTriad>(
    context,
    sensorDelay, solveCalibrationWhenEnoughMeasurements, initializationStartedListener,
    initializationCompletedListener, errorListener, newCalibrationMeasurementAvailableListener,
    readyToSolveCalibrationListener, calibrationSolvingStartedListener,
    calibrationCompletedListener, stoppedListener, qualityScoreMapper
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param sensorType One of the supported accelerometer sensor types.
     * @param sensorDelay Delay of sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param isGroundTruthInitialBias true if estimated bias is assumed to be the true value,
     * false if estimated bias is assumed to be only an initial guess. When [sensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER], bias guess is zero,
     * otherwise when it is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED],
     * bias guess is the device calibrated values.
     * @param location location where device is located at. When location is provided, gravity norm
     * is assumed to be the theoretical value determined by WGS84 Earth model, otherwise, if no
     * location is provided, gravity norm is estimated using a gravity sensor.
     * @param initializationStartedListener listener to notify when initialization starts.
     * @param initializationCompletedListener listener to notify when initialization completes.
     * @param errorListener listener to notify errors.
     * @param unreliableGravityNormEstimationListener listener to notify when gravity norm
     * estimation becomes unreliable. This is only used if no location is provided.
     * @param initialBiasAvailableListener listener to notify when a guess of bias values is
     * obtained.
     * @param newCalibrationMeasurementAvailableListener listener to notify when a new calibration
     * measurement is obtained.
     * @param readyToSolveCalibrationListener listener to notify when calibrator is ready to be
     * solved.
     * @param calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @param calibrationCompletedListener listener to notify when calibration is successfully
     * completed.
     * @param stoppedListener listener to notify when measurement collection stops.
     * @param qualityScoreMapper mapper to convert collected measurements into quality scores,
     * based on the amount of standard deviation (the larger the variability, the worse the score
     * will be).
     */
    constructor(
        context: Context,
        sensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isGroundTruthInitialBias: Boolean = false,
        location: Location? = null,
        initializationStartedListener: OnInitializationStartedListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        initializationCompletedListener: OnInitializationCompletedListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        errorListener: OnErrorListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener? = null,
        initialBiasAvailableListener: OnInitialBiasAvailableListener? = null,
        newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener<SingleSensorStaticIntervalAccelerometerCalibrator, StandardDeviationBodyKinematics>? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        stoppedListener: OnStoppedListener<SingleSensorStaticIntervalAccelerometerCalibrator>? = null,
        qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics> = DefaultAccelerometerQualityScoreMapper()
    ) : this(
        context,
        sensorType,
        sensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        unreliableGravityNormEstimationListener,
        initialBiasAvailableListener,
        newCalibrationMeasurementAvailableListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        qualityScoreMapper
    ) {
        this.isGroundTruthInitialBias = isGroundTruthInitialBias
        this.location = location
        requiredMeasurements = minimumRequiredMeasurements
        robustPreliminarySubsetSize = minimumRequiredMeasurements
    }


    /**
     * Listener used by internal interval detector to handle events when initialization is
     * completed.
     */
    override val intervalDetectorInitializationCompletedListener =
        IntervalDetector.OnInitializationCompletedListener<AccelerometerIntervalDetector> { _, _ ->
            gravityNorm = gravityNormEstimator.averageNorm
            initializationCompletedListener?.onInitializationCompleted(
                this@SingleSensorStaticIntervalAccelerometerCalibrator
            )
        }

    /**
     * Listener used by the internal interval detector when a static period ends and a dynamic
     * period starts. This listener contains accumulated accelerometer average values during static
     * period, that will be used as a measurement to solve calibration.
     */
    private val intervalDetectorDynamicIntervalDetectedListener =
        IntervalDetector.OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector> { _, _, _, _, _, _, _, accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, accumulatedStdX, accumulatedStdY, accumulatedStdZ ->
            // add one measurement
            val kinematics = BodyKinematics(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ)
            val stdNorm = sqrt(
                accumulatedStdX.pow(2.0) + accumulatedStdY.pow(2.0) + accumulatedStdZ.pow(2.0)
            )
            val measurement = StandardDeviationBodyKinematics(kinematics, stdNorm, 0.0)
            measurements.add(measurement)

            val reqMeasurements =
                requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = measurements.size

            newCalibrationMeasurementAvailableListener?.onNewCalibrationMeasurementAvailable(
                this@SingleSensorStaticIntervalAccelerometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@SingleSensorStaticIntervalAccelerometerCalibrator
                )

                // stop interval detector since no more measurements need to be collected
                internalStop(true)

                // build calibrator
                internalCalibrator = buildInternalCalibrator()

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
    private val intervalDetectorMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { _, _, _, bx, by, bz, _, _ ->
            if (isFirstMeasurement) {
                updateInitialBiases(bx, by, bz)
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
            resultUnreliable = true
            unreliableGravityNormEstimationListener?.onUnreliableGravityEstimation(
                this@SingleSensorStaticIntervalAccelerometerCalibrator
            )
        }

    /**
     * Internal interval detector to detect periods when device remains static.
     */
    override val intervalDetector: AccelerometerIntervalDetector =
        AccelerometerIntervalDetector(
            context,
            sensorType,
            sensorDelay,
            intervalDetectorInitializationStartedListener,
            intervalDetectorInitializationCompletedListener,
            intervalDetectorErrorListener,
            dynamicIntervalDetectedListener = intervalDetectorDynamicIntervalDetectedListener,
            measurementListener = intervalDetectorMeasurementListener
        )

    /**
     * Gravity norm estimator. It is used to estimate gravity norm when no location is provided.
     */
    private val gravityNormEstimator: GravityNormEstimator =
        GravityNormEstimator(
            context,
            sensorDelay,
            initialStaticSamples,
            stopMode = StopMode.MAX_SAMPLES_ONLY,
            completedListener = gravityNormCompletedListener,
            unreliableListener = gravityNormUnreliableListener
        )

    /**
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected at static intervals.
     */
    private var internalCalibrator: AccelerometerNonLinearCalibrator? = null

    /**
     * Contains gravity norm (either obtained by the gravity sensor, or determined by current
     * location using WGS84 Earth model). Expressed in meters per squared second (m/s^2).
     */
    var gravityNorm: Double? = null
        private set

    /**
     * Indicates if result is unreliable. This can happen if no location is provided and gravity
     * estimation becomes unreliable. When this happens result of calibration should probably be
     * discarded.
     */
    var resultUnreliable = false
        private set

    /**
     * Gets X-coordinate of bias used as an initial guess and expressed in meters per squared second
     * (m/s^2).
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasX] will be
     * equal to this value, otherwise [estimatedBiasX] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasX].
     */
    var initialBiasX: Double? = null
        private set

    /**
     * Gets Y-coordinate of bias used as an initial guess and expressed in meters per squared second
     * (m/s^2).
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasY] will be
     * equal to this value, otherwise [estimatedBiasY] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasY].
     */
    var initialBiasY: Double? = null
        private set

    /**
     * Gets Z-coordinate of bias used as an initial guess and expressed in meters per squared second
     * (m/s^2).
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasZ] will be
     * equal to this value, otherwise [estimatedBiasZ] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasZ].
     */
    var initialBiasZ: Double? = null
        private set

    /**
     * Gets X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasX] will be
     * equal to this value, otherwise [estimatedBiasX] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasX].
     */
    val initialBiasXAsMeasurement: Acceleration?
        get() {
            val initialBiasX = this.initialBiasX ?: return null
            return Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasX] will be
     * equal to this value, otherwise [estimatedBiasX] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasX].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getInitialBiasXAsMeasurement(result: Acceleration): Boolean {
        val initialBiasX = this.initialBiasX
        return if (initialBiasX != null) {
            result.value = initialBiasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasY] will be
     * equal to this value, otherwise [estimatedBiasY] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasY].
     */
    val initialBiasYAsMeasurement: Acceleration?
        get() {
            val initialBiasY = this.initialBiasY ?: return null
            return Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasY] will be
     * equal to this value, otherwise [estimatedBiasY] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasY].
     */
    fun getInitialBiasYAsMeasurement(result: Acceleration): Boolean {
        val initialBiasY = this.initialBiasY
        return if (initialBiasY != null) {
            result.value = initialBiasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasZ] will be
     * equal to this value, otherwise [estimatedBiasZ] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasZ].
     */
    val initialBiasZAsMeasurement: Acceleration?
        get() {
            val initialBiasZ = this.initialBiasZ ?: return null
            return Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }

    /**
     * Gets Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasZ] will be
     * equal to this value, otherwise [estimatedBiasZ] will be the estimated bias after solving
     * calibration, which will differ from [estimatedBiasZ].
     */
    fun getInitialBiasZAsMeasurement(result: Acceleration): Boolean {
        val initialBiasZ = this.initialBiasZ
        return if (initialBiasZ != null) {
            result.value = initialBiasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets initial bias coordinates used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the values used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasAsTriad]
     * will be equal to this value, otherwise [estimatedBiasAsTriad] will be the estimated bias
     * after solving calibration, which will differ from [estimatedBiasAsTriad].
     */
    val initialBiasAsTriad: AccelerationTriad?
        get() {
            val initialBiasX = this.initialBiasX
            val initialBiasY = this.initialBiasY
            val initialBiasZ = this.initialBiasZ
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
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], this
     * will be equal to the values used internally by the device as part of the accelerometer
     * hardware calibration.
     * If [sensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isGroundTruthInitialBias] is true, this is assumed to be the true bias, and [estimatedBiasAsTriad]
     * will be equal to this value, otherwise [estimatedBiasAsTriad] will be the estimated bias
     * after solving calibration, which will differ from [estimatedBiasAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getInitialBiasAsTriad(result: AccelerationTriad): Boolean {
        val initialBiasX = this.initialBiasX
        val initialBiasY = this.initialBiasY
        val initialBiasZ = this.initialBiasZ
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
     * Indicates whether initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isGroundTruthInitialBias: Boolean = false
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
        get() = intervalDetector.sensor

    /**
     * Gets gravity sensor being used for gravity estimation.
     * This can be used to obtain additional information about the sensor.
     */
    val gravitySensor
        get() = gravityNormEstimator.sensor

    /**
     * Gets or sets initial scaling factors and cross couping errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var initialMa: Matrix
        get() {
            val result = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
            getInitialMa(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            initialSx = value.getElementAtIndex(0)
            initialMyx = value.getElementAtIndex(1)
            initialMzx = value.getElementAtIndex(2)

            initialMxy = value.getElementAtIndex(3)
            initialSy = value.getElementAtIndex(4)
            initialMzy = value.getElementAtIndex(5)

            initialMxz = value.getElementAtIndex(6)
            initialMyz = value.getElementAtIndex(7)
            initialSz = value.getElementAtIndex(8)
        }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getInitialMa(result: Matrix) {
        require(result.rows == BodyKinematics.COMPONENTS && result.columns == BodyKinematics.COMPONENTS)

        result.setElementAtIndex(0, initialSx)
        result.setElementAtIndex(1, initialMyx)
        result.setElementAtIndex(2, initialMzx)

        result.setElementAtIndex(3, initialMxy)
        result.setElementAtIndex(4, initialSy)
        result.setElementAtIndex(5, initialMzy)

        result.setElementAtIndex(6, initialMxz)
        result.setElementAtIndex(7, initialMyz)
        result.setElementAtIndex(8, initialSz)
    }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = if (isCommonAxisUsed) {
            if (isGroundTruthInitialBias) {
                KNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isGroundTruthInitialBias) {
                KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            }
        }

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
    val estimatedMa
        get() = internalCalibrator?.estimatedMa

    /**
     * Gets estimated x-axis scale factor or null if not available.
     */
    override val estimatedSx
        get() = internalCalibrator?.estimatedSx

    /**
     * Gets estimated y-axis scale factor or null if not available.
     */
    override val estimatedSy
        get() = internalCalibrator?.estimatedSy

    /**
     * Gets estimated z-axis scale factor or null if not available.
     */
    override val estimatedSz
        get() = internalCalibrator?.estimatedSz

    /**
     * Gets estimated x-y cross-coupling error or null if not available.
     */
    override val estimatedMxy
        get() = internalCalibrator?.estimatedMxy

    /**
     * Gets estimated x-z cross-coupling error or null if not available.
     */
    override val estimatedMxz
        get() = internalCalibrator?.estimatedMxz

    /**
     * Gets estimated y-x cross-coupling error or null if not available.
     */
    override val estimatedMyx
        get() = internalCalibrator?.estimatedMyx

    /**
     * Gets estimated y-z cross-coupling error or null if not available.
     */
    override val estimatedMyz
        get() = internalCalibrator?.estimatedMyz

    /**
     * Gets estimated z-x cross-coupling error or null if not available.
     */
    override val estimatedMzx
        get() = internalCalibrator?.estimatedMzx

    /**
     * Gets estimated z-y cross-coupling error or null if not available.
     */
    override val estimatedMzy
        get() = internalCalibrator?.estimatedMzy

    /**
     * Gets estimated covariance matrix for estimated accelerometer parameters or null if not
     * available.
     * When bias is known, diagonal elements of the covariance matrix contains variance for the
     * following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx, mzy.
     * When bias is not known, diagonal elements of the covariance matrix contains variance for
     * the following parameters (following indicated order): bx, by, bz, sx, sy, sz, mxy, mxz,
     * myx, myz, mzx, mzy.
     */
    override val estimatedCovariance
        get() = internalCalibrator?.estimatedCovariance

    /**
     * Gets estimated chi square value or null if not available.
     */
    override val estimatedChiSq
        get() = internalCalibrator?.estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided measurements or null if not available.
     */
    override val estimatedMse
        get() = internalCalibrator?.estimatedMse

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasX], otherwise it
     * will be the estimated value obtained after solving calibration, that might differ from
     * [initialBiasX].
     */
    val estimatedBiasX: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasY], otherwise it
     * will be the estimated value obtained after solving calibration, that might differ from
     * [initialBiasY].
     */
    val estimatedBiasY: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasZ], otherwise it
     * will be the estimated value obtained after solving calibration, that might differ from
     * [initialBiasZ].
     */
    val estimatedBiasZ: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasXAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasXAsMeasurement].
     */
    val estimatedBiasXAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasXAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasXAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasXAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasYAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasYAsMeasurement].
     */
    val estimatedBiasYAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasYAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasYAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasYAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasZAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasZAsMeasurement].
     */
    val estimatedBiasZAsMeasurement: Acceleration?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasZAsMeasurement],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasZAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasZAsMeasurement(result: Acceleration): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasAsTriad],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasAsTriad].
     */
    val estimatedBiasAsTriad: AccelerationTriad?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasAsTriad],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasAsTriad(result: AccelerationTriad): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
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
    val estimatedBiasStandardDeviationNorm: Double?
        get() {
            val internalCalibrator = this.internalCalibrator ?: return null
            return if (internalCalibrator is AccelerometerBiasUncertaintySource) {
                internalCalibrator.estimatedBiasStandardDeviationNorm
            } else {
                null
            }
        }

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
            internalCalibrator?.calibrate()
            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(
                SingleSensorStaticIntervalAccelerometerCalibrator::class.qualifiedName,
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
        super.reset()
        gravityNorm = null
        resultUnreliable = false
        initialBiasX = null
        initialBiasY = null
        initialBiasZ = null

        internalCalibrator = null
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
    private fun updateInitialBiases(bx: Float?, by: Float?, bz: Float?) {
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

        this.initialBiasX = initialBiasX
        this.initialBiasY = initialBiasY
        this.initialBiasZ = initialBiasZ

        initialBiasAvailableListener?.onInitialBiasAvailable(
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
    private fun buildInternalCalibrator(): AccelerometerNonLinearCalibrator {
        return AccelerometerInternalCalibratorBuilder(
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
        ).build()
    }

    companion object {
        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope and bias is unknown.
         */
        private const val UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 9

        /**
         * Number of unknowns for the general calibration case when bias is unknown.
         */
        private const val UNKNOWN_BIAS_GENERAL_UNKNOWNS = 12

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope and bias is known.
         */
        private const val KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 6

        /**
         * Number of unknowns for the general calibration case when bias is known.
         */
        private const val KNOWN_BIAS_GENERAL_UNKNOWNS = 9

        /**
         * Required minimum number of measurements when common z-axis is assumed and bias is
         * unknown.
         */
        const val UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when bias is unknown.
         */
        const val UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL = UNKNOWN_BIAS_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of measurements when common z-axis is assumed and bias is known.
         */
        const val KNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when bias is known.
         */
        const val KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL = KNOWN_BIAS_GENERAL_UNKNOWNS + 1

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
        fun onUnreliableGravityEstimation(calibrator: SingleSensorStaticIntervalAccelerometerCalibrator)
    }

    /**
     * Interface to notify when initial bias guess is available.
     * If [isGroundTruthInitialBias] is true, then initial bias is considered the true value after
     * solving calibration, otherwise, initial bias is considered only an initial guess.
     */
    fun interface OnInitialBiasAvailableListener {
        /**
         * Called when initial bias is available.
         * If [isGroundTruthInitialBias] is true, then initial bias is considered the true value
         * after solving calibration, otherwise, initial bias is considered only an initial guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param biasX x-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasY y-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
         */
        fun onInitialBiasAvailable(
            calibrator: SingleSensorStaticIntervalAccelerometerCalibrator,
            biasX: Double,
            biasY: Double,
            biasZ: Double
        )
    }
}