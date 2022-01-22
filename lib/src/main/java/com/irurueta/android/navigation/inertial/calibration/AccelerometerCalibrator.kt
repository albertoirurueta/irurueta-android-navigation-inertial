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
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.StopMode
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AccelerometerBiasUncertaintySource
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.accelerometer.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.Time
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
 * @property accelerometerMeasurementListener listener to notify collected accelerometer
 * measurements.
 * @property gravityMeasurementListener listener to notify collected gravity measurements.
 * @property qualityScoreMapper mapper to convert collected measurements into quality scores,
 * based on the amount of standard deviation (the larger the variability, the worse the score
 * will be).
 */
class AccelerometerCalibrator private constructor(
    val context: Context,
    val sensorType: AccelerometerSensorCollector.SensorType,
    val sensorDelay: SensorDelay,
    val solveCalibrationWhenEnoughMeasurements: Boolean,
    var initializationStartedListener: OnInitializationStartedListener?,
    var initializationCompletedListener: OnInitializationCompletedListener?,
    var errorListener: OnErrorListener?,
    var unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener?,
    var initialBiasAvailableListener: OnInitialBiasAvailableListener?,
    var newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener?,
    var readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener?,
    var calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener?,
    var calibrationCompletedListener: OnCalibrationCompletedListener?,
    var stoppedListener: OnStoppedListener?,
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?,
    var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener?,
    var qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>
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
     * @param accelerometerMeasurementListener listener to notify collected accelerometer
     * measurements.
     * @param gravityMeasurementListener listener to notify collected gravity measurements.
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
        initializationStartedListener: OnInitializationStartedListener? = null,
        initializationCompletedListener: OnInitializationCompletedListener? = null,
        errorListener: OnErrorListener? = null,
        unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener? = null,
        initialBiasAvailableListener: OnInitialBiasAvailableListener? = null,
        newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener? = null,
        stoppedListener: OnStoppedListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null,
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
        accelerometerMeasurementListener,
        gravityMeasurementListener,
        qualityScoreMapper
    ) {
        this.isGroundTruthInitialBias = isGroundTruthInitialBias
        this.location = location
        requiredMeasurements = minimumRequiredMeasurements
        robustPreliminarySubsetSize = minimumRequiredMeasurements
    }

    /**
     * Listener used by internal interval detector to handle events when initialization starts.
     */
    private val intervalDetectorInitializationStartedListener =
        IntervalDetector.OnInitializationStartedListener {
            initializationStartedListener?.onInitializationStarted(
                this@AccelerometerCalibrator
            )
        }

    /**
     * Listener used by internal interval detector to handle events when initialization is
     * completed.
     */
    private val intervalDetectorInitializationCompletedListener =
        IntervalDetector.OnInitializationCompletedListener { _, _ ->
            gravityNorm = gravityNormEstimator.averageNorm
            initializationCompletedListener?.onInitializationCompleted(
                this@AccelerometerCalibrator
            )
        }

    /**
     * Listener used by the internal interval detector to handle errors during interval detection.
     */
    private val intervalDetectorErrorListener =
        IntervalDetector.OnErrorListener { _, reason ->
            stop()
            errorListener?.onError(this@AccelerometerCalibrator, mapErrorReason(reason))
        }

    /**
     * Listener used by the internal interval detector when a static period ends and a dynamic
     * period starts. This listener contains accumulated accelerometer average values during static
     * period, that will be used as a measurement to solve calibration.
     */
    private val intervalDetectorDynamicIntervalDetectedListener =
        IntervalDetector.OnDynamicIntervalDetectedListener { _, _, _, _, _, _, _, accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, accumulatedStdX, accumulatedStdY, accumulatedStdZ ->
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
                this@AccelerometerCalibrator, measurement, measurementsSize, reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@AccelerometerCalibrator
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
                this@AccelerometerCalibrator
            )
        }

    /**
     * Internal interval detector to detect periods when device remains static.
     */
    private val intervalDetector: IntervalDetector =
        IntervalDetector(
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
     * Indicates whether the interval detector has picked the first accelerometer measurement.
     */
    private val isFirstMeasurement =
        intervalDetector.numberOfProcessedMeasurements <= FIRST_MEASUREMENT

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
     * List of measurements that have been collected so far to be used
     * for calibration.
     * Items in returned list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    var measurements = mutableListOf<StandardDeviationBodyKinematics>()
        private set

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    val isReadyToSolveCalibration
        get() = measurements.size >= requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

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
    val initialBiasXAsAcceleration: Acceleration?
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
    fun getInitialBiasXAsAcceleration(result: Acceleration): Boolean {
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
    val initialBiasYAsAcceleration: Acceleration?
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
    fun getInitialBiasYAsAcceleration(result: Acceleration): Boolean {
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
    val initialBiasZAsAcceleration: Acceleration?
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
    fun getInitialBiasZAsAcceleration(result: Acceleration): Boolean {
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
     * Indicates whether calibrator is running.
     * While calibrator is running, calibrator parameters cannot be changed.
     */
    var running: Boolean = false
        private set

    /**
     * Indicates whether initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
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
     * This can be used to obtain aditional information about the sensor.
     */
    val gravitySensor
        get() = gravityNormEstimator.sensor

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous accelerometer noise level during initialization of the interval
     * detector. Window size must always be larger than allowed minimum value, which is 2 and must
     * have an odd value.
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
     * @throws IllegalArgumentException if provided value is zero or negative
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
     * @throws IllegalStateException if calibrator is currently running
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
     * per squared second (m/s^2).
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
     * Gets or sts overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var baseNoiseLevelAbsoluteThresholdAsAcceleration
        get() = intervalDetector.baseNoiseLevelAbsoluteThresholdAsAcceleration
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            check(!running)
            intervalDetector.baseNoiseLevelAbsoluteThresholdAsAcceleration = value
        }

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getBaseNoiseLevelAbsoluteThresholdAsAcceleration(result: Acceleration) {
        intervalDetector.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevel
        get() = intervalDetector.baseNoiseLevel

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelAsAcceleration
        get() = intervalDetector.baseNoiseLevelAsAcceleration

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsAcceleration(result: Acceleration): Boolean {
        return intervalDetector.getBaseNoiseLevelAsAcceleration(result)
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelPsd
        get() = intervalDetector.baseNoiseLevelPsd

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelRootPsd
        get() = intervalDetector.baseNoiseLevelRootPsd

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
     * This is only available once initialization is completed.
     */
    val threshold
        get() = intervalDetector.threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     */
    val thresholdAsAcceleration
        get() = intervalDetector.thresholdAsAcceleration

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getThresholdAsMeasurement(result: Acceleration): Boolean {
        return intervalDetector.getThresholdAsMeasurement(result)
    }

    /**
     * Gets average time interval between accelerometer samples expressed in seconds (s).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val averageTimeInterval
        get() = intervalDetector.averageTimeInterval

    /**
     * Gets average time interval between accelerometer samples.
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
     * @return true if result is available, false otherwise.
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
     * Indicates or specifies whether z-axis is assumed to be common for accelerometer and
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
    val minimumRequiredMeasurements: Int
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
     * Required number of measurements to be collected before calibration can start.
     * The number of required measurements must be greater than [minimumRequiredMeasurements],
     * otherwise at least [minimumRequiredMeasurements] will be collected before calibration can
     * start.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var requiredMeasurements: Int = minimumRequiredMeasurements
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
    var robustPreliminarySubsetSize: Int = minimumRequiredMeasurements
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
    val averageGravityNormAsAcceleration
        get() = if (isGravityNormEstimated) {
            gravityNormEstimator.averageNormAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated average gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    fun getAverageNormAsAcceleration(result: Acceleration): Boolean {
        return gravityNormEstimator.getAverageNormAsMeasurement(result)
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
    val gravityNormStandardDeviationAsAcceleration
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
    fun getGravityNormStandardDeviationAsAcceleration(result: Acceleration): Boolean {
        return gravityNormEstimator.getNormStandardDeviationAsMeasurement(result)
    }

    /**
     * Gets PSD (Power Spectral Density) of gravity norm expressed in (m^2 * s^-3).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityPsd
        get() = gravityNormEstimator.psd

    /**
     * Gets root PSD (Power Spectral Density) of gravity norm expressed in (m * s^-1.5).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityRootPsd
        get() = gravityNormEstimator.rootPsd

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
    val estimatedSx
        get() = internalCalibrator?.estimatedSx

    /**
     * Gets estimated y-axis scale factor or null if not available.
     */
    val estimatedSy
        get() = internalCalibrator?.estimatedSy

    /**
     * Gets estimated z-axis scale factor or null if not available.
     */
    val estimatedSz
        get() = internalCalibrator?.estimatedSz

    /**
     * Gets estimated x-y cross-coupling error or null if not available.
     */
    val estimatedMxy
        get() = internalCalibrator?.estimatedMxy

    /**
     * Gets estimated x-z cross-coupling error or null if not available.
     */
    val estimatedMxz
        get() = internalCalibrator?.estimatedMxz

    /**
     * Gets estimated y-x cross-coupling error or null if not available.
     */
    val estimatedMyx
        get() = internalCalibrator?.estimatedMyx

    /**
     * Gets estimated y-z cross-coupling error or null if not available.
     */
    val estimatedMyz
        get() = internalCalibrator?.estimatedMyz

    /**
     * Gets estimated z-x cross-coupling error or null if not available.
     */
    val estimatedMzx
        get() = internalCalibrator?.estimatedMzx

    /**
     * Gets estimated z-y cross-coupling error or null if not available.
     */
    val estimatedMzy
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
    val estimatedCovariance
        get() = internalCalibrator?.estimatedCovariance

    /**
     * Gets estimated chi square value or null if not available.
     */
    val estimatedChiSq
        get() = internalCalibrator?.estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided measurements or null if not available.
     */
    val estimatedMse
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
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasXAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasXAsAcceleration].
     */
    val estimatedBiasXAsAcceleration: Acceleration?
        get() {
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasXAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasXAsAcceleration].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasXAsAcceleration(result: Acceleration): Boolean {
        val internalCalibrator = this.internalCalibrator ?: return false
        return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasYAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasYAsAcceleration].
     */
    val estimatedBiasYAsAcceleration: Acceleration?
        get() {
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasYAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasYAsAcceleration].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasYAsAcceleration(result: Acceleration): Boolean {
        val internalCalibrator = this.internalCalibrator ?: return false
        return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, this will be equal to [initialBiasZAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasZAsAcceleration].
     */
    val estimatedBiasZAsAcceleration: Acceleration?
        get() {
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
     * If [isGroundTruthInitialBias] is true, result will be equal to [initialBiasZAsAcceleration],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialBiasZAsAcceleration].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedBiasZAsAcceleration(result: Acceleration): Boolean {
        val internalCalibrator = this.internalCalibrator ?: return false
        return when (internalCalibrator) {
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
            val internalCalibrator = this.internalCalibrator ?: return null
            return when (internalCalibrator) {
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
        val internalCalibrator = this.internalCalibrator ?: return false
        return when (internalCalibrator) {
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
     * Once initialization is completed, calibrator determines intervals where device remain static
     * when device has different poses, so that measurements are collected to solve calibration.
     * If [solveCalibrationWhenEnoughMeasurements] is true, calibration is automatically solved
     * once enough measurements are collected, otherwise a call to [calibrate] must be done to solve
     * calibration.
     *
     * @throws IllegalStateException if calibrator is already running or a sensor is missing
     * (either accelerometer or gravity if it is being used when no location is provided).
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        if (isGravityNormEstimated) {
            gravityNormEstimator.start()
        }
        intervalDetector.start()
    }

    /**
     * Stops calibrator.
     * When this is called, no more accelerometer or gravity measurements are collected.
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
     * solving calibration (which is notified using provided [errorListener]).
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
     * When this is called, no more accelerometer or gravity measurements are collected.
     *
     * @param running specifies the running parameter to be set. This is true when stop occurs
     * internally during measurement collection to start solving calibration, otherwise is false
     * when calling public [stop] method.
     */
    private fun internalStop(running: Boolean) {
        intervalDetector.stop()
        if (gravityNormEstimator.running) {
            gravityNormEstimator.stop()
        }
        this.running = running

        stoppedListener?.onStopped(this)
    }

    /**
     * Internally solves calibration using collected measurements without checking pre-requisites
     * (if either calibrator is already running or enough measurements are available).
     * This is called when calibration occurs automatically when enough measurements are collected.
     * When calling [calibrate] method, pre-requisites are checked before calling this method.
     */
    @Throws(IllegalStateException::class)
    private fun internalCalibrate(): Boolean {
        return try {
            calibrationSolvingStartedListener?.onCalibrationSolvingStarted(this)
            internalCalibrator?.calibrate()
            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(AccelerometerCalibrator::class.qualifiedName, "Calibration estimation failed", e)
            errorListener?.onError(this, ErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION)
            running = false
            false
        }
    }

    /**
     * Resets calibrator to its initial state.
     */
    private fun reset() {
        gravityNorm = null
        measurements.clear()
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
        return if (robustMethod == null) {
            buildNonRobustCalibrator()
        } else {
            buildRobustCalibrator()
        }
    }

    /**
     * Internally builds a non-robust accelerometer calibrator based on all provided parameters.
     *
     * @return an internal accelerometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildNonRobustCalibrator(): AccelerometerNonLinearCalibrator {
        val location = this.location
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
            } else if (gravityNorm != null) {
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
            } else if (gravityNorm != null) {
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

        throw IllegalStateException("No calibrator could be built.")
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
                buildRobustKnownBiasAndPositionCalibrator()
            } else {
                buildRobustKnownBiasAndGravityCalibrator()
            }
        } else {
            if (location != null) {
                buildRobustKnownPositionCalibrator()
            } else {
                buildRobustKnownGravityCalibrator()
            }
        }
    }

    /**
     * Internally build a robust accelerometer calibrator when bias and position is known.
     *
     * @return an internal accelerometer calibrator.
     */
    private fun buildRobustKnownBiasAndPositionCalibrator(): RobustKnownBiasAndPositionAccelerometerCalibrator {
        val location = this.location
        checkNotNull(location)

        val baseNoiseLevel = this.baseNoiseLevel
        checkNotNull(baseNoiseLevel)

        val robustThreshold = this.robustThreshold

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
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold and quality scores
        when (result) {
            is RANSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Internally build a robust accelerometer calibrator when bias and gravity is known.
     *
     * @return an internal accelerometer calibrator.
     */
    private fun buildRobustKnownBiasAndGravityCalibrator(): RobustKnownBiasAndGravityNormAccelerometerCalibrator {
        val gravityNorm = this.gravityNorm
        checkNotNull(gravityNorm)

        val baseNoiseLevel = this.baseNoiseLevel
        checkNotNull(baseNoiseLevel)

        val robustThreshold = this.robustThreshold

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
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold
        when (result) {
            is RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
            is LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
                result.qualityScores = buildQualityScores()
            }
        }

        return result
    }

    /**
     * Internally build a robust accelerometer calibrator when position is known.
     *
     * @return an internal accelerometer calibrator.
     */
    private fun buildRobustKnownPositionCalibrator(): RobustKnownPositionAccelerometerCalibrator {
        val location = this.location
        checkNotNull(location)

        val baseNoiseLevel = this.baseNoiseLevel
        checkNotNull(baseNoiseLevel)

        val robustThreshold = this.robustThreshold

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
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold
        when (result) {
            is RANSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is LMedSRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownPositionAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
        }

        return result
    }

    /**
     * Internally build a robust accelerometer calibrator when gravity is known.
     *
     * @return an internal accelerometer calibrator.
     */
    private fun buildRobustKnownGravityCalibrator(): RobustKnownGravityNormAccelerometerCalibrator {
        val gravityNorm = this.gravityNorm
        checkNotNull(gravityNorm)

        val baseNoiseLevel = this.baseNoiseLevel
        checkNotNull(baseNoiseLevel)

        val robustThreshold = this.robustThreshold

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
        result.preliminarySubsetSize =
            robustPreliminarySubsetSize.coerceAtLeast(minimumRequiredMeasurements)

        // set threshold
        when (result) {
            is RANSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is MSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is PROSACRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.threshold = robustThreshold
                } else {
                    result.threshold = robustThresholdFactor * baseNoiseLevel
                }
            }
            is LMedSRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
            is PROMedSRobustKnownGravityNormAccelerometerCalibrator -> {
                if (robustThreshold != null) {
                    result.stopThreshold = robustThreshold
                } else {
                    result.stopThreshold =
                        robustThresholdFactor * robustStopThresholdFactor * baseNoiseLevel
                }
            }
        }

        return result
    }

    /**
     * Builds required quality scroes for PROSAC and PROMedS robust methods.
     * Quality scores are build for each measurement. By default the standard deviation
     * of each measurement is taken into account, so that the larger the standard deviation
     * the poorer the measurement is considered (lower score).
     *
     * @return build qualitys core array.
     */
    private fun buildQualityScores(): DoubleArray {
        val size = measurements.size
        val qualityScores = DoubleArray(size)
        measurements.forEachIndexed { index, measurement ->
            qualityScores[index] = qualityScoreMapper.map(measurement)
        }
        return qualityScores
    }

    companion object {
        /**
         * Indicates when first accelerometer measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1

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
         * Indicates whether by default a common z-axis is assumed for both accelerometer and
         * gyroscope.
         */
        const val DEFAULT_USE_COMMON_Z_AXIS = false

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

        /**
         * Maps interval detector error reasons into error reasons returned by
         * [AccelerometerCalibrator].
         */
        private fun mapErrorReason(reason: IntervalDetector.ErrorReason): ErrorReason {
            return when (reason) {
                IntervalDetector.ErrorReason.UNRELIABLE_SENSOR
                -> ErrorReason.UNRELIABLE_SENSOR
                IntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                -> ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                IntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                -> ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            }
        }
    }

    /**
     * Interface to notify when calibrator starts initialization.
     */
    fun interface OnInitializationStartedListener {
        /**
         * Called when calibrator starts initialization to determine base noise level when device
         * remains static.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onInitializationStarted(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when calibrator successfully completes initialization.
     */
    fun interface OnInitializationCompletedListener {
        /**
         * Called when calibrator successfully completes initialization to determine base noise
         * level when device remains static.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onInitializationCompleted(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when an error occurs.
     */
    fun interface OnErrorListener {
        /**
         * Called when an error is detected, either at initialization because excessive movement
         * forces are detected, because accelerometer sensor becomes unreliable or because obtained
         * measurements produce a numerically unstable calibration solution.
         *
         * @param calibrator calibrator that raised the event.
         * @param errorReason reason why error was detected.
         */
        fun onError(calibrator: AccelerometerCalibrator, errorReason: ErrorReason)
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
        fun onUnreliableGravityEstimation(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when initial bias guess is available.
     * If [isGroundTruthInitialBias] is true, then initial bias is considered the true value after solving
     * calibration, otherwise, initial bias is considered only an initial guess.
     */
    fun interface OnInitialBiasAvailableListener {
        /**
         * Called when initial bias is available.
         * If [isGroundTruthInitialBias] is true, then initial bias is considered the true value after solving
         * calibration, otherwise, initial bias is considered only an initial guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param biasX x-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasY y-coordinate of bias expressed in meters per squared second (m/s^2).
         * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
         */
        fun onInitialBiasAvailable(
            calibrator: AccelerometerCalibrator,
            biasX: Double,
            biasY: Double,
            biasZ: Double
        )
    }

    /**
     * Interface to notify when a new measurement is obtained for calibration purposes.
     * Such measurements are determined each time a static period finishes (when device stops
     * being static).
     * When enough of this measurements are obtained, calibration can actually be solved.
     */
    fun interface OnNewCalibrationMeasurementAvailableListener {
        /**
         * Called when a new measurement for calibration is found.
         * A new measurement each time a static period finishes (when device stops being static for
         * given amount of time).
         * When enough of this measurements are obtained, calibration can actually be solved.
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
            calibrator: AccelerometerCalibrator,
            newMeasurement: StandardDeviationBodyKinematics,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    /**
     * Interface to notify when enough measurements are obtained to start solving calibration.
     */
    fun interface OnReadyToSolveCalibrationListener {
        /**
         * Called when enough measurements are obtained to start solving calibration.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onReadyToSolveCalibration(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when calibration starts being solved.
     */
    fun interface OnCalibrationSolvingStartedListener {
        /**
         * Called when calibration starts being solved after enough measurements are found.
         * Calibration can automatically started when enough measurements are available if
         * [solveCalibrationWhenEnoughMeasurements] is true, otherwise [calibrate] must be called
         * after enough measurements are found, which raises this event.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onCalibrationSolvingStarted(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when calibration is solved and completed.
     */
    fun interface OnCalibrationCompletedListener {
        /**
         * Called when calibration successfully completes.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onCalibrationCompleted(calibrator: AccelerometerCalibrator)
    }

    /**
     * Interface to notify when measurement collection stops.
     * This happens automatically when enough measurements are found after periods when
     * device stops being static, or if an error occurs.
     */
    fun interface OnStoppedListener {
        /**
         * Called when calibration an measurement collection stops.
         * This happens automatically when enough measurements are found after periods when
         * device stops being static, or if an error occurs.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onStopped(calibrator: AccelerometerCalibrator)
    }

    /**
     * Reasons why this calibrator can fail
     */
    enum class ErrorReason {
        /**
         * If a sudden movement is detected during initialization.
         */
        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        /**
         * If overall noise level is excessive during initialization.
         */
        OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

        /**
         * If sensor becomes unreliable.
         */
        UNRELIABLE_SENSOR,

        /**
         * Occurs if obtained measurements cannot yield a numerically stable solution
         * during calibration estimation.
         */
        NUMERICAL_INSTABILITY_DURING_CALIBRATION
    }
}