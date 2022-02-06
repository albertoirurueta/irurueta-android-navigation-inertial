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
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.MagnetometerIntervalDetector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.*
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.Time
import java.util.*
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Collects magnetometer measurements by detecting periods when device captures a constant magnetic
 * field (typically when device orientation remains static).
 * Such static periods are used to obtain measurements and solve calibration parameters.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
 * enough measurements are available, false otherwise.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization completes.
 * @property errorListener listener to notify errors.
 * @property initialHardIronAvailableListener listener to notify when a guess of hard iron values is
 * obtained.
 * @property newCalibrationMeasurementAvailableListener listener to notify when a new calibration
 * measurement is obtained.
 * @property readyToSolveCalibrationListener listener to notify when calibrator is ready to be
 * solved.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration is successfully
 * completed.
 * @property stoppedListener listener to notify when measurement collection stops.
 * @property magnetometerMeasurementListener listener to notify collected magnetometer measurements.
 * @property qualityScoreMapper mapper to convert collected measurements into quality scores,
 * based on the amount ot standard deviation (the larger the variability, the worse the score
 * will be).
 */
class StaticIntervalMagnetometerCalibrator private constructor(
    val context: Context,
    val sensorType: MagnetometerSensorCollector.SensorType,
    val sensorDelay: SensorDelay,
    val solveCalibrationWhenEnoughMeasurements: Boolean,
    var initializationStartedListener: OnInitializationStartedListener?,
    var initializationCompletedListener: OnInitializationCompletedListener?,
    var errorListener: OnErrorListener?,
    var initialHardIronAvailableListener: OnInitialHardIronAvailableListener?,
    var newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener?,
    var readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener?,
    var calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener?,
    var calibrationCompletedListener: OnCalibrationCompletedListener?,
    var stoppedListener: OnStoppedListener?,
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener?,
    var qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location Current device location.
     * @param timestamp Current timestamp
     * @param worldMagneticModel Earth's magnetic model. Null to use default model.
     * @param sensorType One of the supported magnetometer sensor types.
     * @param sensorDelay Delay of sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param initializationStartedListener listener to notify when initialization starts.
     * @param initializationCompletedListener listener to notify when initialization completes.
     * @param errorListener listener to notify errors.
     * @param initialHardIronAvailableListener listener to notify when a guess of hard iron values is
     * obtained.
     * @param newCalibrationMeasurementAvailableListener listener to notify when a new calibration
     * measurement is obtained.
     * @param readyToSolveCalibrationListener listener to notify when calibrator is ready to be
     * solved.
     * @param calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @param calibrationCompletedListener listener to notify when calibration is successfully
     * completed.
     * @param stoppedListener listener to notify when measurement collection stops.
     * @param magnetometerMeasurementListener listener to notify collected magnetometer measurements.
     * @param qualityScoreMapper mapper to convert collected measurements into quality scores,
     * based on the amount ot standard deviation (the larger the variability, the worse the score
     * will be).
     */
    constructor(
        context: Context,
        location: Location,
        timestamp: Date = Date(),
        worldMagneticModel: WorldMagneticModel? = null,
        sensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: OnInitializationStartedListener? = null,
        initializationCompletedListener: OnInitializationCompletedListener? = null,
        errorListener: OnErrorListener? = null,
        initialHardIronAvailableListener: OnInitialHardIronAvailableListener? = null,
        newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener? = null,
        stoppedListener: OnStoppedListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> = DefaultMagnetometerQualityScoreMapper()
    ) : this(
        context,
        sensorType,
        sensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        initialHardIronAvailableListener,
        newCalibrationMeasurementAvailableListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        magnetometerMeasurementListener,
        qualityScoreMapper
    ) {
        this.location = location
        this.timestamp = timestamp
        this.worldMagneticModel = worldMagneticModel
        this.isGroundTruthInitialHardIron = isGroundTruthInitialHardIron
        requiredMeasurements = minimumRequiredMeasurements
        robustPreliminarySubsetSize = minimumRequiredMeasurements
    }

    /**
     * Listener used by internal detector to handle events when initialization starts.
     */
    private val intervalDetectorInitializationStartedListener =
        IntervalDetector.OnInitializationStartedListener<MagnetometerIntervalDetector> {
            initializationStartedListener?.onInitializationStarted(
                this@StaticIntervalMagnetometerCalibrator
            )
        }

    /**
     * Listener used by internal interval detector to handle events when initialization is
     * completed.
     */
    private val intervalDetectorInitializationCompletedListener =
        IntervalDetector.OnInitializationCompletedListener<MagnetometerIntervalDetector> { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(
                this@StaticIntervalMagnetometerCalibrator
            )
        }

    /**
     * Listener used by the internal interval detector to handle errors during interval detection.
     */
    private val intervalDetectorErrorListener =
        IntervalDetector.OnErrorListener<MagnetometerIntervalDetector> { _, reason ->
            stop()
            errorListener?.onError(
                this@StaticIntervalMagnetometerCalibrator,
                mapErrorReason(reason)
            )
        }

    /**
     * Listener used by the internal interval detector when a static period ends and a dynamic
     * period starts. This listener contains accumulated magnetometer average values during static
     * period, that will be used as a measurement to solve calibration.
     */
    private val intervalDetectorDynamicIntervalDetectedListener =
        IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector> { _, _, _, _, _, _, _, accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, accumulatedStdX, accumulatedStdY, accumulatedStdZ ->
            // add one measurement
            val b = BodyMagneticFluxDensity(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ)
            val stdNorm = sqrt(
                accumulatedStdX.pow(2.0) + accumulatedStdY.pow(2.0) + accumulatedStdZ.pow(2.0)
            )
            val measurement = StandardDeviationBodyMagneticFluxDensity(b, stdNorm)
            measurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = measurements.size

            newCalibrationMeasurementAvailableListener?.onNewCalibrationMeasurementAvailable(
                this@StaticIntervalMagnetometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@StaticIntervalMagnetometerCalibrator
                )

                // stop interval detector since no norm measurements need to be collected
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
     * Listener for magnetometer sensor collector.
     * This is used to determine device calibration and obtain initial guesses
     * for magnetometer hard iron (only available if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used, otherwise zero
     * hard iron is assumed on initial guess).
     */
    private val intervalDetectorMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { _, _, _, hardIronX, hardIronY, hardIronZ, _, _ ->
            if (isFirstMeasurement) {
                updateInitialHardIrons(hardIronX, hardIronY, hardIronZ)
            }
        }

    /**
     * Internal interval detector to detect periods when device remains static.
     */
    private val intervalDetector: MagnetometerIntervalDetector =
        MagnetometerIntervalDetector(
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
     * Indicates whether the interval detector has picked the first magnetometer measurement.
     */
    private val isFirstMeasurement
        get() = intervalDetector.numberOfProcessedMeasurements <= FIRST_MEASUREMENT

    /**
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected at static intervals.
     */
    private var internalCalibrator: MagnetometerNonLinearCalibrator? = null

    /**
     * List of measurements that have been collected so far to be used for calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    var measurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()
        private set

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    val isReadyToSolveCalibration
        get() = measurements.size >= requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)

    /**
     * Gets X-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronX] will be equal to this value, otherwise [estimatedHardIronX] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronX].
     */
    var initialHardIronX: Double? = null
        private set

    /**
     * Gets Y-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibrator starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronY] will be equal to this value, otherwise [estimatedHardIronY] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronY].
     */
    var initialHardIronY: Double? = null
        private set

    /**
     * Gets Z-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibrator starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronZ] will be equal to this value, otherwise [estimatedHardIronZ] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronZ].
     */
    var initialHardIronZ: Double? = null
        private set

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronX] will be equal to this value, otherwise [estimatedHardIronX] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronX].
     */
    val initialHardIronXAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            val initialHardIronX = this.initialHardIronX ?: return null
            return MagneticFluxDensity(initialHardIronX, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronX] will be equal to this value, otherwise [estimatedHardIronX] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronX].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronXAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        val initialHardIronX = this.initialHardIronX
        return if (initialHardIronX != null) {
            result.value = initialHardIronX
            result.unit = MagneticFluxDensityUnit.TESLA
            true
        } else {
            false
        }
    }

    /**
     * Gets Y-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronY] will be equal to this value, otherwise [estimatedHardIronY] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronY].
     */
    val initialHardIronYAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            val initialHardIronY = this.initialHardIronY ?: return null
            return MagneticFluxDensity(initialHardIronY, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets Y-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronY] will be equal to this value, otherwise [estimatedHardIronY] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronY].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronYAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        val initialHardIronY = this.initialHardIronY
        return if (initialHardIronY != null) {
            result.value = initialHardIronY
            result.unit = MagneticFluxDensityUnit.TESLA
            true
        } else {
            false
        }
    }

    /**
     * Gets Z-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronZ] will be equal to this value, otherwise [estimatedHardIronZ] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronZ].
     */
    val initialHardIronZAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            val initialHardIronZ = this.initialHardIronZ ?: return null
            return MagneticFluxDensity(initialHardIronZ, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets Z-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the value used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronZ] will be equal to this value, otherwise [estimatedHardIronZ] will be
     * the estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronZ].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronZAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        val initialHardIronZ = this.initialHardIronZ
        return if (initialHardIronZ != null) {
            result.value = initialHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
            true
        } else {
            false
        }
    }

    /**
     * Gets initial hard iron coordinates used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the values used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then magnetometer
     * sensor measurements are assumed to be already hard iron compensated, and the initial hard
     * iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronAsTriad] will be equal to this value, otherwise [estimatedHardIronAsTriad]
     * will be estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronAsTriad].
     */
    val initialHardIronAsTriad: MagneticFluxDensityTriad?
        get() {
            val initialHardIronX = this.initialHardIronX
            val initialHardIronY = this.initialHardIronY
            val initialHardIronZ = this.initialHardIronZ
            return if (initialHardIronX != null && initialHardIronY != null && initialHardIronZ != null) {
                MagneticFluxDensityTriad(
                    MagneticFluxDensityUnit.TESLA,
                    initialHardIronX,
                    initialHardIronY,
                    initialHardIronZ
                )
            } else {
                null
            }
        }

    /**
     * Gets initial hard iron coordinates used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this
     * will be equal to the values used internally by the device as part of the magnetometer
     * hardware calibration.
     * If [sensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then magnetometer
     * sensor measurements are assumed to be already hard iron compensated, and the initial hard
     * iron is assumed to be zero.
     * If [isGroundTruthInitialHardIron] is true, this is assumed to be the true hard iron, and
     * [estimatedHardIronAsTriad] will be equal to this value, otherwise [estimatedHardIronAsTriad]
     * will be estimated hard iron after solving calibration, which will differ from
     * [estimatedHardIronAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getInitialHardIronAsTriad(result: MagneticFluxDensityTriad): Boolean {
        val initialHardIronX = this.initialHardIronX
        val initialHardIronY = this.initialHardIronY
        val initialHardIronZ = this.initialHardIronZ
        return if (initialHardIronX != null && initialHardIronY != null && initialHardIronZ != null) {
            result.setValueCoordinatesAndUnit(
                initialHardIronX,
                initialHardIronY,
                initialHardIronZ,
                MagneticFluxDensityUnit.TESLA
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
     * Indicates whether initial hard iron is considered a ground-truth known bias.
     * When true, estimated hard irons are exactly equal to the initial ones, otherwise
     * initial hard irons are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isGroundTruthInitialHardIron: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Private backing property containing actual location value.
     * This is initialized in the constructor.
     */
    private lateinit var _location: Location

    /**
     * Location of device when running calibration.
     * Both [location] and [timestamp] are used to determine Earth's magnetic field according to
     * provided World Magnetic Model.
     *
     * @throws IllegalStateException if calibrator is already running.
     * @see [WorldMagneticModel]
     */
    var location: Location
        get() = _location
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            _location = value
        }

    /**
     * Private backing property containing current timestamp value.
     * This is initialized in the constructor.
     */
    private lateinit var _timestamp: Date

    /**
     * Sets current timestamp.
     * Both [location] and [timestamp] are used to determine Earth's magnetic field according to
     * provided World Magnetic Model.
     *
     * @throws IllegalStateException if calibrator is already running.
     * @see [WorldMagneticModel]
     */
    var timestamp: Date
        get() = _timestamp
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            _timestamp = value
        }

    /**
     * Sets Earth's magnetic model.
     * Null indicates that default model is being used.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets magnetometer sensor being used for interval detection.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor
        get() = intervalDetector.sensor

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous magnetometer noise level during initialization of the interval
     * detector. Window size must always be larger than allowed minimum value, which is 2 and
     * must have and odd value.
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
     * exceeds this threshold when initialization completes. This threshold is expressed in Teslas
     * (T).
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
    var baseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity
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
    fun getBaseNoiseLevelAbsoluteThresholdAsMagneticFluxDensity(result: MagneticFluxDensity) {
        intervalDetector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization
     * expressed in Teslas (T).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevel
        get() = intervalDetector.baseNoiseLevel

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelAsMagneticFluxDensity
        get() = intervalDetector.baseNoiseLevelAsMeasurement

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getBaseNoiseLevelAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        return intervalDetector.getBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (T^2 * s).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelPsd
        get() = intervalDetector.baseNoiseLevelPsd

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed in (T * s^0.5).
     * This is only available once initialization is completed.
     */
    val baseNoiseLevelRootPsd
        get() = intervalDetector.baseNoiseLevelRootPsd

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in Teslas (T).
     * This is only available once initialization is completed.
     */
    val threshold
        get() = intervalDetector.threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     */
    val thresholdAsMagneticFluxDensity
        get() = intervalDetector.thresholdAsMeasurement

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getThresholdAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
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
     * Gets or sets initial scaling factors and cross couping errors matrix.
     * This is also known as magnetometer soft-iron matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var initialMm: Matrix
        get() {
            val result = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
            getInitialMm(result)
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
     * This is also known as magnetometer soft-iron matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getInitialMm(result: Matrix) {
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
    val minimumRequiredMeasurements: Int
        get() = if (isCommonAxisUsed) {
            if (isGroundTruthInitialHardIron) {
                KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isGroundTruthInitialHardIron) {
                KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
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
     * Gets estimated magnetometer soft-iron matrix containing scale factors and cross coupling
     * errors., or null if not available.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the magnetometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    val estimatedMm
        get() = internalCalibrator?.estimatedMm

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
     * Gets estimated covariance matrix for estimated magnetometer parameters or null if not
     * available.
     * When hard iron is known, diagonal elements of the covariance matrix contains variance for the
     * following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx, mzy.
     * When hard iron is not known, diagonal elements of the covariance matrix contains variance for
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
     * Gets x coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isGroundTruthInitialHardIron] is true, this will be equal to [initialHardIronX],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialHardIronX].
     */
    val estimatedHardIronX: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronX
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronX
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets y coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isGroundTruthInitialHardIron] is true, this will be equal to [initialHardIronY],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialHardIronY].
     */
    val estimatedHardIronY: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronY
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronY
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets z coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isGroundTruthInitialHardIron] is true, this will be equal to [initialHardIronZ],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialHardIronZ].
     */
    val estimatedHardIronZ: Double?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronZ
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronZ
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets x coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronXAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronXAsMagneticFluxDensity].
     */
    val estimatedHardIronXAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronXAsMagneticFluxDensity
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronXAsMagneticFluxDensity
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets x coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronXAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronXAsMagneticFluxDensity].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronXAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
            is UnknownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getEstimatedHardIronXAsMagneticFluxDensity(result)
            }
            is KnownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getHardIronXAsMagneticFluxDensity(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets y coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronYAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronYAsMagneticFluxDensity].
     */
    val estimatedHardIronYAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronYAsMagneticFluxDensity
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronYAsMagneticFluxDensity
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets y coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronYAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronYAsMagneticFluxDensity].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronYAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
            is UnknownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getEstimatedHardIronYAsMagneticFluxDensity(result)
            }
            is KnownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getHardIronYAsMagneticFluxDensity(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets z coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronZAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronZAsMagneticFluxDensity].
     */
    val estimatedHardIronZAsMagneticFluxDensity: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronZAsMagneticFluxDensity
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronZAsMagneticFluxDensity
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets z coordinate of estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to
     * [initialHardIronZAsMagneticFluxDensity], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronZAsMagneticFluxDensity].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronZAsMagneticFluxDensity(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
            is UnknownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getEstimatedHardIronZAsMagneticFluxDensity(result)
            }
            is KnownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getHardIronZAsMagneticFluxDensity(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to [initialHardIronAsTriad],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialHardIronAsTriad].
     */
    val estimatedHardIronAsTriad: MagneticFluxDensityTriad?
        get() {
            return when (val internalCalibrator = this.internalCalibrator) {
                is UnknownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.estimatedHardIronAsTriad
                }
                is KnownHardIronMagnetometerCalibrator -> {
                    internalCalibrator.hardIronAsTriad
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets estimated magnetometer hard iron.
     * If [isGroundTruthInitialHardIron] is true, this will be equal to [initialHardIronAsTriad],
     * otherwise it will be the estimated value obtained after solving calibration, that might
     * differ from [initialHardIronAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronAsTriad(result: MagneticFluxDensityTriad): Boolean {
        return when (val internalCalibrator = this.internalCalibrator) {
            is UnknownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getEstimatedHardIronAsTriad(result)
            }
            is KnownHardIronMagnetometerCalibrator -> {
                internalCalibrator.getHardIronAsTriad(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Starts calibrator.
     * This method starts collecting magnetometer measurements.
     * When calibrator is started, it begins with an initialization stage where magnetometer noise
     * is estimated while device remains static.
     * Once initialization is completed, calibrator determines intervals where device remains static
     * when device has different poses, so that measurements are collected to solve calibration.
     * If [solveCalibrationWhenEnoughMeasurements] is true, calibration is automatically solved
     * once enough measurements are collected, otherwise a call to [calibrate] must be done to solve
     * calibration.
     *
     * @throws IllegalStateException if calibrator is already running or a sensor is missing
     * (either magnetometer or gravity if it is being used when no location is provided).
     */
    fun start() {
        check(!running)

        reset()

        running = true
        intervalDetector.start()
    }

    /**
     * Stops calibrator.
     * When this is called, no more magnetometer or gravity measurements are collected.
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
     * When this is called, no more magnetometer or gravity measurements are collected.
     *
     * @param running specifies the running parameter to be set. This is true when stop occurs
     * internally during measurement collection to start solving calibration, otherwise is false
     * when calling public [stop] method.
     */
    private fun internalStop(running: Boolean) {
        intervalDetector.stop()
        this.running = running

        stoppedListener?.onStopped(this)
    }

    /**
     * Internally solves calibration using collected measurements without checking pre-requisites
     * (if either calibrator is already running or enough measurements are available).
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
            Log.e(
                StaticIntervalMagnetometerCalibrator::class.qualifiedName,
                "Calibration estimation failed",
                e
            )
            errorListener?.onError(this, ErrorReason.NUMERICAL_INSTABILITY_DURING_CALIBRATION)
            running = false
            false
        }
    }

    /**
     * Resets calibrator to its initial state.
     */
    private fun reset() {
        measurements.clear()
        initialHardIronX = null
        initialHardIronY = null
        initialHardIronZ = null

        internalCalibrator = null
    }

    /**
     * Updates initial hard iron values when first magnetometer measurement is received, so that
     * hardware calibrated hard irons are retrieved if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used.
     *
     * @param hardIronX x-coordinate of initial hard iron to be set expressed in Teslas (T).
     * @param hardIronY y-coordinate of initial hard iron to be set expressed in Teslas (T).
     * @param hardIronZ z-coordinate of initial hard iron to be set expressed in Teslas (T).
     */
    private fun updateInitialHardIrons(hardIronX: Float?, hardIronY: Float?, hardIronZ: Float?) {
        val initialHardIronX: Double
        val initialHardIronY: Double
        val initialHardIronZ: Double
        if (hardIronX != null && hardIronY != null && hardIronZ != null) {
            initialHardIronX = hardIronX.toDouble()
            initialHardIronY = hardIronY.toDouble()
            initialHardIronZ = hardIronZ.toDouble()
        } else {
            initialHardIronX = 0.0
            initialHardIronY = 0.0
            initialHardIronZ = 0.0
        }

        this.initialHardIronX = initialHardIronX
        this.initialHardIronY = initialHardIronY
        this.initialHardIronZ = initialHardIronZ

        initialHardIronAvailableListener?.onInitialHardIronAvailable(
            this,
            initialHardIronX,
            initialHardIronY,
            initialHardIronZ
        )
    }

    /**
     * Builds an internal magnetometer calibrator based on all provided parameters.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildInternalCalibrator(): MagnetometerNonLinearCalibrator {
        return if (robustMethod == null) {
            buildNonRobustCalibrator()
        } else {
            buildRobustCalibrator()
        }
    }

    /**
     * Internally builds a non-robust magnetometer calibrator based on all provided parameters.
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
     * Builds required quality scores for PROSAC and PROMedS robust methods.
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

    companion object {
        /**
         * Indicates when first magnetometer measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1

        /**
         * Number of unknowns when common z-axis is assumed for the magnetometer and hard iron is
         * unknown.
         */
        private const val UNKNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS = 9

        /**
         * Number of unknowns for the general calibration case when hard iron is unknown.
         */
        private const val UNKNOWN_HARD_IRON_GENERAL_UNKNOWNS = 12

        /**
         * Number of unknowns when common z-axis is assumed for the magnetometer and hard iron is
         * known.
         */
        private const val KNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS = 6

        /**
         * Number of unknowns for the general calibration case when hard iron is known.
         */
        private const val KNOWN_HARD_IRON_GENERAL_UNKNOWNS = 9

        /**
         * Indicates whether by default a common z-axis is assumed for magnetometer.
         */
        const val DEFAULT_USE_COMMON_Z_AXIS = false

        /**
         * Required minimum number of measurements when common z-axis is assumed and hard iron is
         * unknown.
         */
        const val UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            UNKNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is unknown.
         */
        const val UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL = UNKNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of measurements when common z-axis is assumed and hard iron is known.
         */
        const val KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            KNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is known.
         */
        const val KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL = KNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

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
         * [StaticIntervalMagnetometerCalibrator].
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
        fun onInitializationStarted(calibrator: StaticIntervalMagnetometerCalibrator)
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
        fun onInitializationCompleted(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    /**
     * Interface to notify when an error occurs.
     */
    fun interface OnErrorListener {
        /**
         * Called when an error is detected, either at initialization because excessive noise
         * is detected, because magnetometer sensor becomes unreliable or because obtained
         * measurements produce a numerically unstable calibration solution.
         *
         * @param calibrator calibrator that raised the event.
         * @param errorReason reason why error was detected.
         */
        fun onError(calibrator: StaticIntervalMagnetometerCalibrator, errorReason: ErrorReason)
    }

    /**
     * Interface to notify when initial hard iron guess is available.
     * If [isGroundTruthInitialHardIron] is true, then initial hard iron is considered the true
     * value after solving calibration, otherwise, initial hard iron is considered only an initial
     * guess.
     */
    fun interface OnInitialHardIronAvailableListener {
        /**
         * Called when initial hard iron is available.
         * If [isGroundTruthInitialHardIron] is true, then initial hard iron is considered the true
         * value after solving calibration, otherwise, initial hard iron is considered only on
         * initial guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param hardIronX x-coordinate of hard iron expressed in micro-Teslas (µT).
         * @param hardIronY y-coordinate of hard iron expressed in micro-Teslas (µT).
         * @param hardIronZ z-coordinate of hard iron expressed in micro-Teslas (µT).
         */
        fun onInitialHardIronAvailable(
            calibrator: StaticIntervalMagnetometerCalibrator,
            hardIronX: Double,
            hardIronY: Double,
            hardIronZ: Double
        )
    }

    /**
     * Interface to notify when a new measurement is obtained for calibration purposes.
     * Such measurements are determined each time a static period finishes (when sensor magnetic
     * measurements stop being static).
     * When enough of these measurements are obtained, calibration can actually be solved.
     */
    fun interface OnNewCalibrationMeasurementAvailableListener {
        /**
         * Called when a new measurement for calibration is found.
         * A new measurement each time a static period finishes (when sensor magnetic measurements
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
            calibrator: StaticIntervalMagnetometerCalibrator,
            newMeasurement: StandardDeviationBodyMagneticFluxDensity,
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
        fun onReadyToSolveCalibration(calibrator: StaticIntervalMagnetometerCalibrator)
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
         * @param calibration calibrator that raised the event.
         */
        fun onCalibrationSolvingStarted(calibration: StaticIntervalMagnetometerCalibrator)
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
        fun onCalibrationCompleted(calibrator: StaticIntervalMagnetometerCalibrator)
    }

    /**
     * Interface to notify when measurement collection stops.
     * This happens automatically when enough measurements are found after periods when
     * device stops being static, or if an error occurs.
     */
    fun interface OnStoppedListener {
        /**
         * Called when calibration on measurement collection stops.
         * This happens automatically when enough measurements are found after periods when
         * device stops being static, or if an error occurs.
         *
         * @param calibrator calibrator that raised the event.
         */
        fun onStopped(calibrator: StaticIntervalMagnetometerCalibrator)
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
