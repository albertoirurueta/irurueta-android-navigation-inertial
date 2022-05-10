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
import com.irurueta.android.navigation.inertial.calibration.builder.MagnetometerInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.intervals.IntervalDetector
import com.irurueta.android.navigation.inertial.calibration.intervals.MagnetometerIntervalDetector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.UnknownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import java.util.*
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Collects magnetometer measurements by detecting periods when device captures a constant magnetic
 * field (typically when device orientation remains static).
 * Such static periods are used to obtain measurements and solve calibration parameters.
 * This calibrator DOES NOT require an accelerometer. Only a magnetometer is needed.
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
 * @property qualityScoreMapper mapper to convert collected measurements into quality scores,
 * based on the amount ot standard deviation (the larger the variability, the worse the score
 * will be).
 */
class SingleSensorStaticIntervalMagnetometerCalibrator private constructor(
    context: Context,
    val sensorType: MagnetometerSensorCollector.SensorType,
    sensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: OnInitializationStartedListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    initializationCompletedListener: OnInitializationCompletedListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    errorListener: OnErrorListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    var initialHardIronAvailableListener: OnInitialHardIronAvailableListener?,
    newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener<SingleSensorStaticIntervalMagnetometerCalibrator, StandardDeviationBodyMagneticFluxDensity>?,
    readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    calibrationCompletedListener: OnCalibrationCompletedListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    stoppedListener: OnStoppedListener<SingleSensorStaticIntervalMagnetometerCalibrator>?,
    qualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) : SingleSensorStaticIntervalCalibrator<SingleSensorStaticIntervalMagnetometerCalibrator,
        StandardDeviationBodyMagneticFluxDensity, MagnetometerIntervalDetector,
        MagneticFluxDensityUnit, MagneticFluxDensity, MagneticFluxDensityTriad>(
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
     * @param location Current device location. If location and timestamp are provided, calibration
     * will be based on Earth's magnetic model.If no location and timestamp is provided,
     * calibration will be based on measured magnetic flux density norm at current location.
     * @param timestamp Current timestamp.
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
     * @param qualityScoreMapper mapper to convert collected measurements into quality scores,
     * based on the amount ot standard deviation (the larger the variability, the worse the score
     * will be).
     */
    constructor(
        context: Context,
        location: Location? = null,
        timestamp: Date = Date(),
        worldMagneticModel: WorldMagneticModel? = null,
        sensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: OnInitializationStartedListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        initializationCompletedListener: OnInitializationCompletedListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        errorListener: OnErrorListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        initialHardIronAvailableListener: OnInitialHardIronAvailableListener? = null,
        newCalibrationMeasurementAvailableListener: OnNewCalibrationMeasurementAvailableListener<SingleSensorStaticIntervalMagnetometerCalibrator, StandardDeviationBodyMagneticFluxDensity>? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
        stoppedListener: OnStoppedListener<SingleSensorStaticIntervalMagnetometerCalibrator>? = null,
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
     * Listener used by the internal interval detector when a static period ends and a dynamic
     * period starts. This listener contains accumulated magnetometer average values during static
     * period, that will be used as a measurement to solve calibration.
     */
    private val intervalDetectorDynamicIntervalDetectedListener =
        IntervalDetector.OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector> { _, _, _, _, _, _, _, accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ, accumulatedStdX, accumulatedStdY, accumulatedStdZ ->
            // add one measurement
            val b = BodyMagneticFluxDensity(accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ)

            if (isInitialMagneticFluxDensityNormMeasured && initialMagneticFluxDensityNorm == null) {
                // set initial average magnetic flux density norm measured during initialization
                initialMagneticFluxDensityNorm = b.norm
            }

            val stdNorm = sqrt(
                accumulatedStdX.pow(2.0) + accumulatedStdY.pow(2.0) + accumulatedStdZ.pow(2.0)
            )
            val measurement = StandardDeviationBodyMagneticFluxDensity(b, stdNorm)
            measurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = measurements.size

            newCalibrationMeasurementAvailableListener?.onNewCalibrationMeasurementAvailable(
                this@SingleSensorStaticIntervalMagnetometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@SingleSensorStaticIntervalMagnetometerCalibrator
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
    override val intervalDetector: MagnetometerIntervalDetector =
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
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected at static intervals.
     */
    private var internalCalibrator: MagnetometerNonLinearCalibrator? = null

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
     * [initialHardIronX].
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
     * [initialHardIronY].
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
     * [initialHardIronZ].
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
     * [initialHardIronX].
     */
    val initialHardIronXAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronX].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronXAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * [initialHardIronY].
     */
    val initialHardIronYAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronY].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronYAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * [initialHardIronZ].
     */
    val initialHardIronZAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronZ].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getInitialHardIronZAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * Location of device when running calibration.
     * This is only taken into account if both [location] and [timestamp] are provided to determine
     * Earth's magnetic field according to provided World Magnetic Model.
     * If either location or timestamp is missing, then measurement of magnetic field norm will be
     * used instead.
     *
     * @throws IllegalStateException if calibrator is already running.
     * @see [WorldMagneticModel]
     */
    var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets current timestamp.
     * This is only taken into account if both [location] and [timestamp] are provided to determine
     * Earth's magnetic field according to provided World Magnetic Model.
     * If either location or timestamp is missing, then measurement of magnetic field norm will be
     * used instead.
     *
     * @throws IllegalStateException if calibrator is already running.
     * @see [WorldMagneticModel]
     */
    var timestamp: Date? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
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
     * Norm of average magnetic flux density obtained during initialization and expressed in Teslas.
     */
    var initialMagneticFluxDensityNorm: Double? = null
        private set

    /**
     * Indicates whether initial magnetic flux density norm is measured or not.
     * If either [location] or [timestamp] is missing, magnetic flux density norm is measured,
     * otherwise an estimation is used based on World Magnetic Model.
     */
    val isInitialMagneticFluxDensityNormMeasured: Boolean
        get() = location == null || timestamp == null

    /**
     * Gets magnetometer sensor being used for interval detection.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor
        get() = intervalDetector.sensor

    /**
     * Gets or sets initial scaling factors and cross couping errors matrix.
     * This is also known as magnetometer soft-iron matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var initialMm: Matrix
        get() {
            val result =
                Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)
            getInitialMm(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == MagneticFluxDensityTriad.COMPONENTS && value.columns == MagneticFluxDensityTriad.COMPONENTS)

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
        require(result.rows == MagneticFluxDensityTriad.COMPONENTS && result.columns == MagneticFluxDensityTriad.COMPONENTS)

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
     * Gets estimated covariance matrix for estimated magnetometer parameters or null if not
     * available.
     * When hard iron is known, diagonal elements of the covariance matrix contains variance for the
     * following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx, mzy.
     * When hard iron is not known, diagonal elements of the covariance matrix contains variance for
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
     * [initialHardIronXAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronXAsMeasurement].
     */
    val estimatedHardIronXAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronXAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronXAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronXAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * [initialHardIronYAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronYAsMeasurement].
     */
    val estimatedHardIronYAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronYAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronYAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronYAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * [initialHardIronZAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronZAsMeasurement].
     */
    val estimatedHardIronZAsMeasurement: MagneticFluxDensity?
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
     * [initialHardIronZAsMeasurement], otherwise it will be the estimated value obtained
     * after solving calibration, that might differ from [initialHardIronZAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedHardIronZAsMeasurement(result: MagneticFluxDensity): Boolean {
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
     * Internally solves calibration using collected measurements without checking pre-requisites
     * (if either calibrator is already running or enough measurements are available).
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
                SingleSensorStaticIntervalMagnetometerCalibrator::class.qualifiedName,
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
        initialHardIronX = null
        initialHardIronY = null
        initialHardIronZ = null

        internalCalibrator = null

        initialMagneticFluxDensityNorm = null
    }

    /**
     * Updates initial hard iron values when first magnetometer measurement is received, so that
     * hardware calibrated hard irons are retrieved if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used.
     *
     * @param hardIronX x-coordinate of initial hard iron to be set expressed in micro-Teslas (µT).
     * @param hardIronY y-coordinate of initial hard iron to be set expressed in micro-Teslas (µT).
     * @param hardIronZ z-coordinate of initial hard iron to be set expressed in micro-Teslas (mT).
     */
    private fun updateInitialHardIrons(hardIronX: Float?, hardIronY: Float?, hardIronZ: Float?) {
        val initialHardIronX: Double
        val initialHardIronY: Double
        val initialHardIronZ: Double
        if (hardIronX != null && hardIronY != null && hardIronZ != null) {
            initialHardIronX = MagneticFluxDensityConverter.microTeslaToTesla(hardIronX.toDouble())
            initialHardIronY = MagneticFluxDensityConverter.microTeslaToTesla(hardIronY.toDouble())
            initialHardIronZ = MagneticFluxDensityConverter.microTeslaToTesla(hardIronZ.toDouble())
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
        return MagnetometerInternalCalibratorBuilder(
            measurements,
            robustPreliminarySubsetSize,
            requiredMeasurements,
            initialMagneticFluxDensityNorm,
            location,
            timestamp,
            robustMethod,
            robustConfidence,
            robustMaxIterations,
            robustThreshold,
            robustThresholdFactor,
            robustStopThresholdFactor,
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
        ).build()
    }

    companion object {
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
         * Required minimum number of measurements when common z-axis is assumed and hard iron is
         * unknown.
         */
        const val UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            UNKNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is unknown.
         */
        const val UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL =
            UNKNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of measurements when common z-axis is assumed and hard iron is known.
         */
        const val KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            KNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is known.
         */
        const val KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL =
            KNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

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
            calibrator: SingleSensorStaticIntervalMagnetometerCalibrator,
            hardIronX: Double,
            hardIronY: Double,
            hardIronZ: Double
        )
    }
}
