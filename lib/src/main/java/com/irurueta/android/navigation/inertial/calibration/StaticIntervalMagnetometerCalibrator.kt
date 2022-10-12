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
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.calibration.builder.MagnetometerInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.MagnetometerMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.UnknownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import java.util.*

/**
 * Collects accelerometer and magnetometer measurements by detecting periods when device remains
 * static using the accelerometer, and using such static periods to collect magnetometer
 * measurements to solve calibration parameters.
 * This calibrator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property magnetometerSensorDelay Delay of magnetometer sensor between samples.
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
 * @property generatedMagnetometerMeasurementListener listener to notify when a new magnetometer
 * calibration measurement is generated.
 * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
 * collected and calibrator is ready to solve calibration.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration solving completes.
 * @property stoppedListener listener to notify when calibrator is stopped.
 * @property initialMagnetometerHardIronAvailableListener listener to notify when a guess of hard
 * iron values is obtained.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 * @property magnetometerQualityScoreMapper mapper to convert collected magnetometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class StaticIntervalMagnetometerCalibrator private constructor(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    accelerometerSensorDelay: SensorDelay,
    val magnetometerSensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>?,
    initializationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>?,
    errorListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>?,
    staticIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>?,
    dynamicIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>?,
    staticIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>?,
    dynamicIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>?,
    var generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener?,
    readyToSolveCalibrationListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>?,
    calibrationSolvingStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>?,
    calibrationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>?,
    stoppedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>?,
    var initialMagnetometerHardIronAvailableListener: OnInitialMagnetometerHardIronAvailableListener?,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener?,
    val magnetometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) : BaseStaticIntervalWithMeasurementGeneratorCalibrator<StaticIntervalMagnetometerCalibrator, BodyKinematicsAndMagneticFluxDensity>(
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
     * @param location Current device location. If location and timestamp are provided, calibration
     * will be based on Earth's magnetic model.If no location and timestamp is provided,
     * calibration will be based on measured magnetic flux density norm at current location.
     * @param timestamp Current timestamp.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerSensorDelay Delay of sensor between samples.
     * @param magnetometerSensorDelay Delay of magnetometer sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param isMagnetometerGroundTruthInitialHardIron true if estimated magnetometer hard iron is
     * assumed to be the true value, false if estimated hard iron is assumed to be only an initial
     * guess. When [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER], hard iron guess is zero, otherwise
     * when it is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], hard iron
     * guess is the device calibrated values.
     * @param initializationStartedListener listener to notify when initialization starts.
     * @param initializationCompletedListener listener to notify when initialization completes.
     * @param errorListener listener to notify errors.
     * @param staticIntervalDetectedListener listener to notify when a static interval is detected.
     * @param dynamicIntervalDetectedListener listener to notify when a dynamic interval is detected.
     * @param staticIntervalSkippedListener listener to notify when a static interval is skipped if
     * its duration is too short.
     * @param dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped if
     * its duration is too long.
     * @param generatedMagnetometerMeasurementListener listener to notify when a new magnetometer
     * calibration measurement is generated.
     * @param readyToSolveCalibrationListener listener to notify when enough measurements have been
     * collected and calibrator is ready to solve calibration.
     * @param calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @param calibrationCompletedListener listener to notify when calibration solving completes.
     * @param stoppedListener listener to notify when calibrator is stopped.
     * @param initialMagnetometerHardIronAvailableListener listener to notify when a guess of hard
     * iron values is obtained.
     * @param accuracyChangedListener listener to notify when sensor accuracy changes.
     * @param magnetometerQualityScoreMapper mapper to convert collected magnetometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     */
    constructor(
        context: Context,
        location: Location? = null,
        timestamp: Date = Date(),
        worldMagneticModel: WorldMagneticModel? = null,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isMagnetometerGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalMagnetometerCalibrator>? = null,
        initializationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalMagnetometerCalibrator>? = null,
        errorListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalMagnetometerCalibrator>? = null,
        staticIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>? = null,
        dynamicIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalMagnetometerCalibrator>? = null,
        staticIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>? = null,
        dynamicIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalMagnetometerCalibrator>? = null,
        generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener? = null,
        readyToSolveCalibrationListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalMagnetometerCalibrator>? = null,
        calibrationSolvingStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalMagnetometerCalibrator>? = null,
        calibrationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalMagnetometerCalibrator>? = null,
        stoppedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalMagnetometerCalibrator>? = null,
        initialMagnetometerHardIronAvailableListener: OnInitialMagnetometerHardIronAvailableListener? = null,
        accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null,
        magnetometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> = DefaultMagnetometerQualityScoreMapper()
    ) : this(
        context,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerSensorDelay,
        magnetometerSensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        staticIntervalDetectedListener,
        dynamicIntervalDetectedListener,
        staticIntervalSkippedListener,
        dynamicIntervalSkippedListener,
        generatedMagnetometerMeasurementListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        initialMagnetometerHardIronAvailableListener,
        accuracyChangedListener,
        magnetometerQualityScoreMapper
    ) {
        this.location = location
        this.timestamp = timestamp
        this.worldMagneticModel = worldMagneticModel
        this.isMagnetometerGroundTruthInitialHardIron = isMagnetometerGroundTruthInitialHardIron
        requiredMeasurements = minimumRequiredMeasurements
        magnetometerRobustPreliminarySubsetSize = minimumRequiredMeasurements
    }

    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    private val biasTriad = MagneticFluxDensityTriad()


    /**
     * Listener used by internal generator to handle events when initialization is started.
     */
    private val generatorInitializationStartedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<MagnetometerMeasurementGenerator> {
            initializationStartedListener?.onInitializationStarted(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when initialization is completed.
     */
    private val generatorInitializationCompletedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<MagnetometerMeasurementGenerator> { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when an error occurs.
     */
    private val generatorErrorListener =
        SingleSensorCalibrationMeasurementGenerator.OnErrorListener<MagnetometerMeasurementGenerator> { _, reason ->
            stop()
            errorListener?.onError(
                this@StaticIntervalMagnetometerCalibrator,
                CalibratorErrorReason.mapErrorReason(reason)
            )
        }

    /**
     * Listener used by internal generator to handle events when a static interval is detected.
     */
    private val generatorStaticIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator> {
            staticIntervalDetectedListener?.onStaticIntervalDetected(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is detected.
     */
    private val generatorDynamicIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator> {
            dynamicIntervalDetectedListener?.onDynamicIntervalDetected(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a static interval is skipped.
     */
    private val generatorStaticIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator> {
            staticIntervalSkippedListener?.onStaticIntervalSkipped(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is skipped.
     */
    private val generatorDynamicIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator> {
            dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(this@StaticIntervalMagnetometerCalibrator)
        }

    /**
     * Listener  used by internal generator to handle events when a new measurement is generated.
     */
    private val generatorGeneratedMeasurementListener =
        SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity> { generator, measurement ->
            if (isInitialMagneticFluxDensityNormMeasured && initialMagneticFluxDensityNorm == null) {
                // set initial average magnetic flux density norm measured during initialization
                initialMagneticFluxDensityNorm = generator.initialMagneticFluxDensityNorm
            }

            magnetometerMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = magnetometerMeasurements.size

            generatedMagnetometerMeasurementListener?.onGeneratedMagnetometerMeasurement(
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

                // stop internal generator since no more measurements need to be collected
                internalStop(true)

                // build calibrator
                magnetometerInternalCalibrator = buildMagnetometerInternalCalibrator()

                if (solveCalibrationWhenEnoughMeasurements) {
                    // execute calibration
                    internalCalibrate()
                }
            }
        }

    /**
     * Listener for magnetometer sensor collector.
     * This is used to determine device calibration and obtain initial guesses
     * for magnetometer hard iron values (only available if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used, otherwise zero
     * hard iron is assumed as an initial guess).
     */
    private val generatorMagnetometerMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { _, _, _, hardIronX, hardIronY, hardIronZ, _, _ ->
            if (isFirstMagnetometerMeasurement) {
                val hardIronXTesla = if (hardIronX != null)
                    MagneticFluxDensityConverter.microTeslaToTesla(hardIronX.toDouble())
                else
                    null
                val hardIronYTesla = if (hardIronY != null)
                    MagneticFluxDensityConverter.microTeslaToTesla(hardIronY.toDouble())
                else
                    null
                val hardIronZTesla = if (hardIronZ != null)
                    MagneticFluxDensityConverter.microTeslaToTesla(hardIronZ.toDouble())
                else
                    null
                updateMagnetometerInitialHardIrons(hardIronXTesla, hardIronYTesla, hardIronZTesla)
            }
        }

    /**
     * Internal generator to generate measurements for calibration.
     */
    override val generator = MagnetometerMeasurementGenerator(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        magnetometerSensorType,
        magnetometerSensorDelay,
        generatorInitializationStartedListener,
        generatorInitializationCompletedListener,
        generatorErrorListener,
        generatorStaticIntervalDetectedListener,
        generatorDynamicIntervalDetectedListener,
        generatorStaticIntervalSkippedListener,
        generatorDynamicIntervalSkippedListener,
        generatorGeneratedMeasurementListener,
        magnetometerMeasurementListener = generatorMagnetometerMeasurementListener,
        accuracyChangedListener = accuracyChangedListener
    )

    /**
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected at static intervals.
     */
    private var magnetometerInternalCalibrator: MagnetometerNonLinearCalibrator? = null

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
     * Gets X-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronX] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronX] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronX].
     */
    var magnetometerInitialHardIronX: Double? = null
        private set

    /**
     * Gets Y-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronY] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronY] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronY].
     */
    var magnetometerInitialHardIronY: Double? = null
        private set

    /**
     * Gets Y-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronZ] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronZ] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronZ].
     */
    var magnetometerInitialHardIronZ: Double? = null
        private set

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronX] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronX] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronX].
     */
    val magnetometerInitialHardIronXAsMeasurement: MagneticFluxDensity?
        get() {
            val initialHardIronX = magnetometerInitialHardIronX ?: return null
            return MagneticFluxDensity(initialHardIronX, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronX] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronX] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronX].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getMagnetometerInitialHardIronXAsMeasurement(result: MagneticFluxDensity): Boolean {
        val initialHardIronX = magnetometerInitialHardIronX
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
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronY] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronY] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronY].
     */
    val magnetometerInitialHardIronYAsMeasurement: MagneticFluxDensity?
        get() {
            val initialHardIronY = magnetometerInitialHardIronY ?: return null
            return MagneticFluxDensity(initialHardIronY, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets Y-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronY] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronY] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronY].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getMagnetometerInitialHardIronYAsMeasurement(result: MagneticFluxDensity): Boolean {
        val initialHardIronY = magnetometerInitialHardIronY
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
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronZ] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronZ] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronZ].
     */
    val magnetometerInitialHardIronZAsMeasurement: MagneticFluxDensity?
        get() {
            val initialHardIronZ = magnetometerInitialHardIronZ ?: return null
            return MagneticFluxDensity(initialHardIronZ, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets Z-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronZ] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronZ] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronZ].
     *
     * @param result instance where result will be stored.
     * @return true if initial hard iron is available, false otherwise.
     */
    fun getMagnetometerInitialHardIronZAsMeasurement(result: MagneticFluxDensity): Boolean {
        val initialHardIronZ = magnetometerInitialHardIronZ
        return if (initialHardIronZ != null) {
            result.value = initialHardIronZ
            result.unit = MagneticFluxDensityUnit.TESLA
            true
        } else {
            false
        }
    }

    /**
     * Gets initial hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronAsTriad] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronAsTriad] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronAsTriad].
     */
    val magnetometerInitialHardIronAsTriad: MagneticFluxDensityTriad?
        get() {
            val initialHardIronX = magnetometerInitialHardIronX
            val initialHardIronY = magnetometerInitialHardIronY
            val initialHardIronZ = magnetometerInitialHardIronZ
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
     * Gets initial hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorCollector.SensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronAsTriad] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronAsTriad] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerInitialHardIronAsTriad(result: MagneticFluxDensityTriad): Boolean {
        val initialHardIronX = magnetometerInitialHardIronX
        val initialHardIronY = magnetometerInitialHardIronY
        val initialHardIronZ = magnetometerInitialHardIronZ
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
    var isMagnetometerGroundTruthInitialHardIron: Boolean = false
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
     * Gets accelerometer sensor being used for interval detection.
     * This can be used to obtain additional information about the sensor.
     */
    override val accelerometerSensor
        get() = generator.accelerometerSensor

    /**
     * Gets magnetometer sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor
        get() = generator.magnetometerSensor

    /**
     * Gets or sets initial magnetometer scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var magnetometerInitialMm: Matrix
        get() {
            val result =
                Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)
            getMagnetometerInitialMm(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == MagneticFluxDensityTriad.COMPONENTS && value.columns == MagneticFluxDensityTriad.COMPONENTS)

            magnetometerInitialSx = value.getElementAtIndex(0)
            magnetometerInitialMyx = value.getElementAtIndex(1)
            magnetometerInitialMzx = value.getElementAtIndex(2)

            magnetometerInitialMxy = value.getElementAtIndex(3)
            magnetometerInitialSy = value.getElementAtIndex(4)
            magnetometerInitialMzy = value.getElementAtIndex(5)

            magnetometerInitialMxz = value.getElementAtIndex(6)
            magnetometerInitialMyz = value.getElementAtIndex(7)
            magnetometerInitialSz = value.getElementAtIndex(8)
        }

    /**
     * Gets initial magnetometer scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getMagnetometerInitialMm(result: Matrix) {
        require(result.rows == MagneticFluxDensityTriad.COMPONENTS && result.columns == MagneticFluxDensityTriad.COMPONENTS)

        result.setElementAtIndex(0, magnetometerInitialSx)
        result.setElementAtIndex(1, magnetometerInitialMyx)
        result.setElementAtIndex(2, magnetometerInitialMzx)

        result.setElementAtIndex(3, magnetometerInitialMxy)
        result.setElementAtIndex(4, magnetometerInitialSy)
        result.setElementAtIndex(5, magnetometerInitialMzy)

        result.setElementAtIndex(6, magnetometerInitialMxz)
        result.setElementAtIndex(7, magnetometerInitialMyz)
        result.setElementAtIndex(8, magnetometerInitialSz)
    }

    /**
     * Gets or sets initial x scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-y cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMxy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-z cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMxz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-x cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMyx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-z cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMyz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-x cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMzx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-y cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMzy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Sets initial scaling factors for magnetometer calibration.
     *
     * @param magnetometerInitialSx initial x scaling factor.
     * @param magnetometerInitialSy initial y scaling factor.
     * @param magnetometerInitialSz initial z scaling factor.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setMagnetometerInitialScalingFactors(
        magnetometerInitialSx: Double,
        magnetometerInitialSy: Double,
        magnetometerInitialSz: Double
    ) {
        check(!running)
        this.magnetometerInitialSx = magnetometerInitialSx
        this.magnetometerInitialSy = magnetometerInitialSy
        this.magnetometerInitialSz = magnetometerInitialSz
    }

    /**
     * Sets initial cross coupling errors for magnetometer calibration.
     *
     * @param magnetometerInitialMxy initial x-y cross coupling error.
     * @param magnetometerInitialMxz initial x-z cross coupling error.
     * @param magnetometerInitialMyx initial y-x cross coupling error.
     * @param magnetometerInitialMyz initial y-z cross coupling error.
     * @param magnetometerInitialMzx initial z-x cross coupling error.
     * @param magnetometerInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setMagnetometerInitialCrossCouplingErrors(
        magnetometerInitialMxy: Double,
        magnetometerInitialMxz: Double,
        magnetometerInitialMyx: Double,
        magnetometerInitialMyz: Double,
        magnetometerInitialMzx: Double,
        magnetometerInitialMzy: Double
    ) {
        check(!running)
        this.magnetometerInitialMxy = magnetometerInitialMxy
        this.magnetometerInitialMxz = magnetometerInitialMxz
        this.magnetometerInitialMyx = magnetometerInitialMyx
        this.magnetometerInitialMyz = magnetometerInitialMyz
        this.magnetometerInitialMzx = magnetometerInitialMzx
        this.magnetometerInitialMzy = magnetometerInitialMzy
    }

    /**
     * Sets initial scaling factors and cross couping errors for magnetometer calibration.
     *
     * @param magnetometerInitialSx initial x scaling factor.
     * @param magnetometerInitialSy initial y scaling factor.
     * @param magnetometerInitialSz initial z scaling factor.
     * @param magnetometerInitialMxy initial x-y cross coupling error.
     * @param magnetometerInitialMxz initial x-z cross coupling error.
     * @param magnetometerInitialMyx initial y-x cross coupling error.
     * @param magnetometerInitialMyz initial y-z cross coupling error.
     * @param magnetometerInitialMzx initial z-x cross coupling error.
     * @param magnetometerInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setMagnetometerInitialScalingFactorsAndCrossCouplingErrors(
        magnetometerInitialSx: Double,
        magnetometerInitialSy: Double,
        magnetometerInitialSz: Double,
        magnetometerInitialMxy: Double,
        magnetometerInitialMxz: Double,
        magnetometerInitialMyx: Double,
        magnetometerInitialMyz: Double,
        magnetometerInitialMzx: Double,
        magnetometerInitialMzy: Double
    ) {
        setMagnetometerInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )
        setMagnetometerInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
    }

    /**
     * Indicates or specifies whether z-axis is assumed to be common for magnetometer and
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isMagnetometerCommonAxisUsed: Boolean = DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets minimum number of required measurements to start magnetometer calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredMagnetometerMeasurements: Int
        get() = if (isMagnetometerCommonAxisUsed) {
            if (isMagnetometerGroundTruthInitialHardIron) {
                MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isMagnetometerGroundTruthInitialHardIron) {
                MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            }
        }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = minimumRequiredMagnetometerMeasurements

    /**
     * Indicates robust method used to solve magnetometer calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustMethod: RobustEstimatorMethod? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Confidence of estimated magnetometer calibration result expressed as a value between 0.0
     * and 1.0.
     * By default 99% of confidence is used, which indicates that with a probability of 99%
     * estimation will be accurate because chosen sub-samples will be inliers (in other terms,
     * outliers will be correctly discarded).
     * This property is only taken into account if a not-null [magnetometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0 (both
     * included).
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustConfidence: Double = ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value in 0.0..1.0)
            check(!running)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust magnetometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [magnetometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustMaxIterations: Int = ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0)
            check(!running)
            field = value
        }

    /**
     * Size of preliminary subsets picked while finding a robust magnetometer calibration solution.
     * By default this is the [minimumRequiredMagnetometerMeasurements], which results in the
     * smallest number of iterations to complete robust algorithms.
     * Larger values can be used to ensure that error in each preliminary solution is minimized
     * among more measurements (thus, softening the effect of outliers), but this comes at the
     * expense of larger number of iterations.
     * This properly is only taken into account if a not-null [magnetometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [minimumRequiredMagnetometerMeasurements] at the moment the setter is called.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustPreliminarySubsetSize: Int = 0
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value >= minimumRequiredMagnetometerMeasurements)
            check(!running)
            field = value
        }

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * magnetometer calibration algorithms or not.
     * Threshold varies depending on chosen [magnetometerRobustMethod].
     * By default, if null is provided, the estimated [magnetometerBaseNoiseLevel] will be used to
     * determine a suitable threshold. Otherwise, if a value is provided, such value will be used
     * instead.
     * This properly is only taken into account if a not-null [magnetometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustThreshold: Double? = null
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value == null || value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Factor to be used respect estimated magnetometer base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [magnetometerBaseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement hqving an error greater than tat in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustThresholdFactor: Double = DEFAULT_ROBUST_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to magnetometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustStopThresholdFactor: Double = DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
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
    val estimatedMagnetometerMm
        get() = magnetometerInternalCalibrator?.estimatedMm

    /**
     * Gets estimated magnetometer x-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSx: Double?
        get() = magnetometerInternalCalibrator?.estimatedSx

    /**
     * Gets estimated magnetometer y-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSy: Double?
        get() = magnetometerInternalCalibrator?.estimatedSy

    /**
     * Gets estimated magnetometer z-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSz: Double?
        get() = magnetometerInternalCalibrator?.estimatedSz

    /**
     * Gets estimated magnetometer x-y cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMxy: Double?
        get() = magnetometerInternalCalibrator?.estimatedMxy

    /**
     * Gets estimated magnetometer x-z cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMxz: Double?
        get() = magnetometerInternalCalibrator?.estimatedMxz

    /**
     * Gets estimated magnetometer y-x cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMyx: Double?
        get() = magnetometerInternalCalibrator?.estimatedMyx

    /**
     * Gets estimated magnetometer y-z cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMyz: Double?
        get() = magnetometerInternalCalibrator?.estimatedMyz

    /**
     * Gets estimated magnetometer z-x cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMzx: Double?
        get() = magnetometerInternalCalibrator?.estimatedMzx

    /**
     * Gets estimated magnetometer z-y cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMzy: Double?
        get() = magnetometerInternalCalibrator?.estimatedMzy

    /**
     * Gets estimated covariance matrix for estimated magnetometer parameters or null if not
     * available.
     * When hard iron is known, diagonal elements of the covariance matrix contains variance for the
     * following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx, mzy.
     * When hard iron is not known, diagonal elements of the covariance matrix contains variance for
     * the following parameters (following indicated order): bx, by, bz, sx, sy, sz, mxy, mxz,
     * myx, myz, mzx, mzy.
     */
    val estimatedMagnetometerCovariance: Matrix?
        get() = magnetometerInternalCalibrator?.estimatedCovariance

    /**
     * Gets estimated chi square value for magnetometer or null if not available.
     */
    val estimatedMagnetometerChiSq: Double?
        get() = magnetometerInternalCalibrator?.estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided magnetometer measurements or null if
     * not available.
     */
    val estimatedMagnetometerMse: Double?
        get() = magnetometerInternalCalibrator?.estimatedMse

    /**
     * Gets x coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronX], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronX].
     */
    val estimatedMagnetometerHardIronX: Double?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronY], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronY].
     */
    val estimatedMagnetometerHardIronY: Double?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronZ], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronZ].
     */
    val estimatedMagnetometerHardIronZ: Double?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronXAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronXAsMeasurement].
     */
    val estimatedMagnetometerHardIronXAsMeasurement: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronXAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronXAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedMagnetometerHardIronXAsMeasurement(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronYAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronYAsMeasurement].
     */
    val estimatedMagnetometerHardIronYAsMeasurement: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronYAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronYAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedMagnetometerHardIronYAsMeasurement(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronZAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronZAsMeasurement].
     */
    val estimatedMagnetometerHardIronZAsMeasurement: MagneticFluxDensity?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronZAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronZAsMeasurement].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedMagnetometerHardIronZAsMeasurement(result: MagneticFluxDensity): Boolean {
        return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronAsTriad].
     */
    val estimatedMagnetometerHardIronAsTriad: MagneticFluxDensityTriad?
        get() {
            return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedMagnetometerHardIronAsTriad(result: MagneticFluxDensityTriad): Boolean {
        return when (val internalCalibrator = magnetometerInternalCalibrator) {
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
     * Gets magnetometer measurement base noise level that has been detected during initialization
     * expressed in Teslas (T).
     * This is only available once generator completes initialization.
     */
    val magnetometerBaseNoiseLevel
        get() = generator.magnetometerBaseNoiseLevel

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val magnetometerBaseNoiseLevelAsMeasurement
        get() = generator.magnetometerBaseNoiseLevelAsMeasurement

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerBaseNoiseLevelAsMeasurement(result: MagneticFluxDensity): Boolean {
        return generator.getMagnetometerBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Number of magnetometer measurements that have been processed.
     */
    val numberOfProcessedMagnetometerMeasurements
        get() = generator.numberOfProcessedMagnetometerMeasurements

    /**
     * List of magnetometer measurements that have been collected so far to be used for
     * magnetometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val magnetometerMeasurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    override val isReadyToSolveCalibration: Boolean
        get() = magnetometerMeasurements.size >= requiredMeasurements.coerceAtLeast(
            minimumRequiredMeasurements
        )

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
            magnetometerInternalCalibrator?.calibrate()
            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(
                StaticIntervalMagnetometerCalibrator::class.qualifiedName,
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
        magnetometerMeasurements.clear()

        magnetometerInitialHardIronX = null
        magnetometerInitialHardIronY = null
        magnetometerInitialHardIronZ = null

        magnetometerInternalCalibrator = null

        initialMagneticFluxDensityNorm = null
    }

    /**
     * Updates initial hard iron values when first magnetometer measurement is received, so
     * that hardware calibrated biases are retrieved if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used.
     *
     * @param hardIronX hard iron on device x-axis expressed in Teslas (T). Only
     * available when using [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param hardIronY hard iron on device y-axis expressed in Teslas (T). Only
     * available when using [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param hardIronZ hard iron on device y-axis expressed in Teslas (T). Only
     * available when using [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     */
    private fun updateMagnetometerInitialHardIrons(
        hardIronX: Double?,
        hardIronY: Double?,
        hardIronZ: Double?
    ) {
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

        // convert from device ENU coordinates to local plane NED coordinates
        ENUtoNEDTriadConverter.convert(
            initialHardIronX,
            initialHardIronY,
            initialHardIronZ,
            biasTriad
        )

        magnetometerInitialHardIronX = biasTriad.valueX
        magnetometerInitialHardIronY = biasTriad.valueY
        magnetometerInitialHardIronZ = biasTriad.valueZ

        initialMagnetometerHardIronAvailableListener?.onInitialHardIronAvailable(
            this,
            biasTriad.valueX,
            biasTriad.valueY,
            biasTriad.valueZ
        )
    }

    /**
     * Indicates whether the generator has picked the first accelerometer measurement.
     */
    private val isFirstMagnetometerMeasurement: Boolean
        get() = generator.numberOfProcessedMagnetometerMeasurements <= FIRST_MEASUREMENT

    /**
     * Builds an internal magnetometer calibrator based on all provided parameters.
     *
     * @return an internal magnetometer calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildMagnetometerInternalCalibrator(): MagnetometerNonLinearCalibrator {
        return MagnetometerInternalCalibratorBuilder(
            magnetometerMeasurements,
            magnetometerRobustPreliminarySubsetSize,
            minimumRequiredMagnetometerMeasurements,
            initialMagneticFluxDensityNorm,
            location,
            timestamp,
            magnetometerRobustMethod,
            magnetometerRobustConfidence,
            magnetometerRobustMaxIterations,
            magnetometerRobustThreshold,
            magnetometerRobustThresholdFactor,
            magnetometerRobustStopThresholdFactor,
            isMagnetometerGroundTruthInitialHardIron,
            isMagnetometerCommonAxisUsed,
            magnetometerInitialHardIronX,
            magnetometerInitialHardIronY,
            magnetometerInitialHardIronZ,
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy,
            magnetometerBaseNoiseLevel,
            worldMagneticModel,
            magnetometerQualityScoreMapper
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
         * By default this is 3.0 times [magnetometerBaseNoiseLevel], which considering the noise
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
         * Number of unknowns when common z-axis is assumed for the magnetometer and hard iron is
         * unknown.
         */
        private const val MAGNETOMETER_UNKNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS = 9

        /**
         * Number of unknowns for the general calibration case when hard iron is unknown.
         */
        private const val MAGNETOMETER_UNKNOWN_HARD_IRON_GENERAL_UNKNOWNS = 12

        /**
         * Number of unknowns when common z-axis is assumed for both the magnetometer and hard iron is
         * known.
         */
        private const val MAGNETOMETER_KNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS = 6

        /**
         * Number of unknowns for the general calibration case when hard iron is known.
         */
        private const val MAGNETOMETER_KNOWN_HARD_IRON_GENERAL_UNKNOWNS = 9

        /**
         * Required minimum number of measurements when common z-axis is assumed and hard iron is
         * unknown.
         */
        const val MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            MAGNETOMETER_UNKNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is unknown.
         */
        const val MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL =
            MAGNETOMETER_UNKNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of measurements when common z-axis is assumed and hard iron is
         * known.
         */
        const val MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            MAGNETOMETER_KNOWN_HARD_IRON_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of measurements for the general case when hard iron is known.
         */
        const val MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL =
            MAGNETOMETER_KNOWN_HARD_IRON_GENERAL_UNKNOWNS + 1

        /**
         * Indicates when first sensor measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1
    }

    /**
     * Interface to notify when a new magnetometer calibration measurement is generated.
     */
    fun interface OnGeneratedMagnetometerMeasurementListener {

        /**
         * Called when a new magnetometer calibration measurement is generated.
         *
         * @param calibrator calibrator that raised the event.
         * @param measurement generated magnetometer calibration measurement.
         * @param measurementsFoundSoFar number of measurements that have been found so far.
         * @param requiredMeasurements required number of measurements to solve calibration.
         */
        fun onGeneratedMagnetometerMeasurement(
            calibrator: StaticIntervalMagnetometerCalibrator,
            measurement: StandardDeviationBodyMagneticFluxDensity,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    /**
     * Interface to notify when initial hard iron guess is available.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, then initial hard iron is considered
     * the true value after solving calibration, otherwise, initial hard iron is considered only an
     * initial guess.
     */
    fun interface OnInitialMagnetometerHardIronAvailableListener {

        /**
         * Called when initial hard iron is available.
         * If [isMagnetometerGroundTruthInitialHardIron] is true, then initial hard iron is considered
         * the true value after solving calibration, otherwise, initial hard iron is considered only an
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
}