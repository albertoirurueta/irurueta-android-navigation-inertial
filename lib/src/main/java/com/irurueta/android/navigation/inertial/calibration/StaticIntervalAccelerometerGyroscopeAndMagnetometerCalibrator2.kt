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
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.Acceleration
import com.irurueta.units.AngularSpeed
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.Time
import java.util.*
import kotlin.math.max

/**
 * Collects accelerometer, gyroscope and magnetometer measurements by detecting periods when device
 * remains static or dynamic using the accelerometer and magnetometer, using such periods to
 * determine orientation based on gravity vector at the end of static intervals, and integrating
 * values of gyroscope measurements during dynamic ones, and using static periods to obtain
 * averaged accelerometer and magnetometer values.
 * This calibrator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
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
 * @property generatedAccelerometerMeasurementListener listener to notify when a new accelerometer
 * calibration measurement is generated.
 * @property generatedGyroscopeMeasurementListener listener to notify when a new gyroscope
 * calibration measurement is generated.
 * @property generatedMagnetometerMeasurementListener listener to notify when a new magnetometer
 * calibration measurement is generated.
 * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
 * collected and calibrator is ready to solve calibration.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration solving completes.
 * @property stoppedListener listener to notify when calibrator is stopped.
 * @property initialAccelerometerBiasAvailableListener listener to notify when a guess of bias
 * values is obtained.
 * @property initialGyroscopeBiasAvailableListener listener to notify when a guess of bias values
 * is obtained.
 * @property initialMagnetometerHardIronAvailableListener listener to notify when a guess of
 * magnetometer hard iron is obtained.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 * @property accelerometerQualityScoreMapper mapper to convert collected accelerometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 * @property gyroscopeQualityScoreMapper mapper to convert collected gyroscope measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 * @property magnetometerQualityScoreMapper mapper to convert collected magnetometer measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2 private constructor(
    override val context: Context,
    override val accelerometerSensorType: AccelerometerSensorType,
    val gyroscopeSensorType: GyroscopeSensorType,
    val magnetometerSensorType: MagnetometerSensorType,
    override val accelerometerSensorDelay: SensorDelay,
    val gyroscopeSensorDelay: SensorDelay,
    val magnetometerSensorDelay: SensorDelay,
    override val solveCalibrationWhenEnoughMeasurements: Boolean,
    override var initializationStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var initializationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var errorListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var staticIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var dynamicIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var staticIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var dynamicIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    var generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener?,
    var generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener?,
    var generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener?,
    override var readyToSolveCalibrationListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var calibrationSolvingStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var calibrationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    override var stoppedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>?,
    var unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener?,
    var initialAccelerometerBiasAvailableListener: OnInitialAccelerometerBiasAvailableListener?,
    var initialGyroscopeBiasAvailableListener: OnInitialGyroscopeBiasAvailableListener?,
    var initialMagnetometerHardIronAvailableListener: OnInitialMagnetometerHardIronAvailableListener?,
    override var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener?,
    val accelerometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>,
    val gyroscopeQualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>,
    val magnetometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) : StaticIntervalWithMeasurementGeneratorCalibrator<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2, TimedBodyKinematicsAndMagneticFluxDensity> {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location location where device is located at. When location is provided, gravity norm
     * is assumed to be the theoretical value determined by WGS84 Earth model, otherwise, if no
     * location is provided, gravity norm is estimated using a gravity sensor. Additionally, if
     * location and timestamp are provided, magnetometer calibration will be based on Earth's
     * magnetic model, otherwise magnetometer calibration will be based on measured magnetic flux
     * density norm at current location.
     * @param timestamp Current timestamp.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * @param gyroscopeSensorType One of the supported gyroscope sensor types.
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerSensorDelay Delay of accelerometer sensor between samples.
     * @param gyroscopeSensorDelay Delay of gyroscope sensor between samples.
     * @param magnetometerSensorDelay Delay of magnetometer sensor between samples.
     * @param solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param isAccelerometerGroundTruthInitialBias true if estimated accelerometer bias is assumed
     * to be the true value, false if estimated bias is assumed to be only an initial guess. When
     * [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], bias
     * guess is zero, otherwise when it is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], bias guess is the
     * device calibrated values.
     * @param isGyroscopeGroundTruthInitialBias true if estimated gyroscope bias is assumed to be
     * the true value, false if estimated bias is assumed to be only an initial guess. When
     * [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], bias guess is zero,
     * otherwise when it is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED], bias guess
     * is the device calibrated values.
     * @param isMagnetometerGroundTruthInitialHardIron true if estimated magnetometer hard iron is
     * assumed to be the true value, false if estimated hard iron is assumed to be only an initial
     * guess. When [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER], hard iron guess is zero, otherwise
     * when it is [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], hard iron
     * guess is the device calibrated values.
     * @param initializationStartedListener listener to notify when initialization starts.
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
     * @property generatedGyroscopeMeasurementListener listener to notify when a new gyroscope
     * calibration measurement is generated.
     * @property generatedMagnetometerMeasurementListener listener to notify when a new magnetometer
     * calibration measurement is generated.
     * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
     * collected and calibrator is ready to solve calibration.
     * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @property calibrationCompletedListener listener to notify when calibration solving completes.
     * @property stoppedListener listener to notify when calibrator is stopped.
     * @param unreliableGravityNormEstimationListener listener to notify when gravity norm
     * estimation becomes unreliable. This is only used if no location is provided.
     * @property initialAccelerometerBiasAvailableListener listener to notify when a guess of bias
     * values is obtained.
     * @property initialGyroscopeBiasAvailableListener listener to notify when a guess of bias values
     * is obtained.
     * @property initialMagnetometerHardIronAvailableListener listener to notify when a guess of
     * magnetometer hard iron is obtained.
     * @property accuracyChangedListener listener to notify when sensor accuracy changes.
     * @property accelerometerQualityScoreMapper mapper to convert collected accelerometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     * @property gyroscopeQualityScoreMapper mapper to convert collected gyroscope measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     * @property magnetometerQualityScoreMapper mapper to convert collected magnetometer measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     */
    constructor(
        context: Context,
        location: Location? = null,
        timestamp: Date = Date(),
        worldMagneticModel: WorldMagneticModel? = null,
        accelerometerSensorType: AccelerometerSensorType =
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
        gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
        magnetometerSensorType: MagnetometerSensorType =
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
        accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
        magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isAccelerometerGroundTruthInitialBias: Boolean = false,
        isGyroscopeGroundTruthInitialBias: Boolean = false,
        isMagnetometerGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        initializationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        errorListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        staticIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        dynamicIntervalDetectedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        staticIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        dynamicIntervalSkippedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener? = null,
        generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener? = null,
        generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener? = null,
        readyToSolveCalibrationListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        calibrationSolvingStartedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        calibrationCompletedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        stoppedListener: StaticIntervalWithMeasurementGeneratorCalibrator.OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2>? = null,
        unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener? = null,
        initialAccelerometerBiasAvailableListener: OnInitialAccelerometerBiasAvailableListener? = null,
        initialGyroscopeBiasAvailableListener: OnInitialGyroscopeBiasAvailableListener? = null,
        initialMagnetometerHardIronAvailableListener: OnInitialMagnetometerHardIronAvailableListener? = null,
        accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null,
        accelerometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics> =
            DefaultAccelerometerQualityScoreMapper(),
        gyroscopeQualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> =
            DefaultGyroscopeQualityScoreMapper(),
        magnetometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> =
            DefaultMagnetometerQualityScoreMapper()
    ) : this(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        magnetometerSensorType,
        accelerometerSensorDelay,
        gyroscopeSensorDelay,
        magnetometerSensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        staticIntervalDetectedListener,
        dynamicIntervalDetectedListener,
        staticIntervalSkippedListener,
        dynamicIntervalSkippedListener,
        generatedAccelerometerMeasurementListener,
        generatedGyroscopeMeasurementListener,
        generatedMagnetometerMeasurementListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        unreliableGravityNormEstimationListener,
        initialAccelerometerBiasAvailableListener,
        initialGyroscopeBiasAvailableListener,
        initialMagnetometerHardIronAvailableListener,
        accuracyChangedListener,
        accelerometerQualityScoreMapper,
        gyroscopeQualityScoreMapper,
        magnetometerQualityScoreMapper
    ) {
        this.location = location
        this.timestamp = timestamp
        this.worldMagneticModel = worldMagneticModel

        this.isAccelerometerGroundTruthInitialBias = isAccelerometerGroundTruthInitialBias
        accelerometerRobustPreliminarySubsetSize = minimumRequiredAccelerometerMeasurements

        this.isGyroscopeGroundTruthInitialBias = isGyroscopeGroundTruthInitialBias
        gyroscopeRobustPreliminarySubsetSize = minimumRequiredGyroscopeMeasurements

        this.isMagnetometerGroundTruthInitialHardIron = isMagnetometerGroundTruthInitialHardIron
        magnetometerRobustPreliminarySubsetSize = minimumRequiredMeasurements

        requiredMeasurements = minimumRequiredMeasurements
    }

    /**
     * Internal accelerometer and gyroscope calibrator.
     */
    private val accelerometerAndGyroscopeCalibrator =
        StaticIntervalAccelerometerAndGyroscopeCalibrator(
            context,
            accelerometerSensorType,
            gyroscopeSensorType,
            accelerometerSensorDelay,
            gyroscopeSensorDelay,
            solveCalibrationWhenEnoughMeasurements,
            initializationStartedListener = {
                accelerometerAndGyroscopeInitializationStarted = true
                if (!magnetometerInitializationStarted) {
                    this.initializationStartedListener?.onInitializationStarted(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            initializationCompletedListener = {
                accelerometerAndGyroscopeInitializationCompleted = true
                if (magnetometerInitializationCompleted) {
                    this.initializationCompletedListener?.onInitializationCompleted(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            errorListener = { _, errorReason ->
                stop()
                this.errorListener?.onError(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                    errorReason
                )
            },
            staticIntervalDetectedListener = {
                this.staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            },
            dynamicIntervalDetectedListener = {
                this.dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            },
            staticIntervalSkippedListener = {
                this.staticIntervalSkippedListener?.onStaticIntervalSkipped(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            },
            dynamicIntervalSkippedListener = {
                this.dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            },
            generatedAccelerometerMeasurementListener = { _, measurement, measurementsFoundSoFar, requiredMeasurements ->
                this.generatedAccelerometerMeasurementListener?.onGeneratedAccelerometerMeasurement(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                    measurement,
                    measurementsFoundSoFar,
                    requiredMeasurements
                )
            },
            generatedGyroscopeMeasurementListener = { _, measurement, measurementsFoundSoFar, requiredMeasurements ->
                this.generatedGyroscopeMeasurementListener?.onGeneratedGyroscopeMeasurement(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                    measurement,
                    measurementsFoundSoFar,
                    requiredMeasurements
                )
            },
            readyToSolveCalibrationListener = {
                accelerometerAndGyroscopeReadyToSolveCalibration = true
                if (magnetometerReadyToSolveCalibration) {
                    this.readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            calibrationSolvingStartedListener = {
                accelerometerAndGyroscopeCalibrationSolvingStarted = true
                if (magnetometerReadyToSolveCalibration
                    && !magnetometerCalibrationSolvingStarted
                ) {
                    this.calibrationSolvingStartedListener?.onCalibrationSolvingStarted(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            calibrationCompletedListener = {
                accelerometerAndGyroscopeCalibrationCompleted = true
                if (magnetometerCalibrationCompleted) {
                    this.calibrationCompletedListener?.onCalibrationCompleted(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            stoppedListener = {
                accelerometerAndGyroscopeStopped = true
                if (magnetometerStopped) {
                    this.stoppedListener?.onStopped(
                        this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                    )
                }
            },
            unreliableGravityNormEstimationListener = {
                this.unreliableGravityNormEstimationListener?.onUnreliableGravityEstimation(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            },
            initialAccelerometerBiasAvailableListener = { _, biasX, biasY, biasZ ->
                this.initialAccelerometerBiasAvailableListener?.onInitialBiasAvailable(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                    biasX,
                    biasY,
                    biasZ
                )
            },
            initialGyroscopeBiasAvailableListener = { _, biasX, biasY, biasZ ->
                this.initialGyroscopeBiasAvailableListener?.onInitialBiasAvailable(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                    biasX,
                    biasY,
                    biasZ
                )
            },
            accuracyChangedListener = { accuracy ->
                this.accuracyChangedListener?.onAccuracyChanged(
                    accuracy
                )
            },
            accelerometerQualityScoreMapper = accelerometerQualityScoreMapper,
            gyroscopeQualityScoreMapper = gyroscopeQualityScoreMapper
        )

    /**
     * Internal magnetometer calibrator.
     */
    private val magnetometerCalibrator = SingleSensorStaticIntervalMagnetometerCalibrator(
        context,
        location = null,
        sensorType = magnetometerSensorType,
        sensorDelay = magnetometerSensorDelay,
        solveCalibrationWhenEnoughMeasurements = solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener = {
            magnetometerInitializationStarted = true
            if (!accelerometerAndGyroscopeInitializationStarted) {
                this.initializationStartedListener?.onInitializationStarted(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        initializationCompletedListener = {
            magnetometerInitializationCompleted = true
            if (accelerometerAndGyroscopeInitializationCompleted) {
                this.initializationCompletedListener?.onInitializationCompleted(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        errorListener = { _, errorReason ->
            stop()
            this.errorListener?.onError(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                errorReason
            )
        },
        initialHardIronAvailableListener = { _, hardIronX, hardIronY, hardIronZ ->
            this.initialMagnetometerHardIronAvailableListener?.onInitialHardIronAvailable(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                hardIronX,
                hardIronY,
                hardIronZ
            )
        },
        newCalibrationMeasurementAvailableListener = { _, newMeasurement, measurementsFoundSoFar, requiredMeasurements ->
            this.generatedMagnetometerMeasurementListener?.onGeneratedMagnetometerMeasurement(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
                newMeasurement,
                measurementsFoundSoFar,
                requiredMeasurements
            )
        },
        readyToSolveCalibrationListener = {
            magnetometerReadyToSolveCalibration = true
            if (accelerometerAndGyroscopeReadyToSolveCalibration) {
                this.readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        calibrationSolvingStartedListener = {
            magnetometerCalibrationSolvingStarted = true
            if (accelerometerAndGyroscopeReadyToSolveCalibration
                && !accelerometerAndGyroscopeCalibrationSolvingStarted
            ) {
                this.calibrationSolvingStartedListener?.onCalibrationSolvingStarted(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        calibrationCompletedListener = {
            magnetometerCalibrationCompleted = true
            if (accelerometerAndGyroscopeCalibrationCompleted) {
                this.calibrationCompletedListener?.onCalibrationCompleted(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        stoppedListener = {
            magnetometerStopped = true
            if (accelerometerAndGyroscopeStopped) {
                this.stoppedListener?.onStopped(
                    this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
                )
            }
        },
        qualityScoreMapper = magnetometerQualityScoreMapper
    )

    /**
     * Indicates when internal accelerometer and gyroscope calibrator has started its
     * initialization.
     */
    private var accelerometerAndGyroscopeInitializationStarted = false

    /**
     * Indicates when internal magnetometer calibrator has started its initialization.
     */
    private var magnetometerInitializationStarted = false

    /**
     * Indicates when internal accelerometer and gyroscope calibrator completes initialization.
     */
    private var accelerometerAndGyroscopeInitializationCompleted = false

    /**
     * Indicates when internal magnetometer calibrator completes initialization.
     */
    private var magnetometerInitializationCompleted = false

    /**
     * Indicates when internal accelerometer and gyroscope calibrator is ready to solve calibration.
     */
    private var accelerometerAndGyroscopeReadyToSolveCalibration = false

    /**
     * Indicates when internal magnetometer calibrator is ready to solve calibration.
     */
    private var magnetometerReadyToSolveCalibration = false

    /**
     * Indicates when internal accelerometer and gyroscope calibrator starts solving calibration.
     */
    private var accelerometerAndGyroscopeCalibrationSolvingStarted = false

    /**
     * Indicates when internal magnetometer calibrator starts solving calibration.
     */
    private var magnetometerCalibrationSolvingStarted = false

    /**
     * Indicates when internal accelerometer and gyroscope calibrator completes calibration.
     */
    private var accelerometerAndGyroscopeCalibrationCompleted = false

    /**
     * Indicates when internal magnetometer calibrator completes calibration.
     */
    private var magnetometerCalibrationCompleted = false

    /**
     * Indicates when internal accelerometer and gyroscope calibrator has been stopped.
     */
    private var accelerometerAndGyroscopeStopped = false

    /**
     * Indicates when internal magnetometer calibrator has been stopped.
     */
    private var magnetometerStopped = false

    /**
     * Indicates whether calibrator is running.
     * While calibrator is running, calibrator parameters cannot be changed.
     */
    override val running: Boolean
        get() = accelerometerAndGyroscopeCalibrator.running || magnetometerCalibrator.running

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level during initialization of the internal calibrator
     * measurement generator. Window size must always be larger than allowed minimum value, which
     * is 2 and must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if calibrator is currently running.
     */
    override var windowSize
        get() = accelerometerAndGyroscopeCalibrator.windowSize
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            accelerometerAndGyroscopeCalibrator.windowSize = value
            magnetometerCalibrator.windowSize = value
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
    override var initialStaticSamples
        get() = accelerometerAndGyroscopeCalibrator.initialStaticSamples
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            accelerometerAndGyroscopeCalibrator.initialStaticSamples = value
            magnetometerCalibrator.initialStaticSamples = value
        }

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine a
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    override var thresholdFactor
        get() = accelerometerAndGyroscopeCalibrator.thresholdFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            accelerometerAndGyroscopeCalibrator.thresholdFactor = value
            magnetometerCalibrator.thresholdFactor = value
        }

    /**
     * Gets or sets factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount. This factor
     * is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    override var instantaneousNoiseLevelFactor
        get() = accelerometerAndGyroscopeCalibrator.instantaneousNoiseLevelFactor
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            accelerometerAndGyroscopeCalibrator.instantaneousNoiseLevelFactor = value
            magnetometerCalibrator.instantaneousNoiseLevelFactor = value
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
    override var baseNoiseLevelAbsoluteThreshold by accelerometerAndGyroscopeCalibrator::baseNoiseLevelAbsoluteThreshold

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    override var baseNoiseLevelAbsoluteThresholdAsMeasurement by accelerometerAndGyroscopeCalibrator::baseNoiseLevelAbsoluteThresholdAsMeasurement

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    override fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: Acceleration) {
        accelerometerAndGyroscopeCalibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
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
    var magnetometerBaseNoiseLevelAbsoluteThreshold by magnetometerCalibrator::baseNoiseLevelAbsoluteThreshold

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement by magnetometerCalibrator::baseNoiseLevelAbsoluteThresholdAsMeasurement

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    fun getMagnetometerBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: MagneticFluxDensity) {
        magnetometerCalibrator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result)
    }

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerBaseNoiseLevel by accelerometerAndGyroscopeCalibrator::accelerometerBaseNoiseLevel

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization.
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerBaseNoiseLevelAsMeasurement by accelerometerAndGyroscopeCalibrator::accelerometerBaseNoiseLevelAsMeasurement

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    override fun getAccelerometerBaseNoiseLevelAsMeasurement(result: Acceleration): Boolean {
        return accelerometerAndGyroscopeCalibrator.getAccelerometerBaseNoiseLevelAsMeasurement(
            result
        )
    }

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerBaseNoiseLevelPsd by accelerometerAndGyroscopeCalibrator::accelerometerBaseNoiseLevelPsd

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerBaseNoiseLevelRootPsd by accelerometerAndGyroscopeCalibrator::accelerometerBaseNoiseLevelRootPsd

    /**
     * Gets magnetometer measurement base noise level PSD (Power Spectral Density) expressed
     * (T^2 * s).
     * This is only available once initialization is completed.
     */
    val magnetometerBaseNoiseLevelPsd by magnetometerCalibrator::baseNoiseLevelPsd

    /**
     * Gets magnetometer measurement base noise level root PSD (Power Spectral Density) expressed in
     * (T * s^0.5).
     * This is only available once initialization is completed.
     */
    val magnetometerBaseNoiseLevelRootPsd by magnetometerCalibrator::baseNoiseLevelRootPsd

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
     * This is only available once internal generator completes initialization.
     */
    override val threshold by accelerometerAndGyroscopeCalibrator::threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once internal generator completes initialization.
     */
    override val thresholdAsMeasurement by accelerometerAndGyroscopeCalibrator::thresholdAsMeasurement

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    override fun getThresholdAsMeasurement(result: Acceleration): Boolean {
        return accelerometerAndGyroscopeCalibrator.getThresholdAsMeasurement(result)
    }

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed Teslas (T).
     * This is only available once initialization is completed.
     */
    val magnetometerThreshold by magnetometerCalibrator::threshold

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     */
    val magnetometerThresholdAsMeasurement by magnetometerCalibrator::thresholdAsMeasurement

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerThresholdAsMeasurement(result: MagneticFluxDensity): Boolean {
        return magnetometerCalibrator.getThresholdAsMeasurement(result)
    }

    /**
     * Gets number of samples that have been processed in a static period so far.
     */
    override val processedStaticSamples by accelerometerAndGyroscopeCalibrator::processedStaticSamples

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     */
    override val processedDynamicSamples by accelerometerAndGyroscopeCalibrator::processedDynamicSamples

    /**
     * Indicates whether last static interval must be skipped.
     */
    override val isStaticIntervalSkipped by accelerometerAndGyroscopeCalibrator::isStaticIntervalSkipped

    /**
     * Indicates whether last dynamic interval must be skipped.
     */
    override val isDynamicIntervalSkipped by accelerometerAndGyroscopeCalibrator::isDynamicIntervalSkipped

    /**
     * Gets average time interval between accelerometer samples expressed in seconds (s).
     * This is only available once the internal generator completes initialization.
     */
    override val accelerometerAverageTimeInterval by accelerometerAndGyroscopeCalibrator::accelerometerAverageTimeInterval

    /**
     * Gets average time interval between accelerometer samples.
     * This is only available once the internal generator completes initialization.
     */
    override val accelerometerAverageTimeIntervalAsTime by accelerometerAndGyroscopeCalibrator::accelerometerAverageTimeIntervalAsTime

    /**
     * Gets average time interval between accelerometer measurements.
     * This is only available once the internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    override fun getAccelerometerAverageTimeIntervalAsTime(result: Time): Boolean {
        return accelerometerAndGyroscopeCalibrator.getAccelerometerAverageTimeIntervalAsTime(result)
    }

    /**
     * Gets average time interval between magnetometer samples expressed in seconds (s).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val magnetometerAverageTimeInterval by magnetometerCalibrator::averageTimeInterval

    /**
     * Gets average time interval between magnetometer samples.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val magnetometerAverageTimeIntervalAsTime by magnetometerCalibrator::averageTimeIntervalAsTime

    /**
     * Gets average time interval between magnetometer measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerAverageTimeIntervalAsTime(result: Time): Boolean {
        return magnetometerCalibrator.getAverageTimeIntervalAsTime(result)
    }

    /**
     * Gets estimated variance of time interval between accelerometer measurements expressed in
     * squared seconds (s^2).
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerTimeIntervalVariance by accelerometerAndGyroscopeCalibrator::accelerometerTimeIntervalVariance

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements
     * expressed in seconds (s).
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerTimeIntervalStandardDeviation by accelerometerAndGyroscopeCalibrator::accelerometerTimeIntervalStandardDeviation

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once internal generator completes initialization.
     */
    override val accelerometerTimeIntervalStandardDeviationAsTime by accelerometerAndGyroscopeCalibrator::accelerometerTimeIntervalStandardDeviationAsTime

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once internal generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    override fun getAccelerometerTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return accelerometerAndGyroscopeCalibrator.getAccelerometerTimeIntervalStandardDeviationAsTime(
            result
        )
    }

    /**
     * Gets estimated variance of time interval between magnetometer measurements expressed in
     * squared seconds (s^2).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val magnetometerTimeIntervalVariance by magnetometerCalibrator::timeIntervalVariance

    /**
     * Gets estimated standard deviation of time interval between magnetometer measurements
     * expressed in seconds (s).
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val magnetometerTimeIntervalStandardDeviation by magnetometerCalibrator::timeIntervalStandardDeviation

    /**
     * Gets estimated standard deviation of time interval between magnetometer measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     */
    val magnetometerTimeIntervalStandardDeviationAsTime by magnetometerCalibrator::timeIntervalStandardDeviationAsTime

    /**
     * Gets estimated standard deviation of magnetometer time interval between measurements.
     * Time interval is estimated only while initialization is running, consequently, this is
     * only available once initialization is completed.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false othewise.
     */
    fun getMagnetometerTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return magnetometerCalibrator.getTimeIntervalStandardDeviationAsTime(result)
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
    override var requiredMeasurements by accelerometerAndGyroscopeCalibrator::requiredMeasurements

    /**
     * Number of accelerometer measurements that have been processed.
     */
    override val numberOfProcessedAccelerometerMeasurements by accelerometerAndGyroscopeCalibrator::numberOfProcessedAccelerometerMeasurements

    /**
     * Norm of average magnetic flux density obtained during initialization and expressed in Teslas.
     */
    val initialMagneticFluxDensityNorm by magnetometerCalibrator::initialMagneticFluxDensityNorm

    /**
     * Indicates whether initial magnetic flux density norm is measured or not.
     * If either [location] or [timestamp] is missing, magnetic flux density norm is measured,
     * otherwise an estimation is used based on World Magnetic Model.
     */
    val isInitialMagneticFluxDensityNormMeasured by magnetometerCalibrator::isInitialMagneticFluxDensityNormMeasured

    /**
     * Contains gravity norm (either obtained by the gravity sensor, or determined by current
     * location using WGS84 Earth model). Expressed in meters per squared second (m/s^2).
     */
    val gravityNorm by accelerometerAndGyroscopeCalibrator::gravityNorm

    /**
     * Indicates if accelerometer result is unreliable. This can happen if no location is provided
     * and gravity estimation becomes unreliable. When this happens result of calibration should
     * probably be discarded.
     */
    val accelerometerResultUnreliable by accelerometerAndGyroscopeCalibrator::accelerometerResultUnreliable

    /**
     * Gets x-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasX].
     */
    val accelerometerInitialBiasX by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasX

    /**
     * Gets y-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasY].
     */
    val accelerometerInitialBiasY by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasY

    /**
     * Gets z-coordinate of accelerometer bias used as an initial guess and expressed in meters per
     * squared second (m/s^2).
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasZ].
     */
    val accelerometerInitialBiasZ by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasZ

    /**
     * Gets accelerometer X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasX].
     */
    val accelerometerInitialBiasXAsMeasurement by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasXAsMeasurement

    /**
     * Gets accelerometer X-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
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
        return accelerometerAndGyroscopeCalibrator.getAccelerometerInitialBiasXAsMeasurement(result)
    }

    /**
     * Gets accelerometer Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasY].
     */
    val accelerometerInitialBiasYAsMeasurement by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasYAsMeasurement

    /**
     * Gets accelerometer Y-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
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
        return accelerometerAndGyroscopeCalibrator.getAccelerometerInitialBiasYAsMeasurement(result)
    }

    /**
     * Gets accelerometer Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] will be the estimated bias after solving calibration, which
     * will differ from [accelerometerInitialBiasZ].
     */
    val accelerometerInitialBiasZAsMeasurement by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasZAsMeasurement

    /**
     * Gets accelerometer Z-coordinate of bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
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
        return accelerometerAndGyroscopeCalibrator.getAccelerometerInitialBiasZAsMeasurement(result)
    }

    /**
     * Gets initial bias coordinate used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the values used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedAccelerometerBiasAsTriad] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasAsTriad] will be the estimated bias after solving calibration,
     * which will differ from [estimatedAccelerometerBiasAsTriad].
     */
    val accelerometerInitialBiasAsTriad by accelerometerAndGyroscopeCalibrator::accelerometerInitialBiasAsTriad

    /**
     * Gets initial bias coordinates used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED], this will be equal to
     * the values used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorType.ACCELEROMETER], then
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
        return accelerometerAndGyroscopeCalibrator.getAccelerometerInitialBiasAsTriad(result)
    }

    /**
     * Gets x-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasX] will be equal to this value, otherwise [estimatedGyroscopeBiasX]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasX].
     */
    val gyroscopeInitialBiasX by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasX

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasY] will be equal to this value, otherwise [estimatedGyroscopeBiasY]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasY].
     */
    val gyroscopeInitialBiasY by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasY

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasZ] will be equal to this value, otherwise [estimatedGyroscopeBiasZ]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasZ].
     */
    val gyroscopeInitialBiasZ by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasZ

    /**
     * Gets x-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasX] will be equal to this value, otherwise [estimatedGyroscopeBiasX]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasX].
     */
    val gyroscopeInitialBiasXAsMeasurement by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasXAsMeasurement

    /**
     * Gets x-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasX] will be equal to this value, otherwise [estimatedGyroscopeBiasX]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasX].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getGyroscopeInitialBiasXAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGyroscopeInitialBiasXAsMeasurement(result)
    }

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasY] will be equal to this value, otherwise [estimatedGyroscopeBiasY]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasY].
     */
    val gyroscopeInitialBiasYAsMeasurement by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasYAsMeasurement

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasY] will be equal to this value, otherwise [estimatedGyroscopeBiasY]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasY].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getGyroscopeInitialBiasYAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGyroscopeInitialBiasYAsMeasurement(result)
    }

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasZ] will be equal to this value, otherwise [estimatedGyroscopeBiasZ]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasZ].
     */
    val gyroscopeInitialBiasZAsMeasurement by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasZAsMeasurement

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasZ] will be equal to this value, otherwise [estimatedGyroscopeBiasZ]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasZ].
     *
     * @param result instance where result will be stored.
     * @return true if initial bias is available, false otherwise.
     */
    fun getGyroscopeInitialBiasZAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGyroscopeInitialBiasZAsMeasurement(result)
    }

    /**
     * Gets gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasAsTriad] will be equal to this value, otherwise
     * [estimatedGyroscopeBiasAsTriad] will be the estimated bias after solving calibration, which
     * will differ from [gyroscopeInitialBiasAsTriad].
     */
    val gyroscopeInitialBiasAsTriad by accelerometerAndGyroscopeCalibrator::gyroscopeInitialBiasAsTriad

    /**
     * Gets gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasAsTriad] will be equal to this value, otherwise
     * [estimatedGyroscopeBiasAsTriad] will be the estimated bias after solving calibration, which
     * will differ from [gyroscopeInitialBiasAsTriad].
     */
    fun getGyroscopeInitialBiasAsTriad(result: AngularSpeedTriad): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGyroscopeInitialBiasAsTriad(result)
    }

    /**
     * Gets X-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronX] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronX] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronX].
     */
    val magnetometerInitialHardIronX by magnetometerCalibrator::initialHardIronX

    /**
     * Gets Y-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronY] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronY] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronY].
     */
    val magnetometerInitialHardIronY by magnetometerCalibrator::initialHardIronY

    /**
     * Gets Y-coordinate of hard iron used as an initial guess and expressed in Teslas (T).
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronZ] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronZ] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronZ].
     */
    val magnetometerInitialHardIronZ by magnetometerCalibrator::initialHardIronZ

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronX] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronX] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronX].
     */
    val magnetometerInitialHardIronXAsMeasurement by magnetometerCalibrator::initialHardIronXAsMeasurement

    /**
     * Gets X-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
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
        return magnetometerCalibrator.getInitialHardIronXAsMeasurement(result)
    }

    /**
     * Gets Y-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronY] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronY] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronY].
     */
    val magnetometerInitialHardIronYAsMeasurement by magnetometerCalibrator::initialHardIronYAsMeasurement

    /**
     * Gets Y-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
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
        return magnetometerCalibrator.getInitialHardIronYAsMeasurement(result)
    }

    /**
     * Gets Z-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronZ] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronZ] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronZ].
     */
    val magnetometerInitialHardIronZAsMeasurement by magnetometerCalibrator::initialHardIronZAsMeasurement

    /**
     * Gets Z-coordinate of hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
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
        return magnetometerCalibrator.getInitialHardIronZAsMeasurement(result)
    }

    /**
     * Gets initial hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
     * magnetometer sensor measurements are assumed to be already hard iron compensated, and the
     * initial hard iron is assumed to be zero.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this is assumed to be the true hard
     * iron, and [estimatedMagnetometerHardIronAsTriad] will be equal to this value, otherwise
     * [estimatedMagnetometerHardIronAsTriad] will be the estimated hard iron after solving calibration,
     * which will differ from [magnetometerInitialHardIronAsTriad].
     */
    val magnetometerInitialHardIronAsTriad by magnetometerCalibrator::initialHardIronAsTriad

    /**
     * Gets initial hard iron used as an initial guess.
     * This value is determined once the calibration starts.
     * If [magnetometerSensorType] is
     * [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED], this will be equal to
     * the value used internally by the device as part of the magnetometer hardware calibration.
     * If [magnetometerSensorType] is [MagnetometerSensorType.MAGNETOMETER], then
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
        return magnetometerCalibrator.getInitialHardIronAsTriad(result)
    }

    /**
     * Indicates whether accelerometer initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isAccelerometerGroundTruthInitialBias by accelerometerAndGyroscopeCalibrator::isAccelerometerGroundTruthInitialBias

    /**
     * Indicates whether gyroscope initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isGyroscopeGroundTruthInitialBias by accelerometerAndGyroscopeCalibrator::isGyroscopeGroundTruthInitialBias

    /**
     * Indicates whether initial hard iron is considered a ground-truth known bias.
     * When true, estimated hard irons are exactly equal to the initial ones, otherwise
     * initial hard irons are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isMagnetometerGroundTruthInitialHardIron by magnetometerCalibrator::isGroundTruthInitialHardIron

    /**
     * Location of device when running calibration.
     * If location is provided, WGS84 Earth model is used to determine gravity norm
     * at such location, otherwise gravity norm is estimated during initialization by using the
     * gravity sensor of device.
     * Additionally, if both [location] and [timestamp] are provided, magnetometer calibration
     * uses World Magnetic Model, otherwise magnetometer calibration uses measured norm of magnetic
     * field.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var location
        get() = accelerometerAndGyroscopeCalibrator.location
        @Throws(IllegalStateException::class)
        set(value) {
            accelerometerAndGyroscopeCalibrator.location = value
            magnetometerCalibrator.location = value
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
    var timestamp by magnetometerCalibrator::timestamp

    /**
     * Sets Earth's magnetic model.
     * Null indicates that default model is being used.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var worldMagneticModel by magnetometerCalibrator::worldMagneticModel

    /**
     * Indicates whether gravity norm is estimated during initialization.
     * If location is provided, gravity is not estimated and instead theoretical
     * gravity for provided location is used.
     */
    val isGravityNormEstimated by accelerometerAndGyroscopeCalibrator::isGravityNormEstimated

    /**
     * Gets accelerometer sensor being used for interval detection.
     * This can be used to obtain additional information about the sensor.
     */
    override val accelerometerSensor by accelerometerAndGyroscopeCalibrator::accelerometerSensor

    /**
     * Gets gyroscope sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val gyroscopeSensor by accelerometerAndGyroscopeCalibrator::gyroscopeSensor

    /**
     * Gets magnetometer sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor by magnetometerCalibrator::magnetometerSensor

    /**
     * Gets gravity sensor being used for gravity estimation.
     * This can be used to obtain additional information about the sensor.
     */
    val gravitySensor by accelerometerAndGyroscopeCalibrator::gravitySensor

    /**
     * Gets or sets initial accelerometer scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var accelerometerInitialMa by accelerometerAndGyroscopeCalibrator::accelerometerInitialMa

    /**
     * Gets initial accelerometer scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getAccelerometerInitialMa(result: Matrix) {
        accelerometerAndGyroscopeCalibrator.getAccelerometerInitialMa(result)
    }

    /**
     * Gets or sets initial x scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSx by accelerometerAndGyroscopeCalibrator::accelerometerInitialSx

    /**
     * Gets or sets initial y scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSy by accelerometerAndGyroscopeCalibrator::accelerometerInitialSy

    /**
     * Gets or sets initial z scaling factor for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialSz by accelerometerAndGyroscopeCalibrator::accelerometerInitialSz

    /**
     * Gets or sets initial x-y cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMxy by accelerometerAndGyroscopeCalibrator::accelerometerInitialMxy

    /**
     * Gets or sets initial x-z cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMxz by accelerometerAndGyroscopeCalibrator::accelerometerInitialMxz

    /**
     * Gets or sets initial y-x cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMyx by accelerometerAndGyroscopeCalibrator::accelerometerInitialMyx

    /**
     * Gets or sets initial y-z cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMyz by accelerometerAndGyroscopeCalibrator::accelerometerInitialMyz

    /**
     * Gets or sets initial z-x cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMzx by accelerometerAndGyroscopeCalibrator::accelerometerInitialMzx

    /**
     * Gets or sets initial z-y cross coupling error for accelerometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialMzy by accelerometerAndGyroscopeCalibrator::accelerometerInitialMzy

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
        accelerometerAndGyroscopeCalibrator.setAccelerometerInitialScalingFactors(
            accelerometerInitialSx,
            accelerometerInitialSy,
            accelerometerInitialSz
        )
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
        accelerometerAndGyroscopeCalibrator.setAccelerometerInitialCrossCouplingErrors(
            accelerometerInitialMxy,
            accelerometerInitialMxz,
            accelerometerInitialMyx,
            accelerometerInitialMyz,
            accelerometerInitialMzx,
            accelerometerInitialMzy
        )
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
        accelerometerAndGyroscopeCalibrator.setAccelerometerInitialScalingFactorsAndCrossCouplingErrors(
            accelerometerInitialSx,
            accelerometerInitialSy,
            accelerometerInitialSz,
            accelerometerInitialMxy,
            accelerometerInitialMxz,
            accelerometerInitialMyx,
            accelerometerInitialMyz,
            accelerometerInitialMzx,
            accelerometerInitialMzy
        )
    }

    /**
     * Gets or sets initial gyroscope scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var gyroscopeInitialMg by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMg

    /**
     * Gets initial gyroscope scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getGyroscopeInitialMg(result: Matrix) {
        accelerometerAndGyroscopeCalibrator.getGyroscopeInitialMg(result)
    }

    /**
     * Gets or sets initial x scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSx by accelerometerAndGyroscopeCalibrator::gyroscopeInitialSx

    /**
     * Gets or sets initial y scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSy by accelerometerAndGyroscopeCalibrator::gyroscopeInitialSy

    /**
     * Gets or sets initial z scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSz by accelerometerAndGyroscopeCalibrator::gyroscopeInitialSz

    /**
     * Gets or sets initial x-y cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMxy by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMxy

    /**
     * Gets or sets initial x-z cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMxz by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMxz

    /**
     * Gets or sets initial y-x cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMyx by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMyx

    /**
     * Gets or sets initial y-z cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMyz by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMyz

    /**
     * Gets or sets initial z-x cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMzx by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMzx

    /**
     * Gets or sets initial z-y cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMzy by accelerometerAndGyroscopeCalibrator::gyroscopeInitialMzy

    /**
     * Sets initial scaling factors for gyroscope calibration.
     *
     * @param gyroscopeInitialSx initial x scaling factor.
     * @param gyroscopeInitialSy initial y scaling factor.
     * @param gyroscopeInitialSz initial z scaling factor.
     * @throws IllegalStateException if calibrator is currently running.
     */
    fun setGyroscopeInitialScalingFactors(
        gyroscopeInitialSx: Double,
        gyroscopeInitialSy: Double,
        gyroscopeInitialSz: Double
    ) {
        accelerometerAndGyroscopeCalibrator.setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )
    }

    /**
     * Sets initial cross coupling errors for gyroscope calibration.
     *
     * @param gyroscopeInitialMxy initial x-y cross coupling error.
     * @param gyroscopeInitialMxz initial x-z cross coupling error.
     * @param gyroscopeInitialMyx initial y-x cross coupling error.
     * @param gyroscopeInitialMyz initial y-z cross coupling error.
     * @param gyroscopeInitialMzx initial z-x cross coupling error.
     * @param gyroscopeInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setGyroscopeInitialCrossCouplingErrors(
        gyroscopeInitialMxy: Double,
        gyroscopeInitialMxz: Double,
        gyroscopeInitialMyx: Double,
        gyroscopeInitialMyz: Double,
        gyroscopeInitialMzx: Double,
        gyroscopeInitialMzy: Double
    ) {
        accelerometerAndGyroscopeCalibrator.setGyroscopeInitialCrossCouplingErrors(
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    /**
     * Sets initial scaling factors and cross coupling errors for gyroscope calibration.
     *
     * @param gyroscopeInitialSx initial x scaling factor.
     * @param gyroscopeInitialSy initial y scaling factor.
     * @param gyroscopeInitialSz initial z scaling factor.
     * @param gyroscopeInitialMxy initial x-y cross coupling error.
     * @param gyroscopeInitialMxz initial x-z cross coupling error.
     * @param gyroscopeInitialMyx initial y-x cross coupling error.
     * @param gyroscopeInitialMyz initial y-z cross coupling error.
     * @param gyroscopeInitialMzx initial z-x cross coupling error.
     * @param gyroscopeInitialMzy initial z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    fun setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
        gyroscopeInitialSx: Double,
        gyroscopeInitialSy: Double,
        gyroscopeInitialSz: Double,
        gyroscopeInitialMxy: Double,
        gyroscopeInitialMxz: Double,
        gyroscopeInitialMyx: Double,
        gyroscopeInitialMyz: Double,
        gyroscopeInitialMzx: Double,
        gyroscopeInitialMzy: Double
    ) {
        accelerometerAndGyroscopeCalibrator.setGyroscopeInitialScalingFactorsAndCrossCouplingErrors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy
        )
    }

    /**
     * Gets or sets initial G-dependent cross biases introduced on the gyroscope by the specific
     * forces sensed by the accelerometer.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var gyroscopeInitialGg by accelerometerAndGyroscopeCalibrator::gyroscopeInitialGg

    /**
     * Gets or sets initial magnetometer scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var magnetometerInitialMm by magnetometerCalibrator::initialMm

    /**
     * Gets initial magnetometer scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getMagnetometerInitialMm(result: Matrix) {
        magnetometerCalibrator.getInitialMm(result)
    }

    /**
     * Gets or sets initial x scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSx by magnetometerCalibrator::initialSx

    /**
     * Gets or sets initial y scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSy by magnetometerCalibrator::initialSy

    /**
     * Gets or sets initial z scaling factor for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialSz by magnetometerCalibrator::initialSz

    /**
     * Gets or sets initial x-y cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMxy by magnetometerCalibrator::initialMxy

    /**
     * Gets or sets initial x-z cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMxz by magnetometerCalibrator::initialMxz

    /**
     * Gets or sets initial y-x cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMyx by magnetometerCalibrator::initialMyx

    /**
     * Gets or sets initial y-z cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMyz by magnetometerCalibrator::initialMyz

    /**
     * Gets or sets initial z-x cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMzx by magnetometerCalibrator::initialMzx

    /**
     * Gets or sets initial z-y cross coupling error for magnetometer calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerInitialMzy by magnetometerCalibrator::initialMzy

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
        magnetometerCalibrator.setInitialScalingFactors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz
        )
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
        magnetometerCalibrator.setInitialCrossCouplingErrors(
            magnetometerInitialMxy,
            magnetometerInitialMxz,
            magnetometerInitialMyx,
            magnetometerInitialMyz,
            magnetometerInitialMzx,
            magnetometerInitialMzy
        )
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
        magnetometerCalibrator.setInitialScalingFactorsAndCrossCouplingErrors(
            magnetometerInitialSx,
            magnetometerInitialSy,
            magnetometerInitialSz,
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
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix during accelerometer
     * calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isAccelerometerCommonAxisUsed by accelerometerAndGyroscopeCalibrator::isAccelerometerCommonAxisUsed

    /**
     * Indicates or specifies whether z-axis is assumed to be common for gyroscope. When enabled,
     * this eliminates 3 variables from Mg matrix during gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isGyroscopeCommonAxisUsed by accelerometerAndGyroscopeCalibrator::isGyroscopeCommonAxisUsed

    /**
     * Indicates or specifies whether z-axis is assumed to be common for magnetometer and
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isMagnetometerCommonAxisUsed by magnetometerCalibrator::isCommonAxisUsed

    /**
     * Indicates or specifies whether G-dependent cross biases are being estimated or not. When enabled,
     * this adds 9 variables from Gg matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isGDependentCrossBiasesEstimated by accelerometerAndGyroscopeCalibrator::isGDependentCrossBiasesEstimated

    /**
     * Gets minimum number of required measurements to start accelerometer calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredAccelerometerMeasurements by accelerometerAndGyroscopeCalibrator::minimumRequiredAccelerometerMeasurements

    /**
     * Gets minimum number of required measurements to start gyroscope calibration.
     * Each time that the device is moved, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredGyroscopeMeasurements by accelerometerAndGyroscopeCalibrator::minimumRequiredGyroscopeMeasurements

    /**
     * Gets minimum number of required measurements to start magnetometer calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredMagnetometerMeasurements by magnetometerCalibrator::minimumRequiredMeasurements

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static or moved, new accelerometer and gyroscope
     * measurements are collected.
     * When the required number of measurements is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = max(
            accelerometerAndGyroscopeCalibrator.minimumRequiredMeasurements,
            minimumRequiredMagnetometerMeasurements
        )

    /**
     * Gets estimated average of gravity norm expressed in meters per squared second (m/s^2).
     * This is only available if no location is provided and initialization has completed.
     */
    val averageGravityNorm by accelerometerAndGyroscopeCalibrator::averageGravityNorm

    /**
     * Gets estimated average gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    val averageGravityNormAsMeasurement by accelerometerAndGyroscopeCalibrator::averageGravityNormAsMeasurement

    /**
     * Gets estimated average gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    fun getAverageGravityNormAsMeasurement(result: Acceleration): Boolean {
        return accelerometerAndGyroscopeCalibrator.getAverageGravityNormAsMeasurement(result)
    }

    /**
     * Gets estimated variance of gravity norm expressed in (m^2/s^4).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormVariance by accelerometerAndGyroscopeCalibrator::gravityNormVariance

    /**
     * Gets estimated standard deviation of gravity norm expressed in meters per squared second
     * (m/s^2).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormStandardDeviation by accelerometerAndGyroscopeCalibrator::gravityNormStandardDeviation

    /**
     * Gets estimated standard deviation of gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityNormStandardDeviationAsMeasurement by accelerometerAndGyroscopeCalibrator::gravityNormStandardDeviationAsMeasurement

    /**
     * Gets estimated standard deviation of gravity norm as Acceleration.
     * This is only available if no location is provided and initialization has completed.
     *
     * @param result instance where result will be stored.
     * @return true i result is available, false otherwise.
     */
    fun getGravityNormStandardDeviationAsMeasurement(result: Acceleration): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGravityNormStandardDeviationAsMeasurement(
            result
        )
    }

    /**
     * Gets PSD (Power Spectral Density) of gravity norm expressed in (m^2 * s^-3).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityPsd by accelerometerAndGyroscopeCalibrator::gravityPsd

    /**
     * Gets root PSD (Power Spectral Density) of gravity norm expressed in (m * s^-1.5).
     * This is only available if no location is provided and initialization has completed.
     */
    val gravityRootPsd by accelerometerAndGyroscopeCalibrator::gravityRootPsd

    /**
     * Indicates robust method used to solve accelerometer calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustMethod by accelerometerAndGyroscopeCalibrator::accelerometerRobustMethod

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
    var accelerometerRobustConfidence by accelerometerAndGyroscopeCalibrator::accelerometerRobustConfidence

    /**
     * Maximum number of iterations to attempt to find a robust accelerometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [accelerometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustMaxIterations by accelerometerAndGyroscopeCalibrator::accelerometerRobustMaxIterations

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
    var accelerometerRobustPreliminarySubsetSize by accelerometerAndGyroscopeCalibrator::accelerometerRobustPreliminarySubsetSize

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
    var accelerometerRobustThreshold by accelerometerAndGyroscopeCalibrator::accelerometerRobustThreshold

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
    var accelerometerRobustThresholdFactor by accelerometerAndGyroscopeCalibrator::accelerometerRobustThresholdFactor

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to accelerometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerRobustStopThresholdFactor by accelerometerAndGyroscopeCalibrator::accelerometerRobustStopThresholdFactor

    /**
     * Indicates robust method used to solve gyroscope calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustMethod by accelerometerAndGyroscopeCalibrator::gyroscopeRobustMethod

    /**
     * Confidence of estimated gyroscope calibration result expressed as a value between 0.0
     * and 1.0.
     * By default 99% of confidence is used, which indicates that with a probability of 99%
     * estimation will be accurate because chosen sub-samples will be inliers (in other terms,
     * outliers will be correctly discarded).
     * This property is only taken into account if a not-null [gyroscopeRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0 (both
     * included).
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustConfidence by accelerometerAndGyroscopeCalibrator::gyroscopeRobustConfidence

    /**
     * Maximum number of iterations to attempt to find a robust gyroscope calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [gyroscopeRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustMaxIterations by accelerometerAndGyroscopeCalibrator::gyroscopeRobustMaxIterations

    /**
     * Size of preliminary subsets picked while finding a robust gyroscope calibration solution.
     * By default this is the [minimumRequiredGyroscopeMeasurements], which results in the
     * smallest number of iterations to complete robust algorithms.
     * Larger values can be used to ensure that error in each preliminary solution is minimized
     * among more measurements (thus, softening the effect of outliers), but this comes at the
     * expense of larger number of iterations.
     * This properly is only taken into account if a not-null [gyroscopeRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [minimumRequiredGyroscopeMeasurements] at the moment the setter is called.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustPreliminarySubsetSize by accelerometerAndGyroscopeCalibrator::gyroscopeRobustPreliminarySubsetSize

    /**
     * Threshold to be used to determine whether a measurement is considered an outlier by robust
     * gyroscope calibration algorithms or not.
     * Threshold varies depending on chosen [gyroscopeRobustMethod].
     * By default, if null is provided, the estimated [gyroscopeBaseNoiseLevel] will be used to
     * determine a suitable threshold. Otherwise, if a value is provided, such value will be used
     * instead.
     * This properly is only taken into account if a not-null [gyroscopeRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustThreshold by accelerometerAndGyroscopeCalibrator::gyroscopeRobustThreshold

    /**
     * Factor to be used respect estimated gyroscope base noise level to consider a measurement
     * an outlier when using robust calibration methods.
     * By default this is 3.0 times [gyroscopeBaseNoiseLevel], which considering the noise level
     * as the standard deviation of a Gaussian distribution, should account for 99% of the cases.
     * Any measurement having an error greater than that in the estimated solution, will be
     * considered an outlier and be discarded.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustThresholdFactor by accelerometerAndGyroscopeCalibrator::gyroscopeRobustThresholdFactor

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to accelerometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustStopThresholdFactor by accelerometerAndGyroscopeCalibrator::gyroscopeRobustStopThresholdFactor

    /**
     * Indicates robust method used to solve magnetometer calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustMethod by magnetometerCalibrator::robustMethod

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
    var magnetometerRobustConfidence by magnetometerCalibrator::robustConfidence

    /**
     * Maximum number of iterations to attempt to find a robust magnetometer calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [magnetometerRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustMaxIterations by magnetometerCalibrator::robustMaxIterations

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
    var magnetometerRobustPreliminarySubsetSize by magnetometerCalibrator::robustPreliminarySubsetSize

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
    var magnetometerRobustThreshold by magnetometerCalibrator::robustThreshold

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
    var magnetometerRobustThresholdFactor by magnetometerCalibrator::robustThresholdFactor

    /**
     * Additional factor to be taken into account for robust methods based on LMedS or PROMedS,
     * where factor is not directly related to LMSE, but to a smaller value.
     * This only applies to magnetometer calibration.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var magnetometerRobustStopThresholdFactor by magnetometerCalibrator::robustStopThresholdFactor

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
    val estimatedAccelerometerMa by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMa

    /**
     * Gets estimated accelerometer x-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSx by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerSx

    /**
     * Gets estimated accelerometer y-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSy by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerSy

    /**
     * Gets estimated accelerometer z-axis scale factor or null if not available.
     */
    val estimatedAccelerometerSz by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerSz

    /**
     * Gets estimated accelerometer x-y cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMxy by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMxy

    /**
     * Gets estimated accelerometer x-z cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMxz by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMxz

    /**
     * Gets estimated accelerometer y-x cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMyx by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMyx

    /**
     * Gets estimated accelerometer y-z cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMyz by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMyz

    /**
     * Gets estimated accelerometer z-x cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMzx by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMzx

    /**
     * Gets estimated accelerometer z-y cross-coupling error or null if not available.
     */
    val estimatedAccelerometerMzy by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMzy

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
    val estimatedAccelerometerCovariance by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerCovariance

    /**
     * Gets estimated chi square value for accelerometer or null if not available.
     */
    val estimatedAccelerometerChiSq by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerChiSq

    /**
     * Gets estimated mean square error respect to provided accelerometer measurements or null if
     * not available.
     */
    val estimatedAccelerometerMse by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerMse

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasX].
     */
    val estimatedAccelerometerBiasX by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasX

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasY].
     */
    val estimatedAccelerometerBiasY by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasY

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasZ].
     */
    val estimatedAccelerometerBiasZ by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasZ

    /**
     * Gets x coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasX].
     */
    val estimatedAccelerometerBiasXAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasXAsMeasurement

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
        return accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasXAsMeasurement(
            result
        )
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasY].
     */
    val estimatedAccelerometerBiasYAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasYAsMeasurement

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
        return accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasYAsMeasurement(
            result
        )
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [accelerometerInitialBiasZ].
     */
    val estimatedAccelerometerBiasZAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasZAsMeasurement

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
        return accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasZAsMeasurement(
            result
        )
    }

    /**
     * Gets estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [accelerometerInitialBiasAsTriad].
     */
    val estimatedAccelerometerBiasAsTriad by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasAsTriad

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
        return accelerometerAndGyroscopeCalibrator.getEstimatedAccelerometerBiasAsTriad(result)
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in meters per
     * squared second (m/s^2), or null if not available.
     */
    val estimatedAccelerometerBiasStandardDeviationNorm by accelerometerAndGyroscopeCalibrator::estimatedAccelerometerBiasStandardDeviationNorm

    /**
     * Gets estimated gyroscope scale factors and cross coupling errors, or null if not
     * available.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    val estimatedGyroscopeMg by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMg

    /**
     * Gets estimated gyroscope x-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSx by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeSx

    /**
     * Gets estimated gyroscope y-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSy by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeSy

    /**
     * Gets estimated gyroscope z-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSz by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeSz

    /**
     * Gets estimated gyroscope x-y cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMxy by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMxy

    /**
     * Gets estimated gyroscope x-z cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMxz by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMxz

    /**
     * Gets estimated gyroscope y-x cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMyx by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMyx

    /**
     * Gets estimated gyroscope y-z cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMyz by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMyz

    /**
     * Gets estimated gyroscope z-x cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMzx by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMzx

    /**
     * Gets estimated gyroscope z-y cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMzy by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMzy

    /**
     * Gets estimated gyroscope G-dependent cross biases introduced on the gyroscope by the specific
     * forces sensed by the accelerometer.
     */
    val estimatedGyroscopeGg by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeGg

    /**
     * Gets estimated covariance matrix for estimated gyroscope parameters or null if not
     * available.
     * When bias is known, diagonal elements of the covariance matrix contains variance
     * for the following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx,
     * mzy.
     * When bias is not known, diagonal elements of the covariance matrix contains
     * variance for the following parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, where bx, by, bz corresponds to bias or hard iron coordinates.
     */
    val estimatedGyroscopeCovariance by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeCovariance

    /**
     * Gets estimated chi square value for gyroscope or null if not available.
     */
    val estimatedGyroscopeChiSq by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeChiSq

    /**
     * Gets estimated mean square error respect to provided gyroscope measurements or null if
     * not available.
     */
    val estimatedGyroscopeMse by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeMse

    /**
     * Gets x coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasX].
     */
    val estimatedGyroscopeBiasX by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasX

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasY].
     */
    val estimatedGyroscopeBiasY by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasY

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasZ].
     */
    val estimatedGyroscopeBiasZ by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasZ

    /**
     * Gets x coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasX].
     */
    val estimatedGyroscopeBiasXAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasXAsMeasurement

    /**
     * Gets x coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasX].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedGyroscopeBiasXAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasXAsMeasurement(result)
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasY].
     */
    val estimatedGyroscopeBiasYAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasYAsMeasurement

    /**
     * Gets y coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasY].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedGyroscopeBiasYAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasYAsMeasurement(result)
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasZ].
     */
    val estimatedGyroscopeBiasZAsMeasurement by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasZAsMeasurement

    /**
     * Gets z coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasZ].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedGyroscopeBiasZAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasZAsMeasurement(result)
    }

    /**
     * Gets estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [gyroscopeInitialBiasAsTriad].
     */
    val estimatedGyroscopeBiasAsTriad by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasAsTriad

    /**
     * Gets estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [gyroscopeInitialBiasAsTriad].
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getEstimatedGyroscopeBiasAsTriad(result: AngularSpeedTriad): Boolean {
        return accelerometerAndGyroscopeCalibrator.getEstimatedGyroscopeBiasAsTriad(result)
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in radians per
     * second (rad/s), or null if not available.
     */
    val estimatedGyroscopeBiasStandardDeviationNorm by accelerometerAndGyroscopeCalibrator::estimatedGyroscopeBiasStandardDeviationNorm

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
    val estimatedMagnetometerMm by magnetometerCalibrator::estimatedMm

    /**
     * Gets estimated magnetometer x-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSx by magnetometerCalibrator::estimatedSx

    /**
     * Gets estimated magnetometer y-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSy by magnetometerCalibrator::estimatedSy

    /**
     * Gets estimated magnetometer z-axis scale factor or null if not available.
     */
    val estimatedMagnetometerSz by magnetometerCalibrator::estimatedSz

    /**
     * Gets estimated magnetometer x-y cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMxy by magnetometerCalibrator::estimatedMxy

    /**
     * Gets estimated magnetometer x-z cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMxz by magnetometerCalibrator::estimatedMxz

    /**
     * Gets estimated magnetometer y-x cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMyx by magnetometerCalibrator::estimatedMyx

    /**
     * Gets estimated magnetometer y-z cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMyz by magnetometerCalibrator::estimatedMyz

    /**
     * Gets estimated magnetometer z-x cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMzx by magnetometerCalibrator::estimatedMzx

    /**
     * Gets estimated magnetometer z-y cross-coupling error or null if not available.
     */
    val estimatedMagnetometerMzy by magnetometerCalibrator::estimatedMzy

    /**
     * Gets estimated covariance matrix for estimated magnetometer parameters or null if not
     * available.
     * When hard iron is known, diagonal elements of the covariance matrix contains variance for the
     * following parameters (following indicated order): sx, sy, sz, mxy, mxz, myz, mzx, mzy.
     * When hard iron is not known, diagonal elements of the covariance matrix contains variance for
     * the following parameters (following indicated order): bx, by, bz, sx, sy, sz, mxy, mxz,
     * myx, myz, mzx, mzy.
     */
    val estimatedMagnetometerCovariance by magnetometerCalibrator::estimatedCovariance

    /**
     * Gets estimated chi square value for magnetometer or null if not available.
     */
    val estimatedMagnetometerChiSq by magnetometerCalibrator::estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided magnetometer measurements or null if
     * not available.
     */
    val estimatedMagnetometerMse by magnetometerCalibrator::estimatedMse

    /**
     * Gets x coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronX], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronX].
     */
    val estimatedMagnetometerHardIronX by magnetometerCalibrator::estimatedHardIronX

    /**
     * Gets y coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronY], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronY].
     */
    val estimatedMagnetometerHardIronY by magnetometerCalibrator::estimatedHardIronY

    /**
     * Gets z coordinate of estimated magnetometer hard iron expressed in Teslas (T).
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronZ], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronZ].
     */
    val estimatedMagnetometerHardIronZ by magnetometerCalibrator::estimatedHardIronZ

    /**
     * Gets x coordinate of estimated magnetometer hard iron.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronXAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronXAsMeasurement].
     */
    val estimatedMagnetometerHardIronXAsMeasurement by magnetometerCalibrator::estimatedHardIronXAsMeasurement

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
        return magnetometerCalibrator.getEstimatedHardIronXAsMeasurement(result)
    }

    /**
     * Gets y coordinate of estimated magnetometer hard iron.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronYAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronYAsMeasurement].
     */
    val estimatedMagnetometerHardIronYAsMeasurement by magnetometerCalibrator::estimatedHardIronYAsMeasurement

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
        return magnetometerCalibrator.getEstimatedHardIronYAsMeasurement(result)
    }

    /**
     * Gets z coordinate of estimated magnetometer hard iron.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronZAsMeasurement], otherwise it will be the estimated value
     * obtained after solving calibration, that might differ from
     * [magnetometerInitialHardIronZAsMeasurement].
     */
    val estimatedMagnetometerHardIronZAsMeasurement by magnetometerCalibrator::estimatedHardIronZAsMeasurement

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
        return magnetometerCalibrator.getEstimatedHardIronZAsMeasurement(result)
    }

    /**
     * Gets estimated magnetometer hard iron.
     * If [isMagnetometerGroundTruthInitialHardIron] is true, this will be equal to
     * [magnetometerInitialHardIronAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [magnetometerInitialHardIronAsTriad].
     */
    val estimatedMagnetometerHardIronAsTriad by magnetometerCalibrator::estimatedHardIronAsTriad

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
        return magnetometerCalibrator.getEstimatedHardIronAsTriad(result)
    }

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization
     * expressed in radians per second (rad/s).
     * This is only available once generator completes initialization.
     */
    val gyroscopeBaseNoiseLevel by accelerometerAndGyroscopeCalibrator::gyroscopeBaseNoiseLevel

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val gyroscopeBaseNoiseLevelAsMeasurement by accelerometerAndGyroscopeCalibrator::gyroscopeBaseNoiseLevelAsMeasurement

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getGyroscopeBaseNoiseLevelAsMeasurement(result: AngularSpeed): Boolean {
        return accelerometerAndGyroscopeCalibrator.getGyroscopeBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization
     * expressed in Teslas (T).
     * This is only available once generator completes initialization.
     */
    val magnetometerBaseNoiseLevel by magnetometerCalibrator::baseNoiseLevel

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val magnetometerBaseNoiseLevelAsMeasurement by magnetometerCalibrator::baseNoiseLevelAsMeasurement

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerBaseNoiseLevelAsMeasurement(result: MagneticFluxDensity): Boolean {
        return magnetometerCalibrator.getBaseNoiseLevelAsMeasurement(result)
    }

    /**
     * Number of gyroscope measurements that have been processed.
     */
    val numberOfProcessedGyroscopeMeasurements by accelerometerAndGyroscopeCalibrator::numberOfProcessedGyroscopeMeasurements

    /**
     * List of accelerometer measurements that have been collected so far to be used for
     * accelerometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val accelerometerMeasurements by accelerometerAndGyroscopeCalibrator::accelerometerMeasurements

    /**
     * List of gyroscope measurements that have been collected so far to be used for
     * gyroscope calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val gyroscopeMeasurements by accelerometerAndGyroscopeCalibrator::gyroscopeMeasurements

    /**
     * List of magnetometer measurements that have been collected so far to be used for
     * magnetometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val magnetometerMeasurements by magnetometerCalibrator::measurements

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * accelerometer calibration process can be solved.
     */
    val isReadyToSolveAccelerometerCalibration by accelerometerAndGyroscopeCalibrator::isReadyToSolveAccelerometerCalibration

    /**
     * Indicates whether enough measurements have been picked at dynamic intervals so that the
     * gyroscope calibration process can be solved.
     */
    val isReadyToSolveGyroscopeCalibration by accelerometerAndGyroscopeCalibrator::isReadyToSolveGyroscopeCalibration

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    val isReadyToSolveMagnetometerCalibration by magnetometerCalibrator::isReadyToSolveCalibration

    /**
     * Indicates whether enough measurements have been picked at static or dynamic intervals so that
     * the calibration process can be solved.
     */
    override val isReadyToSolveCalibration: Boolean
        get() = accelerometerAndGyroscopeCalibrator.isReadyToSolveCalibration
                && isReadyToSolveMagnetometerCalibration

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
    override fun start() {
        check(!running)

        reset()

        accelerometerAndGyroscopeCalibrator.start()
        magnetometerCalibrator.start()
    }

    /**
     * Stops calibrator.
     * When this is called, no more sensor measurements are collected.
     */
    override fun stop() {
        accelerometerAndGyroscopeCalibrator.stop()
        magnetometerCalibrator.stop()
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
    override fun calibrate(): Boolean {
        return accelerometerAndGyroscopeCalibrator.calibrate()
                && magnetometerCalibrator.calibrate()
    }

    /**
     * Resets this calibrator.
     */
    private fun reset() {
        accelerometerAndGyroscopeInitializationStarted = false
        magnetometerInitializationStarted = false
        accelerometerAndGyroscopeInitializationCompleted = false
        magnetometerInitializationCompleted = false
        accelerometerAndGyroscopeReadyToSolveCalibration = false
        magnetometerReadyToSolveCalibration = false
        accelerometerAndGyroscopeCalibrationSolvingStarted = false
        magnetometerCalibrationSolvingStarted = false
        accelerometerAndGyroscopeCalibrationCompleted = false
        magnetometerCalibrationCompleted = false
        accelerometerAndGyroscopeStopped = false
        magnetometerStopped = false
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            measurement: StandardDeviationBodyKinematics,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
    }

    /**
     * Interface to notify when a new gyroscope calibration measurement is generated.
     */
    fun interface OnGeneratedGyroscopeMeasurementListener {
        /**
         * Called when a new gyroscope calibration measurement is generated.
         *
         * @param calibrator calibrator that raised the event.
         * @param measurement generated gyroscope calibration measurement.
         * @param measurementsFoundSoFar number of measurements that have been found so far.
         * @param requiredMeasurements required number of measurements to solve calibration.
         */
        fun onGeneratedGyroscopeMeasurement(
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            measurement: BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
        )
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            measurement: StandardDeviationBodyMagneticFluxDensity,
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
        fun onUnreliableGravityEstimation(
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2
        )
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            biasX: Double,
            biasY: Double,
            biasZ: Double
        )
    }

    /**
     * Interface to notify when initial gyroscope bias guess is available.
     * If [isGyroscopeGroundTruthInitialBias] is true, then initial bias is considered the true
     * value after solving calibration, otherwise, initial bias is considered only an initial guess.
     */
    fun interface OnInitialGyroscopeBiasAvailableListener {
        /**
         * Called when initial gyroscope bias is available.
         * If [isGyroscopeGroundTruthInitialBias] is true, then initial bias is considered the true
         * value after solving calibration, otherwise, initial bias is considered only an initial
         * guess.
         *
         * @param calibrator calibrator that raised the event.
         * @param biasX x-coordinate of bias expressed in radians per second (rad/s).
         * @param biasY y-coordinate of bias expressed in radians per second (rad/s).
         * @param biasZ z-coordinate of bias expressed in radians per second (rad/s).
         */
        fun onInitialBiasAvailable(
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            biasX: Double,
            biasY: Double,
            biasZ: Double
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator2,
            hardIronX: Double,
            hardIronY: Double,
            hardIronZ: Double
        )
    }
}