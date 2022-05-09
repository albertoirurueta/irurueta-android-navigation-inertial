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
import com.irurueta.android.navigation.inertial.calibration.builder.GyroscopeInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.builder.MagnetometerInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.*
import com.irurueta.android.navigation.inertial.calibration.noise.AccumulatedMeasurementEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.GravityNormEstimator
import com.irurueta.android.navigation.inertial.calibration.noise.StopMode
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAccelerometerCalibrator
import com.irurueta.navigation.inertial.calibration.accelerometer.UnknownBiasAccelerometerCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasGyroscopeCalibrator
import com.irurueta.navigation.inertial.calibration.gyroscope.UnknownBiasGyroscopeCalibrator
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultMagnetometerQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator
import com.irurueta.navigation.inertial.calibration.magnetometer.UnknownHardIronMagnetometerCalibrator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.*
import java.util.*
import kotlin.math.max

/**
 * Collects accelerometer, gyroscope and magnetometer measurements by detecting periods when device
 * remains static or dynamic using the accelerometer, using such periods to determine orientation
 * based on gravity vector at the end of static intervals, and integrating values of gyroscope
 * measurements during dynamic ones, and using static periods to obtain averaged accelerometer and
 * magnetometer values.
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
class StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator private constructor(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    accelerometerSensorDelay: SensorDelay,
    val gyroscopeSensorDelay: SensorDelay,
    val magnetometerSensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    errorListener: OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    var generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener?,
    var generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener?,
    var generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener?,
    readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    stoppedListener: OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>?,
    var unreliableGravityNormEstimationListener: OnUnreliableGravityEstimationListener?,
    var initialAccelerometerBiasAvailableListener: OnInitialAccelerometerBiasAvailableListener?,
    var initialGyroscopeBiasAvailableListener: OnInitialGyroscopeBiasAvailableListener?,
    var initialMagnetometerHardIronAvailableListener: OnInitialMagnetometerHardIronAvailableListener?,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener?,
    val accelerometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyKinematics>,
    val gyroscopeQualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>,
    val magnetometerQualityScoreMapper: QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity>
) : StaticIntervalWithMeasurementGeneratorCalibrator<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator, TimedBodyKinematicsAndMagneticFluxDensity>(
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
     * [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], bias
     * guess is zero, otherwise when it is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias guess is the
     * device calibrated values.
     * @param isGyroscopeGroundTruthInitialBias true if estimated gyroscope bias is assumed to be
     * the true value, false if estimated bias is assumed to be only an initial guess. When
     * [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], bias guess is zero,
     * otherwise when it is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED], bias guess
     * is the device calibrated values.
     * @param isMagnetometerGroundTruthInitialHardIron true if estimated magnetometer hard iron is
     * assumed to be the true value, false if estimated hard iron is assumed to be only an initial
     * guess. When [magnetometerSensorType] is
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER], hard iron guess is zero, otherwise
     * when it is [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED], hard iron
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
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
        magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isAccelerometerGroundTruthInitialBias: Boolean = false,
        isGyroscopeGroundTruthInitialBias: Boolean = false,
        isMagnetometerGroundTruthInitialHardIron: Boolean = false,
        initializationStartedListener: OnInitializationStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        errorListener: OnErrorListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        generatedAccelerometerMeasurementListener: OnGeneratedAccelerometerMeasurementListener? = null,
        generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener? = null,
        generatedMagnetometerMeasurementListener: OnGeneratedMagnetometerMeasurementListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
        stoppedListener: OnStoppedListener<StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator>? = null,
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
     * Listener used by internal generator to handle events when initialization is started.
     */
    private val generatorInitializationStartedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationStartedListener {
            initializationStartedListener?.onInitializationStarted(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when initialization is completed.
     */
    private val generatorInitializationCompletedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnInitializationCompletedListener { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when an error occurs.
     */
    private val generatorErrorListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnErrorListener { _, reason ->
            stop()
            errorListener?.onError(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
                CalibratorErrorReason.mapErrorReason(reason)
            )
        }

    /**
     * Listener used by internal generator to handle events when a static interval is detected.
     */
    private val generatorStaticIntervalDetectedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalDetectedListener {
            staticIntervalDetectedListener?.onStaticIntervalDetected(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator
            )
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is detected.
     */
    private val generatorDynamicIntervalDetectedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalDetectedListener {
            dynamicIntervalDetectedListener?.onDynamicIntervalDetected(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a static interval is skipped.
     */
    private val generatorStaticIntervalSkippedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnStaticIntervalSkippedListener {
            staticIntervalSkippedListener?.onStaticIntervalSkipped(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is skipped.
     */
    private val generatorDynamicIntervalSkippedListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnDynamicIntervalSkippedListener {
            dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a new accelerometer measurement is generated.
     */
    private val generatorGeneratedAccelerometerMeasurementListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedAccelerometerMeasurementListener { _, measurement ->
            accelerometerMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = accelerometerMeasurements.size

            generatedAccelerometerMeasurementListener?.onGeneratedAccelerometerMeasurement(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            checkMeasurementsAndSolveCalibration()
        }

    /**
     * Listener  used by internal generator to handle events when a new gyroscope measurement is generated.
     */
    private val generatorGeneratedGyroscopeMeasurementListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedGyroscopeMeasurementListener { _, measurement ->
            gyroscopeMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = gyroscopeMeasurements.size

            generatedGyroscopeMeasurementListener?.onGeneratedGyroscopeMeasurement(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            checkMeasurementsAndSolveCalibration()
        }

    /**
     * Listener  used by internal generator to handle events when a new measurement is generated.
     */
    private val generatorGeneratedMagnetometerMeasurementListener =
        AccelerometerGyroscopeAndMagnetometerMeasurementGenerator.OnGeneratedMagnetometerMeasurementListener { generator, measurement ->
            if (isInitialMagneticFluxDensityNormMeasured && initialMagneticFluxDensityNorm == null) {
                // set initial average magnetic flux density norm measured during initialization
                initialMagneticFluxDensityNorm = generator.initialMagneticFluxDensityNorm
            }

            magnetometerMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = magnetometerMeasurements.size

            generatedMagnetometerMeasurementListener?.onGeneratedMagnetometerMeasurement(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            checkMeasurementsAndSolveCalibration()
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
     * Listener for gyroscope sensor collector.
     * This is used to determine device calibration and obtain initial guesses
     * for gyroscope bias (only available if
     * [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED] is used, otherwise zero
     * bias is assumed as an initial guess).
     */
    private val generatorGyroscopeMeasurementListener =
        GyroscopeSensorCollector.OnMeasurementListener { _, _, _, bx, by, bz, _, _ ->
            if (isFirstGyroscopeMeasurement) {
                updateGyroscopeInitialBiases(bx, by, bz)
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
            unreliableGravityNormEstimationListener?.onUnreliableGravityEstimation(this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator)
        }

    /**
     * Internal generator to generate measurements for calibration.
     */
    override val generator = AccelerometerGyroscopeAndMagnetometerMeasurementGenerator(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        gyroscopeSensorType,
        gyroscopeSensorDelay,
        magnetometerSensorType,
        magnetometerSensorDelay,
        generatorInitializationStartedListener,
        generatorInitializationCompletedListener,
        generatorErrorListener,
        generatorStaticIntervalDetectedListener,
        generatorDynamicIntervalDetectedListener,
        generatorStaticIntervalSkippedListener,
        generatorDynamicIntervalSkippedListener,
        generatorGeneratedAccelerometerMeasurementListener,
        generatorGeneratedGyroscopeMeasurementListener,
        generatorGeneratedMagnetometerMeasurementListener,
        accelerometerMeasurementListener = generatorAccelerometerMeasurementListener,
        gyroscopeMeasurementListener = generatorGyroscopeMeasurementListener,
        magnetometerMeasurementListener = generatorMagnetometerMeasurementListener,
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
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected during dynamic intervals.
     */
    private var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? = null

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
     * Gets x-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasX] will be equal to this value, otherwise [estimatedGyroscopeBiasX]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasX].
     */
    var gyroscopeInitialBiasX: Double? = null
        private set

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasY] will be equal to this value, otherwise [estimatedGyroscopeBiasY]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasY].
     */
    var gyroscopeInitialBiasY: Double? = null
        private set

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess and expressed in radians per
     * second (rad/s).
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasZ] will be equal to this value, otherwise [estimatedGyroscopeBiasZ]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasZ].
     */
    var gyroscopeInitialBiasZ: Double? = null
        private set

    /**
     * Gets x-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasX] will be equal to this value, otherwise [estimatedGyroscopeBiasX]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasX].
     */
    val gyroscopeInitialBiasXAsMeasurement: AngularSpeed?
        get() {
            val initialBiasX = gyroscopeInitialBiasX ?: return null
            return AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND)
        }

    /**
     * Gets x-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
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
        val initialBiasX = gyroscopeInitialBiasX
        return if (initialBiasX != null) {
            result.value = initialBiasX
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasY] will be equal to this value, otherwise [estimatedGyroscopeBiasY]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasY].
     */
    val gyroscopeInitialBiasYAsMeasurement: AngularSpeed?
        get() {
            val initialBiasY = gyroscopeInitialBiasY ?: return null
            return AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND)
        }

    /**
     * Gets y-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
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
        val initialBiasY = gyroscopeInitialBiasY
        return if (initialBiasY != null) {
            result.value = initialBiasY
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasZ] will be equal to this value, otherwise [estimatedGyroscopeBiasZ]
     * will be the estimated bias after solving calibration, which will differ from
     * [gyroscopeInitialBiasZ].
     */
    val gyroscopeInitialBiasZAsMeasurement: AngularSpeed?
        get() {
            val initialBiasZ = gyroscopeInitialBiasZ ?: return null
            return AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND)
        }

    /**
     * Gets z-coordinate of gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
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
        val initialBiasZ = gyroscopeInitialBiasZ
        return if (initialBiasZ != null) {
            result.value = initialBiasZ
            result.unit = AngularSpeedUnit.RADIANS_PER_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasAsTriad] will be equal to this value, otherwise
     * [estimatedGyroscopeBiasAsTriad] will be the estimated bias after solving calibration, which
     * will differ from [gyroscopeInitialBiasAsTriad].
     */
    val gyroscopeInitialBiasAsTriad: AngularSpeedTriad?
        get() {
            val initialBiasX = gyroscopeInitialBiasX
            val initialBiasY = gyroscopeInitialBiasY
            val initialBiasZ = gyroscopeInitialBiasZ
            return if (initialBiasX != null && initialBiasY != null && initialBiasZ != null) {
                AngularSpeedTriad(
                    AngularSpeedUnit.RADIANS_PER_SECOND,
                    initialBiasX,
                    initialBiasY,
                    initialBiasZ
                )
            } else {
                null
            }
        }

    /**
     * Gets gyroscope bias used as an initial guess.
     * This value is determined once the calibrator starts.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED],
     * this will be equal to the value used internally by the device as part of the gyroscope
     * hardware calibration.
     * If [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], then gyroscope
     * sensor measurements are assumed to be already bias compensated, and the initial bias is
     * assumed to be zero.
     * If [isGyroscopeGroundTruthInitialBias] is true, this is assumed to be the true bias, and
     * [estimatedGyroscopeBiasAsTriad] will be equal to this value, otherwise
     * [estimatedGyroscopeBiasAsTriad] will be the estimated bias after solving calibration, which
     * will differ from [gyroscopeInitialBiasAsTriad].
     */
    fun getGyroscopeInitialBiasAsTriad(result: AngularSpeedTriad): Boolean {
        val initialBiasX = gyroscopeInitialBiasX
        val initialBiasY = gyroscopeInitialBiasY
        val initialBiasZ = gyroscopeInitialBiasZ
        return if (initialBiasX != null && initialBiasY != null && initialBiasZ != null) {
            result.setValueCoordinatesAndUnit(
                initialBiasX,
                initialBiasY,
                initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )
            true
        } else {
            false
        }
    }

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
     * Indicates whether gyroscope initial bias is considered a ground-truth known bias.
     * When true, estimated biases are exactly equal to initial biases, otherwise
     * initial biases are just an initial guess and estimated ones might differ after
     * solving calibration.
     *
     * @throws IllegalStateException if calibrator is already running.
     */
    var isGyroscopeGroundTruthInitialBias: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
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
     * If location is provided, WGS84 Earth model is used to determine gravity norm
     * at such location, otherwise gravity norm is estimated during initialization by using the
     * gravity sensor of device.
     * Additionally, if both [location] and [timestamp] are provided, magnetometer calibration
     * uses World Magnetic Model, otherwise magnetometer calibration uses measured norm of magnetic
     * field.
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
     * Gets gyroscope sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val gyroscopeSensor
        get() = generator.gyroscopeSensor

    /**
     * Gets magnetometer sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor
        get() = generator.magnetometerSensor

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
     * Gets or sets initial gyroscope scaling factors and cross coupling errors matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var gyroscopeInitialMg: Matrix
        get() {
            val result = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
            getGyroscopeInitialMg(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            gyroscopeInitialSx = value.getElementAtIndex(0)
            gyroscopeInitialMyx = value.getElementAtIndex(1)
            gyroscopeInitialMzx = value.getElementAtIndex(2)

            gyroscopeInitialMxy = value.getElementAtIndex(3)
            gyroscopeInitialSy = value.getElementAtIndex(4)
            gyroscopeInitialMzy = value.getElementAtIndex(5)

            gyroscopeInitialMxz = value.getElementAtIndex(6)
            gyroscopeInitialMyz = value.getElementAtIndex(7)
            gyroscopeInitialSz = value.getElementAtIndex(8)
        }

    /**
     * Gets initial gyroscope scale factors and cross coupling errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided result matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getGyroscopeInitialMg(result: Matrix) {
        require(result.rows == BodyKinematics.COMPONENTS && result.columns == BodyKinematics.COMPONENTS)

        result.setElementAtIndex(0, gyroscopeInitialSx)
        result.setElementAtIndex(1, gyroscopeInitialMyx)
        result.setElementAtIndex(2, gyroscopeInitialMzx)

        result.setElementAtIndex(3, gyroscopeInitialMxy)
        result.setElementAtIndex(4, gyroscopeInitialSy)
        result.setElementAtIndex(5, gyroscopeInitialMzy)

        result.setElementAtIndex(6, gyroscopeInitialMxz)
        result.setElementAtIndex(7, gyroscopeInitialMyz)
        result.setElementAtIndex(8, gyroscopeInitialSz)
    }

    /**
     * Gets or sets initial x scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z scaling factor for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialSz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-y cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMxy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial x-z cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMxz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-x cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMyx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial y-z cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMyz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-x cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMzx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets initial z-y cross coupling error for gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeInitialMzy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

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
        check(!running)
        this.gyroscopeInitialSx = gyroscopeInitialSx
        this.gyroscopeInitialSy = gyroscopeInitialSy
        this.gyroscopeInitialSz = gyroscopeInitialSz
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
        check(!running)
        this.gyroscopeInitialMxy = gyroscopeInitialMxy
        this.gyroscopeInitialMxz = gyroscopeInitialMxz
        this.gyroscopeInitialMyx = gyroscopeInitialMyx
        this.gyroscopeInitialMyz = gyroscopeInitialMyz
        this.gyroscopeInitialMzx = gyroscopeInitialMzx
        this.gyroscopeInitialMzy = gyroscopeInitialMzy
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
        setGyroscopeInitialScalingFactors(
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz
        )
        setGyroscopeInitialCrossCouplingErrors(
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
    var gyroscopeInitialGg: Matrix = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            field = value
        }

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
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix during accelerometer
     * calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isAccelerometerCommonAxisUsed: Boolean =
        StaticIntervalAccelerometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Indicates or specifies whether z-axis is assumed to be common for gyroscope. When enabled,
     * this eliminates 3 variables from Mg matrix during gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isGyroscopeCommonAxisUsed: Boolean =
        StaticIntervalGyroscopeCalibrator.DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Indicates or specifies whether z-axis is assumed to be common for magnetometer and
     * gyroscope. When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isMagnetometerCommonAxisUsed: Boolean =
        StaticIntervalMagnetometerCalibrator.DEFAULT_USE_COMMON_Z_AXIS
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Indicates or specifies whether G-dependent cross biases are being estimated or not. When enabled,
     * this adds 9 variables from Gg matrix.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isGDependentCrossBiasesEstimated: Boolean =
        StaticIntervalGyroscopeCalibrator.DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES
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
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isAccelerometerGroundTruthInitialBias) {
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_KNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                StaticIntervalAccelerometerCalibrator.ACCELEROMETER_UNKNOWN_BIAS_MINIMUM_MEASUREMENTS_GENERAL
            }
        }

    /**
     * Gets minimum number of required measurements to start gyroscope calibration.
     * Each time that the device is moved, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredGyroscopeMeasurements: Int
        get() = if (isGyroscopeGroundTruthInitialBias) {
            // Known bias
            if (isGyroscopeCommonAxisUsed) {
                if (isGDependentCrossBiasesEstimated) {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES
                } else {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS
                }
            } else {
                if (isGDependentCrossBiasesEstimated) {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES
                } else {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL
                }
            }
        } else {
            // Unknown bias
            if (isGyroscopeCommonAxisUsed) {
                if (isGDependentCrossBiasesEstimated) {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES
                } else {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS
                }
            } else {
                if (isGDependentCrossBiasesEstimated) {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES
                } else {
                    StaticIntervalGyroscopeCalibrator.GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL
                }
            }
        }

    /**
     * Gets minimum number of required measurements to start magnetometer calibration.
     * Each time that the device is kept static, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    val minimumRequiredMagnetometerMeasurements: Int
        get() = if (isMagnetometerCommonAxisUsed) {
            if (isMagnetometerGroundTruthInitialHardIron) {
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            } else {
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_COMMON_Z_AXIS
            }
        } else {
            if (isMagnetometerGroundTruthInitialHardIron) {
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_KNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            } else {
                StaticIntervalMagnetometerCalibrator.MAGNETOMETER_UNKNOWN_HARD_IRON_MINIMUM_MEASUREMENTS_GENERAL
            }
        }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is kept static or moved, new accelerometer and gyroscope
     * measurements are collected.
     * When the required number of measurements is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = max(
            max(
                minimumRequiredAccelerometerMeasurements,
                minimumRequiredGyroscopeMeasurements
            ), minimumRequiredMagnetometerMeasurements
        )

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
    var accelerometerRobustConfidence: Double =
        StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_CONFIDENCE
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
    var accelerometerRobustMaxIterations: Int =
        StaticIntervalAccelerometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
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
    var accelerometerRobustThresholdFactor: Double =
        StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
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
    var accelerometerRobustStopThresholdFactor: Double =
        StaticIntervalAccelerometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Indicates robust method used to solve gyroscope calibration.
     * If null, no robust method is used at all, and instead an LMSE solution is found.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustMethod: RobustEstimatorMethod? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

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
    var gyroscopeRobustConfidence: Double =
        StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_CONFIDENCE
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value in 0.0..1.0)
            check(!running)

            field = value
        }

    /**
     * Maximum number of iterations to attempt to find a robust gyroscope calibration solution.
     * By default this is 5000.
     * This property is only taken into account if a not-null [gyroscopeRobustMethod] is
     * specified.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if calibrator is currently running.
     */
    var gyroscopeRobustMaxIterations: Int =
        StaticIntervalGyroscopeCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0)
            check(!running)
            field = value
        }

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
    var gyroscopeRobustPreliminarySubsetSize: Int = 0
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value >= minimumRequiredGyroscopeMeasurements)
            check(!running)
            field = value
        }

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
    var gyroscopeRobustThreshold: Double? = null
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value == null || value > 0.0)
            check(!running)
            field = value
        }

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
    var gyroscopeRobustThresholdFactor: Double =
        StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
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
    var gyroscopeRobustStopThresholdFactor: Double =
        StaticIntervalGyroscopeCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

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
    var magnetometerRobustConfidence: Double =
        StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_CONFIDENCE
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
    var magnetometerRobustMaxIterations: Int =
        StaticIntervalMagnetometerCalibrator.ROBUST_DEFAULT_MAX_ITERATIONS
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
    var magnetometerRobustThresholdFactor: Double =
        StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_THRESHOLD_FACTOR
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
    var magnetometerRobustStopThresholdFactor: Double =
        StaticIntervalMagnetometerCalibrator.DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
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
    val estimatedGyroscopeMg
        get() = gyroscopeInternalCalibrator?.estimatedMg

    /**
     * Gets estimated gyroscope x-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSx: Double?
        get() = gyroscopeInternalCalibrator?.estimatedSx

    /**
     * Gets estimated gyroscope y-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSy: Double?
        get() = gyroscopeInternalCalibrator?.estimatedSy

    /**
     * Gets estimated gyroscope z-axis scale factor or null if not available.
     */
    val estimatedGyroscopeSz: Double?
        get() = gyroscopeInternalCalibrator?.estimatedSz

    /**
     * Gets estimated gyroscope x-y cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMxy: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMxy

    /**
     * Gets estimated gyroscope x-z cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMxz: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMxz

    /**
     * Gets estimated gyroscope y-x cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMyx: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMyx

    /**
     * Gets estimated gyroscope y-z cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMyz: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMyz

    /**
     * Gets estimated gyroscope z-x cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMzx: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMzx

    /**
     * Gets estimated gyroscope z-y cross-coupling error or null if not available.
     */
    val estimatedGyroscopeMzy: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMzy

    /**
     * Gets estimated gyroscope G-dependent cross biases introduced on the gyroscope by the specific
     * forces sensed by the accelerometer.
     */
    val estimatedGyroscopeGg
        get() = gyroscopeInternalCalibrator?.estimatedGg

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
    val estimatedGyroscopeCovariance: Matrix?
        get() = gyroscopeInternalCalibrator?.estimatedCovariance

    /**
     * Gets estimated chi square value for gyroscope or null if not available.
     */
    val estimatedGyroscopeChiSq: Double?
        get() = gyroscopeInternalCalibrator?.estimatedChiSq

    /**
     * Gets estimated mean square error respect to provided gyroscope measurements or null if
     * not available.
     */
    val estimatedGyroscopeMse: Double?
        get() = gyroscopeInternalCalibrator?.estimatedMse

    /**
     * Gets x coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasX].
     */
    val estimatedGyroscopeBiasX: Double?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasX
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasX
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasY].
     */
    val estimatedGyroscopeBiasY: Double?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasY
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasY
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per second (rad/s).
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasZ].
     */
    val estimatedGyroscopeBiasZ: Double?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasZ
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasZ
                }
                else -> {
                    null
                }
            }
        }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasX], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasX].
     */
    val estimatedGyroscopeBiasXAsMeasurement: AngularSpeed?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasAngularSpeedX
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasAngularSpeedX
                }
                else -> {
                    null
                }
            }
        }

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
        return when (val internalCalibrator = gyroscopeInternalCalibrator) {
            is UnknownBiasGyroscopeCalibrator -> {
                internalCalibrator.getEstimatedBiasAngularSpeedX(result)
            }
            is KnownBiasGyroscopeCalibrator -> {
                internalCalibrator.getBiasAngularSpeedX(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasY], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasY].
     */
    val estimatedGyroscopeBiasYAsMeasurement: AngularSpeed?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasAngularSpeedY
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasAngularSpeedY
                }
                else -> {
                    null
                }
            }
        }

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
        return when (val internalCalibrator = gyroscopeInternalCalibrator) {
            is UnknownBiasGyroscopeCalibrator -> {
                internalCalibrator.getEstimatedBiasAngularSpeedY(result)
            }
            is KnownBiasGyroscopeCalibrator -> {
                internalCalibrator.getBiasAngularSpeedY(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasZ], otherwise it will be the estimated value obtained after solving
     * calibration, that might differ from [gyroscopeInitialBiasZ].
     */
    val estimatedGyroscopeBiasZAsMeasurement: AngularSpeed?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasAngularSpeedZ
                }
                is KnownBiasGyroscopeCalibrator -> {
                    internalCalibrator.biasAngularSpeedZ
                }
                else -> {
                    null
                }
            }
        }

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
        return when (val internalCalibrator = gyroscopeInternalCalibrator) {
            is UnknownBiasGyroscopeCalibrator -> {
                internalCalibrator.getEstimatedBiasAngularSpeedZ(result)
            }
            is KnownBiasGyroscopeCalibrator -> {
                internalCalibrator.getBiasAngularSpeedZ(result)
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets estimated gyroscope bias.
     * If [isGyroscopeGroundTruthInitialBias] is true, this will be equal to
     * [gyroscopeInitialBiasAsTriad], otherwise it will be the estimated value obtained after
     * solving calibration, that might differ from [gyroscopeInitialBiasAsTriad].
     */
    val estimatedGyroscopeBiasAsTriad: AngularSpeedTriad?
        get() {
            return when (val internalCalibrator = gyroscopeInternalCalibrator) {
                is UnknownBiasGyroscopeCalibrator -> {
                    internalCalibrator.estimatedBiasAsTriad
                }
                is KnownBiasGyroscopeCalibrator -> {
                    AngularSpeedTriad(
                        AngularSpeedUnit.RADIANS_PER_SECOND,
                        internalCalibrator.biasX,
                        internalCalibrator.biasY,
                        internalCalibrator.biasZ
                    )
                }
                else -> {
                    null
                }
            }
        }

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
        return when (val internalCalibrator = gyroscopeInternalCalibrator) {
            is UnknownBiasGyroscopeCalibrator -> {
                internalCalibrator.getEstimatedBiasAsTriad(result)
            }
            is KnownBiasGyroscopeCalibrator -> {
                result.setValueCoordinatesAndUnit(
                    internalCalibrator.biasX,
                    internalCalibrator.biasY,
                    internalCalibrator.biasZ,
                    AngularSpeedUnit.RADIANS_PER_SECOND
                )
                true
            }
            else -> {
                false
            }
        }
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in radians per
     * second (rad/s), or null if not available.
     */
    val estimatedGyroscopeBiasStandardDeviationNorm: Double?
        get() {
            val internalCalibrator = gyroscopeInternalCalibrator ?: return null
            return if (internalCalibrator is GyroscopeBiasUncertaintySource) {
                internalCalibrator.estimatedBiasStandardDeviationNorm
            } else {
                null
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
     * Gets gyroscope measurement base noise level that has been detected during initialization
     * expressed in radians per second (rad/s).
     * This is only available once generator completes initialization.
     */
    val gyroscopeBaseNoiseLevel
        get() = generator.gyroscopeBaseNoiseLevel

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val gyroscopeBaseNoiseLevelAsMeasurement
        get() = generator.gyroscopeBaseNoiseLevelAsMeasurement

    /**
     * Gets gyroscope measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getGyroscopeBaseNoiseLevelAsMeasurement(result: AngularSpeed): Boolean {
        return generator.getGyroscopeBaseNoiseLevelAsMeasurement(result)
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
     * Number of gyroscope measurements that have been processed.
     */
    val numberOfProcessedGyroscopeMeasurements
        get() = generator.numberOfProcessedGyroscopeMeasurements

    /**
     * Number of magnetometer measurements that have been processed.
     */
    val numberOfProcessedMagnetometerMeasurements
        get() = generator.numberOfProcessedMagnetometerMeasurements

    /**
     * List of accelerometer measurements that have been collected so far to be used for
     * accelerometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val accelerometerMeasurements = mutableListOf<StandardDeviationBodyKinematics>()

    /**
     * List of gyroscope measurements that have been collected so far to be used for
     * gyroscope calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val gyroscopeMeasurements =
        mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()

    /**
     * List of magnetometer measurements that have been collected so far to be used for
     * magnetometer calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val magnetometerMeasurements = mutableListOf<StandardDeviationBodyMagneticFluxDensity>()

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * accelerometer calibration process can be solved.
     */
    val isReadyToSolveAccelerometerCalibration
        get() = accelerometerMeasurements.size >= requiredMeasurements.coerceAtLeast(
            minimumRequiredMeasurements
        )

    /**
     * Indicates whether enough measurements have been picked at dynamic intervals so that the
     * gyroscope calibration process can be solved.
     */
    val isReadyToSolveGyroscopeCalibration
        get() = gyroscopeMeasurements.size >= requiredMeasurements.coerceAtLeast(
            minimumRequiredMeasurements
        )

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    val isReadyToSolveMagnetometerCalibration: Boolean
        get() = magnetometerMeasurements.size >= requiredMeasurements.coerceAtLeast(
            minimumRequiredMeasurements
        )

    /**
     * Indicates whether enough measurements have been picked at static or dynamic intervals so that
     * the calibration process can be solved.
     */
    override val isReadyToSolveCalibration: Boolean
        get() = isReadyToSolveAccelerometerCalibration && isReadyToSolveGyroscopeCalibration && isReadyToSolveMagnetometerCalibration

    /**
     * Starts calibrator.
     * This method starts collecting accelerometer and gyroscope measurements.
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
     * When this is called, no more accelerometer, gyroscope or gravity measurements are collected.
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

            // once accelerometer is calibrated, build gyroscope calibrator using estimated
            // accelerometer parameters
            gyroscopeInternalCalibrator = buildGyroscopeInternalCalibrator()
            gyroscopeInternalCalibrator?.calibrate()

            magnetometerInternalCalibrator?.calibrate()

            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(
                StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator::class.qualifiedName,
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
        gyroscopeMeasurements.clear()
        magnetometerMeasurements.clear()

        gravityNorm = null
        accelerometerResultUnreliable = false
        accelerometerInitialBiasX = null
        accelerometerInitialBiasY = null
        accelerometerInitialBiasZ = null

        gyroscopeInitialBiasX = null
        gyroscopeInitialBiasY = null
        gyroscopeInitialBiasZ = null

        magnetometerInitialHardIronX = null
        magnetometerInitialHardIronY = null
        magnetometerInitialHardIronZ = null

        accelerometerInternalCalibrator = null
        gyroscopeInternalCalibrator = null
        magnetometerInternalCalibrator = null

        initialMagneticFluxDensityNorm = null
    }

    /**
     * Checks current accelerometer and gyroscope calibration measurements, if enough and
     * [solveCalibrationWhenEnoughMeasurements] is true, starts calibration.
     */
    private fun checkMeasurementsAndSolveCalibration() {
        val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
        val accelerometerMeasurementsSize = accelerometerMeasurements.size
        val gyroscopeMeasurementsSize = gyroscopeMeasurements.size
        val magnetometerMeasurements = magnetometerMeasurements.size

        // check if enough measurements have been collected
        val isReadyToCalibrate =
            accelerometerMeasurementsSize >= reqMeasurements
                    && gyroscopeMeasurementsSize >= reqMeasurements
                    && magnetometerMeasurements >= reqMeasurements

        if (isReadyToCalibrate) {
            readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                this@StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator
            )

            // stop internal generator since no more measurements need to be collected
            internalStop(true)

            // build accelerometer and magnetometer calibrators
            accelerometerInternalCalibrator = buildAccelerometerInternalCalibrator()
            magnetometerInternalCalibrator = buildMagnetometerInternalCalibrator()

            if (solveCalibrationWhenEnoughMeasurements) {
                // execute calibration
                internalCalibrate()
            }
        }
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
     * Updates initial biases values when first gyroscope measurement is received, so
     * that hardware calibrated biases are retrieved if
     * [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED] is used.
     *
     * @param bx x-coordinate of initial bias to be set expressed in radians per second (rad/s).
     * @param by y-coordinate of initial bias to be set expressed in radians per second (rad/s).
     * @param bz z-coordinate of initial bias to be set expressed in radians per second (rad/s).
     */
    private fun updateGyroscopeInitialBiases(bx: Float?, by: Float?, bz: Float?) {
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

        gyroscopeInitialBiasX = initialBiasX
        gyroscopeInitialBiasY = initialBiasY
        gyroscopeInitialBiasZ = initialBiasZ

        initialGyroscopeBiasAvailableListener?.onInitialBiasAvailable(
            this,
            initialBiasX,
            initialBiasY,
            initialBiasZ
        )
    }

    /**
     * Updates initial hard iron values when first magnetometer measurement is received, so
     * that hardware calibrated biases are retrieved if
     * [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED] is used.
     *
     * @param hardIronX hard iron on device x-axis expressed in micro-Teslas (µT). Only
     * available when using [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param hardIronY hard iron on device y-axis expressed in micro-Teslas (µT). Only
     * available when using [MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param hardIronZ hard iron on device y-axis expressed in micro-Teslas (µT). Only
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

        magnetometerInitialHardIronX = initialHardIronX
        magnetometerInitialHardIronY = initialHardIronY
        magnetometerInitialHardIronZ = initialHardIronZ

        initialMagnetometerHardIronAvailableListener?.onInitialHardIronAvailable(
            this,
            initialHardIronX,
            initialHardIronY,
            initialHardIronZ
        )
    }

    /**
     * Indicates whether the generator has picked the first gyroscope measurement.
     */
    private val isFirstGyroscopeMeasurement: Boolean
        get() = generator.numberOfProcessedGyroscopeMeasurements <= FIRST_MEASUREMENT

    /**
     * Indicates whether the generator has picked the first accelerometer measurement.
     */
    private val isFirstMagnetometerMeasurement: Boolean
        get() = generator.numberOfProcessedMagnetometerMeasurements <= FIRST_MEASUREMENT

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

    /**
     * Builds an internal gyroscope calibrator based on all provided parameters.
     * To be able to build the gyroscope internal calibrator, first accelerometer calibration must
     * be solved.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildGyroscopeInternalCalibrator(): GyroscopeNonLinearCalibrator {
        val accelerometerSx = estimatedAccelerometerSx
        checkNotNull(accelerometerSx)

        val accelerometerSy = estimatedAccelerometerSy
        checkNotNull(accelerometerSy)

        val accelerometerSz = estimatedAccelerometerSz
        checkNotNull(accelerometerSz)

        val accelerometerMxy = estimatedAccelerometerMxy
        checkNotNull(accelerometerMxy)

        val accelerometerMxz = estimatedAccelerometerMxz
        checkNotNull(accelerometerMxz)

        val accelerometerMyx = estimatedAccelerometerMyx
        checkNotNull(accelerometerMyx)

        val accelerometerMyz = estimatedAccelerometerMyz
        checkNotNull(accelerometerMyz)

        val accelerometerMzx = estimatedAccelerometerMzx
        checkNotNull(accelerometerMzx)

        val accelerometerMzy = estimatedAccelerometerMzy
        checkNotNull(accelerometerMzy)

        return GyroscopeInternalCalibratorBuilder(
            gyroscopeMeasurements,
            gyroscopeRobustPreliminarySubsetSize,
            minimumRequiredGyroscopeMeasurements,
            gyroscopeRobustMethod,
            gyroscopeRobustConfidence,
            gyroscopeRobustMaxIterations,
            gyroscopeRobustThreshold,
            gyroscopeRobustThresholdFactor,
            gyroscopeRobustStopThresholdFactor,
            isGyroscopeGroundTruthInitialBias,
            isGyroscopeCommonAxisUsed,
            gyroscopeInitialBiasX,
            gyroscopeInitialBiasY,
            gyroscopeInitialBiasZ,
            gyroscopeInitialSx,
            gyroscopeInitialSy,
            gyroscopeInitialSz,
            gyroscopeInitialMxy,
            gyroscopeInitialMxz,
            gyroscopeInitialMyx,
            gyroscopeInitialMyz,
            gyroscopeInitialMzx,
            gyroscopeInitialMzy,
            isGDependentCrossBiasesEstimated,
            gyroscopeInitialGg,
            estimatedAccelerometerBiasX,
            estimatedAccelerometerBiasY,
            estimatedAccelerometerBiasZ,
            accelerometerSx,
            accelerometerSy,
            accelerometerSz,
            accelerometerMxy,
            accelerometerMxz,
            accelerometerMyx,
            accelerometerMyz,
            accelerometerMzx,
            accelerometerMzy,
            gyroscopeBaseNoiseLevel,
            gyroscopeQualityScoreMapper
        ).build()
    }

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

    private companion object {
        /**
         * Indicates when first sensor measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
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
            calibrator: StaticIntervalAccelerometerGyroscopeAndMagnetometerCalibrator,
            hardIronX: Double,
            hardIronY: Double,
            hardIronZ: Double
        )
    }
}