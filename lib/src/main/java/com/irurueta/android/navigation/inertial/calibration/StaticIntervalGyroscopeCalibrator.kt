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
import android.util.Log
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.calibration.builder.GyroscopeInternalCalibratorBuilder
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.GyroscopeMeasurementGenerator
import com.irurueta.android.navigation.inertial.calibration.intervals.measurements.SingleSensorCalibrationMeasurementGenerator
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.NavigationException
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.*
import com.irurueta.navigation.inertial.calibration.gyroscope.*
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.QualityScoreMapper
import com.irurueta.numerical.robust.RobustEstimatorMethod
import com.irurueta.units.*

/**
 * Collects accelerometer and magnetometer measurements by detecting periods when device remains
 * static or dynamic using the accelerometer, using such periods to determine orientation based
 * on gravity vector at the end of static intervals, and integrating values of gyroscope
 * measurements during dynamic ones.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
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
 * @property generatedGyroscopeMeasurementListener listener to notify when a new gyroscope
 * calibration measurement is generated.
 * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
 * collected and calibrator is ready to solve calibration.
 * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
 * @property calibrationCompletedListener listener to notify when calibration solving completes.
 * @property stoppedListener listener to notify when calibrator is stopped.
 * @property initialGyroscopeBiasAvailableListener listener to notify when a guess of bias values
 * is obtained.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 * @property gyroscopeQualityScoreMapper mapper to convert collected gyroscope measurements
 * into quality scores, based on the amount of standard deviation (the larger the variability, the
 * worse the score will be).
 */
class StaticIntervalGyroscopeCalibrator private constructor(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    accelerometerSensorDelay: SensorDelay,
    val gyroscopeSensorDelay: SensorDelay,
    solveCalibrationWhenEnoughMeasurements: Boolean,
    initializationStartedListener: OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>?,
    initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>?,
    errorListener: OnErrorListener<StaticIntervalGyroscopeCalibrator>?,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>?,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>?,
    staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>?,
    dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>?,
    var generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener?,
    readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>?,
    calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>?,
    calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>?,
    stoppedListener: OnStoppedListener<StaticIntervalGyroscopeCalibrator>?,
    var initialGyroscopeBiasAvailableListener: OnInitialGyroscopeBiasAvailableListener?,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener?,
    val gyroscopeQualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
) : StaticIntervalWithMeasurementGeneratorCalibrator<StaticIntervalGyroscopeCalibrator, TimedBodyKinematics>(
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
     * @property context Android context.
     * @property accelerometerSensorType One of the supported accelerometer sensor types.
     * @property gyroscopeSensorType One of the supported gyroscope sensor types.
     * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
     * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
     * @property solveCalibrationWhenEnoughMeasurements true to automatically solve calibration once
     * enough measurements are available, false otherwise.
     * @param isGyroscopeGroundTruthInitialBias true if estimated gyroscope bias is assumed to be
     * the true value, flase if estimated bias is assumed to be only an initial guess. When
     * [gyroscopeSensorType] is [GyroscopeSensorCollector.SensorType.GYROSCOPE], bias guess is zero,
     * otherwise when it is [GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED], bias guess
     * is the device calibrated values.
     * @property initializationStartedListener listener to notify when initialization starts.
     * @property initializationCompletedListener listener to notify when initialization completes.
     * @property errorListener listener to notify errors.
     * @property staticIntervalDetectedListener listener to notify when a static interval is detected.
     * @property dynamicIntervalDetectedListener listener to notify when a dynamic interval is detected.
     * @property staticIntervalSkippedListener listener to notify when a static interval is skipped if
     * its duration is too short.
     * @property dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped if
     * its duration is too long.
     * @property generatedGyroscopeMeasurementListener listener to notify when a new gyroscope
     * calibration measurement is generated.
     * @property readyToSolveCalibrationListener listener to notify when enough measurements have been
     * collected and calibrator is ready to solve calibration.
     * @property calibrationSolvingStartedListener listener to notify when calibration solving starts.
     * @property calibrationCompletedListener listener to notify when calibration solving completes.
     * @property stoppedListener listener to notify when calibrator is stopped.
     * @property initialGyroscopeBiasAvailableListener listener to notify when a guess of bias values
     * is obtained.
     * @property accuracyChangedListener listener to notify when sensor accuracy changes.
     * @property gyroscopeQualityScoreMapper mapper to convert collected gyroscope measurements
     * into quality scores, based on the amount of standard deviation (the larger the variability, the
     * worse the score will be).
     */
    constructor(
        context: Context,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
        gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
        solveCalibrationWhenEnoughMeasurements: Boolean = true,
        isGyroscopeGroundTruthInitialBias: Boolean = false,
        initializationStartedListener: OnInitializationStartedListener<StaticIntervalGyroscopeCalibrator>? = null,
        initializationCompletedListener: OnInitializationCompletedListener<StaticIntervalGyroscopeCalibrator>? = null,
        errorListener: OnErrorListener<StaticIntervalGyroscopeCalibrator>? = null,
        staticIntervalDetectedListener: OnStaticIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>? = null,
        dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<StaticIntervalGyroscopeCalibrator>? = null,
        staticIntervalSkippedListener: OnStaticIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>? = null,
        dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<StaticIntervalGyroscopeCalibrator>? = null,
        generatedGyroscopeMeasurementListener: OnGeneratedGyroscopeMeasurementListener? = null,
        readyToSolveCalibrationListener: OnReadyToSolveCalibrationListener<StaticIntervalGyroscopeCalibrator>? = null,
        calibrationSolvingStartedListener: OnCalibrationSolvingStartedListener<StaticIntervalGyroscopeCalibrator>? = null,
        calibrationCompletedListener: OnCalibrationCompletedListener<StaticIntervalGyroscopeCalibrator>? = null,
        stoppedListener: OnStoppedListener<StaticIntervalGyroscopeCalibrator>? = null,
        initialGyroscopeBiasAvailableListener: OnInitialGyroscopeBiasAvailableListener? = null,
        accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null,
        gyroscopeQualityScoreMapper: QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> = DefaultGyroscopeQualityScoreMapper()
    ) : this(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        accelerometerSensorDelay,
        gyroscopeSensorDelay,
        solveCalibrationWhenEnoughMeasurements,
        initializationStartedListener,
        initializationCompletedListener,
        errorListener,
        staticIntervalDetectedListener,
        dynamicIntervalDetectedListener,
        staticIntervalSkippedListener,
        dynamicIntervalSkippedListener,
        generatedGyroscopeMeasurementListener,
        readyToSolveCalibrationListener,
        calibrationSolvingStartedListener,
        calibrationCompletedListener,
        stoppedListener,
        initialGyroscopeBiasAvailableListener,
        accuracyChangedListener,
        gyroscopeQualityScoreMapper
    ) {
        this.isGyroscopeGroundTruthInitialBias = isGyroscopeGroundTruthInitialBias
        requiredMeasurements = minimumRequiredMeasurements
        gyroscopeRobustPreliminarySubsetSize = minimumRequiredMeasurements
    }

    /**
     * Listener used by internal generator to handle events when initialization is started.
     */
    private val generatorInitializationStartedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationStartedListener<GyroscopeMeasurementGenerator> {
            initializationStartedListener?.onInitializationStarted(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when initialization is completed.
     */
    private val generatorInitializationCompletedListener =
        SingleSensorCalibrationMeasurementGenerator.OnInitializationCompletedListener<GyroscopeMeasurementGenerator> { _, _ ->
            initializationCompletedListener?.onInitializationCompleted(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when an error occurs.
     */
    private val generatorErrorListener =
        SingleSensorCalibrationMeasurementGenerator.OnErrorListener<GyroscopeMeasurementGenerator> { _, reason ->
            stop()
            errorListener?.onError(
                this@StaticIntervalGyroscopeCalibrator,
                CalibratorErrorReason.mapErrorReason(reason)
            )
        }

    /**
     * Listener used by internal generator to handle events when a static interval is detected.
     */
    private val generatorStaticIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalDetectedListener<GyroscopeMeasurementGenerator> {
            staticIntervalDetectedListener?.onStaticIntervalDetected(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is detected.
     */
    private val generatorDynamicIntervalDetectedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalDetectedListener<GyroscopeMeasurementGenerator> {
            dynamicIntervalDetectedListener?.onDynamicIntervalDetected(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a static interval is skipped.
     */
    private val generatorStaticIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnStaticIntervalSkippedListener<GyroscopeMeasurementGenerator> {
            staticIntervalSkippedListener?.onStaticIntervalSkipped(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener used by internal generator to handle events when a dynamic interval is skipped.
     */
    private val generatorDynamicIntervalSkippedListener =
        SingleSensorCalibrationMeasurementGenerator.OnDynamicIntervalSkippedListener<GyroscopeMeasurementGenerator> {
            dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(this@StaticIntervalGyroscopeCalibrator)
        }

    /**
     * Listener  used by internal generator to handle events when a new measurement is generated.
     */
    private val generatorGeneratedMeasurementListener =
        SingleSensorCalibrationMeasurementGenerator.OnGeneratedMeasurementListener<GyroscopeMeasurementGenerator, BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> { _, measurement ->
            gyroscopeMeasurements.add(measurement)

            val reqMeasurements = requiredMeasurements.coerceAtLeast(minimumRequiredMeasurements)
            val measurementsSize = gyroscopeMeasurements.size

            generatedGyroscopeMeasurementListener?.onGeneratedGyroscopeMeasurement(
                this@StaticIntervalGyroscopeCalibrator,
                measurement,
                measurementsSize,
                reqMeasurements
            )

            // check if enough measurements have been collected
            val isReadyToCalibrate = measurementsSize >= reqMeasurements
            if (isReadyToCalibrate) {
                readyToSolveCalibrationListener?.onReadyToSolveCalibration(
                    this@StaticIntervalGyroscopeCalibrator
                )

                // stop internal generator since no more measurements need to be collected
                internalStop(true)

                // build calibrator
                gyroscopeInternalCalibrator = buildGyroscopeInternalCalibrator()

                if (solveCalibrationWhenEnoughMeasurements) {
                    // execute calibration
                    internalCalibrate()
                }
            }
        }

    /**
     * Listener for accelerometer sensor collector.
     * This is used to determine accelerometer calibration and obtain an initial guess
     * for accelerometer bias (only available if
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED] is used, otherwise zero
     * bias is assumed as an initial guess).
     */
    private val generatorAccelerometerMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { _, _, _, bx, by, bz, _, _ ->
            if (isFirstAccelerometerMeasurement) {
                updateAccelerometerBiases(bx, by, bz)
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
     * Internal generator to generate measurements for calibration.
     */
    override val generator = GyroscopeMeasurementGenerator(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        gyroscopeSensorType,
        gyroscopeSensorDelay,
        generatorInitializationStartedListener,
        generatorInitializationCompletedListener,
        generatorErrorListener,
        generatorStaticIntervalDetectedListener,
        generatorDynamicIntervalDetectedListener,
        generatorStaticIntervalSkippedListener,
        generatorDynamicIntervalSkippedListener,
        generatorGeneratedMeasurementListener,
        accelerometerMeasurementListener = generatorAccelerometerMeasurementListener,
        gyroscopeMeasurementListener = generatorGyroscopeMeasurementListener,
        accuracyChangedListener = accuracyChangedListener
    )

    /**
     * Internal calibrator used to solve the calibration parameters once enough measurements are
     * collected during dynamic intervals.
     */
    private var gyroscopeInternalCalibrator: GyroscopeNonLinearCalibrator? = null

    /**
     * Gets or sets x-coordinate of accelerometer bias used as an initial guess and expressed in
     * meters per squared second (m/s^2).
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasX: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets y-coordinate of accelerometer bias used as an initial guess and expressed in
     * meters per squared second (m/s^2).
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasY: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets z-coordinate of accelerometer bias used as an initial guess and expressed in
     * meters per squared second (m/s^2).
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasZ: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets x-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasXAsMeasurement: Acceleration
        get() = Acceleration(
            accelerometerInitialBiasX,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            accelerometerInitialBiasX = AccelerationConverter.convert(
                value.value,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            ).toDouble()
        }

    /**
     * Gets x-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasX] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasX] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @param result instance where result will be stored.
     */
    fun getAccelerometerInitialBiasXAsMeasurement(result: Acceleration) {
        result.value = accelerometerInitialBiasX
        result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
    }

    /**
     * Gets or sets y-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasYAsMeasurement: Acceleration
        get() = Acceleration(
            accelerometerInitialBiasY,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            accelerometerInitialBiasY = AccelerationConverter.convert(
                value.value,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            ).toDouble()
        }

    /**
     * Gets y-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasY] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasY] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @param result instance where result will be stored.
     */
    fun getAccelerometerInitialBiasYAsMeasurement(result: Acceleration) {
        result.value = accelerometerInitialBiasY
        result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
    }

    /**
     * Gets or sets z-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasZAsMeasurement: Acceleration
        get() = Acceleration(
            accelerometerInitialBiasZ,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            accelerometerInitialBiasZ = AccelerationConverter.convert(
                value.value,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            ).toDouble()
        }

    /**
     * Gets z-coordinate of accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasZ] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasZ] is obtained from hardware calibration once the accelerometer
     * starts.
     *
     * @param result instance where result will be stored.
     */
    fun getAccelerometerInitialBiasZAsMeasurement(result: Acceleration) {
        result.value = accelerometerInitialBiasZ
        result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
    }

    /**
     * Gets or sets accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasAsTriad] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasAsTriad] is obtained from hardware calibration once the
     * accelerometer starts.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var accelerometerInitialBiasAsTriad: AccelerationTriad
        get() = AccelerationTriad(
            AccelerationUnit.METERS_PER_SQUARED_SECOND,
            accelerometerInitialBiasX,
            accelerometerInitialBiasY,
            accelerometerInitialBiasZ
        )
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            accelerometerInitialBiasX = AccelerationConverter.convert(
                value.valueX,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            accelerometerInitialBiasY = AccelerationConverter.convert(
                value.valueY,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            accelerometerInitialBiasZ = AccelerationConverter.convert(
                value.valueZ,
                value.unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }

    /**
     * Gets accelerometer bias used as an initial guess.
     * This value can be automatically determined once the calibrator starts.
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     * If [isAccelerometerGroundTruthInitialBias] is true, provided value is assumed to be the true
     * bias, and [estimatedAccelerometerBiasAsTriad] will be equal to this value, otherwise
     * [estimatedAccelerometerBiasAsTriad] is obtained from hardware calibration once the
     * accelerometer starts.
     *
     * @param result instance where result will be stored.
     */
    fun getAccelerometerInitialBiasAsTriad(result: AccelerationTriad) {
        result.setValueCoordinatesAndUnit(
            accelerometerInitialBiasX,
            accelerometerInitialBiasY,
            accelerometerInitialBiasZ,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
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
     * Indicates or specifies whether z-axis is assumed to be common for gyroscope. When enabled,
     * this eliminates 3 variables from Mg matrix during gyroscope calibration.
     *
     * @throws IllegalStateException if calibrator is currently running.
     */
    var isGyroscopeCommonAxisUsed: Boolean = DEFAULT_USE_COMMON_Z_AXIS
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
    var isGDependentCrossBiasesEstimated: Boolean = DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
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
                    GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES
                } else {
                    GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS
                }
            } else {
                if (isGDependentCrossBiasesEstimated) {
                    GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES
                } else {
                    GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL
                }
            }
        } else {
            // Unknown bias
            if (isGyroscopeCommonAxisUsed) {
                if (isGDependentCrossBiasesEstimated) {
                    GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES
                } else {
                    GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS
                }
            } else {
                if (isGDependentCrossBiasesEstimated) {
                    GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES
                } else {
                    GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL
                }
            }
        }

    /**
     * Gets minimum number of required measurements to start calibration.
     * Each time that the device is moved, a new measurement is collected.
     * When the required number of measurements for all sensors is collected, calibration can start.
     */
    override val minimumRequiredMeasurements: Int
        get() = minimumRequiredGyroscopeMeasurements

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
    var gyroscopeRobustConfidence: Double = ROBUST_DEFAULT_CONFIDENCE
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
    var gyroscopeRobustMaxIterations: Int = ROBUST_DEFAULT_MAX_ITERATIONS
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
    var gyroscopeRobustThresholdFactor: Double = DEFAULT_ROBUST_THRESHOLD_FACTOR
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
    var gyroscopeRobustStopThresholdFactor: Double = DEFAULT_ROBUST_STOP_THRESHOLD_FACTOR
        @Throws(IllegalArgumentException::class, IllegalStateException::class)
        set(value) {
            require(value > 0.0)
            check(!running)
            field = value
        }

    /**
     * Gets x-coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    var estimatedAccelerometerBiasX: Double? = null
        private set

    /**
     * Gets y-coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    var estimatedAccelerometerBiasY: Double? = null
        private set

    /**
     * Gets z-coordinate of estimated accelerometer bias expressed in meters per squared second
     * (m/s^2).
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    var estimatedAccelerometerBiasZ: Double? = null
        private set

    /**
     * Gets x-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    val estimatedAccelerometerBiasXAsMeasurement: Acceleration?
        get() {
            val biasX = estimatedAccelerometerBiasX
            return if (biasX != null) {
                Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND)
            } else {
                null
            }
        }

    /**
     * Gets x-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasX] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     *
     * @param result instance where result will be stored.
     * @return true if estimated bias is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasXAsMeasurement(result: Acceleration): Boolean {
        val biasX = estimatedAccelerometerBiasX
        return if (biasX != null) {
            result.value = biasX
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets y-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    val estimatedAccelerometerBiasYAsMeasurement: Acceleration?
        get() {
            val biasY = estimatedAccelerometerBiasY
            return if (biasY != null) {
                Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND)
            } else {
                null
            }
        }

    /**
     * Gets y-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasY] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     *
     * @param result instance where result will be stored.
     * @return true if estimated bias is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasYAsMeasurement(result: Acceleration): Boolean {
        val biasY = estimatedAccelerometerBiasY
        return if (biasY != null) {
            result.value = biasY
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets z-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    val estimatedAccelerometerBiasZAsMeasurement: Acceleration?
        get() {
            val biasZ = estimatedAccelerometerBiasZ
            return if (biasZ != null) {
                Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND)
            } else {
                null
            }
        }

    /**
     * Gets z-coordinate of estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasZ] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     *
     * @param result instance where result will be stored.
     * @return true if estimated bias is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasZAsMeasurement(result: Acceleration): Boolean {
        val biasZ = estimatedAccelerometerBiasZ
        return if (biasZ != null) {
            result.value = biasZ
            result.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasAsTriad] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    val estimatedAccelerometerBiasAsTriad: AccelerationTriad?
        get() {
            val biasX = estimatedAccelerometerBiasX
            val biasY = estimatedAccelerometerBiasY
            val biasZ = estimatedAccelerometerBiasZ
            return if (biasX != null && biasY != null && biasZ != null) {
                AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX, biasY, biasZ)
            } else {
                null
            }
        }

    /**
     * Gets estimated accelerometer bias.
     * If [isAccelerometerGroundTruthInitialBias] is true, this will be equal to
     * [accelerometerInitialBiasAsTriad] once the accelerometer has started, otherwise depending on
     * [accelerometerSensorType] this will be one of the following:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     *
     * @param result instance where result will be stored.
     * @return true if estimated bias is available, false otherwise.
     */
    fun getEstimatedAccelerometerBiasAsTriad(result: AccelerationTriad): Boolean {
        val biasX = estimatedAccelerometerBiasX
        val biasY = estimatedAccelerometerBiasY
        val biasZ = estimatedAccelerometerBiasZ
        return if (biasX != null && biasY != null && biasZ != null) {
            result.setValueCoordinatesAndUnit(
                biasX,
                biasY,
                biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
            true
        } else {
            false
        }
    }

    /**
     * Indicates whether [estimatedAccelerometerBiasAsTriad] is equal to
     * [accelerometerInitialBiasAsTriad], or if it is initialized once accelerometer sensor
     * starts.
     * If this is true, [estimatedAccelerometerBiasAsTriad] will be equal to
     * [accelerometerInitialBiasAsTriad]
     * If this is false, [estimatedAccelerometerBiasAsTriad] will be set once the accelerometer is
     * started and depending on [accelerometerSensorType], it will be set as:
     * If [accelerometerSensorType] is
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED], bias will be equal to
     * the value used internally by the device as part of the accelerometer hardware calibration.
     * If [accelerometerSensorType] is [AccelerometerSensorCollector.SensorType.ACCELEROMETER], then
     * accelerometer sensor measurements are assumed to be already bias compensated, and the initial
     * bias is assumed to be zero.
     */
    var isAccelerometerGroundTruthInitialBias: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer scaling factors and cross coupling errors matrix.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMa: Matrix
        get() {
            val result = Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS)
            getAccelerometerMa(result)
            return result
        }
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            require(value.rows == BodyKinematics.COMPONENTS && value.columns == BodyKinematics.COMPONENTS)

            accelerometerSx = value.getElementAtIndex(0)
            accelerometerMyx = value.getElementAtIndex(1)
            accelerometerMzx = value.getElementAtIndex(2)

            accelerometerMxy = value.getElementAtIndex(3)
            accelerometerSy = value.getElementAtIndex(4)
            accelerometerMzy = value.getElementAtIndex(5)

            accelerometerMxz = value.getElementAtIndex(6)
            accelerometerMyz = value.getElementAtIndex(7)
            accelerometerSz = value.getElementAtIndex(8)
        }

    /**
     * Gets accelerometer scaling factors and cross coupling errors matrix.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Throws(IllegalArgumentException::class)
    fun getAccelerometerMa(result: Matrix) {
        require(result.rows == BodyKinematics.COMPONENTS && result.columns == BodyKinematics.COMPONENTS)

        result.setElementAtIndex(0, accelerometerSx)
        result.setElementAtIndex(1, accelerometerMyx)
        result.setElementAtIndex(2, accelerometerMzx)

        result.setElementAtIndex(3, accelerometerMxy)
        result.setElementAtIndex(4, accelerometerSy)
        result.setElementAtIndex(5, accelerometerMzy)

        result.setElementAtIndex(6, accelerometerMxz)
        result.setElementAtIndex(7, accelerometerMyz)
        result.setElementAtIndex(8, accelerometerSz)
    }

    /**
     * Gets or sets accelerometer initial x scaling factor.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerSx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer initial y scaling factor.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerSy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer initial z scaling factor.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerSz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer x-y cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMxy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer x-z cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMxz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer y-x cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMyx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer y-z cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMyz: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer z-x cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMzx: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets accelerometer z-y cross coupling error.
     * This is only taken into account if G-dependent cross biases are used when
     * [isGDependentCrossBiasesEstimated] is true.
     *
     * @throws IllegalStateException ir calibrator is already running.
     */
    var accelerometerMzy: Double = 0.0
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Sets accelerometer scaling factors.
     *
     * @param accelerometerSx x scaling factor.
     * @param accelerometerSy y scaling factor.
     * @param accelerometerSz z scaling factor.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setAccelerometerScalingFactors(
        accelerometerSx: Double,
        accelerometerSy: Double,
        accelerometerSz: Double
    ) {
        check(!running)
        this.accelerometerSx = accelerometerSx
        this.accelerometerSy = accelerometerSy
        this.accelerometerSz = accelerometerSz
    }

    /**
     * Sets cross coupling errors for accelerometer calibration.
     *
     * @param accelerometerMxy x-y cross coupling error.
     * @param accelerometerMxz x-z cross coupling error.
     * @param accelerometerMyx y-x cross coupling error.
     * @param accelerometerMyz y-z cross coupling error.
     * @param accelerometerMzx z-x cross coupling error.
     * @param accelerometerMzy z-y cross coupling error.
     * @throws IllegalStateException if calibrator is currently running.
     */
    @Throws(IllegalStateException::class)
    fun setAccelerometerCrossCouplingErrors(
        accelerometerMxy: Double,
        accelerometerMxz: Double,
        accelerometerMyx: Double,
        accelerometerMyz: Double,
        accelerometerMzx: Double,
        accelerometerMzy: Double
    ) {
        check(!running)
        this.accelerometerMxy = accelerometerMxy
        this.accelerometerMxz = accelerometerMxz
        this.accelerometerMyx = accelerometerMyx
        this.accelerometerMyz = accelerometerMyz
        this.accelerometerMzx = accelerometerMzx
        this.accelerometerMzy = accelerometerMzy
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
     * Number of gyroscope measurements that have been processed.
     */
    val numberOfProcessedGyroscopeMeasurements
        get() = generator.numberOfProcessedGyroscopeMeasurements

    /**
     * List of gyroscope measurements that have been collected so far to be used for
     * gyroscope calibration.
     * Items in return list can be modified if needed, but beware that this might
     * have consequences on solved calibration result.
     */
    val gyroscopeMeasurements =
        mutableListOf<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>()

    /**
     * Indicates whether enough measurements have been picked at static intervals so that the
     * calibration process can be solved.
     */
    override val isReadyToSolveCalibration
        get() = gyroscopeMeasurements.size >= requiredMeasurements.coerceAtLeast(
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
            gyroscopeInternalCalibrator?.calibrate()
            calibrationCompletedListener?.onCalibrationCompleted(this)
            running = false
            true
        } catch (e: NavigationException) {
            Log.e(
                StaticIntervalGyroscopeCalibrator::class.qualifiedName,
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
        gyroscopeMeasurements.clear()

        gyroscopeInitialBiasX = null
        gyroscopeInitialBiasY = null
        gyroscopeInitialBiasZ = null

        gyroscopeInternalCalibrator = null
    }

    /**
     * Updates accelerometer biases values when first accelerometer measurement is received,
     * so that hardware calibrated biases are retrieved if
     * [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED] is used and
     * [isAccelerometerGroundTruthInitialBias] is false.
     *
     * @param bx x-coordinate of accelerometer bias expressed in meters per squared second (m/s^2).
     * @param by y-coordinate of accelerometer bias expressed in meters per squared second (m/s^2).
     * @param bz z-coordinate of accelerometer bias expressed in meters per squared second (m/s^2).
     */
    private fun updateAccelerometerBiases(bx: Float?, by: Float?, bz: Float?) {
        if (isAccelerometerGroundTruthInitialBias) {
            estimatedAccelerometerBiasX = accelerometerInitialBiasX
            estimatedAccelerometerBiasY = accelerometerInitialBiasY
            estimatedAccelerometerBiasZ = accelerometerInitialBiasZ
        } else {
            val biasX: Double
            val biasY: Double
            val biasZ: Double
            if (bx != null && by != null && bz != null) {
                biasX = bx.toDouble()
                biasY = by.toDouble()
                biasZ = bz.toDouble()
            } else {
                biasX = 0.0
                biasY = 0.0
                biasZ = 0.0
            }

            estimatedAccelerometerBiasX = biasX
            estimatedAccelerometerBiasY = biasY
            estimatedAccelerometerBiasZ = biasZ
        }
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
     * Indicates whether the generator has picked the first gyroscope measurement.
     */
    private val isFirstGyroscopeMeasurement: Boolean
        get() = generator.numberOfProcessedGyroscopeMeasurements <= FIRST_MEASUREMENT

    /**
     * Builds an internal gyroscope calibrator based on all provided parameters.
     *
     * @return an internal gyroscope calibrator.
     * @throws IllegalStateException if no suitable calibrator can be built.
     */
    @Throws(IllegalStateException::class)
    private fun buildGyroscopeInternalCalibrator(): GyroscopeNonLinearCalibrator {
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
         * By default this is 3.0 times [gyroscopeBaseNoiseLevel], which considering the noise
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
         * Indicates whether G-dependent cross biases are being estimated by default.
         */
        const val DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES = false

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope when G-dependent cross biases are being estimated.
         */
        private const val GYROSCOPE_UNKNOWN_BIAS_COMMON_Z_AXIS_AND_CROSS_BIASES_UNKNOWNS = 18

        /**
         * Number of unknowns for the general case when G-dependant cross biases are being
         * estimated.
         */
        private const val GYROSCOPE_UNKNOWN_BIAS_GENERAL_AND_CROSS_BIASES_UNKNOWNS = 21

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope when G-dependent cross biases are not being estimated.
         */
        private const val GYROSCOPE_UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 9

        /**
         * Number of unknowns for the general case when G-dependent cross biases are not being
         * estimated.
         */
        private const val GYROSCOPE_UNKNOWN_BIAS_GENERAL_UNKNOWNS = 12

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and
         * gyroscope when G-dependent cross biases are being estimated.
         */
        private const val GYROSCOPE_KNOWN_BIAS_COMMON_Z_AXIS_AND_CROSS_BIASES_UNKNOWNS = 15

        /**
         * Number of unknowns for the general case when G-dependent cross biases are being
         * estimated.
         */
        private const val GYROSCOPE_KNOWN_BIAS_GENERAL_AND_CROSS_BIASES_UNKNOWNS = 18

        /**
         * Number of unknowns when common z-axis is assumed for both the accelerometer and gyroscope
         * when G-dependent cross biases are not being estimated.
         */
        private const val GYROSCOPE_KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS = 6

        /**
         * Number of unknowns for the general calibration case when bias is known.
         */
        private const val GYROSCOPE_KNOWN_BIAS_GENERAL_UNKNOWNS = 9

        /**
         * Required minimum number of sequences when common z-axis is assumed and G-dependent cross
         * biases are being estimated.
         */
        const val GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES =
            GYROSCOPE_KNOWN_BIAS_COMMON_Z_AXIS_AND_CROSS_BIASES_UNKNOWNS + 1

        /**
         * Required minimum number of sequences for the general case and G-dependent cross biases
         * are being estimated.
         */
        const val GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES =
            GYROSCOPE_KNOWN_BIAS_GENERAL_AND_CROSS_BIASES_UNKNOWNS + 1

        /**
         * Required minimum number of sequences when common z-axis is assumed and G-dependent cross
         * biases are being ignored.
         */
        const val GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS =
            GYROSCOPE_KNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Requires minimum number of sequences for the general case and G-dependent cross biases
         * are being ignored.
         */
        const val GYROSCOPE_KNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL =
            GYROSCOPE_KNOWN_BIAS_GENERAL_UNKNOWNS + 1

        /**
         * Required minimum number of sequences when common z-axis is assumed and G-dependent cross
         * biases are being estimated.
         */
        const val GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES =
            GYROSCOPE_UNKNOWN_BIAS_COMMON_Z_AXIS_AND_CROSS_BIASES_UNKNOWNS + 1

        /**
         * Required minimum number of sequences for the general case and G-dependent cross biases
         * are being estimated.
         */
        const val GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES =
            GYROSCOPE_UNKNOWN_BIAS_GENERAL_AND_CROSS_BIASES_UNKNOWNS + 1

        /**
         * Required minimum number of sequences when common z-axis is assumed and G-dependent cross
         * biases are being ignored.
         */
        const val GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_COMMON_Z_AXIS =
            GYROSCOPE_UNKNOWN_BIAS_COMMON_Z_AXIS_UNKNOWNS + 1

        /**
         * Required minimum number of sequences for the general case and G-dependent cross biases
         * are being ignored.
         */
        const val GYROSCOPE_UNKNOWN_BIAS_MINIMUM_SEQUENCES_GENERAL =
            GYROSCOPE_UNKNOWN_BIAS_GENERAL_UNKNOWNS + 1

        /**
         * Indicates when first sensor measurement is obtained.
         */
        private const val FIRST_MEASUREMENT = 1
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
            calibrator: StaticIntervalGyroscopeCalibrator,
            measurement: BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>,
            measurementsFoundSoFar: Int,
            requiredMeasurements: Int
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
            calibrator: StaticIntervalGyroscopeCalibrator,
            biasX: Double,
            biasY: Double,
            biasZ: Double
        )
    }
}