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
package com.irurueta.android.navigation.inertial.calibration.intervals.measurements

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit

/**
 * Generates measurements that can later be used by magnetometer calibrators.
 * Measurements are generated by taking into account static and dynamic intervals on the device,
 * when the device is kept static (e.g. motionless), or when same force is applied to the device
 * changing its position or orientation.
 * Measurement generator converts device ENU measurements into measurements expressed in local
 * tangent plane NED coordinates.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property magnetometerSensorType One of the supported gyroscope sensor types.
 * @property magnetometerSensorDelay Delay of magnetometer sensor between samples.
 * @property initializationStartedListener listener to notify when initialization starts.
 * @property initializationCompletedListener listener to notify when initialization successfully
 * completes.
 * @property errorListener listener to notify errors.
 * @property staticIntervalDetectedListener listener to notify when a static interval is detected.
 * @property dynamicIntervalDetectedListener listener to notify when a dynamic interval is detected.
 * @property staticIntervalSkippedListener listener to notify when a static interval is skipped if
 * its duration is too short.
 * @property dynamicIntervalSkippedListener listener to notify when a dynamic interval is skipped if
 * its duration is too long.
 * @property generatedMeasurementListener listener to notify when a new calibration measurement is
 * generated.
 * @property resetListener listener to notify when generator is restarted.
 * @property accelerometerMeasurementListener listener to notify when a new accelerometer
 * measurement is received.
 * @property magnetometerMeasurementListener listener to notify when a new magnetometer measurement
 * is received.
 * @property accuracyChangedListener listener to notify when sensor accuracy changes.
 */
class MagnetometerMeasurementGenerator(
    context: Context,
    accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val magnetometerSensorType: MagnetometerSensorType =
        MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    initializationStartedListener: OnInitializationStartedListener<MagnetometerMeasurementGenerator>? = null,
    initializationCompletedListener: OnInitializationCompletedListener<MagnetometerMeasurementGenerator>? = null,
    errorListener: OnErrorListener<MagnetometerMeasurementGenerator>? = null,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<MagnetometerMeasurementGenerator>? = null,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<MagnetometerMeasurementGenerator>? = null,
    staticIntervalSkippedListener: OnStaticIntervalSkippedListener<MagnetometerMeasurementGenerator>? = null,
    dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<MagnetometerMeasurementGenerator>? = null,
    generatedMeasurementListener: OnGeneratedMeasurementListener<MagnetometerMeasurementGenerator, StandardDeviationBodyMagneticFluxDensity>? = null,
    resetListener: OnResetListener<MagnetometerMeasurementGenerator>? = null,
    accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : SingleSensorCalibrationMeasurementGenerator<MagnetometerMeasurementGenerator,
        StandardDeviationBodyMagneticFluxDensity, MagnetometerMeasurementsGenerator,
        MagnetometerMeasurementsGeneratorListener, BodyKinematicsAndMagneticFluxDensity>(
    context,
    accelerometerSensorType,
    accelerometerSensorDelay,
    initializationStartedListener,
    initializationCompletedListener,
    errorListener,
    staticIntervalDetectedListener,
    dynamicIntervalDetectedListener,
    staticIntervalSkippedListener,
    dynamicIntervalSkippedListener,
    generatedMeasurementListener,
    resetListener,
    accelerometerMeasurementListener,
    accuracyChangedListener
) {
    /**
     * Triad containing acceleration samples converted from device ENU coordinates to local plane
     * NED coordinates.
     * This is reused for performance reasons.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Triad containing magnetic flux density samples converted from device ENU coordinates to local
     * plane NED coordinates.
     * This is reused for performance reasons.
     */
    private val b = MagneticFluxDensityTriad()

    /**
     * Listener for internal measurement generator.
     */
    override val measurementsGeneratorListener =
        object : MagnetometerMeasurementsGeneratorListener {
            override fun onInitializationStarted(generator: MagnetometerMeasurementsGenerator?) {
                initializationStartedListener?.onInitializationStarted(
                    this@MagnetometerMeasurementGenerator
                )
            }

            override fun onInitializationCompleted(
                generator: MagnetometerMeasurementsGenerator?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@MagnetometerMeasurementGenerator,
                    baseNoiseLevel
                )
            }

            override fun onError(
                generator: MagnetometerMeasurementsGenerator?,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                errorListener?.onError(
                    this@MagnetometerMeasurementGenerator,
                    mapErrorReason(reason)
                )
            }

            override fun onStaticIntervalDetected(generator: MagnetometerMeasurementsGenerator?) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@MagnetometerMeasurementGenerator
                )
            }

            override fun onDynamicIntervalDetected(generator: MagnetometerMeasurementsGenerator?) {
                dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@MagnetometerMeasurementGenerator
                )
            }

            override fun onStaticIntervalSkipped(generator: MagnetometerMeasurementsGenerator?) {
                staticIntervalSkippedListener?.onStaticIntervalSkipped(
                    this@MagnetometerMeasurementGenerator
                )
            }

            override fun onDynamicIntervalSkipped(generator: MagnetometerMeasurementsGenerator?) {
                dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(
                    this@MagnetometerMeasurementGenerator
                )
            }

            override fun onGeneratedMeasurement(
                generator: MagnetometerMeasurementsGenerator?,
                measurement: StandardDeviationBodyMagneticFluxDensity
            ) {
                generatedMeasurementListener?.onGeneratedMeasurement(
                    this@MagnetometerMeasurementGenerator,
                    measurement
                )
            }

            override fun onReset(generator: MagnetometerMeasurementsGenerator?) {
                resetListener?.onReset(this@MagnetometerMeasurementGenerator)
            }
        }

    /**
     * Internal measurements generator for magnetometer calibration.
     */
    override val measurementsGenerator =
        MagnetometerMeasurementsGenerator(measurementsGeneratorListener)

    /**
     * Processes an accelerometer measurement to generate an instance of type [BodyKinematics] to be
     * used by the internal measurement generator.
     * Since [MagnetometerMeasurementsGenerator] only requires acceleration information, no data is
     * provided for angular rates in processed result.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param diffSeconds elapsed seconds since accelerometer started.
     * @param result instance where processed sample result will be stored.
     */
    override fun processSample(
        ax: Float,
        ay: Float,
        az: Float,
        diffSeconds: Double,
        result: BodyKinematicsAndMagneticFluxDensity
    ) {
        // convert from devic ENU coordinates to local plane NED coordinates
        ENUtoNEDConverter.convert(ax.toDouble(), ay.toDouble(), az.toDouble(), acceleration)

        // set accelerometer information
        kinematics.fx = acceleration.valueX
        kinematics.fy = acceleration.valueY
        kinematics.fz = acceleration.valueZ
        result.kinematics = kinematics

        // set magnetometer information
        result.magneticFluxDensity = magneticFluxDensity
    }

    /**
     * Body kinematics being reused for efficiency purposes.
     * For magnetometer calibration purposes, only specific force is required.
     */
    private val kinematics = BodyKinematics()

    /**
     * Body magnetic flux density being reused for efficiency purposes.
     */
    private val magneticFluxDensity = BodyMagneticFluxDensity()

    /**
     * Estimates accumulated average and noise of magnetometer values during static periods.
     */
    private val magnetometerAccumulatedNoiseEstimator =
        AccumulatedMagneticFluxDensityTriadNoiseEstimator()

    /**
     * Sample used by internal measurement generator.
     */
    override val sample = BodyKinematicsAndMagneticFluxDensity()

    /**
     * Internal listener for magnetometer sensor collector.
     * Handles measurements collected by the magnetometer sensor so that they are processed
     * by the internal measurement generator.
     */
    private val magnetometerCollectorMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
            val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
            val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())

            // convert from device ENU coordinates to local plane NED coordinates
            ENUtoNEDConverter.convert(bxTesla, byTesla, bzTesla, b)

            magneticFluxDensity.bx = b.valueX
            magneticFluxDensity.by = b.valueY
            magneticFluxDensity.bz = b.valueZ

            if (status == Status.INITIALIZING) {
                magnetometerAccumulatedNoiseEstimator.addTriad(bxTesla, byTesla, bzTesla)
            }

            numberOfProcessedMagnetometerMeasurements++

            if (magnetometerBaseNoiseLevel == null && (status == Status.INITIALIZATION_COMPLETED || status == Status.STATIC_INTERVAL || status == Status.DYNAMIC_INTERVAL)) {
                magnetometerBaseNoiseLevel =
                    magnetometerAccumulatedNoiseEstimator.standardDeviationNorm
                initialMagneticFluxDensityNorm = magnetometerAccumulatedNoiseEstimator.avgNorm
            }

            magnetometerMeasurementListener?.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )
        }

    /**
     * Magnetometer sensor collector.
     * Collects magnetometer measurements.
     */
    private val magnetometerCollector = MagnetometerSensorCollector(
        context,
        magnetometerSensorType,
        magnetometerSensorDelay,
        magnetometerCollectorMeasurementListener,
        collectorAccuracyChangedListener
    )

    /**
     * Gets magnetometer sensor being used to obtain measurements, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor
        get() = magnetometerCollector.sensor

    /**
     * Number of magnetometer measurements that have been processed.
     */
    var numberOfProcessedMagnetometerMeasurements: Int = 0
        private set

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization
     * expressed in Teslas (T).
     * This is only available once generator completes initialization.
     */
    var magnetometerBaseNoiseLevel: Double? = null
        private set

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     */
    val magnetometerBaseNoiseLevelAsMeasurement: MagneticFluxDensity?
        get() {
            val value = magnetometerBaseNoiseLevel ?: return null
            return MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets magnetometer measurement base noise level that has been detected during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getMagnetometerBaseNoiseLevelAsMeasurement(result: MagneticFluxDensity): Boolean {
        val value = magnetometerBaseNoiseLevel ?: return false
        result.value = value
        result.unit = MagneticFluxDensityUnit.TESLA
        return true
    }

    /**
     * Norm of average magnetic flux density obtained during initialization and expressed in
     * Teslas (T).
     * This is only available once generator completes initialization.
     */
    var initialMagneticFluxDensityNorm: Double? = null
        private set

    /**
     * Gets norm of average magnetic flux density obtained during initialization.
     * This is only available once generator completes initialization.
     */
    val initialMagneticFluxDensityNormAsMeasurement: MagneticFluxDensity?
        get() {
            val value = initialMagneticFluxDensityNorm ?: return null
            return MagneticFluxDensity(value, MagneticFluxDensityUnit.TESLA)
        }

    /**
     * Gets norm of average magnetic flux density obtained during initialization.
     * This is only available once generator completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getInitialMagneticFluxDensityNormAsMeasurement(result: MagneticFluxDensity): Boolean {
        val value = initialMagneticFluxDensityNorm ?: return false
        result.value = value
        result.unit = MagneticFluxDensityUnit.TESLA
        return true
    }

    /**
     * Starts collection of sensor measurements.
     *
     * @throws IllegalStateException if detector is already running or sensor is not available.
     */
    @Throws(IllegalStateException::class)
    override fun start() {
        super.start()

        reset()

        if (!magnetometerCollector.start()) {
            stop()
            throw IllegalStateException("Unavailable magnetometer sensor")
        }
    }

    /**
     * Stops collection of sensor measurements.
     */
    override fun stop() {
        magnetometerCollector.stop()
        super.stop()
    }

    /**
     * Resets generator to its initial state.
     */
    private fun reset() {
        numberOfProcessedMagnetometerMeasurements = 0
        magnetometerBaseNoiseLevel = null
        initialMagneticFluxDensityNorm = null
        magnetometerAccumulatedNoiseEstimator.reset()
    }
}