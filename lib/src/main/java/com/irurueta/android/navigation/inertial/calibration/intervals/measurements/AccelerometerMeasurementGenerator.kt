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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGenerator
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGeneratorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector

/**
 * Generates measurements that can later be used by accelerometer calibrators.
 * Measurements are generated by taking into account static and dynamic intervals on the device,
 * when the device is kept static (e.g. motionless), or when some force is applied to the device
 * changing its position or orientation.
 */
class AccelerometerMeasurementGenerator(
    context: Context,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    initializationStartedListener: OnInitializationStartedListener<AccelerometerMeasurementGenerator>? = null,
    initializationCompletedListener: OnInitializationCompletedListener<AccelerometerMeasurementGenerator>? = null,
    errorListener: OnErrorListener<AccelerometerMeasurementGenerator>? = null,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<AccelerometerMeasurementGenerator>? = null,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<AccelerometerMeasurementGenerator>? = null,
    staticIntervalSkippedListener: OnStaticIntervalSkippedListener<AccelerometerMeasurementGenerator>? = null,
    dynamicIntervalSkippedListener: OnDynamicIntervalSkippedListener<AccelerometerMeasurementGenerator>? = null,
    generatedMeasurementListener: OnGeneratedMeasurementListener<AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics>? = null,
    resetListener: OnResetListener<AccelerometerMeasurementGenerator>? = null,
    accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : CalibrationMeasurementGenerator<
        AccelerometerMeasurementGenerator, StandardDeviationBodyKinematics,
        AccelerometerMeasurementsGenerator, AccelerometerMeasurementsGeneratorListener,
        BodyKinematics>(
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
     * Listener for internal measurement generator.
     */
    override val measurementsGeneratorListener =
        object : AccelerometerMeasurementsGeneratorListener {
            override fun onInitializationStarted(generator: AccelerometerMeasurementsGenerator?) {
                initializationStartedListener?.onInitializationStarted(
                    this@AccelerometerMeasurementGenerator
                )
            }

            override fun onInitializationCompleted(
                generator: AccelerometerMeasurementsGenerator?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@AccelerometerMeasurementGenerator,
                    baseNoiseLevel
                )
            }

            override fun onError(
                generator: AccelerometerMeasurementsGenerator?,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                errorListener?.onError(
                    this@AccelerometerMeasurementGenerator,
                    mapErrorReason(reason)
                )
            }

            override fun onStaticIntervalDetected(generator: AccelerometerMeasurementsGenerator?) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@AccelerometerMeasurementGenerator
                )
            }

            override fun onDynamicIntervalDetected(generator: AccelerometerMeasurementsGenerator?) {
                dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@AccelerometerMeasurementGenerator
                )
            }

            override fun onStaticIntervalSkipped(generator: AccelerometerMeasurementsGenerator?) {
                staticIntervalSkippedListener?.onStaticIntervalSkipped(
                    this@AccelerometerMeasurementGenerator
                )
            }

            override fun onDynamicIntervalSkipped(generator: AccelerometerMeasurementsGenerator?) {
                dynamicIntervalSkippedListener?.onDynamicIntervalSkipped(
                    this@AccelerometerMeasurementGenerator
                )
            }

            override fun onGeneratedMeasurement(
                generator: AccelerometerMeasurementsGenerator?,
                measurement: StandardDeviationBodyKinematics
            ) {
                generatedMeasurementListener?.onGeneratedMeasurement(
                    this@AccelerometerMeasurementGenerator,
                    measurement
                )
            }

            override fun onReset(generator: AccelerometerMeasurementsGenerator?) {
                resetListener?.onReset(this@AccelerometerMeasurementGenerator)
            }
        }

    /**
     * Internal measurements generator for accelerometer calibration.
     */
    override val measurementsGenerator =
        AccelerometerMeasurementsGenerator(measurementsGeneratorListener)

    /**
     * Processes an accelerometer measurement to generate an instance of type [BodyKinematics] to be
     * used by the internal measurement generator.
     * Since [AccelerometerMeasurementsGenerator] only requires acceleration information, no data is
     * provided for angular rates in processed result.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param bx bias on device x-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param by bias on device y-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param bz bias on device z-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param diffSeconds elapsed seconds since accelerometer started.
     * @param result instance where processed sample result will be stored.
     */
    override fun processSample(
        ax: Float,
        ay: Float,
        az: Float,
        bx: Float?,
        by: Float?,
        bz: Float?,
        diffSeconds: Double,
        result: BodyKinematics
    ) {
        // set accelerometer information
        result.fx = ax.toDouble()
        result.fy = ay.toDouble()
        result.fz = az.toDouble()
    }

    /**
     * Body kinematics used by internal measurement generator.
     * For accelerometer calibration purposes, only specific force is required.
     */
    override val sample = BodyKinematics()
}