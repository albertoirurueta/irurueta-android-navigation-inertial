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
package com.irurueta.android.navigation.inertial.old.calibration.intervals

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.old.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.MagneticFluxDensityTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.TimeConverter

/**
 * Detects static or motion intervals of time by using the magnetometer of the device to determine
 * whether the device is moving or not.
 * When detector is started, initialization occurs to determine the magnetometer noise level while
 * keeping device static. Once the detector is initialized, then static or dynamic intervals can be
 * detected.
 * This detector uses accumulated average values during static intervals, and windowed
 * averages as "instantaneous" values during dynamic intervals.
 * Length of windows, as well as thresholds to determine when changes between static and dynamic
 * intervals occur can be easily configured.
 * This interval detector converts sensor measurements from device ENU coordinates to local plane
 * NED coordinates. Thus, all values referring to a given x-y-z coordinate refers to local plane
 * NED system of coordinates.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property initializationStartedListener listener to notify when this detector starts
 * initialization after being started.
 * @property initializationCompletedListener listener to notify when this detector completes
 * initialization after being started.
 * @property errorListener listener to notify errors such as sudden motion during initialization or
 * sensor unreliability.
 * @property staticIntervalDetectedListener listener to notify when a new static interval is
 * detected.
 * @property dynamicIntervalDetectedListener listener to notify when a new dynamic interval is
 * detected.
 * @property resetListener listener to notify when a reset occurs.
 * @property measurementListener listener to notify collected magnetometer measurements.
 * @property accuracyChangedListener listener to notify when magnetometer accuracy changes.
 */
class MagnetometerIntervalDetector(
    context: Context,
    val sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    initializationStartedListener: OnInitializationStartedListener<MagnetometerIntervalDetector>? = null,
    initializationCompletedListener: OnInitializationCompletedListener<MagnetometerIntervalDetector>? = null,
    errorListener: OnErrorListener<MagnetometerIntervalDetector>? = null,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<MagnetometerIntervalDetector>? = null,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<MagnetometerIntervalDetector>? = null,
    resetListener: OnResetListener<MagnetometerIntervalDetector>? = null,
    var measurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : IntervalDetector<MagnetometerIntervalDetector, MagnetometerSensorCollector,
        MagneticFluxDensityUnit, MagneticFluxDensity, MagneticFluxDensityTriad,
        MagneticFluxDensityTriadStaticIntervalDetector,
        MagneticFluxDensityTriadStaticIntervalDetectorListener>(
    context,
    sensorDelay,
    initializationStartedListener,
    initializationCompletedListener,
    errorListener,
    staticIntervalDetectedListener,
    dynamicIntervalDetectedListener,
    resetListener,
    accuracyChangedListener
) {

    /**
     * Listener for internal interval detector.
     */
    override val internalDetectorListener =
        object : MagneticFluxDensityTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?
            ) {
                initializationStartedListener?.onInitializationStarted(this@MagnetometerIntervalDetector)
            }

            override fun onInitializationCompleted(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@MagnetometerIntervalDetector,
                    baseNoiseLevel
                )
            }

            override fun onError(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                accumulatedNoiseLevel: Double,
                instantaneousNoiseLevel: Double,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                stop()
                errorListener?.onError(this@MagnetometerIntervalDetector, mapErrorReason(reason))
            }

            override fun onStaticIntervalDetected(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@MagnetometerIntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ
                )
            }

            override fun onDynamicIntervalDetected(
                detector: MagneticFluxDensityTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double,
                accumulatedAvgX: Double,
                accumulatedAvgY: Double,
                accumulatedAvgZ: Double,
                accumulatedStdX: Double,
                accumulatedStdY: Double,
                accumulatedStdZ: Double
            ) {
                dynamicIntervalDetectedListener?.onDynamicIntervalDetected(
                    this@MagnetometerIntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ,
                    accumulatedAvgX,
                    accumulatedAvgY,
                    accumulatedAvgZ,
                    accumulatedStdX,
                    accumulatedStdY,
                    accumulatedStdZ
                )
            }

            override fun onReset(detector: MagneticFluxDensityTriadStaticIntervalDetector?) {
                resetListener?.onReset(this@MagnetometerIntervalDetector)
            }
        }

    /**
     * Internal interval detector.
     * Processes magnetometer measurements and detects static and dynamic intervals.
     */
    override val internalDetector =
        MagneticFluxDensityTriadStaticIntervalDetector(internalDetectorListener)

    /**
     * Triad containing magnetic flux density samples converted from device ENU coordinates to local
     * plane NED coordinates.
     * This is reused for performance reasons.
     */
    private val b = MagneticFluxDensityTriad()

    /**
     * Internal listener for magnetometer sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val internalMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            val status = status
            if (status == Status.INITIALIZING) {
                // during initialization phase, also estimate time interval duration.
                if (numberOfProcessedMeasurements > 0) {
                    val diff = timestamp - initialTimestamp
                    val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
                    timeIntervalEstimator.addTimestamp(diffSeconds)
                } else {
                    initialTimestamp = timestamp
                }
            }

            val bxT = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
            val byT = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
            val bzT = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())

            // convert from device ENU coordinates to local plane NED coordinates
            ENUtoNEDConverter.convert(bxT, byT, bzT, b)

            internalDetector.process(b.valueX, b.valueY, b.valueZ)
            numberOfProcessedMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
                initialized = true
            }

            measurementListener?.onMeasurement(
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
    override val collector =
        MagnetometerSensorCollector(
            context,
            sensorType,
            sensorDelay,
            internalMeasurementListener,
            internalAccuracyChangedListener
        )
}