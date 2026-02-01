/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.intervals.AngularSpeedTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.AngularSpeedTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.TimeConverter

/**
 * Detects static or motion intervals of time by using the gyroscope of the device to determine
 * whether the device is moving or not.
 * When detector is started, initialization occurs to determine the gyroscope noise level while
 * keeping device static. Once the detector is initialized, then static or dynamic intervals can be
 * detected.
 * This detector uses accumulated average values during static intervals, and windowed averages as
 * "instantaneous" values during dynamic intervals.
 * Length of windows, as well as thresholds to determine when changes between static and dynamic
 * intervals occur can be easily configured.
 * This interval detector converts sensor measurements from device ENU coordinates to local plane
 * NED coordinates. Thus, all values referring to a given x-y-z coordinate refers to local plane
 * NED system of coordinates.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
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
 */
class GyroscopeIntervalDetector(
    context: Context,
    val sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    initializationStartedListener: OnInitializationStartedListener<GyroscopeIntervalDetector>? = null,
    initializationCompletedListener: OnInitializationCompletedListener<GyroscopeIntervalDetector>? = null,
    errorListener: OnErrorListener<GyroscopeIntervalDetector>? = null,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<GyroscopeIntervalDetector>? = null,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<GyroscopeIntervalDetector>? = null,
    resetListener: OnResetListener<GyroscopeIntervalDetector>? = null,
) : IntervalDetector<GyroscopeIntervalDetector, GyroscopeSensorCollector, AngularSpeedUnit,
        AngularSpeed, AngularSpeedTriad, AngularSpeedTriadStaticIntervalDetector,
        AngularSpeedTriadStaticIntervalDetectorListener>(
    context,
    sensorDelay,
    initializationStartedListener,
    initializationCompletedListener,
    errorListener,
    staticIntervalDetectedListener,
    dynamicIntervalDetectedListener,
    resetListener
) {
    /**
     * Listener for internal interval detector.
     */
    override val internalDetectorListener =
        object : AngularSpeedTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(
                detector: AngularSpeedTriadStaticIntervalDetector?
            ) {
                initializationStartedListener?.onInitializationStarted(
                    this@GyroscopeIntervalDetector
                )

            }

            override fun onInitializationCompleted(
                detector: AngularSpeedTriadStaticIntervalDetector?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@GyroscopeIntervalDetector,
                    baseNoiseLevel
                )
            }

            override fun onError(
                detector: AngularSpeedTriadStaticIntervalDetector?,
                accumulatedNoiseLevel: Double,
                instantaneousNoiseLevel: Double,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                stop()
                errorListener?.onError(this@GyroscopeIntervalDetector,
                    mapErrorReason(reason))
            }

            override fun onStaticIntervalDetected(
                detector: AngularSpeedTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@GyroscopeIntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ
                )
            }

            override fun onDynamicIntervalDetected(
                detector: AngularSpeedTriadStaticIntervalDetector?,
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
                    this@GyroscopeIntervalDetector,
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

            override fun onReset(detector: AngularSpeedTriadStaticIntervalDetector?) {
                resetListener?.onReset(this@GyroscopeIntervalDetector)
            }
        }

    /**
     * Internal interval detector.
     * Processes gyroscope measurements and detects static and dynamic intervals.
     */
    override val internalDetector =
        AngularSpeedTriadStaticIntervalDetector(internalDetectorListener)

    /**
     * Triad containing angular speed samples converted from device ENU coordinates to local plane
     * NED coordinates.
     * This is reused for performance reasons.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * Gyroscope sensor measurement expressed in NED coordinates.
     */
    private val nedMeasurement = GyroscopeSensorMeasurement()

    /**
     * Internal listener for accelerometer sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val measurementListener = SensorCollector.OnMeasurementListener<
            GyroscopeSensorMeasurement, GyroscopeSensorCollector> { _, measurement ->
        val status = status
        if (status == Status.INITIALIZING) {
            // during initialization phase, also estimate time interval duration.
            if (numberOfProcessedMeasurements > 0) {
                val diff = measurement.timestamp - initialTimestamp
                val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
                timeIntervalEstimator.addTimestamp(diffSeconds)
            } else {
                initialTimestamp = measurement.timestamp
            }
        }

        // convert from device ENU coordinates to local plane NED coordinates
        measurement.toNed(nedMeasurement)
        nedMeasurement.toTriad(angularSpeed)

        internalDetector.process(angularSpeed)
        numberOfProcessedMeasurements++

        if (status == Status.INITIALIZATION_COMPLETED) {
            // once initialized, set time interval into internal detector
            internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
            initialized = true
        }
    }

    /**
     * Listener to detect when accuracy of accelerometer sensor changes.
     * When sensor becomes unreliable, an error is notified.
     */
    private val accuracyChangedListener = SensorCollector.OnAccuracyChangedListener<GyroscopeSensorMeasurement, GyroscopeSensorCollector> { _, accuracy ->
        if (accuracy == SensorAccuracy.UNRELIABLE) {
            stop()
            unreliable = true
            errorListener?.onError(
                this,
                ErrorReason.UNRELIABLE_SENSOR
            )
        }
    }

    /**
     * Gyroscope sensor collector.
     * Collects gyroscope measurements.
     */
    override val collector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        accuracyChangedListener,
        measurementListener
    )
}