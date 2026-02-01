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
import com.irurueta.android.navigation.inertial.old.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.old.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
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
    var measurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
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
    resetListener,
    accuracyChangedListener
) {
    /**
     * Listener for interval detector.
     */
    override val internalDetectorListener =
        object : AngularSpeedTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(detector: AngularSpeedTriadStaticIntervalDetector?) {
                initializationStartedListener?.onInitializationStarted(this@GyroscopeIntervalDetector)
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
                errorListener?.onError(this@GyroscopeIntervalDetector, mapErrorReason(reason))
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
     * Internal listener for gyroscope sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val internalMeasurementListener =
        GyroscopeSensorCollector.OnMeasurementListener { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
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

            // convert from device ENU coordinates to local plane NED coordinates
            ENUtoNEDConverter.convert(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble(),
                angularSpeed
            )

            internalDetector.process(angularSpeed.valueX, angularSpeed.valueY, angularSpeed.valueZ)
            numberOfProcessedMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
                initialized = true
            }

            measurementListener?.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Gyroscope sensor collector.
     * Collects gyroscope measurements.
     */
    override val collector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        internalMeasurementListener,
        internalAccuracyChangedListener
    )
}