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
package com.irurueta.android.navigation.inertial.calibration.intervals

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetectorListener
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.TimeConverter

/**
 * Detects static or motion intervals of time by using the accelerometer of the
 * device to determine whether the device is moving or not.
 * When detector is started, initialization occurs to determine the accelerometer noise level while
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
 * @property sensorType One of the supported accelerometer sensor types.
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
 * @property measurementListener listener to notify collected accelerometer measurements.
 * @property accuracyChangedListener listener to notify when accelerometer accuracy changes.
 */
class AccelerometerIntervalDetector(
    context: Context,
    val sensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    initializationStartedListener: OnInitializationStartedListener<AccelerometerIntervalDetector>? = null,
    initializationCompletedListener: OnInitializationCompletedListener<AccelerometerIntervalDetector>? = null,
    errorListener: OnErrorListener<AccelerometerIntervalDetector>? = null,
    staticIntervalDetectedListener: OnStaticIntervalDetectedListener<AccelerometerIntervalDetector>? = null,
    dynamicIntervalDetectedListener: OnDynamicIntervalDetectedListener<AccelerometerIntervalDetector>? = null,
    resetListener: OnResetListener<AccelerometerIntervalDetector>? = null,
    var measurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
    accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) : IntervalDetector<AccelerometerIntervalDetector, AccelerometerSensorCollector, AccelerationUnit,
        Acceleration, AccelerationTriad, AccelerationTriadStaticIntervalDetector,
        AccelerationTriadStaticIntervalDetectorListener>(
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
        object : AccelerationTriadStaticIntervalDetectorListener {
            override fun onInitializationStarted(
                detector: AccelerationTriadStaticIntervalDetector?
            ) {
                initializationStartedListener?.onInitializationStarted(this@AccelerometerIntervalDetector)
            }

            override fun onInitializationCompleted(
                detector: AccelerationTriadStaticIntervalDetector?,
                baseNoiseLevel: Double
            ) {
                initializationCompletedListener?.onInitializationCompleted(
                    this@AccelerometerIntervalDetector,
                    baseNoiseLevel
                )
            }

            override fun onError(
                detector: AccelerationTriadStaticIntervalDetector?,
                accumulatedNoiseLevel: Double,
                instantaneousNoiseLevel: Double,
                reason: TriadStaticIntervalDetector.ErrorReason
            ) {
                stop()
                errorListener?.onError(this@AccelerometerIntervalDetector, mapErrorReason(reason))
            }

            override fun onStaticIntervalDetected(
                detector: AccelerationTriadStaticIntervalDetector?,
                instantaneousAvgX: Double,
                instantaneousAvgY: Double,
                instantaneousAvgZ: Double,
                instantaneousStdX: Double,
                instantaneousStdY: Double,
                instantaneousStdZ: Double
            ) {
                staticIntervalDetectedListener?.onStaticIntervalDetected(
                    this@AccelerometerIntervalDetector,
                    instantaneousAvgX,
                    instantaneousAvgY,
                    instantaneousAvgZ,
                    instantaneousStdX,
                    instantaneousStdY,
                    instantaneousStdZ
                )
            }

            override fun onDynamicIntervalDetected(
                detector: AccelerationTriadStaticIntervalDetector?,
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
                    this@AccelerometerIntervalDetector,
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

            override fun onReset(detector: AccelerationTriadStaticIntervalDetector?) {
                resetListener?.onReset(this@AccelerometerIntervalDetector)
            }
        }

    /**
     * Internal interval detector.
     * Processes accelerometer measurements and detects static and dynamic intervals.
     */
    override val internalDetector =
        AccelerationTriadStaticIntervalDetector(internalDetectorListener)

    /**
     * Triad containing acceleration samples converted from device ENU coordinates to local plane
     * NED coordinates.
     * This is reused for performance reasons.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Internal listener for accelerometer sensor collector.
     * Handles measurements collected by the sensor collector so that they are processed by
     * the internal interval detector.
     */
    private val internalMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { ax, ay, az, bx, by, bz, timestamp, accuracy ->
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
            ENUtoNEDTriadConverter.convert(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                acceleration
            )

            internalDetector.process(acceleration.valueX, acceleration.valueY, acceleration.valueZ)
            numberOfProcessedMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                internalDetector.timeInterval = timeIntervalEstimator.averageTimeInterval
                initialized = true
            }

            measurementListener?.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Accelerometer sensor collector.
     * Collects accelerometer measurements.
     */
    override val collector =
        AccelerometerSensorCollector(
            context,
            sensorType,
            sensorDelay,
            internalMeasurementListener,
            internalAccuracyChangedListener
        )
}