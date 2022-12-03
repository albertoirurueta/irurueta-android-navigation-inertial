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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad

/**
 * Estimates sensed specific force component associated to gravity by either using the OS gravity
 * sensor or by low-pass filtering accelerometer measurements.
 * It must be noticed that usage of [AccelerometerSensorType.ACCELEROMETER] is
 * discouraged for posing purposes if [useAccelerometer] is enabled, as it will yield poor gravity
 * estimation values.
 * Instead either [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED] must be used
 * or [useAccelerometer] must be disabled to use gravity sensor.
 *
 * @property context Android context.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property estimationListener listener to notify when a new gravity measurement is available.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property accelerometerMeasurementListener listener to notify new accelerometer measurements.
 * (Only used if [useAccelerometer] is true).
 * @property gravityMeasurementListener listener to notify new gravity measurements.
 * (Only used if [useAccelerometer] is false).
 */
class GravityEstimator(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val useAccelerometer: Boolean = false,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    var estimationListener: OnEstimationListener? = null,
    val accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
    var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null
) {
    /**
     * Array to be reused containing result of averaging filter for accelerometer measurements.
     */
    private val accelerometerAveragingFilterOutput = DoubleArray(AveragingFilter.OUTPUT_LENGTH)

    /**
     * Triad to be reused for ENU to NED coordinates conversion.
     */
    private val triad = AccelerationTriad()

    /**
     * Internal gravity sensor collector.
     */
    private val gravitySensorCollector = GravitySensorCollector(
        context,
        sensorDelay,
        { gx, gy, gz, g, timestamp, accuracy ->
            ENUtoNEDTriadConverter.convert(gx.toDouble(), gy.toDouble(), gz.toDouble(), triad)
            estimationListener?.onEstimation(
                this@GravityEstimator,
                triad.valueX,
                triad.valueY,
                triad.valueZ,
                timestamp
            )
            gravityMeasurementListener?.onMeasurement(gx, gy, gz, g, timestamp, accuracy)
        })

    /**
     * Internal accelerometer sensor collector.
     */
    private val accelerometerSensorCollector = AccelerometerSensorCollector(
        context,
        accelerometerSensorType,
        sensorDelay,
        { ax, ay, az, bx, by, bz, timestamp, accuracy ->
            accelerometerMeasurementListener?.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )

            val currentAx = if (bx != null)
                ax.toDouble() - bx.toDouble()
            else
                ax.toDouble()
            val currentAy = if (by != null)
                ay.toDouble() - by.toDouble()
            else
                ay.toDouble()
            val currentAz = if (bz != null)
                az.toDouble() - bz.toDouble()
            else
                az.toDouble()

            if (accelerometerAveragingFilter.filter(
                    currentAx,
                    currentAy,
                    currentAz,
                    accelerometerAveragingFilterOutput,
                    timestamp
                )
            ) {
                ENUtoNEDTriadConverter.convert(
                    accelerometerAveragingFilterOutput[0],
                    accelerometerAveragingFilterOutput[1],
                    accelerometerAveragingFilterOutput[2],
                    triad
                )

                estimationListener?.onEstimation(
                    this@GravityEstimator,
                    triad.valueX,
                    triad.valueY,
                    triad.valueZ,
                    timestamp
                )
            }
        })

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        check(!running)

        reset()
        running = if (useAccelerometer) {
            accelerometerSensorCollector.start()
        } else {
            gravitySensorCollector.start()
        }
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        gravitySensorCollector.stop()
        accelerometerSensorCollector.stop()
        running = false
    }

    /**
     * Resets accelerometer averaging filter.
     */
    private fun reset() {
        accelerometerAveragingFilter.reset()
    }

    /**
     * Interface to notify when a new measurement is available.
     */
    fun interface OnEstimationListener {
        /**
         * Called when a new gravity measurement is available.
         *
         * @param estimator gravity estimator that raised this event.
         * @param gx x-coordinate of sensed specific force containing gravity component expressed
         * in NED coordinates.
         * @param gy y-coordinate of sensed specific force containing gravity component expressed
         * in NED coordinates.
         * @param gz z-coordinate of sensed specific force containing gravity component expressed
         * in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         */
        fun onEstimation(
            estimator: GravityEstimator,
            gx: Double,
            gy: Double,
            gz: Double,
            timestamp: Long
        )
    }
}