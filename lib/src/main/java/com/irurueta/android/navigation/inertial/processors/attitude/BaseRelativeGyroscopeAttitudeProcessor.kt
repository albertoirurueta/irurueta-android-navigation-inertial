/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.units.TimeConverter

/**
 * Base class to estimate relative attitude of device respect to an arbitrary initial attitude using
 * gyroscope measurements only.
 *
 * @property processorListener listener to notify new relative attitudes.
 */
abstract class BaseRelativeGyroscopeAttitudeProcessor(var processorListener: OnProcessedListener?) {

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Instance to be reused which contains integrated attitude of all gyroscope samples.
     */
    protected val internalAttitude = Quaternion()

    /**
     * Triad to be reused for ENU to NED coordinates conversion.
     */
    protected val triad = AngularSpeedTriad()

    /**
     * Time interval expressed in seconds between consecutive gyroscope measurements
     */
    var timeIntervalSeconds = 0.0
        private set

    /**
     * Instance to be reused which contains integrated attitude of all gyroscope samples using NED
     * coordinates system.
     */
    val attitude = Quaternion()

    /**
     * Processes a gyroscope sensor measurement to integrate angular speed values to obtain
     * an accumulated attitude from an arbitrary attitude of origin.
     *
     * @param measurement gyroscope measurement expressed in ENU android coordinates system to be
     * processed
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gyroscope measurement. If null, the timestamp from gyroscope measurement is used.
     * @return true if a new relative attitude is estimated, false otherwise.
     */
    abstract fun process(
        measurement: GyroscopeSensorMeasurement,
        timestamp: Long = measurement.timestamp
    ): Boolean

    /**
     * Resets this processor to its initial state.
     */
    fun reset() {
        previousTimestamp = -1L
        timeIntervalSeconds = 0.0

        resetQuaternion(attitude)
        resetQuaternion(internalAttitude)
    }

    /**
     * Converts gyroscope measurement values to NED coordinates and stores the result into [triad].
     *
     * @param measurement a gyroscope measurement.
     */
    protected fun updateTriad(measurement: GyroscopeSensorMeasurement) {
        val wx = measurement.wx
        val wy = measurement.wy
        val wz = measurement.wz
        val bx = measurement.bx
        val by = measurement.by
        val bz = measurement.bz

        val currentWx = if (bx != null)
            wx.toDouble() - bx.toDouble()
        else
            wx.toDouble()

        val currentWy = if (by != null)
            wy.toDouble() - by.toDouble()
        else
            wy.toDouble()

        val currentWz = if (bz != null)
            wz.toDouble() - bz.toDouble()
        else
            wz.toDouble()

        ENUtoNEDConverter.convert(currentWx, currentWy, currentWz, triad)
    }

    /**
     * Updates current time interval estimation between gyroscope measurements.
     *
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gyroscope measurement. If null, the timestamp from gyroscope measurement is used.
     * @return true if it is the 1st measurement by the time interval estimator, false otherwise.
     */
    protected fun updateTimeInterval(timestamp: Long): Boolean {
        val isFirst = previousTimestamp <= 0
        if (!isFirst) {
            val diff = timestamp - previousTimestamp
            timeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isFirst
    }

    internal companion object {
        /**
         * Resets provided quaternion to the identity
         */
        internal fun resetQuaternion(q: Quaternion) {
            q.a = 1.0
            q.b = 0.0
            q.c = 0.0
            q.d = 0.0
            q.normalize()
        }
    }

    /**
     * Interface to notify when a new relative attitude has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new relative attitude measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param attitude estimated relative attitude.
         * @param accuracy gyroscope sensor accuracy.
         */
        fun onProcessed(
            processor: BaseRelativeGyroscopeAttitudeProcessor,
            attitude: Quaternion,
            accuracy: SensorAccuracy?
        )
    }
}