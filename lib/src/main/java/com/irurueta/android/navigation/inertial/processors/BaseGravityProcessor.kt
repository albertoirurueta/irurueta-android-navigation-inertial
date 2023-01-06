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
package com.irurueta.android.navigation.inertial.processors

import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorMeasurement
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.AccelerationUnit

/**
 * Base class for a gravity processor.
 * Collects accelerometer or gravity sensor measurements and processed them and converts
 * them to NED coordinates.
 *
 * @property processorListener listener to notify new gravity measurements.
 */
abstract class BaseGravityProcessor<T : SensorMeasurement<T>>(
    var processorListener: OnProcessedListener<T>?
) {

    /**
     * Triad to be reused for ENU to NED coordinates conversion.
     */
    protected val triad = AccelerationTriad()

    /**
     * X-coordinates of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    var gx: Double = 0.0
        protected set

    /**
     * Y-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    var gy: Double = 0.0
        protected set

    /**
     * Z-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    var gz: Double = 0.0
        protected set

    /**
     * Gets a new triad containing gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gravity: AccelerationTriad
        get() = AccelerationTriad(gx, gy, gz)

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        result.setValueCoordinatesAndUnit(gx, gy, gz, AccelerationUnit.METERS_PER_SQUARED_SECOND)
    }

    /**
     * Time in nanoseconds at which the measurement was made. Each measurement will be monotonically
     * increasing using the same time base as [android.os.SystemClock.elapsedRealtimeNanos].
     */
    var timestamp: Long = 0L
        protected set

    /**
     * Accuracy of last sensed gravity measurement.
     */
    var accuracy: SensorAccuracy? = null
        protected set

    /**
     * Processes a gravity or accelerometer sensor measurement collected by a collector or a syncer.
     * @param measurement measurement expressed in ENU android coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * accelerometer or gravity measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     *
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.collectors.BufferedGravitySensorCollector
     * @see com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
     */
    abstract fun process(measurement: T, timestamp: Long = measurement.timestamp): Boolean

    /**
     * Resets this processor to its initial values.
     */
    fun reset() {
        gx = 0.0
        gy = 0.0
        gz = 0.0
        timestamp = 0L
        accuracy = null
    }

    /**
     * Interface to notify when a new gravity measurement has been processed.
     */
    fun interface OnProcessedListener<T : SensorMeasurement<T>> {
        /**
         * Called when a new gravity measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param gx x-coordinate of sensed gravity component of specific force expressed in NED
         * coordinates and meters per squared second (m/s^2).
         * @param gy y-coordinate of sensed gravity component of specific force expressed in NED
         * coordinates and meters per squared second (m/s^2).
         * @param gz z-coordinate of sensed gravity component of specific force expressed in NED
         * coordinates and meters per squared second (m/s^2).
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy gravity or accelerometer sensor accuracy.
         */
        fun onProcessed(
            processor: BaseGravityProcessor<T>,
            gx: Double,
            gy: Double,
            gz: Double,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}