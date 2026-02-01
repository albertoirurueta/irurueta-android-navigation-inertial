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

import android.hardware.SensorManager
import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.NEDGravity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.units.AccelerationUnit
import kotlin.math.sqrt

/**
 * Base class for a gravity processor.
 * Collects accelerometer or gravity sensor measurements and processed them and converts
 * them to NED coordinates.
 *
 * @property location current device location.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 * @property processorListener listener to notify new gravity measurements.
 */
abstract class BaseGravityProcessor<T : SensorMeasurement<T>>(
    var location: Location?,
    var adjustGravityNorm: Boolean,
    var processorListener: OnProcessedListener<T>?
) {
    /**
     * Gravity expressed in NED coordinates.
     */
    private val nedGravity = NEDGravity()

    /**
     * Position expressed in NED coordinates. Only used if location is provided.
     */
    private val nedPosition = NEDPosition()

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
     * Gets norm of current gravity.
     */
    val gravityNorm: Double
        get() = sqrt(gx * gx + gy * gy + gz * gz)

    /**
     * Gets NED position from provided location.
     *
     * @param result instance where converted location expressed in NED coordinates will be stored.
     * @return true if location is available and conversion is computed, false otherwise.
     */
    fun getNedPosition(result: NEDPosition): Boolean {
        val location = this.location
        return if (location != null) {
            location.toNEDPosition(result)
            true
        } else {
            false
        }
    }

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        result.setValueCoordinatesAndUnit(
            gx,
            gy,
            gz,
            AccelerationUnit.METERS_PER_SQUARED_SECOND
        )
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
     * Gets expected gravity norm at provided location.
     * If No location is provided, average gravity at sea level is returned instead.
     * This value will be used to adjust to adjust gravity norm if [adjustGravityNorm] is true.
     */
    val expectedGravityNorm: Double
        get() {
            val location = this.location
            return if (location != null) {
                location.toNEDPosition(nedPosition)
                NEDGravityEstimator.estimateGravity(nedPosition, nedGravity)
                nedGravity.norm
            } else {
                GRAVITY_EARTH
            }
        }

    /**
     * Processes a gravity or accelerometer sensor measurement collected by a collector or a syncer.
     * @param measurement measurement expressed in ENU android coordinates system to be processed.
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * accelerometer or gravity measurement. If not set, the timestamp from measurement is used.
     * @return true if a new gravity is estimated, false otherwise.
     *
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndGyroscopeSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
     * @see com.irurueta.android.navigation.inertial.old.collectors.BufferedGravitySensorCollector
     * @see com.irurueta.android.navigation.inertial.old.collectors.GravitySensorCollector
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
     * Adjusts norm (only if [adjustGravityNorm] is true).
     */
    protected fun adjustNorm() {
        if (!adjustGravityNorm) {
            return
        }

        val factor = expectedGravityNorm / gravityNorm
        gx *= factor
        gy *= factor
        gz *= factor
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

    private companion object {
        /**
         * Constant defining average acceleration due to gravity at Earth's sea level.
         * This is roughly 9.81 m/s¨2.
         */
        const val GRAVITY_EARTH = SensorManager.GRAVITY_EARTH.toDouble()
    }
}