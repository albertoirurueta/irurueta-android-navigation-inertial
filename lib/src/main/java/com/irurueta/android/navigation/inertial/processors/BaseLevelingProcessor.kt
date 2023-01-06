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

import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.AccelerationUnit

/**
 * Base class to estimate leveling of device (roll and pitch angle) by using estimated gravity
 * vector.
 *
 * @property processorListener listener to notify new leveling measurements.
 */
abstract class BaseLevelingProcessor(var processorListener: OnProcessedListener?) {

    /**
     *
     * Instance to be reused containing estimated leveling attitude (roll and pitch angles) in NED
     * coordinates.
     */
    var attitude = Quaternion()
        protected set

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
     * Processes provided gravity components estimated using a [BaseGravityProcessor].
     */
    abstract fun process(gx: Double, gy: Double, gz: Double)

    /**
     * Resets this processor to its initial values.
     */
    fun reset() {
        gx = 0.0
        gy = 0.0
        gz = 0.0
        attitude.setFromEulerAngles(0.0, 0.0, 0.0)
    }

    /**
     * Interface to notify when a new leveled attitude has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new leveled attitude measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param attitude estimated leveled attitude.
         */
        fun onProcessed(
            processor: BaseLevelingProcessor,
            attitude: Quaternion
        )
    }
}