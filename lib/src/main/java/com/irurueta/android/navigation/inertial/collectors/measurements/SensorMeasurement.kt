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

package com.irurueta.android.navigation.inertial.collectors.measurements

import java.util.Objects

/**
 * Base class for sensor measurements.
 *
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
@Suppress("UNCHECKED_CAST")
abstract class SensorMeasurement<T : SensorMeasurement<T>>(
    var timestamp: Long,
    var accuracy: SensorAccuracy?,
    var sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) {
    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    open fun copyFrom(other: T) {
        timestamp = other.timestamp
        accuracy = other.accuracy
    }

    /**
     * Copies values to provided measurement.
     *
     * @param result instance where result will be stored.
     */
    fun copyTo(result: T) {
        result.copyFrom(this as T)
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    abstract fun copy(): T

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    abstract fun toNedOrThrow(result: T)

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    abstract fun toNedOrThrow(): T

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    abstract fun toEnuOrThrow(result: T)

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    abstract fun toEnuOrThrow(): T

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    abstract fun toNed(result: T)

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    abstract fun toNed(): T

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    abstract fun toEnu(result: T)

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    abstract fun toEnu(): T

    /** Checks whether this measurement is equal to the provided one.
     *
     * @param other other measurement to check against.
     * @return true if both measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return this === other || other is SensorMeasurement<*> &&
            timestamp == other.timestamp &&
            accuracy == other.accuracy
    }

    /**
     * Returns a hash code value for the object.
     *
     * @return a hash code value for this object.
     */
    override fun hashCode(): Int {
        return Objects.hash(timestamp, accuracy)
    }
}