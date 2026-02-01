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

import com.irurueta.android.navigation.inertial.collectors.converters.AttitudeSensorMeasurementCoordinateSystemConverter
import com.irurueta.geometry.Quaternion
import java.util.Objects

/**
 * Rotation sensor measurement.
 *
 * @property attitude quaternion containing measured device attitude in ENU coordinates.
 * @property headingAccuracy heading accuracy expressed in radians or null if not available.
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorType attitude sensor type.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
class AttitudeSensorMeasurement(
    var attitude: Quaternion = Quaternion(),
    var headingAccuracy: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    var sensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) : SensorMeasurement<AttitudeSensorMeasurement>(timestamp, accuracy, sensorCoordinateSystem) {

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: AttitudeSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    override fun copyFrom(other: AttitudeSensorMeasurement) {
        super.copyFrom(other)
        attitude.fromRotation(other.attitude)
        headingAccuracy = other.headingAccuracy
        sensorType = other.sensorType
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    override fun copy(): AttitudeSensorMeasurement {
        val result = AttitudeSensorMeasurement()
        copyTo(result)
        return result
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(result: AttitudeSensorMeasurement) {
        AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): AttitudeSensorMeasurement {
        return AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: AttitudeSensorMeasurement) {
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): AttitudeSensorMeasurement {
        return AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toNed(result: AttitudeSensorMeasurement) {
        AttitudeSensorMeasurementCoordinateSystemConverter.toNed(this, result)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toNed(): AttitudeSensorMeasurement {
        return AttitudeSensorMeasurementCoordinateSystemConverter.toNed(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toEnu(result: AttitudeSensorMeasurement) {
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(): AttitudeSensorMeasurement {
        return AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(this)
    }

    /**
     * Indicates whether some other object is "equal to" this one.
     *
     * @param other the reference object with which to compare.
     * @return true if this object is the same as the [other] argument, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && other is AttitudeSensorMeasurement &&
                attitude == other.attitude &&
                headingAccuracy == other.headingAccuracy &&
                sensorType == other.sensorType
    }

    /**
     * Returns a hash code value for the object.
     *
     * @return a hash code value for this object.
     */
    override fun hashCode(): Int {
        return Objects.hash(super.hashCode(), attitude, headingAccuracy, timestamp, accuracy, sensorType)
    }
}