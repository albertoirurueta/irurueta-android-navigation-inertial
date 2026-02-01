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

import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorMeasurementCoordinateSystemConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorMeasurementNormConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorMeasurementTriadConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import java.util.Objects

/**
 * Gravity sensor measurement.
 *
 * @property gx gravity acceleration on device x-axis expressed in meters per squared second
 * (m/s^2) and in ENU coordinates system.
 * @property gy gravity acceleration on device y-axis expressed in meters per squared second
 * (m/s^2) and in ENU coordinates system.
 * @property gz gravity acceleration on device z-axis expressed in meters per squared second
 * (m/s^2) and in ENU coordinates system.
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
class GravitySensorMeasurement(
    var gx: Float = 0.0f,
    var gy: Float = 0.0f,
    var gz: Float = 0.0f,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) : SensorMeasurement<GravitySensorMeasurement>(timestamp, accuracy, sensorCoordinateSystem),
    TriadConvertible<AccelerationTriad>{

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: GravitySensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    override fun copyFrom(other: GravitySensorMeasurement) {
        super.copyFrom(other)
        gx = other.gx
        gy = other.gy
        gz = other.gz
        sensorCoordinateSystem = other.sensorCoordinateSystem
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    override fun copy(): GravitySensorMeasurement {
        val result = GravitySensorMeasurement()
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
    override fun toNedOrThrow(result: GravitySensorMeasurement) {
        GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): GravitySensorMeasurement {
        return GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: GravitySensorMeasurement) {
        GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): GravitySensorMeasurement {
        return GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toNed(result: GravitySensorMeasurement) {
        GravitySensorMeasurementCoordinateSystemConverter.toNed(this, result)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toNed(): GravitySensorMeasurement {
        return GravitySensorMeasurementCoordinateSystemConverter.toNed(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toEnu(result: GravitySensorMeasurement) {
        GravitySensorMeasurementCoordinateSystemConverter.toEnu(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(): GravitySensorMeasurement {
        return GravitySensorMeasurementCoordinateSystemConverter.toEnu(this)
    }

    /**
     * Converts this measurement into a triad and stores the result in the provided output
     * instance.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @param result instance where result will be stored.
     */
    override fun toTriad(result: AccelerationTriad) {
        return GravitySensorMeasurementTriadConverter.convert(this, result)
    }

    /**
     * Converts this measurement into a triad and returns a new instance containing the result.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toTriad(): AccelerationTriad {
        return GravitySensorMeasurementTriadConverter.convert(this)
    }

    /**
     * Converts this measurement into its norm value.
     * Conversion takes into account any available sensor bias data.
     *
     * @return norm value.
     */
    fun toNorm(): Double {
        return GravitySensorMeasurementNormConverter.convert(this)
    }

    /**
     * Checks whether this measurement is equal to the provided one.
     *
     * @param other other measurement to check against.
     * @return true if both measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && (other is GravitySensorMeasurement) &&
            gx == other.gx &&
            gy == other.gy &&
            gz == other.gz &&
            sensorCoordinateSystem == other.sensorCoordinateSystem
    }

    /**
     * Returns a hash code value for the object.
     *
     * @return a hash code value for this object.
     */
    override fun hashCode(): Int {
        return Objects.hash(super.hashCode(), gx, gy, gz, sensorCoordinateSystem)
    }
}