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

import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorMeasurementCoordinateSystemConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorMeasurementNormConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorMeasurementTriadConverter
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import java.util.Objects

/**
 * Gyroscope sensor measurement.
 *
 * @property wx angular speed around device x-axis expressed in radians per second (rad/s) and
 * in ENU coordinates system.
 * @property wy angular speed around device y-axis expressed in radians per second (rad/s) and
 * in ENU coordinates system.
 * @property wz angular speed around device z-axis expressed in radians per second (rad/s) and
 * in ENU coordinates system.
 * @property bx estimated drift around device x-axis expressed in radians per second (rad/s)
 * and in ENU coordinates system.
 * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
 * remains constant with calibrated bias value.
 * @property by estimated drift around device y-axis expressed in radians per second (rad/s)
 * and in ENU coordinates system.
 * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
 * remains constant with calibrated bias value.
 * @property bz estimated drift around device z-axis expressed in radians per second (rad/s)
 * and in ENU coordinates system.
 * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
 * remains constant with calibrated bias value.
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorType gyroscope sensor type.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
class GyroscopeSensorMeasurement(
    var wx: Float = 0.0f,
    var wy: Float = 0.0f,
    var wz: Float = 0.0f,
    var bx: Float? = null,
    var by: Float? = null,
    var bz: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    var sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) : SensorMeasurement<GyroscopeSensorMeasurement>(timestamp, accuracy, sensorCoordinateSystem),
    TriadConvertible<AngularSpeedTriad> {

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: GyroscopeSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    override fun copyFrom(other: GyroscopeSensorMeasurement) {
        super.copyFrom(other)
        wx = other.wx
        wy = other.wy
        wz = other.wz
        bx = other.bx
        by = other.by
        bz = other.bz
        sensorType = other.sensorType
        sensorCoordinateSystem = other.sensorCoordinateSystem
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    override fun copy(): GyroscopeSensorMeasurement {
        val result = GyroscopeSensorMeasurement()
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
    override fun toNedOrThrow(result: GyroscopeSensorMeasurement) {
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): GyroscopeSensorMeasurement {
        return GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: GyroscopeSensorMeasurement) {
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): GyroscopeSensorMeasurement {
        return GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toNed(result: GyroscopeSensorMeasurement) {
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(this, result)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toNed(): GyroscopeSensorMeasurement {
        return GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toEnu(result: GyroscopeSensorMeasurement) {
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            this, result
        )
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(): GyroscopeSensorMeasurement {
        return GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(this)
    }

    /**
     * Converts this measurement into a triad and returns a new instance containing the result.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toTriad(result: AngularSpeedTriad) {
        GyroscopeSensorMeasurementTriadConverter.convert(this, result)
    }

    /**
     * Converts this measurement into a triad and returns a new instance containing the result.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toTriad(): AngularSpeedTriad {
        val triad = AngularSpeedTriad()
        toTriad(triad)
        return triad
    }

    /**
     * Converts this measurement into its norm value.
     * Conversion takes into account any available sensor bias data.
     *
     * @return norm value.
     */
    fun toNorm(): Double {
        return GyroscopeSensorMeasurementNormConverter.convert(this)
    }

    /**
     * Checks whether this measurement is equal to the provided one.
     *
     * @param other other measurement to check against.
     * @return true if both measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && (other is GyroscopeSensorMeasurement) &&
                wx == other.wx &&
                wy == other.wy &&
                wz == other.wz &&
                bx == other.bx &&
                by == other.by &&
                bz == other.bz &&
                sensorType == other.sensorType &&
                sensorCoordinateSystem == other.sensorCoordinateSystem
    }

    /**
     * Returns a hash code value for the object.
     *
     * @return a hash code value for this object.
     */
    override fun hashCode(): Int {
        return Objects.hash(
            super.hashCode(),
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            sensorType,
            sensorCoordinateSystem
        )
    }
}