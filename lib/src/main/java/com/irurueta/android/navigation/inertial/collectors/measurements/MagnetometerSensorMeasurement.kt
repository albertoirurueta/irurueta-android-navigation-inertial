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

import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorMeasurementCoordinateSystemConverter
import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorMeasurementNormConverter
import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorMeasurementTriadConverter
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import java.util.Objects

/**
 * Magnetometer sensor measurement.
 *
 * @property bx magnetic field on device x-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system.
 * @property by magnetic field on device y-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system.
 * @property bz magnetic field on device z-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system.
 * @property hardIronX hard iron on device x-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system. Only available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
 * If available, this value remains constant with calibrated bias value.
 * @property hardIronY hard iron on device y-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system. Only available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
 * If available, this value remains constant with calibrated bias value.
 * @property hardIronZ hard iron on device y-axis expressed in micro-Teslas (µT) and in ENU
 * coordinates system. Only available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
 * If available, this value remains constant with calibrated bias value.
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorType magnetometer sensor type.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
class MagnetometerSensorMeasurement(
    var bx: Float = 0.0f,
    var by: Float = 0.0f,
    var bz: Float = 0.0f,
    var hardIronX: Float? = null,
    var hardIronY: Float? = null,
    var hardIronZ: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    var sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) : SensorMeasurement<MagnetometerSensorMeasurement>(timestamp, accuracy, sensorCoordinateSystem),
    TriadConvertible<MagneticFluxDensityTriad> {

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: MagnetometerSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    override fun copyFrom(other: MagnetometerSensorMeasurement) {
        super.copyFrom(other)
        bx = other.bx
        by = other.by
        bz = other.bz
        hardIronX = other.hardIronX
        hardIronY = other.hardIronY
        hardIronZ = other.hardIronZ
        sensorType = other.sensorType
        sensorCoordinateSystem = other.sensorCoordinateSystem
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    override fun copy(): MagnetometerSensorMeasurement {
        val result = MagnetometerSensorMeasurement()
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
    override fun toNedOrThrow(result: MagnetometerSensorMeasurement) {
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): MagnetometerSensorMeasurement {
        return MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: MagnetometerSensorMeasurement) {
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): MagnetometerSensorMeasurement {
        return MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toNed(result: MagnetometerSensorMeasurement) {
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(
            this, result
        )
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toNed(): MagnetometerSensorMeasurement {
        return MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toEnu(result: MagnetometerSensorMeasurement) {
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(): MagnetometerSensorMeasurement {
        return MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(this)
    }

    /**
     * Converts this measurement into a triad and stores the result in the provided output
     * instance.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toTriad(result: MagneticFluxDensityTriad) {
        MagnetometerSensorMeasurementTriadConverter.convert(this, result)
    }

    /**
     * Converts this measurement into a triad and returns a new instance containing the result.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toTriad(): MagneticFluxDensityTriad {
        return MagnetometerSensorMeasurementTriadConverter.convert(this)
    }

    /**
     * Converts this measurement into its norm value.
     * Conversion keeps into account any available sensor bias (hard iron) data.
     *
     * @return norm value.
     */
    fun toNorm(): Double {
        return MagnetometerSensorMeasurementNormConverter.convert(this)
    }

    /**
     * Checks whether this measurement is equal to the provided one.
     *
     * @param other other measurement to check against.
     * @return true if both measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && (other is MagnetometerSensorMeasurement) &&
                bx == other.bx &&
                by == other.by &&
                bz == other.bz &&
                hardIronX == other.hardIronX &&
                hardIronY == other.hardIronY &&
                hardIronZ == other.hardIronZ &&
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
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            sensorType,
            sensorCoordinateSystem
        )
    }
}