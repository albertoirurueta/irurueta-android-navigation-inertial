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

import com.irurueta.android.navigation.inertial.collectors.converters.AccelerometerSensorMeasurementCoordinateSystemConverter
import com.irurueta.android.navigation.inertial.collectors.converters.AccelerometerSensorMeasurementNormConverter
import com.irurueta.android.navigation.inertial.collectors.converters.AccelerometerSensorMeasurementTriadConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import java.util.Objects

/**
 * Accelerometer sensor measurement.
 *
 * @property ax acceleration on device x-axis expressed in meters per squared second (m/s^2)
 * and in ENU coordinates system.
 * @property ay acceleration on device y-axis expressed in meters per squared second (m/s^2)
 * and in ENU coordinates system.
 * @property az acceleration on device z-axis expressed in meters per squared second (m/s^2)
 * and in ENU coordinates system.
 * @property bx bias on device x-axis expressed in meters per squared second (m/s^2) and in
 * ENU coordinates system. Only available when using
 * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value remains constant
 * with calibrated bias value.
 * @property by bias on device y-axis expressed in meters per squared second (m/s^2) and in
 * ENU coordinates system. Only available when using
 * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value remains constant
 * with calibrated bias value.
 * @property bz bias on device z-axis expressed in meters per squared second (m/s^2) and in
 * ENU coordinates system. Only available when using
 * [AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value remains constant
 * with calibrated bias value.
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement received from a given sensor will be monotonically increasing using the same time
 * base as [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 * @property sensorType accelerometer sensor type.
 * @property sensorCoordinateSystem coordinate system in which sensor measurements are expressed.
 */
class AccelerometerSensorMeasurement(
    var ax: Float = 0.0f,
    var ay: Float = 0.0f,
    var az: Float = 0.0f,
    var bx: Float? = null,
    var by: Float? = null,
    var bz: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    var sensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    sensorCoordinateSystem: SensorCoordinateSystem = SensorCoordinateSystem.ENU
) : SensorMeasurement<AccelerometerSensorMeasurement>(timestamp, accuracy, sensorCoordinateSystem),
    TriadConvertible<AccelerationTriad> {

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: AccelerometerSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    override fun copyFrom(other: AccelerometerSensorMeasurement) {
        super.copyFrom(other)
        ax = other.ax
        ay = other.ay
        az = other.az
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
    override fun copy(): AccelerometerSensorMeasurement {
        val result = AccelerometerSensorMeasurement()
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
    override fun toNedOrThrow(result: AccelerometerSensorMeasurement) {
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): AccelerometerSensorMeasurement {
        return AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: AccelerometerSensorMeasurement) {
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): AccelerometerSensorMeasurement {
        return AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(this)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in NED coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toNed(result: AccelerometerSensorMeasurement) {
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(
            this, result)
    }

    /**
     * Converts this measurement to North-East-Down (NED) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in NED coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toNed(): AccelerometerSensorMeasurement {
        return AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(this)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and stores the
     * result in the provided output instance.
     * If this measurement is already expressed in ENU coordinates system, this measurement
     * is copied into output instance.
     *
     * @param result instance where converted measurement will be stored.
     */
    override fun toEnu(result: AccelerometerSensorMeasurement) {
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(
            this, result)
    }

    /**
     * Converts this measurement to East-North-Up (ENU) coordinates system and returns a new
     * instance containing the result.
     * If this measurement is already expressed in ENU coordinates system, a copy of this
     * measurement is returned.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(): AccelerometerSensorMeasurement {
        return AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(this)
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
        AccelerometerSensorMeasurementTriadConverter.convert(this, result)
    }

    /**
     * Converts this measurement into a triad and returns a new instance containing the result.
     * Conversion keeps into account any available sensor bias data, and preserves coordinates
     * system.
     *
     * @return a new instance containing converted measurement.
     */
    override fun toTriad(): AccelerationTriad {
        return AccelerometerSensorMeasurementTriadConverter.convert(this)
    }

    /**
     * Converts this measurement into its norm value.
     * Conversion takes into account any available sensor bias data.
     *
     * @return norm value.
     */
    fun toNorm(): Double {
        return AccelerometerSensorMeasurementNormConverter.convert(this)
    }

    /**
     * Checks whether this measurement is equal to the provided one.
     *
     * @param other other measurement to check against.
     * @return true if both measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && (other is AccelerometerSensorMeasurement) &&
                ax == other.ax &&
                ay == other.ay &&
                az == other.az &&
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
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            sensorType,
            sensorCoordinateSystem
        )
    }
}