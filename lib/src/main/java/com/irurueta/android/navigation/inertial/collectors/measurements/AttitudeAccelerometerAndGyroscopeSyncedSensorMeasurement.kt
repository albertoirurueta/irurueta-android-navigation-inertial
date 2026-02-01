/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
 * Contains synced attitude, accelerometer and gyroscope measurements, which are assumed to belong
 * to the same timestamp.
 *
 * @property attitudeMeasurement an attitude measurement (either relative or absolute). Notice that
 * this instance might be reused between consecutive
 * [AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement] measurements, and that its timestamp
 * might differ from the global [timestamp] property of this [SyncedSensorMeasurement]
 * @property accelerometerMeasurement an accelerometer measurement. Notice that this instance might
 * be reused between consecutive [AccelerometerAndGyroscopeSyncedSensorMeasurement] measurements,
 * and that its timestamp might differ from the global [timestamp] property of this
 * [SyncedSensorMeasurement].
 * @property gyroscopeMeasurement a gyroscope measurement. Notice that this instance might be reused
 * between consecutive [AccelerometerAndGyroscopeSyncedSensorMeasurement] measurements, and that its
 * timestamp might differ from the global [timestamp] property of this [SyncedSensorMeasurement].
 * @property timestamp timestamp expressed in nanoseconds following
 * [android.os.SystemClock.elapsedRealtimeNanos] monotonic clock when synced [SensorMeasurement] are
 * assumed to occur.
 */
class AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
    var attitudeMeasurement: AttitudeSensorMeasurement? = null,
    var accelerometerMeasurement: AccelerometerSensorMeasurement? = null,
    var gyroscopeMeasurement: GyroscopeSensorMeasurement? = null,
    timestamp: Long = 0L
) : SyncedSensorMeasurement<AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement>(
    timestamp
) {
    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided synced sensor measurement.
     *
     * @param other other synced measurement to copy from.
     */
    override fun copyFrom(other: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) {
        super.copyFrom(other)
        attitudeMeasurement = other.attitudeMeasurement?.copy()
        accelerometerMeasurement = other.accelerometerMeasurement?.copy()
        gyroscopeMeasurement = other.gyroscopeMeasurement?.copy()
    }

    /**
     * Creates a new copy of this synced sensor measurement.
     *
     * @return a new copy of this synced sensor measurement.
     */
    override fun copy(): AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement {
        val result = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        copyTo(result)
        return result
    }

    /**
     * Converts this synced sensor measurement to North-East-Down (NED) coordinates system.
     *
     * @param result instance where converted synced sensor measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(result: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) {
        // check requirements
        val attitudeMeasurement = this.attitudeMeasurement
        if (attitudeMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                attitudeMeasurement
            )
        }
        val accelerometerMeasurement = this.accelerometerMeasurement
        if (accelerometerMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                accelerometerMeasurement
            )
        }
        val gyroscopeMeasurement = this.gyroscopeMeasurement
        if (gyroscopeMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                gyroscopeMeasurement
            )
        }

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAttitudeMeasurement = result.attitudeMeasurement
        if (resultAttitudeMeasurement != null) {
            attitudeMeasurement?.toNedOrThrow(resultAttitudeMeasurement)
        }
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toNedOrThrow(resultAccelerometerMeasurement)
        }
        val resultGyroscopeMeasurement = result.gyroscopeMeasurement
        if (resultGyroscopeMeasurement != null) {
            gyroscopeMeasurement?.toNedOrThrow(resultGyroscopeMeasurement)
        }
    }

    /**
     * Converts this synced sensor measurement to North-East-Down (NED) coordinates system.
     *
     * @return a new instance containing converted synced sensor measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(): AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement {
        val output = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        toNedOrThrow(output)
        return output
    }

    /**
     * Converts this synced sensor measurement to East-North-Up (ENU) coordinates system.
     *
     * @param result instance where converted synced sensor measurement will be stored.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(result: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) {
        // check requirements
        val attitudeMeasurement = this.attitudeMeasurement
        if (attitudeMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                attitudeMeasurement
            )
        }
        val accelerometerMeasurement = this.accelerometerMeasurement
        if (accelerometerMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                accelerometerMeasurement
            )
        }
        val gyroscopeMeasurement = this.gyroscopeMeasurement
        if (gyroscopeMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                gyroscopeMeasurement
            )
        }

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAttitudeMeasurement = result.attitudeMeasurement
        if (resultAttitudeMeasurement != null) {
            attitudeMeasurement?.toEnuOrThrow(resultAttitudeMeasurement)
        }
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toEnuOrThrow(resultAccelerometerMeasurement)
        }
        val resultGyroscopeMeasurement = result.gyroscopeMeasurement
        if (resultGyroscopeMeasurement != null) {
            gyroscopeMeasurement?.toEnuOrThrow(resultGyroscopeMeasurement)
        }
    }

    /**
     * Converts this synced sensor measurement to East-North-Up (ENU) coordinates system.
     *
     * @return a new instance containing converted synced sensor measurement.
     * @throws IllegalArgumentException if current coordinate system is not supported.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(): AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement {
        val output = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        toEnuOrThrow(output)
        return output
    }

    /**
     * Converts this synced sensor measurement to North-East-Down (NED) coordinates system and
     * stores the result in the provided output instance.
     * If this synced sensor measurement is already expressed in NED coordinates system, this
     * synced sensor measurement is copied into output instance.
     *
     * @param result instance where converted synced sensor measurement will be stored.
     */
    override fun toNed(result: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) {
        val attitudeMeasurement = this.attitudeMeasurement
        val accelerometerMeasurement = this.accelerometerMeasurement
        val gyroscopeMeasurement = this.gyroscopeMeasurement

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAttitudeMeasurement = result.attitudeMeasurement
        if (resultAttitudeMeasurement != null) {
            attitudeMeasurement?.toNed(resultAttitudeMeasurement)
        }
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toNed(resultAccelerometerMeasurement)
        }
        val resultGyroscopeMeasurement = result.gyroscopeMeasurement
        if (resultGyroscopeMeasurement != null) {
            gyroscopeMeasurement?.toNed(resultGyroscopeMeasurement)
        }
    }

    /**
     * Converts this synced sensor measurement to North-East-Down (NED) coordinates system and
     * returns a new instance containing the result.
     * If this synced sensor measurement is already expressed in NED coordinates system, a copy
     * of this synced sensor measurement is returned.
     *
     * @return a new instance containing converted synced sensor measurement.
     */
    override fun toNed(): AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement {
        val output = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        toNed(output)
        return output
    }

    /**
     * Converts this synced sensor measurement to East-North-Up (ENU) coordinates system and
     * stores the result in the provided output instance.
     * If this synced sensor measurement is already expressed in ENU coordinates system, this
     * synced sensor measurement is copied into output instance.
     *
     * @param result instance where converted synced sensor measurement will be stored.
     */
    override fun toEnu(result: AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) {
        val attitudeMeasurement = this.attitudeMeasurement
        val accelerometerMeasurement = this.accelerometerMeasurement
        val gyroscopeMeasurement = this.gyroscopeMeasurement

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAttitudeMeasurement = result.attitudeMeasurement
        if (resultAttitudeMeasurement != null) {
            attitudeMeasurement?.toEnu(resultAttitudeMeasurement)
        }
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toEnu(resultAccelerometerMeasurement)
        }
        val resultGyroscopeMeasurement = result.gyroscopeMeasurement
        if (resultGyroscopeMeasurement != null) {
            gyroscopeMeasurement?.toEnu(resultGyroscopeMeasurement)
        }
    }

    /**
     * Converts this synced sensor measurement to East-North-Up (ENU) coordinates system and
     * returns a new instance containing the result.
     * If this synced sensor measurement is already expressed in ENU coordinates system, a copy
     * of this synced sensor measurement is returned.
     *
     * @return a new instance containing converted synced sensor measurement.
     */
    override fun toEnu(): AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement {
        val output = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        toEnu(output)
        return output
    }

    /**
     * Checks whether this synced sensor measurement is equal to the provided one.
     *
     * @param other other synced sensor measurement to check against.
     * @return true if both synced sensor measurements are equal, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other) && (other is AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement) &&
                attitudeMeasurement == other.attitudeMeasurement &&
                accelerometerMeasurement == other.accelerometerMeasurement &&
                gyroscopeMeasurement == other.gyroscopeMeasurement
    }

    /**
     * Returns a hash code value for the object.
     *
     * @return a hash code value for this object.
     */
    override fun hashCode(): Int {
        return Objects.hash(
            super.hashCode(),
            attitudeMeasurement,
            accelerometerMeasurement,
            gyroscopeMeasurement
        )
    }
}