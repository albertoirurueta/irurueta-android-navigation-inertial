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
 * Contains synced accelerometer, gravity and gyroscope measurements, which are assumed to belong to
 * the same timestamp.
 *
 * @property accelerometerMeasurement an accelerometer measurement. Notice that this instance might
 * be reused between consecutive [AccelerometerGravityAndGyroscopeSyncedSensorMeasurement]
 * measurements, and that its timestamp might differ from the global [timestamp] property of this
 * [SyncedSensorMeasurement].
 * @property gravityMeasurement a gravity measurement. Notice that this instance might be reused
 * between consecutive [AccelerometerGravityAndGyroscopeSyncedSensorMeasurement] measurements, and
 * that its timestamp might differ from the global [timestamp] property of this
 * [SyncedSensorMeasurement].
 * @property gyroscopeMeasurement a gyroscope measurement. Notice that this instance might be reused
 * between consecutive [AccelerometerGravityAndGyroscopeSyncedSensorMeasurement] measurements, and
 * that its timestamp might differ from the global [timestamp] property of this
 * [SyncedSensorMeasurement].
 * @property timestamp timestamp expressed in nanoseconds following
 * [android.os.SystemClock.elapsedRealtimeNanos] monotonic clock when synced [SensorMeasurement] are
 * assumed to occur.
 */
class AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
    var accelerometerMeasurement: AccelerometerSensorMeasurement? = null,
    var gravityMeasurement: GravitySensorMeasurement? = null,
    var gyroscopeMeasurement: GyroscopeSensorMeasurement? = null,
    timestamp: Long = 0L
) : SyncedSensorMeasurement<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement>(
    timestamp
) {
    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    constructor(other: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) : this() {
        copyFrom(other)
    }

    /**
     * Copies values from provided synced sensor measurement.
     *
     * @param other other synced measurement to copy from.
     */
    override fun copyFrom(other: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) {
        super.copyFrom(other)
        accelerometerMeasurement = other.accelerometerMeasurement?.copy()
        gravityMeasurement = other.gravityMeasurement?.copy()
        gyroscopeMeasurement = other.gyroscopeMeasurement?.copy()
    }

    /**
     * Creates a new copy of this synced sensor measurement.
     *
     * @return a new copy of this synced sensor measurement.
     */
    override fun copy(): AccelerometerGravityAndGyroscopeSyncedSensorMeasurement {
        val result = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
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
    override fun toNedOrThrow(result: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) {
        // check requirements
        val accelerometerMeasurement = this.accelerometerMeasurement
        if (accelerometerMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                accelerometerMeasurement
            )
        }
        val gravityMeasurement = this.gravityMeasurement
        if (gravityMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                gravityMeasurement
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
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toNedOrThrow(resultAccelerometerMeasurement)
        }
        val resultGravityMeasurement = result.gravityMeasurement
        if (resultGravityMeasurement != null) {
            gravityMeasurement?.toNedOrThrow(resultGravityMeasurement)
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
    override fun toNedOrThrow(): AccelerometerGravityAndGyroscopeSyncedSensorMeasurement {
        val output = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
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
    override fun toEnuOrThrow(result: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) {
        // check requirements
        val accelerometerMeasurement = this.accelerometerMeasurement
        if (accelerometerMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                accelerometerMeasurement
            )
        }
        val gravityMeasurement = this.gravityMeasurement
        if (gravityMeasurement != null) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                gravityMeasurement
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
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toEnuOrThrow(resultAccelerometerMeasurement)
        }
        val resultGravityMeasurement = result.gravityMeasurement
        if (resultGravityMeasurement != null) {
            gravityMeasurement?.toEnuOrThrow(resultGravityMeasurement)
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
    override fun toEnuOrThrow(): AccelerometerGravityAndGyroscopeSyncedSensorMeasurement {
        val output = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
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
    override fun toNed(result: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) {
        val accelerometerMeasurement = this.accelerometerMeasurement
        val gravityMeasurement = this.gravityMeasurement
        val gyroscopeMeasurement = this.gyroscopeMeasurement

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toNed(resultAccelerometerMeasurement)
        }
        val resultGravityMeasurement = result.gravityMeasurement
        if (resultGravityMeasurement != null) {
            gravityMeasurement?.toNed(resultGravityMeasurement)
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
    override fun toNed(): AccelerometerGravityAndGyroscopeSyncedSensorMeasurement {
        val output = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
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
    override fun toEnu(result: AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) {
        val accelerometerMeasurement = this.accelerometerMeasurement
        val gravityMeasurement = this.gravityMeasurement
        val gyroscopeMeasurement = this.gyroscopeMeasurement

        // copy current values (in case they are not null)
        result.copyFrom(this)

        // convert measurements
        val resultAccelerometerMeasurement = result.accelerometerMeasurement
        if (resultAccelerometerMeasurement != null) {
            accelerometerMeasurement?.toEnu(resultAccelerometerMeasurement)
        }
        val resultGravityMeasurement = result.gravityMeasurement
        if (resultGravityMeasurement != null) {
            gravityMeasurement?.toEnu(resultGravityMeasurement)
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
    override fun toEnu(): AccelerometerGravityAndGyroscopeSyncedSensorMeasurement {
        val output = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
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
        return super.equals(other) && (other is AccelerometerGravityAndGyroscopeSyncedSensorMeasurement) &&
                accelerometerMeasurement == other.accelerometerMeasurement &&
                gravityMeasurement == other.gravityMeasurement &&
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
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement
        )
    }
}