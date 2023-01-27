/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.collectors

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
    var sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
) : SensorMeasurement<MagnetometerSensorMeasurement>(timestamp, accuracy) {

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
}