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
 */
class AccelerometerSensorMeasurement(
    var ax: Float = 0.0f,
    var ay: Float = 0.0f,
    var az: Float = 0.0f,
    var bx: Float? = null,
    var by: Float? = null,
    var bz: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null
) : SensorMeasurement<AccelerometerSensorMeasurement>(timestamp, accuracy) {

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
}