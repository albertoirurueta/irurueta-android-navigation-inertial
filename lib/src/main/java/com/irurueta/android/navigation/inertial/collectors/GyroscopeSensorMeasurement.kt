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
    var sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
) : SensorMeasurement<GyroscopeSensorMeasurement>(timestamp, accuracy) {

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
}