/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.Quaternion

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
 */
class AttitudeSensorMeasurement(
    var attitude: Quaternion = Quaternion(),
    var headingAccuracy: Float? = null,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null,
    var sensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE
) : SensorMeasurement<AttitudeSensorMeasurement>(timestamp, accuracy) {

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
}