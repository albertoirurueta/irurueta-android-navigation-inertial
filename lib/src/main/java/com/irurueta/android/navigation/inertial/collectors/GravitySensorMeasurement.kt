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
 */
class GravitySensorMeasurement(
    var gx: Float = 0.0f,
    var gy: Float = 0.0f,
    var gz: Float = 0.0f,
    timestamp: Long = 0L,
    accuracy: SensorAccuracy? = null
) : SensorMeasurement<GravitySensorMeasurement>(timestamp, accuracy) {

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
}