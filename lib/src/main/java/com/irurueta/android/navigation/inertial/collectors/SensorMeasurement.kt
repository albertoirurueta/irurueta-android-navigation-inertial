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
 * Base class for sensor measurements.
 *
 * @property timestamp relative timestamp in nanoseconds at which the measurement was made. Each
 * measurement will be monotonically increasing using the same time base as
 * [android.os.SystemClock.elapsedRealtimeNanos].
 * @property accuracy sensor accuracy.
 */
abstract class SensorMeasurement<T : SensorMeasurement<T>>(
    var timestamp: Long,
    var accuracy: SensorAccuracy?
) {
    /**
     * Copies values from provided measurement.
     *
     * @param other other measurement to copy from.
     */
    open fun copyFrom(other: T) {
        timestamp = other.timestamp
        accuracy = other.accuracy
    }

    /**
     * Copies values to provided measurement.
     *
     * @param result instance where result will be stored.
     */
    fun copyTo(result: T) {
        @Suppress("UNCHECKED_CAST")
        result.copyFrom(this as T)
    }

    /**
     * Creates a new copy of this measurement.
     *
     * @return a new copy of this measurement.
     */
    abstract fun copy(): T
}