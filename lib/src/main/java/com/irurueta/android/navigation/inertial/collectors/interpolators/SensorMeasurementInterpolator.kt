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

package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import kotlin.math.abs

/**
 * Interpolates measurements.
 */
abstract class SensorMeasurementInterpolator<T : SensorMeasurement<T>> {

    /**
     * Finds the closest measurement to desired timestamp.
     *
     * @param measurements a collection or buffer of measurements where the search is performed.
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the measurements in the buffer).
     * @return the closest measurement (if available)
     */
    fun findClosest(
        measurements: Collection<T>,
        targetNanoSeconds: Long
    ): T? {
        val (previousMeasurement, nextMeasurement) = findPreviousAndNextMeasurements(
            measurements,
            targetNanoSeconds
        )

        if (previousMeasurement != null && nextMeasurement != null) {
            // pick closest
            val prevTimestamp = previousMeasurement.timestamp
            val nextTimestamp = nextMeasurement.timestamp
            return if (abs(targetNanoSeconds - prevTimestamp) < abs(targetNanoSeconds - nextTimestamp)) {
                previousMeasurement
            } else {
                nextMeasurement
            }

        } else if (previousMeasurement == null && nextMeasurement != null) {
            return nextMeasurement
        } else if (previousMeasurement != null) {
            return previousMeasurement
        } else {
            return null
        }
    }

    /**
     * Interpolates buffered sensor measurements according to desired timestamp.
     * This method finds the closest measurements to desired timestamp and computes a linear
     * interpolation between them.
     *
     * @param measurements a collection or buffer of measurements where the search is performed.
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the measurements in the buffer).
     * @param result the resulting sensor measurement.
     * @return true if interpolation was performed, false otherwise.
     */
    fun interpolate(
        measurements: Collection<T>,
        targetNanoSeconds: Long,
        result: T
    ): Boolean {
        val (previousMeasurement, nextMeasurement) = findPreviousAndNextMeasurements(
            measurements,
            targetNanoSeconds
        )

        if (previousMeasurement != null && nextMeasurement != null) {
            // Linear interpolation
            val alpha = if (nextMeasurement.timestamp == previousMeasurement.timestamp) {
                0.0f
            } else {
                (targetNanoSeconds - previousMeasurement.timestamp).toFloat() /
                        (nextMeasurement.timestamp - previousMeasurement.timestamp).toFloat()
            }

            interpolate(
                previousMeasurement,
                nextMeasurement,
                alpha,
                targetNanoSeconds,
                result
            )
            return true
        } else if (previousMeasurement == null && nextMeasurement != null) {
            nextMeasurement.copyTo(result)
            result.timestamp = targetNanoSeconds
            return true
        } else if (previousMeasurement != null) {
            previousMeasurement.copyTo(result)
            result.timestamp = targetNanoSeconds
            return true
        } else {
            return false
        }
    }

    /**
     * Interpolates 2 measurements.
     *
     * @param measurement1 the first measurement.
     * @param measurement2 the second measurement.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the other measurements).
     * @param result the resulting measurement.
     */
    protected abstract fun interpolate(
        measurement1: T,
        measurement2: T,
        alpha: Float,
        targetNanoSeconds: Long,
        result: T
    )

    /**
     * Interpolates between two values.
     *
     * @param value1 the first value.
     * @param value2 the second value.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @return the interpolated value.
     */
    protected fun interpolate(value1: Float, value2: Float, alpha: Float): Float {
        return value1 + alpha * (value2 - value1)
    }

    /**
     * Finds within a buffer or collection of measurements, the measurements placed immediately
     * before and after respect to desired timestamp.
     *
     * @param measurements a collection or buffer of measurements where the search is performed.
     */
    protected fun findPreviousAndNextMeasurements(
        measurements: Collection<T>,
        targetNanoSeconds: Long
    ): Pair<T?, T?> {

        var previousMeasurement: T? = null
        var nextMeasurement: T? = null
        var previousTimestamp = Long.MIN_VALUE
        var nextTimestamp = Long.MAX_VALUE

        for (s in measurements) {
            if (s.timestamp in previousTimestamp..targetNanoSeconds) {
                previousMeasurement = s
                previousTimestamp = s.timestamp
            }
            if (s.timestamp in targetNanoSeconds..nextTimestamp) {
                nextMeasurement = s
                nextTimestamp = s.timestamp
            }
        }

        return if (previousMeasurement == null) {
            if (nextMeasurement == null) {
                Pair(null, null)
            } else {
                Pair(null, nextMeasurement)
            }
        } else {
            if (nextMeasurement == null) {
                Pair(previousMeasurement, null)
            } else {
                Pair(previousMeasurement, nextMeasurement)
            }
        }
    }
}