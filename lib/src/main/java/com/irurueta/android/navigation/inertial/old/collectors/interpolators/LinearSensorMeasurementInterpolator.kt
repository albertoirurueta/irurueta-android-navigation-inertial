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
package com.irurueta.android.navigation.inertial.old.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement

/**
 * Implementation of a linear interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 * @param <M> Type of sensor measurement.
 */
abstract class LinearSensorMeasurementInterpolator<M : SensorMeasurement<M>>(val copyIfNotInitialized: Boolean) :
    SensorMeasurementInterpolator<LinearSensorMeasurementInterpolator<M>, M> {

    /**
     * Oldest pushed measurement.
     */
    protected abstract val measurement0: M

    /**
     * Newest pushed measurement.
     */
    protected abstract val measurement1: M

    /**
     * Indicates whether [measurement0] exists.
     */
    private var hasMeasurement0 = false

    /**
     * Indicates whether [measurement1] exists.
     */
    private var hasMeasurement1 = false

    /**
     * Pushes previous measurement into collection of processed measurements.
     *
     * @param previousMeasurement previous measurement to be pushed.
     */
    override fun push(previousMeasurement: M) {
        if (hasMeasurement1) {
            measurement0.copyFrom(measurement1)
            hasMeasurement0 = true
        }
        measurement1.copyFrom(previousMeasurement)
        hasMeasurement1 = true
    }

    /**
     * Interpolates provided current measurement.
     *
     * @param currentMeasurement current measurement to be interpolated with previous ones.
     * @param timestamp timestamp to perform interpolation respect previous measurements.
     * @param result instance where result of interpolation will be stored.
     * @return true if interpolation has been computed and result instance contains expected value,
     * false if result of interpolation must be discarded.
     */
    override fun interpolate(currentMeasurement: M, timestamp: Long, result: M): Boolean {
        if (!hasMeasurement0 || !hasMeasurement1) {
            return if (copyIfNotInitialized) {
                result.copyFrom(currentMeasurement)
                result.timestamp = timestamp
                true
            } else {
                false
            }
        }


        val timestamp0 = measurement0.timestamp
        val timestamp1 = measurement1.timestamp
        val diff = timestamp1 - timestamp0

        val currentDiff = timestamp - timestamp0

        val value = currentDiff.toDouble() / diff.toDouble()
        interpolate(measurement0, measurement1, value, timestamp, result)

        result.accuracy = measurement0.accuracy
        result.timestamp = timestamp

        return true
    }

    /**
     * Resets this interpolator.
     */
    override fun reset() {
        hasMeasurement0 = false
        hasMeasurement1 = false
    }

    /**
     * Linearly interpolates [measurement0] and [measurement1] with provided factor and stores
     * the result of interpolation into provided result measurement.
     *
     * @param measurement0 oldest pushed measurement to linearly interpolate from.
     * @param measurement1 newest pushed measurement to linearly interpolate from.
     * @param factor factor to interpolate measurements with.
     * @param timestamp timestamp to be set into result instance.
     * @param result instance where result of interpolation is stored.
     */
    protected abstract fun interpolate(
        measurement0: M,
        measurement1: M,
        factor: Double,
        timestamp: Long,
        result: M
    )

    /**
     * Linearly interpolates a float value.
     *
     * @param value0 oldest value to linearly interpolate from.
     * @param value1 newest value to linearly interpolate from.
     * @param factor factor to interpolate values with.
     * @return interpolated value.
     */
    protected fun interpolate(value0: Float, value1: Float, factor: Double): Float {
        return (value0.toDouble() + factor * (value1.toDouble() - value0.toDouble())).toFloat()
    }
}