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
 * Implementation of a quadratic interpolator.
 */
abstract class QuadraticSensorMeasurementInterpolator<M : SensorMeasurement<M>>(val copyIfNotInitialized: Boolean) :
    SensorMeasurementInterpolator<LinearSensorMeasurementInterpolator<M>, M> {

    /**
     * Oldest pushed measurement.
     */
    protected abstract val measurement0: M

    /**
     * Pushed measurement between oldest and newest ones.
     */
    protected abstract val measurement1: M

    /**
     * Newest pushed measurement.
     */
    protected abstract val measurement2: M

    /**
     * Indicates whether [measurement0] exists.
     */
    private var hasMeasurement0 = false

    /**
     * Indicates whether [measurement1] exists.
     */
    private var hasMeasurement1 = false

    /**
     * Indicates whether [measurement2] exists.
     */
    private var hasMeasurement2 = false

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
        if (hasMeasurement2) {
            measurement1.copyFrom(measurement2)
            hasMeasurement1 = true
        }
        measurement2.copyFrom(previousMeasurement)
        hasMeasurement2 = true
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
        if (!hasMeasurement0 || !hasMeasurement1 || !hasMeasurement2) {
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
        val timestamp2 = measurement2.timestamp

        val diff = (timestamp1 - timestamp0).toDouble()

        val diff2 = timestamp2 - timestamp0
        val x = diff2.toDouble() / diff

        val currentDiff = timestamp - timestamp0
        val factor = currentDiff.toDouble() / diff

        interpolate(measurement0, measurement1, measurement2, x, factor, timestamp, result)

        result.accuracy = measurement0.accuracy
        result.timestamp = timestamp

        return true
    }

    /**
     * Quadratically interpolates [measurement0], [measurement1] and [measurement2], by fitting a
     * second degree polynomial, assuming that [measurement2] is at normalized x value, and
     * providing a factor to interpolate new measurements.
     *
     * @param measurement0 oldest pushed measurement to quadratically interpolate from.
     * @param measurement1 pushed measurement between oldest and newest to quadratically interpolate
     * from.
     * @param measurement2 newest pushed measurement to quadratically interpolate from.
     * @param x normalized x value for [measurement2] to fit second degree polynomial.
     * @param factor value to evaluate fitted polynomial to interpolate new measurements.
     * @param timestamp timestamp to be set into result instance.
     * @param result instance where result of interpolation is stored.
     */
    protected abstract fun interpolate(
        measurement0: M,
        measurement1: M,
        measurement2: M,
        x: Double,
        factor: Double,
        timestamp: Long,
        result: M
    )

    /**
     * Quadratically interpolates a float value.
     *
     * @param value0 oldest value to quadratically interpolate from.
     * @param value1 value between oldest and newest to quadratically interpolate from.
     * @param value2 = newest value to quadratically interpolate from.
     * @param x normalized x value for [measurement2] to fit second degree polynomial.
     * @param factor value to evaluate fitted polynomial to interpolate new measurements.
     * @return interpolated value.
     */
    protected fun interpolate(
        value0: Float,
        value1: Float,
        value2: Float,
        x: Double,
        factor: Double
    ): Float {
        // This interpolator attempts to estimate parameters of a second degree polynomial:
        // y = f(x) = a*x^2 + b*x + c

        // measurement 0 will always be placed at x = 0
        // measurement 1 will always be placed at x = 1
        // measurement 2 will be placed at an arbitrary x value

        // Consequently:
        // y0 = f(0) = c
        // y1 = f(1) = a + b + c = a + b + y0
        // y2 = f(factor2) = a*x^2 + b*x + c = a*x^2  + b*x + y0

        // Taking the second equation, we can obtain b as:
        // b = y1 - y0 - a

        // Replacing this into third equation:
        // y2 = a*x^2 + (y1 - y0 - a)*x + y0 = a*(x^2 - x) + (y1 - y0)*x + y0

        // Then:
        // a = (y2 - (y1 - y0)*x - y0) / (x^2 - x)
        // b = y1 - y0 - a
        // c = y0
        val value0D = value0.toDouble()
        val value1D = value1.toDouble()
        val value2D = value2.toDouble()

        val a = (value2D - (value1D - value0D) * x - value0D) / (x * x - x)
        val b = value1D - value0D - a

        return (a * factor * factor + b * factor + value0).toFloat()
    }

    /**
     * Resets this interpolator.
     */
    override fun reset() {
        hasMeasurement0 = false
        hasMeasurement1 = false
        hasMeasurement2 = false
    }

}