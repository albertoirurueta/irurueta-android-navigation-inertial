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
package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement

/**
 * Accelerometer quadratic interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class AccelerometerQuadraticSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    QuadraticSensorMeasurementInterpolator<AccelerometerSensorMeasurement>(copyIfNotInitialized),
    AccelerometerSensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = AccelerometerSensorMeasurement()

    /**
     * Pushed measurement between oldest and newest ones.
     */
    override val measurement1 = AccelerometerSensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement2 = AccelerometerSensorMeasurement()

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
    override fun interpolate(
        measurement0: AccelerometerSensorMeasurement,
        measurement1: AccelerometerSensorMeasurement,
        measurement2: AccelerometerSensorMeasurement,
        x: Double,
        factor: Double,
        timestamp: Long,
        result: AccelerometerSensorMeasurement
    ) {
        result.ax = interpolate(measurement0.ax, measurement1.ax, measurement2.ax, x, factor)
        result.ay = interpolate(measurement0.ay, measurement1.ay, measurement2.ay, x, factor)
        result.az = interpolate(measurement0.az, measurement1.az, measurement2.az, x, factor)

        val bx0 = measurement0.bx
        val bx1 = measurement1.bx
        val bx2 = measurement2.bx
        result.bx = if (bx0 != null && bx1 != null && bx2 != null) {
            interpolate(bx0, bx1, bx2, x, factor)
        } else {
            null
        }

        val by0 = measurement0.by
        val by1 = measurement1.by
        val by2 = measurement2.by
        result.by = if (by0 != null && by1 != null && by2 != null) {
            interpolate(by0, by1, by2, x, factor)
        } else {
            null
        }

        val bz0 = measurement0.bz
        val bz1 = measurement1.bz
        val bz2 = measurement2.bz
        result.bz = if (bz0 != null && bz1 != null && bz2 != null) {
            interpolate(bz0, bz1, bz2, x, factor)
        } else {
            null
        }
    }
}