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

import com.irurueta.android.navigation.inertial.collectors.GravitySensorMeasurement

/**
 * Gravity quadratic interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class GravityQuadraticSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    QuadraticSensorMeasurementInterpolator<GravitySensorMeasurement>(copyIfNotInitialized),
    GravitySensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = GravitySensorMeasurement()

    /**
     * Pushed measurement between oldest and newest ones.
     */
    override val measurement1 = GravitySensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement2 = GravitySensorMeasurement()

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
        measurement0: GravitySensorMeasurement,
        measurement1: GravitySensorMeasurement,
        measurement2: GravitySensorMeasurement,
        x: Double,
        factor: Double,
        timestamp: Long,
        result: GravitySensorMeasurement
    ) {
        result.gx = interpolate(measurement0.gx, measurement1.gx, measurement2.gx, x, factor)
        result.gy = interpolate(measurement0.gy, measurement1.gy, measurement2.gy, x, factor)
        result.gz = interpolate(measurement0.gz, measurement1.gz, measurement2.gz, x, factor)
    }
}