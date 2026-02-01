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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement

/**
 * Magnetometer quadratic interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class MagnetometerQuadraticSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    QuadraticSensorMeasurementInterpolator<MagnetometerSensorMeasurement>(copyIfNotInitialized),
    MagnetometerSensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = MagnetometerSensorMeasurement()

    /**
     * Pushed measurement between oldest and newest ones.
     */
    override val measurement1 = MagnetometerSensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement2 = MagnetometerSensorMeasurement()

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
        measurement0: MagnetometerSensorMeasurement,
        measurement1: MagnetometerSensorMeasurement,
        measurement2: MagnetometerSensorMeasurement,
        x: Double,
        factor: Double,
        timestamp: Long,
        result: MagnetometerSensorMeasurement
    ) {
        result.bx = interpolate(measurement0.bx, measurement1.bx, measurement2.bx, x, factor)
        result.by = interpolate(measurement0.by, measurement1.by, measurement2.by, x, factor)
        result.bz = interpolate(measurement0.bz, measurement1.bz, measurement2.bz, x, factor)

        val hardIronX0 = measurement0.hardIronX
        val hardIronX1 = measurement1.hardIronX
        val hardIronX2 = measurement2.hardIronX
        result.hardIronX = if (hardIronX0 != null && hardIronX1 != null && hardIronX2 != null) {
            interpolate(hardIronX0, hardIronX1, hardIronX2, x, factor)
        } else {
            null
        }

        val hardIronY0 = measurement0.hardIronY
        val hardIronY1 = measurement1.hardIronY
        val hardIronY2 = measurement2.hardIronY
        result.hardIronY = if (hardIronY0 != null && hardIronY1 != null && hardIronY2 != null) {
            interpolate(hardIronY0, hardIronY1, hardIronY2, x, factor)
        } else {
            null
        }

        val hardIronZ0 = measurement0.hardIronZ
        val hardIronZ1 = measurement1.hardIronZ
        val hardIronZ2 = measurement2.hardIronZ
        result.hardIronZ = if (hardIronZ0 != null && hardIronZ1 != null && hardIronZ2 != null) {
            interpolate(hardIronZ0, hardIronZ1, hardIronZ2, x, factor)
        } else {
            null
        }
    }
}