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

import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorMeasurement

/**
 * Magnetometer linear interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class MagnetometerLinearSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    LinearSensorMeasurementInterpolator<MagnetometerSensorMeasurement>(copyIfNotInitialized),
    MagnetometerSensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = MagnetometerSensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement1 = MagnetometerSensorMeasurement()

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
    override fun interpolate(
        measurement0: MagnetometerSensorMeasurement,
        measurement1: MagnetometerSensorMeasurement,
        factor: Double,
        timestamp: Long,
        result: MagnetometerSensorMeasurement
    ) {
        result.bx = interpolate(measurement0.bx, measurement1.bx, factor)
        result.by = interpolate(measurement0.by, measurement1.by, factor)
        result.bz = interpolate(measurement0.bz, measurement1.bz, factor)

        val hardIronX0 = measurement0.hardIronX
        val hardIronX1 = measurement1.hardIronX
        result.hardIronX = if (hardIronX0 != null && hardIronX1 != null) {
            interpolate(hardIronX0, hardIronX1, factor)
        } else {
            null
        }

        val hardIronY0 = measurement0.hardIronY
        val hardIronY1 = measurement1.hardIronY
        result.hardIronY = if (hardIronY0 != null && hardIronY1 != null) {
            interpolate(hardIronY0, hardIronY1, factor)
        } else {
            null
        }

        val hardIronZ0 = measurement0.hardIronZ
        val hardIronZ1 = measurement1.hardIronZ
        result.hardIronZ = if (hardIronZ0 != null && hardIronZ1 != null) {
            interpolate(hardIronZ0, hardIronZ1, factor)
        } else {
            null
        }
    }
}