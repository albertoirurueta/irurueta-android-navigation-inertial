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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement

/**
 * Accelerometer linear interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class AccelerometerLinearSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    LinearSensorMeasurementInterpolator<AccelerometerSensorMeasurement>(copyIfNotInitialized),
    AccelerometerSensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = AccelerometerSensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement1 = AccelerometerSensorMeasurement()

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
        measurement0: AccelerometerSensorMeasurement,
        measurement1: AccelerometerSensorMeasurement,
        factor: Double,
        timestamp: Long,
        result: AccelerometerSensorMeasurement
    ) {
        result.ax = interpolate(measurement0.ax, measurement1.ax, factor)
        result.ay = interpolate(measurement0.ay, measurement1.ay, factor)
        result.az = interpolate(measurement0.az, measurement1.az, factor)

        val bx0 = measurement0.bx
        val bx1 = measurement1.bx
        result.bx = if (bx0 != null && bx1 != null) interpolate(bx0, bx1, factor) else null

        val by0 = measurement0.by
        val by1 = measurement1.by
        result.by = if (by0 != null && by1 != null) interpolate(by0, by1, factor) else null

        val bz0 = measurement0.bz
        val bz1 = measurement1.bz
        result.bz = if (bz0 != null && bz1 != null) interpolate(bz0, bz1, factor) else null
    }
}