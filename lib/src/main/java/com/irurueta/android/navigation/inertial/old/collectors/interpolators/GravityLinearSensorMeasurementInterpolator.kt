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

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement

/**
 * Gravity linear interpolator.
 *
 * @property copyIfNotInitialized true indicates that measurement is copied into result if
 * interpolator is not completely initialized.
 */
class GravityLinearSensorMeasurementInterpolator(copyIfNotInitialized: Boolean = true) :
    LinearSensorMeasurementInterpolator<GravitySensorMeasurement>(copyIfNotInitialized),
    GravitySensorMeasurementInterpolator {

    /**
     * Oldest pushed measurement.
     */
    override val measurement0 = GravitySensorMeasurement()

    /**
     * Newest pushed measurement.
     */
    override val measurement1 = GravitySensorMeasurement()

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
        measurement0: GravitySensorMeasurement,
        measurement1: GravitySensorMeasurement,
        factor: Double,
        timestamp: Long,
        result: GravitySensorMeasurement
    ) {
        result.gx = interpolate(measurement0.gx, measurement1.gx, factor)
        result.gy = interpolate(measurement0.gy, measurement1.gy, factor)
        result.gz = interpolate(measurement0.gz, measurement1.gz, factor)
    }
}