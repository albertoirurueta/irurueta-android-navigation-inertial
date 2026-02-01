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

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement

/**
 * Interpolates gravity measurements.
 */
class GravitySensorMeasurementInterpolator() :
    SensorMeasurementInterpolator<GravitySensorMeasurement>() {

    /**
     * Interpolates between two gravity measurements.
     *
     * @param measurement1 the first gravity measurement.
     * @param measurement2 the second gravity measurement.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the other measurements).
     * @param result the resulting gravity measurement.
     */
    override fun interpolate(
        measurement1: GravitySensorMeasurement,
        measurement2: GravitySensorMeasurement,
        alpha: Float,
        targetNanoSeconds: Long,
        result: GravitySensorMeasurement
    ) {
        val gx1 = measurement1.gx
        val gy1 = measurement1.gy
        val gz1 = measurement1.gz

        val gx2 = measurement2.gx
        val gy2 = measurement2.gy
        val gz2 = measurement2.gz

        val gx = interpolate(gx1, gx2, alpha)
        val gy = interpolate(gy1, gy2, alpha)
        val gz = interpolate(gz1, gz2, alpha)

        result.gx = gx
        result.gy = gy
        result.gz = gz
        result.timestamp = targetNanoSeconds
        result.accuracy = measurement1.accuracy
        result.sensorCoordinateSystem = measurement1.sensorCoordinateSystem
    }
}