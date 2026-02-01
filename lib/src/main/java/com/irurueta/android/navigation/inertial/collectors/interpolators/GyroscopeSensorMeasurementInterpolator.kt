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

import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement

/**
 * Interpolates gyroscope measurements.
 */
class GyroscopeSensorMeasurementInterpolator() :
    SensorMeasurementInterpolator<GyroscopeSensorMeasurement>() {

    /**
     * Interpolates between two gyroscope measurements.
     *
     * @param measurement1 the first gyroscope measurement.
     * @param measurement2 the second gyroscope measurement.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the other measurements).
     * @param result the resulting gyroscope measurement.
     */
    override fun interpolate(
        measurement1: GyroscopeSensorMeasurement,
        measurement2: GyroscopeSensorMeasurement,
        alpha: Float,
        targetNanoSeconds: Long,
        result: GyroscopeSensorMeasurement
    ) {
        val wx1 = measurement1.wx
        val wy1 = measurement1.wy
        val wz1 = measurement1.wz
        val bx1 = measurement1.bx
        val by1 = measurement1.by
        val bz1 = measurement1.bz

        val wx2 = measurement2.wx
        val wy2 = measurement2.wy
        val wz2 = measurement2.wz
        val bx2 = measurement2.bx
        val by2 = measurement2.by
        val bz2 = measurement2.bz

        val wx = interpolate(wx1, wx2, alpha)
        val wy = interpolate(wy1, wy2, alpha)
        val wz = interpolate(wz1, wz2, alpha)
        val bx = if (bx1 != null && bx2 != null) {
            interpolate(bx1, bx2, alpha)
        } else {
            null
        }
        val by = if (by1 != null && by2 != null) {
            interpolate(by1, by2, alpha)
        } else {
            null
        }
        val bz = if (bz1 != null && bz2 != null) {
            interpolate(bz1, bz2, alpha)
        } else {
            null
        }

        result.wx = wx
        result.wy = wy
        result.wz = wz
        result.bx = bx
        result.by = by
        result.bz = bz
        result.timestamp = targetNanoSeconds
        result.accuracy = measurement1.accuracy
        result.sensorType = measurement1.sensorType
        result.sensorCoordinateSystem = measurement1.sensorCoordinateSystem
    }
}