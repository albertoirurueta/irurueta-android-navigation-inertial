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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement

/**
 * Interpolates magnetometer measurements.
 */
class MagnetometerSensorMeasurementInterpolator() :
    SensorMeasurementInterpolator<MagnetometerSensorMeasurement>() {

    /**
     * Interpolates between two magnetometer measurements.
     *
     * @param measurement1 the first magnetometer measurement.
     * @param measurement2 the second magnetometer measurement.
     * @param alpha the interpolation factor (as a value between 0.0f and 1.0f).
     * @param targetNanoSeconds the target timestamp of resulting measurement expressed in
     * nanoseconds (using the same clock as the other measurements).
     * @param result the resulting magnetometer measurement.
     */
    override fun interpolate(
        measurement1: MagnetometerSensorMeasurement,
        measurement2: MagnetometerSensorMeasurement,
        alpha: Float,
        targetNanoSeconds: Long,
        result: MagnetometerSensorMeasurement
    ) {
        val bx1 = measurement1.bx
        val by1 = measurement1.by
        val bz1 = measurement1.bz
        val hardIronX1 = measurement1.hardIronX
        val hardIronY1 = measurement1.hardIronY
        val hardIronZ1 = measurement1.hardIronZ

        val bx2 = measurement2.bx
        val by2 = measurement2.by
        val bz2 = measurement2.bz
        val hardIronX2 = measurement2.hardIronX
        val hardIronY2 = measurement2.hardIronY
        val hardIronZ2 = measurement2.hardIronZ

        val bx = interpolate(bx1, bx2, alpha)
        val by = interpolate(by1, by2, alpha)
        val bz = interpolate(bz1, bz2, alpha)
        val hardIronX = if (hardIronX1 != null && hardIronX2 != null) {
            interpolate(hardIronX1, hardIronX2, alpha)
        } else {
            null
        }
        val hardIronY = if (hardIronY1 != null && hardIronY2 != null) {
            interpolate(hardIronY1, hardIronY2, alpha)
        } else {
            null
        }
        val hardIronZ = if (hardIronZ1 != null && hardIronZ2 != null) {
            interpolate(hardIronZ1, hardIronZ2, alpha)
        } else {
            null
        }

        result.bx = bx
        result.by = by
        result.bz = bz
        result.hardIronX = hardIronX
        result.hardIronY = hardIronY
        result.hardIronZ = hardIronZ
        result.timestamp = targetNanoSeconds
        result.accuracy = measurement1.accuracy
        result.sensorType = measurement1.sensorType
        result.sensorCoordinateSystem = measurement1.sensorCoordinateSystem
    }
}