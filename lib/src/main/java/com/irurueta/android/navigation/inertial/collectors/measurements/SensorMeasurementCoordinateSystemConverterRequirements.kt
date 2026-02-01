/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.collectors.measurements

/**
 * Contains helper methods to check requirements for converting sensor measurements between
 * NED (North-East-Down) and ENU (East-North-Up) coordinates systems.
 *
 * @see SensorMeasurement for more information about sensor measurements.
 */
object SensorMeasurementCoordinateSystemConverterRequirements {

    /**
     * Ensures provided measurement is expressed in ENU coordinates system.
     *
     * @param measurement input measurement to be checked.
     * @throws IllegalArgumentException if measurement is not expressed in ENU coordinates
     * system.
     */
    fun requireENUSensorMeasurement(measurement: SensorMeasurement<*>) {
        if (measurement.sensorCoordinateSystem != SensorCoordinateSystem.ENU) {
            throw IllegalArgumentException(
                "Input measurement is not expressed in ENU coordinates system"
            )
        }
    }

    /**
     * Ensures provided measurement is expressed in NED coordinates system.
     *
     * @param measurement input measurement to be checked.
     * @throws IllegalArgumentException if measurement is not expressed in NED coordinates
     * system.
     */
    fun requireNEDSensorMeasurement(measurement: SensorMeasurement<*>) {
        if (measurement.sensorCoordinateSystem != SensorCoordinateSystem.NED) {
            throw IllegalArgumentException(
                "Input measurement is not expressed in NED coordinates system"
            )
        }
    }
}