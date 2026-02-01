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

package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurementCoordinateSystemConverterRequirements

/**
 * Converts magnetometer sensor measurements between different coordinate systems.
 *
 * By default, sensor measurements are expressed in ENU (East-North-Up) coordinates system.
 * However, in some cases it might be required to convert them to NED (North-East-Down)
 * coordinates system.
 *
 * The conversion is performed by swapping x and y coordinates and negating z coordinate.
 *
 * For example, given a measurement (ax, ay, az) in ENU coordinates system, the corresponding
 * measurement in NED coordinates system would be (ay, ax, -az).
 */
object MagnetometerSensorMeasurementCoordinateSystemConverter :
    SensorMeasurementCoordinateSystemConverter<MagnetometerSensorMeasurement> {

    /**
     * Converts provided magnetometer sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input magnetometer measurement to be converted.
     * @param output instance where converted magnetometer measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(
        input: MagnetometerSensorMeasurement,
        output: MagnetometerSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
            input
        )
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.NED
    }

    /**
     * Converts provided magnetometer sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input magnetometer measurement to be converted.
     * @return a new instance containing converted magnetometer measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(input: MagnetometerSensorMeasurement): MagnetometerSensorMeasurement {
        val output = MagnetometerSensorMeasurement()
        toNedOrThrow(input, output)
        return output
    }

    /**
     * Converts provided magnetometer sensor measurement to ENU coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input magnetometer measurement to be converted.
     * @param output instance where converted magnetometer measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(
        input: MagnetometerSensorMeasurement,
        output: MagnetometerSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
            input)
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
    }

    /**
     * Converts provided magnetometer sensor measurement to ENU coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input magnetometer measurement to be converted.
     * @return a new instance containing converted magnetometer measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(input: MagnetometerSensorMeasurement): MagnetometerSensorMeasurement {
        val output = MagnetometerSensorMeasurement()
        toEnuOrThrow(input, output)
        return output
    }

    /**
     * Converts provided magnetometer sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     * If the measurement is already expressed in NED coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toNed(
        input: MagnetometerSensorMeasurement,
        output: MagnetometerSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.NED) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.NED
        }
    }

    /**
     * Converts provided magnetometer sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     * If the measurement is already expressed in NED coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun toNed(input: MagnetometerSensorMeasurement): MagnetometerSensorMeasurement {
        val output = MagnetometerSensorMeasurement()
        toNed(input, output)
        return output
    }

    /**
     * Converts provided magnetometer sensor measurement to ENU coordinates system and stores the
     * result in the provided output instance.
     * If the measurement is already expressed in ENU coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toEnu(
        input: MagnetometerSensorMeasurement,
        output: MagnetometerSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.ENU) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
        }
    }

    /**
     * Converts provided magnetometer sensor measurement to ENU coordinates system and returns a
     * new instance containing the result.
     * If the measurement is already expressed in ENU coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(input: MagnetometerSensorMeasurement): MagnetometerSensorMeasurement {
        val output = MagnetometerSensorMeasurement()
        toEnu(input, output)
        return output
    }

    /**
     * Performs internal conversion of magnetometer sensor measurements by swapping x and y
     * coordinates and negating z coordinate.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    private fun internalConvert(
        input: MagnetometerSensorMeasurement,
        output: MagnetometerSensorMeasurement
    ) {
        output.bx = input.by
        output.by = input.bx
        output.bz = -input.bz

        output.hardIronX = input.hardIronY
        output.hardIronY = input.hardIronX
        val inputHardIronZ = input.hardIronZ
        output.hardIronZ = if (inputHardIronZ != null) -inputHardIronZ else null

        output.timestamp = input.timestamp
        output.accuracy = input.accuracy
        output.sensorType = input.sensorType
    }
}