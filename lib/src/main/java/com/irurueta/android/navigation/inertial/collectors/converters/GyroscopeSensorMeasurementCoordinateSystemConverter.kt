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

import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurementCoordinateSystemConverterRequirements

/**
 * Converts gyroscope sensor measurements between different coordinate systems.
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
object GyroscopeSensorMeasurementCoordinateSystemConverter :
    SensorMeasurementCoordinateSystemConverter<GyroscopeSensorMeasurement> {

    /**
     * Converts provided gyroscope sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input gyroscope measurement to be converted.
     * @param output instance where converted gyroscope measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(
        input: GyroscopeSensorMeasurement,
        output: GyroscopeSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
            input
        )
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.NED
    }

    /**
     * Converts provided gyroscope sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input gyroscope measurement to be converted.
     * @return a new instance containing converted gyroscope measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(input: GyroscopeSensorMeasurement): GyroscopeSensorMeasurement {
        val output = GyroscopeSensorMeasurement()
        toNedOrThrow(input, output)
        return output
    }

    /**
     * Converts provided gyroscope sensor measurement to ENU coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input gyroscope measurement to be converted.
     * @param output instance where converted gyroscope measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(
        input: GyroscopeSensorMeasurement,
        output: GyroscopeSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
            input)
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
    }

    /**
     * Converts provided gyroscope sensor measurement to ENU coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input gyroscope measurement to be converted.
     * @return a new instance containing converted gyroscope measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(input: GyroscopeSensorMeasurement): GyroscopeSensorMeasurement {
        val output = GyroscopeSensorMeasurement()
        toEnuOrThrow(input, output)
        return output
    }

    /**
     * Converts provided gyroscope sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     * If the measurement is already expressed in NED coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toNed(
        input: GyroscopeSensorMeasurement,
        output: GyroscopeSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.NED) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.NED
        }
    }

    /**
     * Converts provided gyroscope sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     * If the measurement is already expressed in NED coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun toNed(input: GyroscopeSensorMeasurement): GyroscopeSensorMeasurement {
        val output = GyroscopeSensorMeasurement()
        toNed(input, output)
        return output
    }

    /**
     * Converts provided gyroscope sensor measurement to ENU coordinates system and stores the
     * result in the provided output instance.
     * If the measurement is already expressed in ENU coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toEnu(
        input: GyroscopeSensorMeasurement,
        output: GyroscopeSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.ENU) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
        }
    }

    /**
     * Converts provided gyroscope sensor measurement to ENU coordinates system and returns a new
     * instance containing the result.
     * If the measurement is already expressed in ENU coordinates system, a copy of input
     * measurement is returned.
     */
    override fun toEnu(input: GyroscopeSensorMeasurement): GyroscopeSensorMeasurement {
        val output = GyroscopeSensorMeasurement()
        toEnu(input, output)
        return output
    }

    /**
     * Performs internal conversion of gyroscope sensor measurements by swapping x and y
     * coordinates and negating z coordinate.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    private fun internalConvert(
        input: GyroscopeSensorMeasurement,
        output: GyroscopeSensorMeasurement
    ) {
        output.wx = input.wy
        output.wy = input.wx
        output.wz = -input.wz

        output.bx = input.by
        output.by = input.bx
        val inputBz = input.bz
        output.bz = if (inputBz != null) -inputBz else null

        output.timestamp = input.timestamp
        output.accuracy = input.accuracy
        output.sensorType = input.sensorType
    }
}