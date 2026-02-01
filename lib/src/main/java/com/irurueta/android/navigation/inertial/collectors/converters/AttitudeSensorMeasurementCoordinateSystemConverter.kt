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

import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurementCoordinateSystemConverterRequirements

/**
 * Converts attitude sensor measurements between different coordinate systems.
 *
 * By default, sensor measurements are expressed in ENU (East-North-Up) coordinates system.
 * However, in some cases it might be required to convert them to NED (North-East-Down)
 * coordinates system.
 */
object AttitudeSensorMeasurementCoordinateSystemConverter :
    SensorMeasurementCoordinateSystemConverter<AttitudeSensorMeasurement> {

    /**
     * Converts provided attitude sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input attitude measurement to be converted.
     * @param output instance where converted attitude measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(
        input: AttitudeSensorMeasurement,
        output: AttitudeSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
            input
        )
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.NED
    }

    /**
     * Converts provided attitude sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input attitude measurement to be converted.
     * @return a new instance containing converted attitude measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in ENU coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toNedOrThrow(input: AttitudeSensorMeasurement): AttitudeSensorMeasurement {
        val output = AttitudeSensorMeasurement()
        toNedOrThrow(input, output)
        return output
    }

    /**
     * Converts provided attitude sensor measurement to ENU coordinates system and stores the
     * result in the provided output instance.
     *
     * @param input input attitude measurement to be converted.
     * @param output instance where converted attitude measurement will be stored.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(
        input: AttitudeSensorMeasurement,
        output: AttitudeSensorMeasurement
    ) {
        SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
            input)
        internalConvert(input, output)
        output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
    }

    /**
     * Converts provided attitude sensor measurement to ENU coordinates system and returns a
     * new instance containing the result.
     *
     * @param input input attitude measurement to be converted.
     * @return a new instance containing converted attitude measurement.
     * @throws IllegalArgumentException if input measurements is not expressed in NED coordinates
     * system.
     */
    @Throws(IllegalArgumentException::class)
    override fun toEnuOrThrow(input: AttitudeSensorMeasurement): AttitudeSensorMeasurement {
        val output = AttitudeSensorMeasurement()
        toEnuOrThrow(input, output)
        return output
    }

    /**
     * Converts provided attitude sensor measurement to NED coordinates system and stores the
     * result in the provided output instance.
     * If the measurement is already expressed in NED coordinates system, input measurement
     * is copied into output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toNed(
        input: AttitudeSensorMeasurement,
        output: AttitudeSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.NED) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.NED
        }
    }

    /**
     * Converts provided attitude sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     * If the measurement is already expressed in NED coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun toNed(input: AttitudeSensorMeasurement): AttitudeSensorMeasurement {
        val output = AttitudeSensorMeasurement()
        toNed(input, output)
        return output
    }

    /**
     * Converts provided attitude sensor measurement to NED coordinates system and returns a
     * new instance containing the result.
     * If the measurement is already expressed in NED coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun toEnu(
        input: AttitudeSensorMeasurement,
        output: AttitudeSensorMeasurement
    ) {
        if (input.sensorCoordinateSystem == SensorCoordinateSystem.ENU) {
            output.copyFrom(input)
        } else {
            internalConvert(input, output)
            output.sensorCoordinateSystem = SensorCoordinateSystem.ENU
        }
    }

    /**
     * Converts provided attitude sensor measurement to ENU coordinates system and returns a new
     * instance containing the result.
     * If the measurement is already expressed in ENU coordinates system, a copy of input
     * measurement is returned.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun toEnu(input: AttitudeSensorMeasurement): AttitudeSensorMeasurement {
        val output = AttitudeSensorMeasurement()
        toEnu(input, output)
        return output
    }

    /**
     * Performs internal conversion of attitude sensor measurements by converting attitude
     * quaternion from ENU to NED or from NED to ENU.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    private fun internalConvert(
        input: AttitudeSensorMeasurement,
        output: AttitudeSensorMeasurement
    ) {
        QuaternionCoordinateSystemConverter.convert(input.attitude, output.attitude)
        output.timestamp = input.timestamp
        output.accuracy = input.accuracy
        output.headingAccuracy = input.headingAccuracy
        output.sensorType = input.sensorType
    }
}