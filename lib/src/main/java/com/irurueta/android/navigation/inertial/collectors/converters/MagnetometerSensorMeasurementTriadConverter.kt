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
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.units.MagneticFluxDensityUnit

/**
 * Converts [com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement] into [com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad].
 * Conversion keeps into account any available sensor bias data, and preserves coordinates system.
 */
object MagnetometerSensorMeasurementTriadConverter :
    SensorMeasurementTriadConverter<MagnetometerSensorMeasurement, MagneticFluxDensityTriad> {

    /**
     * Converts provided sensor measurement into a triad and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun convert(
        input: MagnetometerSensorMeasurement,
        output: MagneticFluxDensityTriad
    ) {
        val bx = input.bx.toDouble()
        val by = input.by.toDouble()
        val bz = input.bz.toDouble()
        val hardIronX = input.hardIronX?.toDouble()
        val hardIronY = input.hardIronY?.toDouble()
        val hardIronZ = input.hardIronZ?.toDouble()

        val x = if (hardIronX != null) bx + hardIronX else bx
        val y = if (hardIronY != null) by + hardIronY else by
        val z = if (hardIronZ != null) bz + hardIronZ else bz
        output.setValueCoordinates(x, y, z)
        output.unit = MagneticFluxDensityUnit.MICROTESLA
    }

    /**
     * Converts provided sensor measurement into a triad and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun convert(input: MagnetometerSensorMeasurement): MagneticFluxDensityTriad {
        val triad = MagneticFluxDensityTriad()
        convert(input, triad)
        return triad
    }
}