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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.AccelerationUnit

/**
 * Converts [com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement] into [com.irurueta.navigation.inertial.calibration.AccelerationTriad].
 * Conversion keeps into account any available sensor bias data, and preserves coordinates system.
 */
object AccelerometerSensorMeasurementTriadConverter :
    SensorMeasurementTriadConverter<AccelerometerSensorMeasurement, AccelerationTriad> {

    /**
     * Converts provided sensor measurement into a triad and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun convert(
        input: AccelerometerSensorMeasurement,
        output: AccelerationTriad
    ) {
        val ax = input.ax.toDouble()
        val ay = input.ay.toDouble()
        val az = input.az.toDouble()
        val bx = input.bx?.toDouble()
        val by = input.by?.toDouble()
        val bz = input.bz?.toDouble()

        val x = if (bx != null) ax + bx else ax
        val y = if (by != null) ay + by else ay
        val z = if (bz != null) az + bz else az
        output.setValueCoordinates(x, y, z)
        output.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND
    }

    /**
     * Converts provided sensor measurement into a triad and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun convert(input: AccelerometerSensorMeasurement): AccelerationTriad {
        val triad = AccelerationTriad()
        convert(input, triad)
        return triad
    }
}