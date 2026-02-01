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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.units.AngularSpeedUnit

/**
 * Converts a [com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement] into an [com.irurueta.navigation.inertial.calibration.AngularSpeedTriad].
 * Conversion keeps into account any available sensor bias data, and preserves coordinates system.
 */
object GyroscopeSensorMeasurementTriadConverter :
    SensorMeasurementTriadConverter<GyroscopeSensorMeasurement, AngularSpeedTriad> {

    /**
     * Converts provided sensor measurement into a triad and stores the result
     * in the provided output instance.
     *
     * @param input input measurement to be converted.
     * @param output instance where converted measurement will be stored.
     */
    override fun convert(
        input: GyroscopeSensorMeasurement,
        output: AngularSpeedTriad
    ) {
        val wx = input.wx.toDouble()
        val wy = input.wy.toDouble()
        val wz = input.wz.toDouble()
        val bx = input.bx?.toDouble()
        val by = input.by?.toDouble()
        val bz = input.bz?.toDouble()

        val x = if (bx != null) wx + bx else wx
        val y = if (by != null) wy + by else wy
        val z = if (bz != null) wz + bz else wz
        output.setValueCoordinates(x, y, z)
        output.unit = AngularSpeedUnit.RADIANS_PER_SECOND
    }

    /**
     * Converts provided sensor measurement into a triad and returns a new
     * instance containing the result.
     *
     * @param input input measurement to be converted.
     * @return a new instance containing converted measurement.
     */
    override fun convert(input: GyroscopeSensorMeasurement): AngularSpeedTriad {
        val triad = AngularSpeedTriad()
        convert(input, triad)
        return triad
    }
}