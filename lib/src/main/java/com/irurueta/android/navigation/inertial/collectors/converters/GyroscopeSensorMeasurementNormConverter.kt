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
import kotlin.math.sqrt

/**
 * Converts a [com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement] into its norm value.
 * Conversion takes into account any available sensor bias data.
 */
object GyroscopeSensorMeasurementNormConverter:
    SensorMeasurementNormConverter<GyroscopeSensorMeasurement> {

    /**
     * Converts provided sensor measurement into its norm value.
     *
     * @param input input measurement to be converted.
     * @return norm value.
     */
    override fun convert(input: GyroscopeSensorMeasurement): Double {
        val wx = input.wx.toDouble()
        val wy = input.wy.toDouble()
        val wz = input.wz.toDouble()
        val bx = input.bx?.toDouble()
        val by = input.by?.toDouble()
        val bz = input.bz?.toDouble()

        val x = if (bx != null) wx + bx else wx
        val y = if (by != null) wy + by else wy
        val z = if (bz != null) wz + bz else wz

        return sqrt(x * x + y * y + z * z)
    }
}