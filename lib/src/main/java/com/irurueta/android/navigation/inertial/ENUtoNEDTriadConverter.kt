/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.units.Measurement
import kotlin.math.sqrt

/**
 * Converts between ENU (East, North, Up) coordinates system and NED (North, East, Down).
 * It must be noticed that Android uses ENU coordinates system for its sensors.
 * ENU is a RHS (Right Handed System) that matches the coordinate system used by OpenGL tod display
 * 3D scenes.
 * This converter can be used to convert from ENU to NED and conversely from NED to ENU.
 */
object ENUtoNEDTriadConverter {
    /**
     * Minus half of square root of 2.0.
     */
    private val VALUE = -1.0 / sqrt(2.0)

    /**
     * Rotation representing the conversion between ENU to NED and NED to ENU.
     * This rotation is equivalent to rotation matrix below:
     * [0   1   0]
     * [1   0   0]
     * [0   0  -1]
     * Which is equivalent to roll = 0 rad, pitch = -[Math.PI] rad, yaw = -[Math.PI] / 2.0 rad,
     * which is also equivalent to roll = -[Math.PI] rad, pitch = 0 rad, yaw = [Math.PI] / 2.0 rad.
     */
    private val CONVERSION_ROTATION = Quaternion(0.0, VALUE, VALUE, 0.0)

    /**
     * Converts from ENU to NED or from NED to ENU.
     *
     * @param input input triad to be converted.
     * @param output instance where result of conversion will be stored.
     */
    fun <T : Triad<U, M>, U : Enum<*>, M : Measurement<U>> convert(input: T, output: T) {
        convert(input.valueX, input.valueY, input.valueZ, output)
    }

    /**
     * Converts from ENU to NED or from NED to ENU.
     *
     * @param valueX x-coordinate of input triad to be converted.
     * @param valueY y-coordinate of input triad to be converted.
     * @param valueZ z-coordinate of input triad to be converted.
     * @param output instance where result of conversion will be stored.
     */
    fun <T : Triad<U, M>, U : Enum<*>, M : Measurement<U>> convert(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        output: T
    ) {
        output.setValueCoordinates(valueY, valueX, -valueZ)
    }

    /**
     * Converts a rotation from ENU to NED or from NED to ENU, taking into account that:
     * Renu = CONVERSION_ROTATION * Rned * CONVERSION_ROTATION
     * And also:
     * Rned = CONVERSION_ROTATION * Renu * CONVERSION_ROTATION
     *
     * @param input Input quaternion to be converted.
     * @param output Instance where result will be stored.
     */
    fun convert(input: Quaternion, output: Quaternion) {
        Quaternion.product(CONVERSION_ROTATION, input, output)
        output.normalize()
        Quaternion.product(output, CONVERSION_ROTATION, output)
        output.normalize()
    }

    /**
     * Converts a rotation from ENU to NED or from NED to ENU, taking into account that:
     * Renu = CONVERSION_ROTATION * Rned * CONVERSION_ROTATION
     * And also:
     * Rned = CONVERSION_ROTATION * Renu * CONVERSION_ROTATION
     *
     * @param value Input quaternion to be converted.
     * @return new converted quaternion.
     */
    fun convertAndReturnNew(value: Quaternion): Quaternion {
        val result = Quaternion()
        convert(value, result)
        return result
    }

    /**
     * Obtains a rotation to convert from ENU to NED and from NED to ENU coordinate systems.
     *
     * @param result instance where result will be stored.
     */
    fun conversionRotation(result: Rotation3D) {
        result.fromRotation(CONVERSION_ROTATION)
    }

    /**
     * Obtains a rotation to convert from ENU to NED and from NED to ENU coordinate systems.
     */
    val conversionRotation: Quaternion
        get() = Quaternion(CONVERSION_ROTATION)

    /**
     * Obtains a matrix to convert from ENU to NED and from NED to ENU coordinate systems.
     *
     * @param result instance where result will be stored.
     */
    fun conversionRotationMatrix(result: Matrix) {
        if (result.rows != Rotation3D.INHOM_COORDS || result.columns != Rotation3D.INHOM_COORDS) {
            result.resize(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        }
        result.setElementAt(0, 1, 1.0)
        result.setElementAt(1, 0, 1.0)
        result.setElementAt(2, 2, -1.0)
    }

    /**
     * Obtains a matrix to convert from ENU to NED and from NED to ENU coordinate systems.
     */
    val conversionRotationMatrix: Matrix
        get() {
            val result = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
            conversionRotationMatrix(result)
            return result
        }
}