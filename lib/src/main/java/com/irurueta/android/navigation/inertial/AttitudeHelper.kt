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

import android.hardware.SensorEvent
import com.irurueta.algebra.Matrix
import com.irurueta.geometry.MatrixRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Converts [SensorEvent] values into a Quaternion representing a 3D rotation that represents
 * the device attitude.
 * Depending on the sensor being used, attitude will be absolute or relative and might more or less
 * accurate, have drift and use more or less power. following the system coordinates
 * Android uses NEU (North, East Up) system coordinates as indicated here:
 * https://developer.android.com/reference/android/hardware/SensorEvent#sensor.type_rotation_vector,
 * where:
 * - X axis is defined as the vector product Y.Z (It is tangential to the ground at the device's
 * current location and roughly points East).
 * - Y axis is tangential to the ground at the device's current location and points towards magnetic
 * north.
 * - Z axis points towards the sky and is perpendicular to the ground.
 * However, this helper can also return attitudes in NED (North, East, Down) coordinates.
 */
object AttitudeHelper {

    /**
     * Minimum required length of values array.
     */
    private const val MIN_LENGTH = 4

    /**
     * Value indicating that heading accuracy is not available.
     */
    const val UNAVAILABLE_HEADING_ACCURACY = -1.0f

    /**
     * Converts array of values contained in a [SensorEvent] into a 3D rotation expressed in NEU
     * (North, East, Up) system coordinates.
     *
     * @param values array of values to be converted.
     * @param result instance where converted attitude will be stored.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNEU(values: FloatArray, result: Quaternion): Double? {
        // Array contains the following values:
        // values[0]: x*sin(θ/2)
        // values[1]: y*sin(θ/2)
        // values[2]: z*sin(θ/2) (pointing towards the sky)
        // values[3]: cos(θ/2) (only for SDK 18 or later)
        // values[4]: estimated heading Accuracy (in radians) (-1 if unavailable)
        // where θ is the rotation angle, and x, y, z represent the rotation axis.

        require(values.size >= MIN_LENGTH)

        convertQuaternion(values, 0, result)

        val headingAccuracy =
            if (values.size > MIN_LENGTH && values[4] != UNAVAILABLE_HEADING_ACCURACY) {
                values[4].toDouble()
            } else {
                null
            }

        return headingAccuracy
    }

    /**
     * Converts an array of values contained in a [SensorEvent] into a 3D rotation expressed in NED
     * (North, Easth, Down) system coordinates.
     *
     * @param values array of values to be converted.
     * @param result instance where converted attitude will be stored.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(values: FloatArray, result: Quaternion): Double? {
        val headingAccuracy = convertToNEU(values, result)
        result.conjugate(result)
        return headingAccuracy
    }

    /**
     * Converts an array of values contained in a [SensorEvent] into a coordinate transformation
     * and the equivalent 3D rotation expressed in NED (North, Easth, Down) system coordinates.
     *
     * @param values array of values to be converted.
     * @param resultC coordinate transformation instance where converted attitude will be stored.
     * @param resultQ quaternion instance where converted attitude will be stored.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(
        values: FloatArray,
        resultC: CoordinateTransformation,
        resultQ: Quaternion? = null,
        matrix: Matrix? = null
    ): Double? {
        val q: Quaternion = resultQ ?: Quaternion()
        val headingAccuracy = convertToNED(values, q)
        val transformationMatrix: Matrix = if (matrix != null) {
            // reuse provided matrix if available
            if (matrix.rows != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS || matrix.columns != MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS) {
                matrix.resize(
                    MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                    MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
                )
            }
            matrix
        } else {
            Matrix(
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
                MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
            )
        }
        q.toMatrixRotation(transformationMatrix)

        resultC.sourceType = FrameType.BODY_FRAME
        resultC.destinationType = FrameType.LOCAL_NAVIGATION_FRAME
        resultC.matrix = transformationMatrix
        return headingAccuracy
    }

    /**
     * Converts a [SensorEvent] values array at provided position into a quaternion.
     *
     * @param values values array to be converted.
     * @param offset starting position where array conversion starts.
     * @param result instance where result is stored as a rotation.
     */
    internal fun convertQuaternion(values: FloatArray, offset: Int, result: Quaternion) {
        // Quaternions follow the expression:
        // a = cos(θ/2)
        // b = x*sin(θ/2)
        // c = y*sin(θ/2)
        // d = z*sin(θ/2)
        // It is assumed that array starting at provided offset has the following expression:
        // values[offset]: x*sin(θ/2)
        // values[offset + 1]: y*sin(θ/2)
        // values[offset + 2]: z*sin(θ/2) (pointing towards the sky)
        // values[offset + 3]: cos(θ/2) (only for SDK 18 or later)
        result.b = values[offset].toDouble()
        result.c = values[offset + 1].toDouble()
        result.d = values[offset + 2].toDouble()
        result.a = values[offset + 3].toDouble()

        result.normalize()
    }
}