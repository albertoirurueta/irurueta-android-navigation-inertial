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

import android.content.Context
import android.hardware.SensorEvent
import com.irurueta.algebra.Matrix
import com.irurueta.geometry.MatrixRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Converts [SensorEvent] values into a Quaternion representing a 3D rotation that represents
 * the device attitude.
 * Depending on the sensor being used, attitude will be absolute or relative and might be more or
 * less accurate, have drift and use more or less power. following the system coordinates
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
     * Converts array of values contained in a [SensorEvent] into a 3D rotation expressed in ENU
     * (East, North, Up) system coordinates.
     *
     * @param context Android context.
     * @param values array of values to be converted.
     * @param result instance where converted attitude will be stored.
     * @param displayOrientationResult instance containing a quaternion indicating display
     * orientation as a rotation on z-axis (yaw angle). If provided, this is meant to be reused.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToENU(
        context: Context,
        values: FloatArray,
        result: Quaternion,
        displayOrientationResult: Quaternion? = null
    ): Double? {
        // Array contains the following values:
        // values[0]: x*sin(θ/2)
        // values[1]: y*sin(θ/2)
        // values[2]: z*sin(θ/2) (pointing towards the sky)
        // values[3]: cos(θ/2) (only for SDK 18 or later)
        // values[4]: estimated heading Accuracy (in radians) (-1 if unavailable)
        // where θ is the rotation angle, and x, y, z represent the rotation axis.

        require(values.size >= MIN_LENGTH)

        convertQuaternion(context, values, result, displayOrientationResult)

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
     * @param context Android context.
     * @param values array of values to be converted.
     * @param result instance where converted attitude will be stored.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @param displayOrientationResult instance containing a quaternion indicating display
     * orientation as a rotation on z-axis (yaw angle). If provided, this is meant to be reused.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(
        context: Context,
        values: FloatArray,
        result: Quaternion,
        displayOrientationResult: Quaternion? = null
    ): Double? {
        val headingAccuracy = convertToENU(context, values, result, displayOrientationResult)
        result.conjugate(result)
        return headingAccuracy
    }

    /**
     * Converts an array of values contained in a [SensorEvent] into a coordinate transformation
     * and the equivalent 3D rotation expressed in NED (North, Easth, Down) system coordinates.
     *
     * @param context Android context.
     * @param values array of values to be converted.
     * @param resultC coordinate transformation instance where converted attitude will be stored.
     * @param resultQ quaternion instance where converted attitude will be stored.
     * @param displayOrientationResult instance containing a quaternion indicating display
     * orientation as a rotation on z-axis (yaw angle). If provided, this is meant to be reused.
     * @param matrix result quaternion expressed in matrix form. If provided, this is meant to be
     * reused.
     * @return accuracy of heading attitude angle expressed in radians or null if not available.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(
        context: Context,
        values: FloatArray,
        resultC: CoordinateTransformation,
        resultQ: Quaternion? = null,
        displayOrientationResult: Quaternion? = null,
        matrix: Matrix? = null
    ): Double? {
        val q: Quaternion = resultQ ?: Quaternion()
        val headingAccuracy = convertToNED(context, values, q, displayOrientationResult)
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
        resultC.destinationType = FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        resultC.matrix = transformationMatrix
        return headingAccuracy
    }

    /**
     * Converts a [SensorEvent] values array at provided position into a quaternion.
     *
     * @param context Android context.
     * @param values values array to be converted.
     * @param result instance where result is stored as a rotation.
     * @param displayOrientationResult instance containing a quaternion indicating display
     * orientation. If provided, this is meant to be reused.
     */
    private fun convertQuaternion(
        context: Context,
        values: FloatArray,
        result: Quaternion,
        displayOrientationResult: Quaternion? = null
    ) {
        // Quaternions follow the expression:
        // a = cos(θ/2)
        // b = x*sin(θ/2)
        // c = y*sin(θ/2)
        // d = z*sin(θ/2)
        // It is assumed that array starting at provided offset has the following expression:
        // values[0]: x*sin(θ/2)
        // values[1]: y*sin(θ/2)
        // values[2]: z*sin(θ/2) (pointing towards the sky)
        // values[3]: cos(θ/2) (only for SDK 18 or later)
        result.b = values[0].toDouble()
        result.c = values[1].toDouble()
        result.d = values[2].toDouble()
        result.a = values[3].toDouble()

        result.normalize()

        val q = displayOrientationResult ?: Quaternion()
        val displayRotationRadians = DisplayOrientationHelper.getDisplayRotationRadians(context)
        q.setFromEulerAngles(0.0, 0.0, displayRotationRadians)
        result.combine(q)
    }
}