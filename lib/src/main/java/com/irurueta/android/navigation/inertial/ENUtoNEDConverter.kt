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
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
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
object ENUtoNEDConverter {
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
     * Converts provided acceleration triad from ENU to NED or from NED to ENU.
     *
     * @param input acceleration triad to be converted.
     * @return new acceleration triad containing result of conversion.
     */
    fun convertAndReturnNew(input: AccelerationTriad) : AccelerationTriad {
        val result = AccelerationTriad()
        convert(input, result)
        return result
    }

    /**
     * Converts provided angular speed triad from ENU to NED or from NED to ENU.
     *
     * @param input angular speed triad to be converted.
     * @return new angular speed triad containing result of conversion.
     */
    fun convertAndReturnNew(input: AngularSpeedTriad) : AngularSpeedTriad {
        val result = AngularSpeedTriad()
        convert(input, result)
        return result
    }

    /**
     * Converts provided magnetic flux density triad from ENU to NED or from NED to ENU.
     *
     * @param input magnetic flux density to be converted.
     * @return new magnetic flux density triad containing result of conversion.
     */
    fun convertAndReturnNew(input: MagneticFluxDensityTriad) : MagneticFluxDensityTriad {
        val result = MagneticFluxDensityTriad()
        convert(input, result)
        return result
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
        // Xenu =  [0   1   0 ] * Xned
        //         [1   0   0 ]
        //	       [0   0   -1]

        // Xned =  [0   1   0 ] * Xenu
        //         [1   0   0 ]
        //         [0   0   -1]

        // C = 	[0  1   0 ]
        //      [1  0   0 ]
        //      [0  0   -1]

        // Notice that	conversion = C, and C * C = I -> C = C^-1

        // Xrotated_ned = Rned * Xned = Rned * C * Xenu
        // Xrotated_enu = Renu * Xenu = Renu * C * Xned

        // Xrotated_enu = C * Xrotated_ned = C * Rned * C * Xenu = Renu * Xenu
        // Xrotated_ned = C * Xrotated_enu = C * Renu * C * Xned = Rned * Xned

        // Hence:
        // Renu = C * Rned * C
        // Rned = C * Renu * C

        Quaternion.product(CONVERSION_ROTATION, input, output)
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
     * Converts an euclidean 3D transformation from ENU to NED or from NED to ENU, taking into
     * account that:
     * Tenu = Chom * Tned * Chom
     * Tned = Chom * Tenu * Chom
     *
     * @param input Input euclidean 3D transformation to be converted.
     * @param output Instance where result will be stored.
     */
    fun convert(input: EuclideanTransformation3D, output: EuclideanTransformation3D) {
        // T =  [R  t] is an euclidean 3D transformation
        //      [0  1]

        // Given a point X = [x y z 1]^T = [Xinhom 1]^T, then it's transformation becomes:
        // X' = T * X = [R  t][Xinhom] = [R*Xinhom + t]
        //              [0  1][1     ]   [1           ]

        // A transformed point in ENU coordinates can be expressed as:
        // Xenu' = Tenu * Xenu
        // where Tenu is
        // Tenu =   [Renu   tenu]
        //          [0      1   ]
        // A transformed point in NED coordinates can be expressed as:
        // Xned' = Tned * Xned
        // where Tned is
        // Tned = [Rned     tned]
        //        [0        1   ]

        // we know that:
        // Rned = C * Renu * C
        // Where C is the conversion matrix
        // and likewise, the converted translation term is:
        // tned = C * tenu

        // Consequently:
        // Tned = [Rned     tned] = [C * Renu * C   C *tenu ] = [C * Renu   C * tenu] * [C  0] =
        //        [0        1   ]   [0              1       ]   [0          1       ]   [0  1]

        // [C   0] *[Renu tenu] * [C    0]
        // [0   1]  [0    1   ]   [0    1]

        // Tned = [C    0] * Tenu * [C  0]
        //        [0    1]          [0  1]

        // Finally, if we consider Chom as an extended version of C in homogeneous coordinates:
        // Chom = [C    0]
        //        [0    1]

        // Then transformation becomes
        // Tned = Chom * Tenu * Chom

        val inputRotation = if (input.rotation is Quaternion) {
            input.rotation as Quaternion
        } else {
            input.rotation.toQuaternion()
        }

        val inputTranslation = input.translation

        val outputRotation = if (output.rotation is Quaternion) {
            output.rotation as Quaternion
        } else {
            output.rotation.toQuaternion()
        }

        val outputTranslation = output.translation

        convert(inputRotation, outputRotation)
        convertPoint(inputTranslation, outputTranslation)
    }

    /**
     * Converts an euclidean 3D transformation from ENU to NED or from NED to ENU, taking into
     * account that:
     * Tenu = Chom * Tned * Chom
     * Tned = Chom * Tenu * Chom
     *
     * @param input Input euclidean 3D transformation t be converted.
     * @return converted euclidean 3D transformation.
     */
    fun convertAndReturnNew(input: EuclideanTransformation3D): EuclideanTransformation3D {
        val result = EuclideanTransformation3D()
        convert(input, result)
        return result
    }

    /**
     * Converts 3D inhomogeneous point from ENU to NED or from NED to ENU.
     *
     * @param valueX x-coordinate of inhomogeneous 3D input point to be converted
     * @param valueY y-coordinate of inhomogeneous 3D input point to be converted.
     * @param valueZ z-coordinate of inhomogeneous 3D input point to be converted.
     * @param output array instance where result of conversion will be stored.
     * @throws IllegalArgumentException if provided output array does not have length 3.
     */
    @Throws(IllegalArgumentException::class)
    fun convertPoint(valueX: Double, valueY: Double, valueZ: Double, output: DoubleArray) {
        require(output.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        output[0] = valueY
        output[1] = valueX
        output[2] = -valueZ
    }

    /**
     * Converts 3D inhomogeneous point from ENU to NED or from NED to ENU.
     *
     * @param input array containing point coordinates of 3D inhomogeneous point to be converted.
     * @param output array instance where result of conversion will be stored.
     * @throws IllegalArgumentException if provided arrays do not have length 3.
     */
    @Throws(IllegalArgumentException::class)
    fun convertPoint(input: DoubleArray, output: DoubleArray) {
        require(input.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        convertPoint(input[0], input[1], input[2], output)
    }

    /**
     * Converts 3D inhomogeneous point from ENU to NED or from NED to ENU.
     *
     * @param input point to be converted.
     * @param output instance where result of conversion will be stored.
     */
    fun convertPoint(input: Point3D, output: Point3D) {
        output.setInhomogeneousCoordinates(input.inhomY, input.inhomX, -input.inhomZ)
    }

    /**
     * Converts 3D inhomogeneous point from ENU to NED or from NED to ENU.
     *
     * @param input array containing point coordinates of 3D inhomogeneous point to be converted.
     * @return array containing converted point coordinates.
     * @throws IllegalArgumentException if provided input array does not have length 3.
     */
    @Throws(IllegalArgumentException::class)
    fun convertPointAndReturnNew(input: DoubleArray): DoubleArray {
        val result = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        convertPoint(input, result)
        return result
    }

    /**
     * Converts 3D inhomogeneous point from ENU to NED or from NED to ENU.
     *
     * @param input point to be converted.
     * @return new point containing result of conversion.
     */
    fun convertPointAndReturnNew(input: Point3D) : InhomogeneousPoint3D {
        val result = InhomogeneousPoint3D()
        convertPoint(input, result)
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