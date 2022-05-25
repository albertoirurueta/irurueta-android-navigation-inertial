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
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion

/**
 * Converts [SensorEvent] values into a Quaternion representing a 3D rotation that represents the
 * device attitude and a 3D point that represents the device relative translation.
 * Attitude is absolute respect to Earth and translation is relative to an arbitrary starting point.
 * Optionally, delta values respect last processed [SensorEvent] can also be obtained.
 */
object PoseHelper {

    /**
     * Minimum required length of values array.
     */
    private const val MIN_LENGTH = 15

    /**
     * Converts array of values contained in a [SensorEvent] into a 3D rotation expressed in NEU
     * (North, East, Up) system coordinates and a relative translation.
     *
     * @param values array of values to be converted.
     * @param attitudeResult instance where converted attitude will be stored.
     * @param translationResult instance where converted relative translation will be stored.
     * @param deltaAttitudeResult if provided, instance where converted delta attitude will be
     * stored. Delta attitude refers to the variation of attitude since last received [SensorEvent].
     * @param deltaTranslationResult if provided, instance where converted relative delta
     * translation will be stored. Delta translation refers to the variation of translation since
     * last received [SensorEvent].
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNEU(
        values: FloatArray,
        attitudeResult: Quaternion,
        translationResult: Point3D,
        deltaAttitudeResult: Quaternion? = null,
        deltaTranslationResult: Point3D? = null
    ): Float {
        // Array contains the following values:
        // values[0]: x*sin(θ/2)
        // values[1]: y*sin(θ/2)
        // values[2]: z*sin(θ/2) (pointing towards the sky)
        // values[3]: cos(θ/2)
        // values[4]: translation along x axis from an arbitrary origin
        // values[5]: translation along y axis from an arbitrary origin
        // values[6]: translation along z axis from an arbitrary origin
        // values[7]: delta quaternion rotation x*sin(θ/2)
        // values[8]: delta quaternion rotation y*sin(θ/2)
        // values[9]: delta quaternion rotation z*sin(θ/2)
        // values[10]: delta quaternion rotation cos(θ/2)
        // values[11]: delta translation along x axis
        // values[12]: delta translation along y axis.
        // values[13]: delta translation along z axis.
        // values[14]: sequence number
        // where θ is the rotation angle, and x, y, z represent the rotation axis.
        // On the other hand, a Quaternion defined by values (a,b,c,d) is defined so that:
        // a = cos(θ/2)
        // b = x*sin(θ/2)
        // c = y*sin(θ/2)
        // d = z*sin(θ/2)

        require(values.size >= MIN_LENGTH)

        AttitudeHelper.convertQuaternion(values, 0, attitudeResult)
        convertTranslation(values, 4, translationResult)
        if (deltaAttitudeResult != null) {
            AttitudeHelper.convertQuaternion(values, 7, deltaAttitudeResult)
        }
        if (deltaTranslationResult != null) {
            convertTranslation(values, 11, deltaTranslationResult)
        }
        return values[14]
    }

    /**
     * Converts array of values contained in a [SensorEvent] into a 3D rotation expressed in NEU
     * (North, East, Up) system coordinates and a relative translation.
     *
     * @param values array of values to be converted.
     * @param attitudeResult instance where converted attitude will be stored.
     * @param translationResult array where converted relative translation will be stored. Must have
     * size 3.
     * @param deltaAttitudeResult if provided, instance where converted delta attitude will be
     * stored. Delta attitude refers to the variation of attitude since last received [SensorEvent].
     * @param deltaTranslationResult if provided, array where converted relative delta
     * translation will be stored. Delta translation refers to the variation of translation since
     * last received [SensorEvent]. Must be null or have size 3.
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements, or if translation arrays have invalid size.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNEU(
        values: FloatArray,
        attitudeResult: Quaternion,
        translationResult: DoubleArray,
        deltaAttitudeResult: Quaternion? = null,
        deltaTranslationResult: DoubleArray? = null
    ): Float {
        require(values.size >= MIN_LENGTH)
        require(translationResult.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        require(deltaTranslationResult == null
                || deltaTranslationResult.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        AttitudeHelper.convertQuaternion(values, 0, attitudeResult)
        convertTranslation(values, 4, translationResult)
        if (deltaAttitudeResult != null) {
            AttitudeHelper.convertQuaternion(values, 7, deltaAttitudeResult)
        }
        if (deltaTranslationResult != null) {
            convertTranslation(values, 11, deltaTranslationResult)
        }
        return values[14]
    }

    /**
     * Converts array of values contained in [SensorEvent] into a 3D euclidean transformation
     * expressed in NEU (North, East, Up) system coordinates.
     *
     * @param values array of values to be converted.
     * @param transformationResult instance where converted 3D euclidean transformation will be
     * stored.
     * @param deltaTransformationResult if provided, instance where converted 3D euclidean
     * transformation of pose changes since last event will be stored.
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNEU(
        values: FloatArray,
        transformationResult: EuclideanTransformation3D,
        deltaTransformationResult: EuclideanTransformation3D? = null
    ): Float {
        val rotation = transformationResult.rotation
        val rotationQ = if (rotation is Quaternion) {
            rotation
        } else {
            // if rotation is not a quaternion, obtain a copy as a quaternion
            rotation.toQuaternion()
        }
        val translation = transformationResult.translation
        val deltaRotation = deltaTransformationResult?.rotation
        val deltaRotationQ = if (deltaRotation is Quaternion?) {
            deltaRotation
        } else {
            // if rotation is not a quaternion, obtain a copy as a quaternion
            deltaRotation?.toQuaternion()
        }
        val deltaTranslation = deltaTransformationResult?.translation
        val result = convertToNEU(values, rotationQ, translation, deltaRotationQ, deltaTranslation)

        // Rotation only needs to be set if a copy of rotation was made
        if (rotation !== rotationQ) {
            transformationResult.rotation = rotationQ
        }
        if (deltaRotation != deltaRotationQ) {
            deltaTransformationResult?.rotation = deltaRotationQ
        }

        return result
    }

    /**
     * Converts array of values contained in [SensorEvent] into a 3D euclidean transformation
     * expressed in NED (North, East, Down) system coordinates.
     *
     * @param values array of values to be converted.
     * @param transformationResult instance where converted 3D euclidean transformation will be
     * stored.
     * @param deltaTransformationResult if provided, instance where converted 3D euclidean
     * transformation of pose changes since last event will be stored.
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    fun convertToNED(
        values: FloatArray,
        transformationResult: EuclideanTransformation3D,
        deltaTransformationResult: EuclideanTransformation3D? = null
    ): Float {
        val result = convertToNEU(values, transformationResult, deltaTransformationResult)
        transformationResult.inverse()
        deltaTransformationResult?.inverse()
        return result
    }

    /**
     * Converts array of values contained in [SensorEvent] into a 3D rotation expressed in NED
     * (North, East, Down) system coordinates and a relative translation. If provided, result can
     * also be stored as 3D euclidean transformations.
     *
     * @param values array of values to be converted.
     * @param attitudeResult instance where converted attitude will be stored.
     * @param translationResult instance where converted relative translation will be stored.
     * @param deltaAttitudeResult if provided, instance where converted delta attitude will be
     * stored. Delta attitude refers to the variation of attitude since last received [SensorEvent].
     * @param deltaTranslationResult if provided, instance where converted relative delta
     * translation will be stored. Delta translation refers to the variation of translation since
     * last received [SensorEvent].
     * @param transformationResult if provided, instance where converted 3D euclidean transformation
     * will be stored.
     * @param deltaTransformationResult if provided, instance where converted 3D euclidean
     * transformation of pose changes since last event will be stored.
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(
        values: FloatArray,
        attitudeResult: Quaternion,
        translationResult: Point3D,
        deltaAttitudeResult: Quaternion? = null,
        deltaTranslationResult: Point3D? = null,
        transformationResult: EuclideanTransformation3D? = null,
        deltaTransformationResult: EuclideanTransformation3D? = null
    ): Float {
        val transformationResult2 = transformationResult ?: EuclideanTransformation3D()
        val deltaTransformationResult2 =
            if (deltaAttitudeResult != null || deltaTranslationResult != null) {
                deltaTransformationResult ?: EuclideanTransformation3D()
            } else {
                deltaTransformationResult
            }

        val result = convertToNED(values, transformationResult2, deltaTransformationResult2)

        transformationResult2.rotation.toQuaternion(attitudeResult)
        transformationResult2.getTranslationPoint(translationResult)
        if (deltaAttitudeResult != null) {
            deltaTransformationResult2?.rotation?.toQuaternion(deltaAttitudeResult)
        }
        if (deltaTranslationResult != null) {
            deltaTransformationResult2?.getTranslationPoint(deltaTranslationResult)
        }

        return result
    }

    /**
     * Converts array of values contained in [SensorEvent] into a 3D rotation expressed in NED
     * (North, East, Down) system coordinates and a relative translation. If provided, result can
     * also be stored as 3D euclidean transformations.
     *
     * @param values array of values to be converted.
     * @param attitudeResult instance where converted attitude will be stored.
     * @param translationResult array where converted relative translation will be stored. Must have
     * size 3.
     * @param deltaAttitudeResult if provided, instance where converted delta attitude will be
     * stored. Delta attitude refers to the variation of attitude since last received [SensorEvent].
     * @param deltaTranslationResult if provided, array where converted relative delta
     * translation will be stored. Delta translation refers to the variation of translation since
     * last received [SensorEvent]. Must be null or have size 3.
     * @param transformationResult if provided, instance where converted 3D euclidean transformation
     * will be stored.
     * @param deltaTransformationResult if provided, instance where converted 3D euclidean
     * transformation of pose changes since last event will be stored.
     * @return sequence number.
     * @throws IllegalArgumentException if provided values array does not have at least [MIN_LENGTH]
     * elements, or if translation arrays have invalid size.
     */
    @Throws(IllegalArgumentException::class)
    fun convertToNED(
        values: FloatArray,
        attitudeResult: Quaternion,
        translationResult: DoubleArray,
        deltaAttitudeResult: Quaternion? = null,
        deltaTranslationResult: DoubleArray? = null,
        transformationResult: EuclideanTransformation3D? = null,
        deltaTransformationResult: EuclideanTransformation3D? = null
    ): Float {
        require(translationResult.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        require(deltaTranslationResult == null
                || deltaTranslationResult.size == Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        val transformationResult2 = transformationResult ?: EuclideanTransformation3D()
        val deltaTransformationResult2 =
            if (deltaAttitudeResult != null || deltaTranslationResult != null) {
                deltaTransformationResult ?: EuclideanTransformation3D()
            } else {
                deltaTransformationResult
            }

        val result = convertToNED(values, transformationResult2, deltaTransformationResult2)

        transformationResult2.rotation.toQuaternion(attitudeResult)
        translationResult[0] = transformationResult2.translationX
        translationResult[1] = transformationResult2.translationY
        translationResult[2] = transformationResult2.translationZ

        if (deltaAttitudeResult != null) {
            deltaTransformationResult2?.rotation?.toQuaternion(deltaAttitudeResult)
        }
        if (deltaTranslationResult != null) {
            deltaTranslationResult[0] = deltaTransformationResult2?.translationX ?: 0.0
            deltaTranslationResult[1] = deltaTransformationResult2?.translationY ?: 0.0
            deltaTranslationResult[2] = deltaTransformationResult2?.translationZ ?: 0.0
        }

        return result
    }

    /**
     * Converts a [SensorEvent] values array at provided position into a translation point.
     *
     * @param values values array to be converted.
     * @param offset starting point where array conversion starts.
     * @param result instance where result is stored as a 3D point.
     */
    private fun convertTranslation(values: FloatArray, offset: Int, result: Point3D) {
        // Array at provided offset contains the following:
        // values[offset] = x
        // values[offset + 1] = y
        // values[offset + 2] = z
        result.setInhomogeneousCoordinates(
            values[offset].toDouble(),
            values[offset + 1].toDouble(),
            values[offset + 2].toDouble()
        )
        result.normalize()
    }

    /**
     * Converts a [SensorEvent] values array at provided position into an array containing
     * translation coordinates.
     *
     * @param values values array to be converted.
     * @param offset starting point where array conversion starts.
     * @param result instance where result is stored as an array of coordinates.
     */
    private fun convertTranslation(values: FloatArray, offset: Int, result: DoubleArray) {
        result[0] = values[offset].toDouble()
        result[1] = values[offset + 1].toDouble()
        result[2] = values[offset + 2].toDouble()
    }
}