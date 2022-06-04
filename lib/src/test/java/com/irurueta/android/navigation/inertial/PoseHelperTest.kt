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
import com.irurueta.geometry.*
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class PoseHelperTest {

    @Test
    fun convertToNEU_whenTranslationPoints_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val result = PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(attitudeResult.equals(attitude2, ABSOLUTE_ERROR))
        assertTrue(translationResult.equals(translation2, ABSOLUTE_ERROR))
        assertTrue(deltaAttitudeResult.equals(deltaAttitude, ABSOLUTE_ERROR))
        assertTrue(deltaTranslationResult.equals(deltaTranslation, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNEU_whenTranslationPointsAndOnlyRequiredParameters_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val result = PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(attitudeResult.equals(attitude2, ABSOLUTE_ERROR))
        assertTrue(translationResult.equals(translation2, ABSOLUTE_ERROR))
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNEU_whenTranslationPointsAndInvalidValuesSize_throwsIllegalArgumentException() {
        val values = FloatArray(14)

        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )
    }

    @Test
    fun convertToNEU_whenTranslationArrays_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val result = PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(attitudeResult.equals(attitude2, ABSOLUTE_ERROR))
        assertArrayEquals(translationResult, translation2, ABSOLUTE_ERROR)
        assertTrue(deltaAttitudeResult.equals(deltaAttitude, ABSOLUTE_ERROR))
        assertArrayEquals(deltaTranslationResult, deltaTranslation, ABSOLUTE_ERROR)
    }

    @Test
    fun convertToNEU_whenTranslationArraysAndOnlyRequiredParameters_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val result = PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(attitudeResult.equals(attitude2, ABSOLUTE_ERROR))
        assertArrayEquals(translationResult, translation2, ABSOLUTE_ERROR)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNEU_whenTranslationArraysInvalidValuesSize_returnsExpectedValues() {
        val values = FloatArray(14)

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNEU_whenInvalidTranslationResultArray_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(2)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNEU_whenInvalidDeltaTranslationResultArray_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(2)
        PoseHelper.convertToNEU(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult
        )
    }

    @Test
    fun convertToNEU_whenTransformationsWithQuaternionRotations_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        assertTrue(transformationResult.rotation is Quaternion)
        assertTrue(deltaTransformationResult.rotation is Quaternion)

        val result = PoseHelper.convertToNEU(
            values,
            transformationResult,
            deltaTransformationResult
        )

        // check
        assertTrue(transformationResult.rotation is Quaternion)
        assertTrue(deltaTransformationResult.rotation is Quaternion)

        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(transformationResult.rotation.equals(attitude2, ABSOLUTE_ERROR))
        assertTrue(transformationResult.translationPoint.equals(translation2, ABSOLUTE_ERROR))
        assertTrue(deltaTransformationResult.rotation.equals(deltaAttitude, ABSOLUTE_ERROR))
        assertTrue(
            deltaTransformationResult.translationPoint.equals(
                deltaTranslation,
                ABSOLUTE_ERROR
            )
        )
    }

    @Test
    fun convertToNEU_whenTransformationsWithNonQuaternionRotations_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val transformationResult =
            EuclideanTransformation3D(Rotation3D.create(Rotation3DType.MATRIX_ROTATION3D))
        val deltaTransformationResult =
            EuclideanTransformation3D(Rotation3D.create(Rotation3DType.MATRIX_ROTATION3D))
        assertFalse(transformationResult.rotation is Quaternion)
        assertFalse(deltaTransformationResult.rotation is Quaternion)

        val result = PoseHelper.convertToNEU(
            values,
            transformationResult,
            deltaTransformationResult
        )

        // check
        assertTrue(transformationResult.rotation is Quaternion)
        assertTrue(deltaTransformationResult.rotation is Quaternion)

        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(transformationResult.rotation.equals(attitude2, ABSOLUTE_ERROR))
        assertTrue(transformationResult.translationPoint.equals(translation2, ABSOLUTE_ERROR))
        assertTrue(deltaTransformationResult.rotation.equals(deltaAttitude, ABSOLUTE_ERROR))
        assertTrue(
            deltaTransformationResult.translationPoint.equals(
                deltaTranslation,
                ABSOLUTE_ERROR
            )
        )
    }

    @Test
    fun convertToNEU_whenOnlyTransformationResult_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val transformationResult = EuclideanTransformation3D()
        val result = PoseHelper.convertToNEU(
            values,
            transformationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(transformationResult.rotation.equals(attitude2, ABSOLUTE_ERROR))
        assertTrue(transformationResult.translationPoint.equals(translation2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenTransformations_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()

        val result = PoseHelper.convertToNED(
            values,
            transformationResult,
            deltaTransformationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenOnlyTransformationResult_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val transformationResult = EuclideanTransformation3D()

        val result = PoseHelper.convertToNED(
            values,
            transformationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenTransformationsQuaternionsAndPoints_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()

        val result = PoseHelper.convertToNED(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertEquals(translationResult, transformationResult.translationPoint)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertEquals(deltaTranslationResult, deltaTransformationResult.translationPoint)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenRequiredQuaternionsAndPoints_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()

        val result = PoseHelper.convertToNED(
            values,
            attitudeResult,
            translationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        val invTransformation2 = transformation2.inverseAndReturnNew()
        val transformationResult =
            EuclideanTransformation3D(attitudeResult, translationResult.asArray())
        assertTrue(
            invTransformation2.asMatrix().equals(transformationResult.asMatrix(), ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenTransformationsQuaternionsAndArrays_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation)

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()

        val result = PoseHelper.convertToNED(
            values,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertArrayEquals(translationResult, transformationResult.translation, 0.0)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertArrayEquals(deltaTranslationResult, deltaTransformationResult.translation, 0.0)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenRequiredQuaternionsAndArrays_returnsExpectedValues() {
        // Notice that: attitude2 = attitude1 * deltaAttitude
        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = doubleArrayOf(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation[0].toFloat()
        val yDelta = deltaTranslation[1].toFloat()
        val zDelta = deltaTranslation[2].toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        val result = PoseHelper.convertToNED(
            values,
            attitudeResult,
            translationResult
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        val invTransformation2 = transformation2.inverseAndReturnNew()
        val transformationResult =
            EuclideanTransformation3D(attitudeResult, translationResult)
        assertTrue(
            invTransformation2.asMatrix().equals(transformationResult.asMatrix(), ABSOLUTE_ERROR)
        )
    }

    @Test
    fun convertToNED_whenECEFStartPositionFramesTransformationsQuaternionsAndPoints_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertEquals(translationResult, transformationResult.translationPoint)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertEquals(deltaTranslationResult, deltaTransformationResult.translationPoint)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult.inhomX,
            startEcefPosition.y + translationResult.inhomY,
            startEcefPosition.z + translationResult.inhomZ
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult.inhomX / TIME_INTERVAL,
            deltaTranslationResult.inhomY / TIME_INTERVAL,
            deltaTranslationResult.inhomZ / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult2 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult.equals(frameResult2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenECEFStartPositionFrameOnly_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult1 = NEDFrame()
        val frameResult2 = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result1 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult1,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        val result2 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult2
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result1)
        assertEquals(SEQUENCE_NUMBER, result2)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertEquals(translationResult, transformationResult.translationPoint)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertEquals(deltaTranslationResult, deltaTransformationResult.translationPoint)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult.inhomX,
            startEcefPosition.y + translationResult.inhomY,
            startEcefPosition.z + translationResult.inhomZ
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult.inhomX / TIME_INTERVAL,
            deltaTranslationResult.inhomY / TIME_INTERVAL,
            deltaTranslationResult.inhomZ / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult3 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult1.equals(frameResult3, ABSOLUTE_ERROR))
        assertEquals(frameResult1.coordinateTransformation, frameResult3.coordinateTransformation)
        assertEquals(frameResult1.position, frameResult2.position)
        assertEquals(NEDVelocity(), frameResult2.velocity)
    }

    @Test
    fun convertToNED_whenECEFStartPositionFramesTransformationsQuaternionsAndArrays_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertArrayEquals(translationResult, transformationResult.translation, 0.0)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertArrayEquals(deltaTranslationResult, deltaTransformationResult.translation, 0.0)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult[0],
            startEcefPosition.y + translationResult[1],
            startEcefPosition.z + translationResult[2]
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult[0] / TIME_INTERVAL,
            deltaTranslationResult[1] / TIME_INTERVAL,
            deltaTranslationResult[2] / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult2 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult.equals(frameResult2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenECEFStartPositionFrameAttitudeAndTranslationArrayOnly_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult1 = NEDFrame()
        val frameResult2 = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result1 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult1,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        val result2 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult2,
            attitudeResult,
            translationResult,
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result1)
        assertEquals(SEQUENCE_NUMBER, result2)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertArrayEquals(translationResult, transformationResult.translation, 0.0)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertArrayEquals(deltaTranslationResult, deltaTransformationResult.translation, 0.0)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult[0],
            startEcefPosition.y + translationResult[1],
            startEcefPosition.z + translationResult[2]
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult[0] / TIME_INTERVAL,
            deltaTranslationResult[1] / TIME_INTERVAL,
            deltaTranslationResult[2] / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult3 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult1.equals(frameResult3, ABSOLUTE_ERROR))
        assertEquals(frameResult1.coordinateTransformation, frameResult3.coordinateTransformation)
        assertEquals(frameResult1.position, frameResult2.position)
        assertEquals(NEDVelocity(), frameResult2.velocity)
    }

    @Test
    fun convertToNED_whenNEDStartPositionFramesTransformationsQuaternionsAndPoints_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result = PoseHelper.convertToNED(
            values,
            startNedPosition,
            frameResult,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertEquals(translationResult, transformationResult.translationPoint)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertEquals(deltaTranslationResult, deltaTransformationResult.translationPoint)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult.inhomX,
            startEcefPosition.y + translationResult.inhomY,
            startEcefPosition.z + translationResult.inhomZ
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult.inhomX / TIME_INTERVAL,
            deltaTranslationResult.inhomY / TIME_INTERVAL,
            deltaTranslationResult.inhomZ / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult2 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult.equals(frameResult2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenNEDStartPositionFrameOnly_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult1 = NEDFrame()
        val frameResult2 = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = InhomogeneousPoint3D()
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = InhomogeneousPoint3D()
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result1 = PoseHelper.convertToNED(
            values,
            startNedPosition,
            frameResult1,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        val result2 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult2
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result1)
        assertEquals(SEQUENCE_NUMBER, result2)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertEquals(translationResult, transformationResult.translationPoint)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertEquals(deltaTranslationResult, deltaTransformationResult.translationPoint)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult.inhomX,
            startEcefPosition.y + translationResult.inhomY,
            startEcefPosition.z + translationResult.inhomZ
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult.inhomX / TIME_INTERVAL,
            deltaTranslationResult.inhomY / TIME_INTERVAL,
            deltaTranslationResult.inhomZ / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult3 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult1.equals(frameResult3, ABSOLUTE_ERROR))
        assertEquals(frameResult1.coordinateTransformation, frameResult3.coordinateTransformation)
        assertEquals(frameResult1.position, frameResult2.position)
        assertEquals(NEDVelocity(), frameResult2.velocity)
    }

    @Test
    fun convertToNED_whenNEDStartPositionFramesTransformationsQuaternionsAndArrays_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result = PoseHelper.convertToNED(
            values,
            startNedPosition,
            frameResult,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertArrayEquals(translationResult, transformationResult.translation, 0.0)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertArrayEquals(deltaTranslationResult, deltaTransformationResult.translation, 0.0)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult[0],
            startEcefPosition.y + translationResult[1],
            startEcefPosition.z + translationResult[2]
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult[0] / TIME_INTERVAL,
            deltaTranslationResult[1] / TIME_INTERVAL,
            deltaTranslationResult[2] / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult2 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult.equals(frameResult2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenNEDStartPositionFrameAttitudeAndTranslationArrayOnly_returnsExpectedValues() {
        val startNedPosition = createNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation().asArray()
        val translation2 = createTranslation().asArray()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2[0] - translation1[0],
            translation2[1] - translation1[1],
            translation2[2] - translation1[2]
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2)
        val deltaTransformation =
            EuclideanTransformation3D(deltaAttitude, deltaTranslation.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2[0].toFloat()
        val y2 = translation2[1].toFloat()
        val z2 = translation2[2].toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        val frameResult1 = NEDFrame()
        val frameResult2 = NEDFrame()
        val attitudeResult = Quaternion()
        val translationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val deltaAttitudeResult = Quaternion()
        val deltaTranslationResult = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        val transformationResult = EuclideanTransformation3D()
        val deltaTransformationResult = EuclideanTransformation3D()
        val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        val endEcefPosition = ECEFPosition()
        val endNedPosition = NEDPosition()
        val endVelocity = NEDVelocity()

        val result1 = PoseHelper.convertToNED(
            values,
            startNedPosition,
            frameResult1,
            attitudeResult,
            translationResult,
            deltaAttitudeResult,
            deltaTranslationResult,
            transformationResult,
            deltaTransformationResult,
            rotationMatrix,
            endEcefPosition,
            endNedPosition,
            endVelocity,
            TIME_INTERVAL
        )

        val result2 = PoseHelper.convertToNED(
            values,
            startEcefPosition,
            frameResult2,
            attitudeResult,
            translationResult,
        )

        // check
        assertEquals(SEQUENCE_NUMBER, result1)
        assertEquals(SEQUENCE_NUMBER, result2)
        assertEquals(attitudeResult, transformationResult.rotation)
        assertArrayEquals(translationResult, transformationResult.translation, 0.0)
        assertEquals(deltaAttitudeResult, deltaTransformationResult.rotation)
        assertArrayEquals(deltaTranslationResult, deltaTransformationResult.translation, 0.0)
        assertTrue(
            transformationResult.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )

        val endEcefPosition2 = ECEFPosition(
            startEcefPosition.x + translationResult[0],
            startEcefPosition.y + translationResult[1],
            startEcefPosition.z + translationResult[2]
        )
        assertTrue(rotationMatrix.equals(attitudeResult.asInhomogeneousMatrix(), ABSOLUTE_ERROR))
        assertTrue(endEcefPosition.equals(endEcefPosition2, ABSOLUTE_ERROR))
        val endNedPosition2 = NEDPosition()
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult[0] / TIME_INTERVAL,
            deltaTranslationResult[1] / TIME_INTERVAL,
            deltaTranslationResult[2] / TIME_INTERVAL
        )
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition2,
            endNedVelocity
        )
        assertTrue(endNedPosition2.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(endVelocity.equals(endNedVelocity, ABSOLUTE_ERROR))
        val c = CoordinateTransformation(
            rotationMatrix,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frameResult3 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frameResult1.equals(frameResult3, ABSOLUTE_ERROR))
        assertEquals(frameResult1.coordinateTransformation, frameResult3.coordinateTransformation)
        assertEquals(frameResult1.position, frameResult2.position)
        assertEquals(NEDVelocity(), frameResult2.velocity)
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_POS = -10.0
        const val MAX_POS = 10.0

        const val SEQUENCE_NUMBER = 1.0f

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 100.0

        const val TIME_INTERVAL = 0.02

        fun createNEDPosition(): NEDPosition {
            val randomizer = UniformRandomizer()
            val latitude =
                Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
            val longitude =
                Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
            return NEDPosition(latitude, longitude, height)
        }

        fun createQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }

        fun createTranslation(): Point3D {
            val randomizer = UniformRandomizer()
            val x = randomizer.nextDouble(MIN_POS, MAX_POS)
            val y = randomizer.nextDouble(MIN_POS, MAX_POS)
            val z = randomizer.nextDouble(MIN_POS, MAX_POS)
            return InhomogeneousPoint3D(x, y, z)
        }
    }
}