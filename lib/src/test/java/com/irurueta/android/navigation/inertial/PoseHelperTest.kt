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

import com.irurueta.geometry.*
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
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        assertTrue(
            deltaTransformationResult.asMatrix()
                .equals(deltaTransformation.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
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

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_POS = -10.0
        const val MAX_POS = 10.0

        const val SEQUENCE_NUMBER = 1.0f

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