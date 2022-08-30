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
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.geometry.MatrixRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.mockk
import io.mockk.spyk
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeHelperTest {

    @Test
    fun convertToENU_whenAccuracyAvailable_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val quaternionResult = Quaternion()
        val result = AttitudeHelper.convertToENU(context, values, quaternionResult)
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        assertTrue(quaternionResult.equals(attitude, ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNEU_whenAccuracyNotAvailable_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val values = floatArrayOf(b, c, d, a, AttitudeHelper.UNAVAILABLE_HEADING_ACCURACY)

        val quaternionResult = Quaternion()
        assertNull(AttitudeHelper.convertToENU(context, values, quaternionResult))
        assertTrue(quaternionResult.equals(attitude, ABSOLUTE_ERROR))
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNEU_whenInvalidValuesSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val values = FloatArray(3)
        val quaternionResult = Quaternion()
        AttitudeHelper.convertToENU(context, values, quaternionResult)
    }

    @Test
    fun convertToNED_whenAccuracyAvailable_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val quaternionResult = Quaternion()
        val result = AttitudeHelper.convertToNED(context, values, quaternionResult)
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        assertTrue(quaternionResult.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenAccuracyNotAvailable_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val values = floatArrayOf(b, c, d, a, AttitudeHelper.UNAVAILABLE_HEADING_ACCURACY)

        val quaternionResult = Quaternion()
        assertNull(AttitudeHelper.convertToNED(context, values, quaternionResult))
        assertTrue(quaternionResult.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNED_whenInvalidValuesSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val values = FloatArray(3)
        val quaternionResult = Quaternion()
        AttitudeHelper.convertToNED(context, values, quaternionResult)
    }

    @Test
    fun convertToNED_whenCoordinateTransformation_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val result = AttitudeHelper.convertToNED(context, values, transformationResult)
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult = Quaternion()
        transformationResult.asRotation(quaternionResult)
        assertTrue(quaternionResult.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenCoordinateTransformationAndQuaternion_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val result =
            AttitudeHelper.convertToNED(context, values, transformationResult, quaternionResult1)
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionAndMatrixWithValidSize_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                null,
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)

        assertEquals(FrameType.BODY_FRAME, transformationResult.sourceType)
        assertEquals(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            transformationResult.destinationType
        )
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionMatrixWithValidSizeAndDisplayOrientation_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                Quaternion(),
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionAndMatrixWithInvalidRows_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            1,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                null,
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionMatrixWithInvalidRowsAndDisplayOrientation_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            1,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                Quaternion(),
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionAndMatrixWithInvalidColumns_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            1
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                null,
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)
    }

    @Test
    fun convertToNED_whenCoordinateTransformationQuaternionMatrixWithInvalidColumnsAndDisplayOrientation_returnsExpectedValue() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)

        val transformationResult = CoordinateTransformation(
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        val quaternionResult1 = Quaternion()
        val matrix1 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            1
        )
        val result =
            AttitudeHelper.convertToNED(
                context,
                values,
                transformationResult,
                quaternionResult1,
                Quaternion(),
                matrix1
            )
        requireNotNull(result)
        assertEquals(accuracy.toDouble(), result, ABSOLUTE_ERROR)
        val quaternionResult2 = Quaternion()
        transformationResult.asRotation(quaternionResult2)

        assertEquals(quaternionResult1, quaternionResult2)
        assertTrue(quaternionResult2.equals(attitude.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val matrix2 = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        quaternionResult1.toMatrixRotation(matrix2)
        assertEquals(matrix1, matrix2)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNED_whenCoordinateTransformationQuaternionMatrixAndInvalidValuesSize_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val values = FloatArray(3)
        val transformationResult = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val quaternionResult = Quaternion()
        val matrix = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        AttitudeHelper.convertToNED(
            context,
            values,
            transformationResult,
            quaternionResult,
            null,
            matrix
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertToNED_whenCoordinateTransformationQuaternionMatrixInvalidValuesSizeAndDisplayOrientation_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val values = FloatArray(3)
        val transformationResult = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val quaternionResult = Quaternion()
        val matrix = Matrix(
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
            MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
        )
        AttitudeHelper.convertToNED(
            context,
            values,
            transformationResult,
            quaternionResult,
            Quaternion(),
            matrix
        )
    }

    @Test
    fun checkCoordinateTransformationIsTheConjugateOfQuaternionWithSameEulerAngles() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val c = CoordinateTransformation(
            roll,
            pitch,
            yaw,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // q1 is the conjugate of q2 because Q2 is expressed in NED coordinates and Q1 in NEU coordinates
        val q1 = Quaternion(roll, pitch, yaw)
        val q2 = Quaternion()
        c.asRotation(q2)

        q1.normalize()
        q2.normalize()

        val q3 = q1.conjugateAndReturnNew()

        assertEquals(q3, q2)
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        fun createQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }
    }
}