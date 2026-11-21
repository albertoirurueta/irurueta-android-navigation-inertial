package com.irurueta.android.navigation.inertial.processors.attitude.kalman

import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.processors.attitude.PhiMatrixMethod
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.numerical.ExponentialMatrixEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert
import org.junit.Test

class ApproximatedPhiMatrixEstimatorTest {
    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val estimator = ApproximatedPhiMatrixEstimator()

        Assert.assertNull(estimator.a)
        Assert.assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
    }

    @Test
    fun constructor_whenValidMatrix_returnsExpectedValues() {
        val a = Matrix(ROWS, COLUMNS)
        val estimator = ApproximatedPhiMatrixEstimator(a)

        Assert.assertSame(a, estimator.a)
        Assert.assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
    }

    @Test
    fun constructor_whenInvalidARows_throwsIllegalArgumentException() {
        val a = Matrix(1, COLUMNS)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            ApproximatedPhiMatrixEstimator(a)
        }
    }

    @Test
    fun constructor_whenInvalidAColumns_throwsIllegalArgumentException() {
        val a = Matrix(ROWS, 1)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            ApproximatedPhiMatrixEstimator(a)
        }
    }

    @Test
    fun a_whenInvalidRows_throwsIllegalArgumentException() {
        val estimator = ApproximatedPhiMatrixEstimator()

        // check initial value
        Assert.assertNull(estimator.a)

        val a = Matrix(1, COLUMNS)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            estimator.a = a
        }
    }

    @Test
    fun a_whenInvalidColumns_throwsIllegalArgumentException() {
        val estimator = ApproximatedPhiMatrixEstimator()

        // check initial value
        Assert.assertNull(estimator.a)

        val a = Matrix(ROWS, 1)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            estimator.a = a
        }
    }

    @Test
    fun a_whenValid_setsExpectedValue() {
        val estimator = ApproximatedPhiMatrixEstimator()

        // check initial value
        Assert.assertNull(estimator.a)

        // set new value
        val a = Matrix(ROWS, COLUMNS)
        estimator.a = a

        // check
        Assert.assertSame(a, estimator.a)
    }

    @Test
    fun method_returnsExpectedValue() {
        val estimator = ApproximatedPhiMatrixEstimator()

        Assert.assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
    }

    @Test
    fun estimate_returnsExpectedResult() {
        val a = generateProcessEquationMatrix()

        val estimator = ApproximatedPhiMatrixEstimator(a)

        val result = Matrix(ROWS, COLUMNS)
        estimator.estimate(TIME_INTERVAL, result)

        val expected = Matrix.identity(ROWS, COLUMNS)
            .addAndReturnNew(a.multiplyByScalarAndReturnNew(TIME_INTERVAL))
            .addAndReturnNew(a.multiplyAndReturnNew(a)
                .multiplyByScalarAndReturnNew(0.5 * TIME_INTERVAL * TIME_INTERVAL))

        Assert.assertEquals(expected, result)

        // precise result
        val exponentialEstimator = ExponentialMatrixEstimator()
        val at = a.multiplyByScalarAndReturnNew(TIME_INTERVAL)
        val expAt = exponentialEstimator.exponential(at)

        Assert.assertTrue(expAt.equals(result, ABSOLUTE_ERROR))
    }

    @Test
    fun estimate_whenNoMatrix_throwsIllegalStateException() {
        val estimator = ApproximatedPhiMatrixEstimator()

        val result = Matrix(ROWS, COLUMNS)
        Assert.assertThrows(IllegalStateException::class.java) {
            estimator.estimate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun estimate_whenInvalidResultRows_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()

        val estimator = PrecisePhiMatrixEstimator(a)

        val result = Matrix(1, COLUMNS)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            estimator.estimate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun estimate_whenInvalidResultColumns_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()

        val estimator = PrecisePhiMatrixEstimator(a)

        val result = Matrix(ROWS, 1)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            estimator.estimate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun estimate_whenNegativeTimeInterval_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()

        val estimator = PrecisePhiMatrixEstimator(a)

        val result = Matrix(ROWS, COLUMNS)
        Assert.assertThrows(IllegalArgumentException::class.java) {
            estimator.estimate(-TIME_INTERVAL, result)
        }
    }

    private companion object {
        const val ROWS = 9

        const val COLUMNS = 9

        const val MIN_ANGULAR_SPEED = -Math.PI

        const val MAX_ANGULAR_SPEED = Math.PI

        const val TIME_INTERVAL = 0.02

        const val ABSOLUTE_ERROR = 1e-3

        fun generateProcessEquationMatrix(): Matrix {
            val randomizer = UniformRandomizer()
            val wx = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val wy = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val wz = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val angularSpeedTriad =
                AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, wx, wy, wz)
            val angularSpeedMatrix = angularSpeedTriad.valuesAsMatrix
            val angularSpeedSkewMatrix = Utils.skewMatrix(angularSpeedMatrix)
            angularSpeedSkewMatrix.multiplyByScalar(-1.0)

            val a = Matrix(ROWS, COLUMNS)
            a.setSubmatrix(0, 0, 2, 2, angularSpeedSkewMatrix)

            val identity3 = Matrix.identity(3, 3)
            identity3.multiplyByScalar(-0.5)
            a.setSubmatrix(0, 3, 2, 5, identity3)

            return a
        }
    }
}