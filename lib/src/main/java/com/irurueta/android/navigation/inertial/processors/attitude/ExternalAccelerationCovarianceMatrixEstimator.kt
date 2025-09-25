/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.algebra.AlgebraException
import com.irurueta.algebra.Matrix
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import kotlin.math.pow

/**
 * Estimates external acceleration covariance matrix.
 *
 * @property accelerometerNoiseStandardDeviation standard deviation for accelerometer noise
 * expressed in m/s^2.
 */
abstract class ExternalAccelerationCovarianceMatrixEstimator(
    val accelerometerNoiseStandardDeviation: Double
) {
    /**
     * Kalman's filter post-predicted state covariance matrix.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x9.
     */
    var kalmanErrorCovPostPredicted =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            field.copyFrom(value)
        }

    /**
     * Accelerometer measurement matrix
     *
     * @throws IllegalArgumentException if provided matrix is not 3x9
     */
    var ha = Matrix(AccelerationTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == AccelerationTriad.COMPONENTS)
            require(value.columns == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            field.copyFrom(value)
        }

    /**
     * Estimated external acceleration covariance matrix.
     */
    val qab: Matrix = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Contains partial accelerometer residual covariance matrix.
     */
    val partialAccelerometerResidualCovarianceMatrix =
        Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Transpose of [ha].
     */
    val haTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, AccelerationTriad.COMPONENTS)

    /**
     * Covariance measurement matrix of accelerometer measurements expressed in (m/s^2)^2.
     */
    private val ra =
        Matrix.identity(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)
            .multiplyByScalarAndReturnNew(accelerometerNoiseStandardDeviation.pow(2.0))

    /**
     * Temporary value to compute [partialAccelerometerResidualCovarianceMatrix].
     */
    private val accelerometerMuTmp =
        Matrix(AccelerationTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Computes estimation of external acceleration covariance matrix.
     *
     * @throws AlgebraException if there are numerical errors or instabilities.
     */
    @Throws(AlgebraException::class)
    abstract fun estimate()

    /**
     * Computes partial accelerometer residual covariance matrix.
     * This is used to estimate external acceleration covariance matrix
     */
    protected fun computePartialAccelerometerResidualCovarianceMatrix() {
        ha.transpose(haTransposed)

        // H_a * P__next_prev
        ha.multiply(kalmanErrorCovPostPredicted, accelerometerMuTmp)
        // H_a * P__next_prev * H_a'
        accelerometerMuTmp.multiply(haTransposed, partialAccelerometerResidualCovarianceMatrix)
        // H_a * P__next_prev * H_a' + R_a
        partialAccelerometerResidualCovarianceMatrix.add(ra)
    }

    companion object {
        /**
         * Default standard deviation for accelerometer noise expressed in m/s^2.
         */
        const val DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION = 0.015

        /**
         * Kalman filter's number of dynamic parameters.
         * Filter state contains:
         * - error of orientation quaternion respect to Earth (in local navigation system expressed
         * in NED coordinates). Because error is small, real value is assumed to be 1, and only 3
         * axis components are store.
         * - bias of gyroscope respect to body expressed in NED coordinates. 3x1 vector
         * - bias of accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        internal const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 9

        /**
         * Shifts contents of matrix by one column to the right.
         *
         * @param matrix matrix to be shifted.
         */

        // shift one matrix column to the right
        fun circularShiftByOneColumn(matrix: Matrix) {
            val rows = matrix.rows
            val columns = matrix.columns
            for (i in 0 until rows) {
                var prevValue = matrix.getElementAt(i, columns - 1)
                for (j in 0 until columns) {
                    val value = matrix.getElementAt(i, j)

                    matrix.setElementAt(i, j, prevValue)
                    prevValue = value
                }
            }
        }
    }
}