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
package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.algebra.AlgebraException
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.SingularValueDecomposer
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import kotlin.math.max

/**
 * Estimates external acceleration covariance matrix using Suh's method.
 *
 * @property accelerometerNoiseStandardDeviation standard deviation for accelerometer noise
 * expressed in m/s^2.
 */
class SuhExternalAccelerationCovarianceMatrixEstimator(
    accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION
) : ExternalAccelerationCovarianceMatrixEstimator(accelerometerNoiseStandardDeviation) {

    // ra
    /**
     * Matrix containing last [SUH_M1] accelerometer residuals.
     *
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    var accelerometerResiduals = Matrix(AccelerationTriad.COMPONENTS, SUH_M1)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == AccelerationTriad.COMPONENTS)
            require(value.columns == SUH_M1)
            field.copyFrom(value)
        }

    // ra'
    /**
     * Transposed of [accelerometerResiduals].
     */
    private val accelerometerResidualsTransposed = Matrix(SUH_M1, AccelerationTriad.COMPONENTS)

    // temp
    /**
     * Temporary value containing scaled product of ra * ra', which is equivalent to the
     * covariance matrix of [accelerometerResiduals].
     */
    private val accelerometerU = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    // lambda
    /**
     * Contains last [SUH_M2] singular values of [accelerometerU].
     */
    private val accelerometerLambda = Matrix(AccelerationTriad.COMPONENTS, SUH_M2 + 1)

    /**
     * Contains last [SUH_M2] average values of accelerometer residuals.
     */
    private val accelerometerMu = Matrix(AccelerationTriad.COMPONENTS, SUH_M2 + 1)

    /**
     * Decomposes [accelerometerU] in singular values.
     */
    private val singularValueDecomposer = SingularValueDecomposer()

    /**
     * Transposed right singular vectors.
     */
    private val vTransposed = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Contains one right singular vector of [accelerometerMu].
     */
    private val u = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Transposed of [u].
     */
    private val uTransposed = Matrix(1, AccelerationTriad.COMPONENTS)

    /**
     * Temporary value to compute [accelerometerMu].
     */
    private val accelerometerMuTmp3 = Matrix(1, AccelerationTriad.COMPONENTS)

    /**
     * Temporary value to compute [accelerometerMu].
     */
    private val accelerometerMuTmp4 = Matrix(1, 1)

    /**
     * Temporary value to estimate [qab].
     */
    private val qabTemp = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Resets this estimator.
     */
    fun reset() {
        accelerometerResiduals.initialize(0.0)
    }

    /**
     * Computes estimation of external acceleration covariance matrix.
     *
     * @throws AlgebraException if there are numerical errors or instabilities.
     */
    @Throws(AlgebraException::class)
    override fun estimate() {
        // ra
        accelerometerResiduals.transpose(accelerometerResidualsTransposed)

        // ra * ra'
        accelerometerResiduals.multiply(accelerometerResidualsTransposed, accelerometerU)
        // U = 1 / M1 * (ra * ra')
        accelerometerU.multiplyByScalar(1.0 / SUH_M1)

        circularShiftByOneColumn(accelerometerLambda)
        circularShiftByOneColumn(accelerometerMu)

        singularValueDecomposer.inputMatrix = accelerometerU
        singularValueDecomposer.decompose()

        val singularValues = singularValueDecomposer.singularValues
        accelerometerLambda.setSubmatrix(0, 0, 2, 0, singularValues)

        // u = v
        val v = singularValueDecomposer.v
        v.transpose(vTransposed)

        /*
            % [Eq. post-32 Suh]
            for i = 1:3 % NB: k = 1 is the most recent sample, whereas k = (M_2 + 1) is the oldest one
                mu(i,1) = u(:,i)' * (H_a * P__next_prev * H_a' + R_a) * u(:,i);
            end
         */
        computePartialAccelerometerResidualCovarianceMatrix()

        for (i in 0 until SUH_M2 + 1) {
            v.getSubmatrix(0, i, AccelerationTriad.COMPONENTS - 1, i, u)
            u.transpose(uTransposed)

            //first column (k = 0) of accelerometerMu contains most recent sample, whereas
            // k = SUH_M2 is the oldest one
            uTransposed.multiply(partialAccelerometerResidualCovarianceMatrix, accelerometerMuTmp3)
            // accelerometerMuTmp4 is 1x1
            accelerometerMuTmp3.multiply(u, accelerometerMuTmp4)

            accelerometerMu.setElementAt(i, 0, accelerometerMuTmp4.getElementAtIndex(0))
        }

        // Compute acceleration mode (it is able to detect external acceleration)
        when (computeAccelerationMode()) {
            AccelerationMode.EXTERNAL_ACCELERATION_MODE -> {
                qab.initialize(0.0)
                for (i in 0 until AccelerationTriad.COMPONENTS) {
                    val scale = max(
                        accelerometerLambda.getElementAt(i, 0) - accelerometerMu.getElementAt(i, 0),
                        0.0
                    )

                    v.getSubmatrix(0, i, AccelerationTriad.COMPONENTS - 1, i, u)
                    u.transpose(uTransposed)

                    u.multiply(uTransposed, qabTemp)

                    qabTemp.multiplyByScalar(scale)

                    qab.add(qabTemp)
                }
            }

            AccelerationMode.NO_EXTERNAL_ACCELERATION_MODE -> {
                qab.initialize(0.0)
            }
        }
    }

    /**
     * Computes acceleration mode depending on whether external acceleration is detected or not.
     *
     * @return acceleration mode.
     */
    private fun computeAccelerationMode(): AccelerationMode {
        for (j in SUH_M2 downTo 0) {
            var maxTemp = -Double.MAX_VALUE
            for (i in 0 until AccelerationTriad.COMPONENTS) {
                maxTemp = max(
                    accelerometerLambda.getElementAt(i, j) - accelerometerMu.getElementAt(i, j),
                    maxTemp
                )
            }
            if (maxTemp >= SUH_GAMMA) {
                // external acceleration detected
                return AccelerationMode.EXTERNAL_ACCELERATION_MODE
            }
        }

        // no external acceleration detected
        return AccelerationMode.NO_EXTERNAL_ACCELERATION_MODE
    }

    /**
     * Indicates acceleration mode.
     */
    private enum class AccelerationMode {
        /**
         * No external acceleration is detected.
         */
        NO_EXTERNAL_ACCELERATION_MODE,

        /**
         * External acceleration is detected.
         */
        EXTERNAL_ACCELERATION_MODE
    }

    companion object {

        /**
         * Default standard deviation for accelerometer noise expressed in m/s^2.
         */
        const val DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION = 0.015

        /**
         * Number of previous residuals being kept.
         */
        private const val SUH_M1 = 3

        /**
         * Number of previous residuals being kept minus one.
         */
        private const val SUH_M2 = 2

        /**
         * Threshold to detect external acceleration.
         */
        private const val SUH_GAMMA = 0.1
    }
}