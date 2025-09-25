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

import android.hardware.SensorManager
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import kotlin.math.pow

/**
 * Corrects Kalman state from accelerometer measurement.
 */
class KalmanAccelerometerCorrector(
    val externalAccelerationCovarianceMatrixEstimationMethod: ExternalAccelerationCovarianceMatrixEstimationMethod = DEFAULT_EXTERNAL_ACCELERATION_COVARIANCE_MATRIX_ESTIMATION_METHOD
) {
    /**
     * Standard deviation for accelerometer noise expressed in m/s^2.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION
        @Throws(IllegalArgumentException::class)
        set(value) {
            if (field != value) {
                require(value >= 0.0)
                Matrix.identity(ra)
                ra.multiplyByScalar(value.pow(2.0))
            }
            field = value
        }

    /**
     * Gets expected gravity norm at current location.
     * If no location is available, average gravity at sea level is returned instead.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gravityNorm: Double = EARTH_GRAVITY
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
            nedGravityMatrix.initialize(0.0)
            nedGravityMatrix.setElementAtIndex(2, value)
        }


    /**
     * Contains predicted leveled local NED attitude respect to Earth for next step.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
    private val predictedNedQ : Quaternion = Quaternion()

/*    var predictedNedQ: Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }*/

    /**
     * Contains predicted body attitude for next step.
     * Because rotation is defined respect to body, rotation converts from body coordinates
     * to leveled local NED coordinates respect to Earth:
     * pn = previousBodyQ * pb
     */
    var predictedBodyQ: Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }

    // y_a = C*g  b_a + n_a + a_b
    /**
     * Contains accelerometer data expressed in NED coordinates respect to body.
     * This is reused for performance reasons.
     */
    var nedBodyAccelerationTriad: AccelerationTriad = AccelerationTriad()
        set(value) {
            field.copyFrom(value)
        }

    /**
     * Kalman state error covariance after prediction stage.
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

    // x_hat__next_prev
    /**
     * Kalman state after prediction.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x1.
     */
    var kalmanStatePostPredicted = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == 1)
            field.copyFrom(value)
        }

    /**
     * Contains corrected leveled local NED attitude respect to Earth.
     */
//    val nedQ: Quaternion = Quaternion()

    /**
     * Contains corrected body attitude.
     */
    val bodyQ: Quaternion = Quaternion()

    /**
     * Accelerometer corrected Kalman error covariance.
     */
    val kalmanErrorCovAccelerometerCorrected =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // x_hat_a__next_next
    /**
     * Corrected Kalman state after processing accelerometer measurement.
     */
    val kalmanStateAccelerometerCorrected = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Contains gyroscope bias expressed in radians per second (rad/s).
     */
    val bg = Matrix(AngularSpeedTriad.COMPONENTS, 1)

    /**
     * Contains accelerometer bias expressed in meters per squared second (m/s^2).
     */
    val ba = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Contains external acceleration in body coordinates.
     */
    val externalAcceleration = AccelerationTriad()

    /**
     * Jacobian of [predictedNedQ] respect to estimated [errorQ].
     */
    val predictedNedQJacobian: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Column vector representing gravity in local leveled NED coordinates.
     */
    private val nedGravityMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Contains external acceleration in body coordinates expressed in meters per squared second
     * (m/s^2) as a column matrix.
     */
    private val externalAccelerationMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Contains inverse of [predictedNedQ].
     */
//    private val invPredictedNedQ = Quaternion()

    /**
     * Contains inverse of predicted NED attitude for next step in matrix form.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
/*    private val invPredictedNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)*/

    /**
     * Contains predicted NED attitude for next step in matrix form.
     * Converts from leveled NED local coordinates to body NED coordinates.
     * pb = predictedNedRotationMatrix * pn
     */
    private val predictedNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Column vector representing gravity in body coordinates taking into account current
     * orientation.
     */
    private val bodyGravityMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Skew matrix representation of gravity expressed in body coordinates.
     */
    private val skewBodyGravityMatrix: Matrix =
        Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Accelerometer measurement matrix.
     */
    private val ha = Matrix(AccelerationTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Transpose of [ha].
     */
    private val haTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, AccelerationTriad.COMPONENTS)

    /**
     * 3x3 identity matrix.
     */
    private val identity3: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Contains [nedBodyAccelerationTriad] in matrix form.
     */
    private val bodyAccelerationMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    //za
    /**
     * Contains accelerometer measurement error.
     */
    private val accelerometerMeasurementError = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Accelerometer residual covariance matrix.
     */
    private val sa = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Inverse of [sa].
     */
    private val invSa = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Accelerometer Kalman gain.
     */
    private val ka = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, AccelerationTriad.COMPONENTS)

    /**
     * Transpose of [ka]
     */
    private val kaTransposed =
        Matrix(AccelerationTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // ra
    /**
     * Matrix containing last [SUH_M1] accelerometer residuals.
     */
    private val accelerometerResiduals = Matrix(AccelerationTriad.COMPONENTS, SUH_M1)

    /**
     * Contains the most recent residual in the accelerometer measurement update.
     */
    private val accelerometerResidual = Matrix(AccelerationTriad.COMPONENTS, 1)

    // temp = eye(9) - K_a * H_a;
    /**
     * Temporary value to update accelerometer error covariance matrix.
     */
    private val aTemp = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * 9x9 identity matrix.
     */
    private val identity9 =
        Matrix.identity(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Transposed of [aTemp].
     */
    private val aTempTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * External acceleration covariance matrix
     */
    private val qab = Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Covariance measurement matrix of accelerometer measurements expressed in (m/s^2)^2.
     */
    private val ra =
        Matrix.identity(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)
            .multiplyByScalarAndReturnNew(accelerometerNoiseStandardDeviation.pow(2.0))

    /**
     * Temporary value to compute accelerometer corrected error covariance.
     */
    private val errorCovAccelerometerCorrectedTmp1 =
        Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    /**
     * Temporary value to compute accelerometer corrected error covariance.
     */
    private val errorCovAccelerometerCorrectedTmp2 =
        Matrix(AccelerationTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary value to compute accelerometer corrected error covariance.
     */
    private val errorCovAccelerometerCorrectedTmp3 =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary value to compute accelerometer corrected error covariance.
     */
    private val errorCovAccelerometerCorrectedTmp4 =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Quaternion containing error component of estimation.
     */
    private val errorQ: Quaternion = Quaternion()

    /**
     * External acceleration covariance matrix estimator using Suh's method.
     */
    private val suhQabEstimator =
        SuhExternalAccelerationCovarianceMatrixEstimator(accelerometerNoiseStandardDeviation)

    /**
     * External acceleration covariance matrix estimator using Sabatini's method.
     */
    private val sabatiniQabEstimator = SabatiniExternalAccelerationCovarianceMatrixEstimator()

    /**
     * Resets this corrector instance to its initial state.
     */
    fun reset() {
        accelerometerResiduals.initialize(0.0)
        qab.initialize(0.0)
    }

    /**
     * Computes accelerometer measurement correction.
     */
    fun correct() {
        // compute accelerometer measurement matrix
        predictedBodyQ.normalize()
//        predictedNedQ.normalize()
        //predictedNedQ.inverse(invPredictedNedQ)
        predictedBodyQ.inverse(predictedNedQ)
        predictedNedQ.normalize()
//        predictedBodyQ.toMatrixRotation(predictedNedRotationMatrix)
        predictedNedQ.toMatrixRotation(predictedNedRotationMatrix)
//        predictedNedQ.fromQuaternion(invPredictedNedQ)
//        invPredictedNedQ.toMatrixRotation(invPredictedNedRotationMatrix)

        predictedNedRotationMatrix.multiply(nedGravityMatrix, bodyGravityMatrix)
//        invPredictedNedRotationMatrix.multiply(nedGravityMatrix, bodyGravityMatrix)
        Utils.skewMatrix(bodyGravityMatrix, skewBodyGravityMatrix)
        skewBodyGravityMatrix.multiplyByScalar(2.0)

        ha.initialize(0.0)
        ha.setSubmatrix(0, 0, 2, 2, skewBodyGravityMatrix)
        ha.setSubmatrix(0, 6, 2, 8, identity3)

        // compute accelerometer measurement error (za)
        nedBodyAccelerationTriad.getValuesAsMatrix(bodyAccelerationMatrix)
        bodyAccelerationMatrix.subtract(bodyGravityMatrix, accelerometerMeasurementError)

        // Estimate external acceleration covariance matrix
        // Compute accelerometer residual covariance matrix
        estimateExternalAccelerationCovarianceMatrixAndAccelerometerResidualCovarianceMatrix()

        // Compute accelerometer Kalman gain
        /*
        % [Eq. 19a Suh]
        % K_a: Accelerometer Kalman gain
        K_a = P__next_prev * H_a' / S_a;
         */
        // TODO: no need to transpose again --> ha.transpose(haTransposed)
        Utils.inverse(sa, invSa)
        kalmanErrorCovPostPredicted.multiply(haTransposed, ka)
        ka.multiply(invSa)

        /*
        % shift each row of r_a 1 position to the right (i.e., age r_a's of 1 sample)
        r_a = circshift(r_a,1,2);
        */
        ExternalAccelerationCovarianceMatrixEstimator.circularShiftByOneColumn(
            accelerometerResiduals
        )

        /*
        %% 6. Compute residual in the accelerometer measurement update:
        % [Eq. 19b (partial) Suh and Eq. pre-29 Suh]
        % r_a: Residual in the accelerometer measurement update
        r_a(:,1) = z_a - H_a * x_hat__next_prev;     % r_a(:,1) = the most recent r_a
         */
        ha.multiply(kalmanStatePostPredicted, accelerometerResidual)
        accelerometerResidual.multiplyByScalar(-1.0)
        accelerometerResidual.add(accelerometerMeasurementError)
        accelerometerResiduals.setSubmatrix(0, 0, 2, 0, accelerometerResidual)

        /*
        %% 7. Update the state vector:
        % [Eq. 19b Suh]
        % x_hat_a__next_next: Updated state
        x_hat_a__next_next = x_hat__next_prev + K_a * (r_a(:,1));
        */
        ka.multiply(accelerometerResidual, kalmanStateAccelerometerCorrected)
        kalmanStateAccelerometerCorrected.add(kalmanStatePostPredicted)

        // Kalman state contains: [q_e; b_g; b_a]

        // obtain gyroscope bias
        kalmanStateAccelerometerCorrected.getSubmatrix(3, 0, 5, 0, bg)

        // obtain accelerometer bias
        kalmanStateAccelerometerCorrected.getSubmatrix(6, 0, 8, 0, ba)

        // obtain external acceleration (a_b)
        // y_a = C*g + b_a + n_a + a_b
        // y_a = measured body acceleration (nedBodyAccelerationTriad)
        // C = predicted NED attitude (predictedNedQ)
        // g = gravity in NED coordinates (nedGravityMatrix)
        // b_a = accelerometer bias (ba)
        // n_a = accelerometer noise
        // a_b = external acceleration (externalAccelerationMatrix)

        // C*g -> bodyGravityMatrix

        // a_b = y_a - C*g - b_a - n_a
        bodyAccelerationMatrix.subtract(bodyGravityMatrix, externalAccelerationMatrix)
        externalAccelerationMatrix.subtract(ba)
        externalAcceleration.setValueCoordinates(externalAccelerationMatrix)

        /*
        %% 8. Update the covariance matrix:
        % [Eq. 19c Suh]
        % P_a__next_next: State covariance matrix
        temp = eye(9) - K_a * H_a;
        */
        ka.multiply(ha, aTemp)
        aTemp.multiplyByScalar(-1.0)
        aTemp.add(identity9)

        /*
        P_a__next_next = temp * P__next_prev * temp' + K_a * (R_a + Q_hat_a_b) * K_a';
        */
        aTemp.transpose(aTempTransposed)
        ka.transpose(kaTransposed)

        // R_a + Q_hat_a_b
        qab.add(ra, errorCovAccelerometerCorrectedTmp1)

        //  K_a * (R_a + Q_hat_a_b) * K_a'
        errorCovAccelerometerCorrectedTmp1.multiply(
            kaTransposed,
            errorCovAccelerometerCorrectedTmp2
        )
        ka.multiply(errorCovAccelerometerCorrectedTmp2, errorCovAccelerometerCorrectedTmp3)

        // temp * P__next_prev * temp'
        kalmanErrorCovPostPredicted.multiply(
            aTempTransposed,
            errorCovAccelerometerCorrectedTmp4
        )

        aTemp.multiply(errorCovAccelerometerCorrectedTmp4, kalmanErrorCovAccelerometerCorrected)
        kalmanErrorCovAccelerometerCorrected.add(errorCovAccelerometerCorrectedTmp3)

        /*
        %% 9. Update the attitude quaternion

        % [Eq. 20a Suh]
        % Extract the imaginary part of the attitude error quaternion from the state vector
        % q_e : Orientation error (3x1) vector (i.e, the vector part of the error quaternion)
        q_e = x_hat_a__next_next(1:3);
        */
        /*
        % [Eq. 6 Suh]
        % Create the attitude error quaternion
        % q_tilde_e: Orientation error (4x1) quaternion
        q_tilde_e = [1; q_e];
        */
        errorQ.a = 1.0
        errorQ.b = kalmanStateAccelerometerCorrected.getElementAtIndex(0)
        errorQ.c = kalmanStateAccelerometerCorrected.getElementAtIndex(1)
        errorQ.d = kalmanStateAccelerometerCorrected.getElementAtIndex(2)

//        errorQ.normalize()

        /*
        % [Eq. 20b Suh] (and [Eq. 5 Suh])
        % Compute the new attitude quaternion
        q_hat__next_next = quatMultiplication(q_hat__next_prev, q_tilde_e);
        */
        Quaternion.product(predictedBodyQ, errorQ, bodyQ, predictedNedQJacobian, null)
//        Quaternion.product(predictedNedQ, errorQ, nedQ, predictedNedQJacobian, null)

        /*
        % [Eq. 20c Suh]
        % Normalize the quaternion
        q_hat__next_next = q_hat__next_next / norm(q_hat__next_next);
        */
        bodyQ.normalize()
//        nedQ.normalize()

        /*
        % Ensure quaternion scalar part is non-negative:
        if (q_hat__next_next(1) < 0)
            q_hat__next_next = -q_hat__next_next;
        end
        */
        if (bodyQ.a < 0.0) {
            bodyQ.a *= -1.0
            bodyQ.b *= -1.0
            bodyQ.c *= -1.0
            bodyQ.d *= -1.0
        }

/*        if (nedQ.a < 0.0) {
            nedQ.a *= -1.0
            nedQ.b *= -1.0
            nedQ.c *= -1.0
            nedQ.d *= -1.0
        }*/

        /*
        % [Eq. 20d Suh]
        % Set q_e to zero (i.e., set the first 3 elements of the state array to 0)
        x_hat_a__next_next(1:3) = zeros(3,1);
        */
        for (i in 0 until AccelerationTriad.COMPONENTS) {
            kalmanStateAccelerometerCorrected.setElementAtIndex(i, 0.0)
        }
    }

    /**
     * Computes external acceleration covariance matrix and accelerometer residual covariance
     * matrix.
     */
    private fun estimateExternalAccelerationCovarianceMatrixAndAccelerometerResidualCovarianceMatrix() {
        when (externalAccelerationCovarianceMatrixEstimationMethod) {
            ExternalAccelerationCovarianceMatrixEstimationMethod.SUH -> {
                suhQabEstimator.ha = ha
                suhQabEstimator.kalmanErrorCovPostPredicted = kalmanErrorCovPostPredicted
                suhQabEstimator.accelerometerResiduals = accelerometerResiduals

                suhQabEstimator.estimate()

                qab.copyFrom(suhQabEstimator.qab)

                sa.copyFrom(suhQabEstimator.partialAccelerometerResidualCovarianceMatrix)
                haTransposed.copyFrom(suhQabEstimator.haTransposed)
            }

            ExternalAccelerationCovarianceMatrixEstimationMethod.SABATINI -> {
                sabatiniQabEstimator.bodyAccelerationMatrix = bodyAccelerationMatrix
                sabatiniQabEstimator.gravity = gravityNorm

                sabatiniQabEstimator.ha = ha
                sabatiniQabEstimator.kalmanErrorCovPostPredicted = kalmanErrorCovPostPredicted

                sabatiniQabEstimator.estimate()

                qab.copyFrom(sabatiniQabEstimator.qab)

                sa.copyFrom(sabatiniQabEstimator.partialAccelerometerResidualCovarianceMatrix)
                haTransposed.copyFrom(sabatiniQabEstimator.haTransposed)
            }

            ExternalAccelerationCovarianceMatrixEstimationMethod.ZEROES -> {
                sabatiniQabEstimator.ha = ha
                sabatiniQabEstimator.kalmanErrorCovPostPredicted = kalmanErrorCovPostPredicted

                sabatiniQabEstimator.estimate()

                // always assume no external acceleration
                qab.initialize(0.0)

                sa.copyFrom(sabatiniQabEstimator.partialAccelerometerResidualCovarianceMatrix)
                haTransposed.copyFrom(sabatiniQabEstimator.haTransposed)
            }
        }

        // Compute accelerometer residual covariance matrix
        /*
        % [Eq. 19a (partial) Suh]
        % S_a: Accelerometer residual covariance matrix
        S_a = H_a * P__next_prev * H_a' + R_a + Q_hat_a_b;
        */
        sa.add(qab)
    }

    init {
        nedGravityMatrix.initialize(0.0)
        nedGravityMatrix.setElementAtIndex(2, gravityNorm)
    }

    companion object {
        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Default standard deviation for accelerometer noise expressed in m/s^2.
         */
        const val DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION = 0.015

        /**
         * Default external acceleration covariance matrix estimation method.
         */
        val DEFAULT_EXTERNAL_ACCELERATION_COVARIANCE_MATRIX_ESTIMATION_METHOD =
            ExternalAccelerationCovarianceMatrixEstimationMethod.SUH

        /**
         * Kalman filter's number of dynamic parameters.
         * Filter state contains:
         * - error of orientation quaternion respect to Earth (in local navigation system expressed
         * in NED coordinates). Because error is small, real value is assumed to be 1, and only 3
         * axis components are store.
         * - bias of gyroscope respect to body expressed in NED coordinates. 3x1 vector
         * - bias of accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 9

        /**
         * Number of previous residuals being kept.
         */
        private const val SUH_M1 = 3
    }
}