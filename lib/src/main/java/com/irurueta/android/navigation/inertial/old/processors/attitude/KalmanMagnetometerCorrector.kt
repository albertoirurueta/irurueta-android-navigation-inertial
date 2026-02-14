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

import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import kotlin.math.pow

/**
 * Corrects Kalman state from magnetometer measurement.
 */
class KalmanMagnetometerCorrector {

    /**
     * Standard deviation for magnetometer noise expressed in Teslas (T).
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var magnetometerNoiseStandardDeviation: Double = DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION
        @Throws(IllegalArgumentException::class)
        set(value) {
            if (field != value) {
                require(value >= 0.0)
                Matrix.identity(rm)
                rm.multiplyByScalar(value.pow(2.0))
            }
            field = value
        }

    /**
     * Gets or sets corrected leveled local NED attitude respect to Earth.
     */
    private val correctedNedQ: Quaternion = Quaternion()
/*    var correctedNedQ: Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }*/

    /**
     * Gets or sets corrected body attitude.
     */
    var correctedBodyQ: Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }

    /**
     * Gets or sets magnetometer data expressed in NED coordinates respect to body.
     */
    var nedBodyMagneticFluxDensityTriad: MagneticFluxDensityTriad = MagneticFluxDensityTriad()
        set(value) {
            field.copyFrom(value)
        }

    /**
     * Accelerometer corrected Kalman error covariance.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x9
     */
    var kalmanErrorCovAccelerometerCorrected =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            field.copyFrom(value)
        }

    // x_hat_a__next_next
    /**
     * Corrected Kalman state after processing accelerometer measurement.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x1
     */
    var kalmanStateAccelerometerCorrected = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == 1)
            field.copyFrom(value)
        }

    /**
     * Corrected Kalman state after processing magnetometer measurement.
     */
    val kalmanStateMagnetometerCorrected = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Magnetometer corrected Kalman error covariance.
     */
    val kalmanErrorCovMagnetometerCorrected =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains inverse of corrected leveled localNED attitude respect to Earth in matrix form.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
//    private val invCorrectedNedRotationMatrix: Matrix =
//        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Contains corrected leveled local NED attitude respect to Earth in matrix form.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
    private val correctedNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Magnetometer measurement matrix.
     */
    private val hm = Matrix(MagneticFluxDensityTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Transposed of [hm].
     */
    private val hmTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Contains [nedBodyMagneticFluxDensityTriad] in matrix form obtained from magnetometer sensor.
     */
    private val measuredBodyMagneticFluxDensityMatrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Expected magnetic flux density based on current location and orientation.
     */
    private val expectedBodyMagneticFluxDensityMatrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Skew matrix representation of magnetic flux density expressed in body coordinates.
     */
    private val skewBodyMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Magnetic flux density at current location expressed in local NED reference system.
     */
    var magneticFluxDensity = MagneticFluxDensityTriad()
        set(value) {
            field.copyFrom(value)
        }

    /**
     * Contains inverse of [correctedNedQ].
     */
//    private val invCorrectedNedQ = Quaternion()

    /**
     * [magneticFluxDensity] expressed in column matrix form.
     */
    private val magneticFluxDensityMatrix = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    //zm
    /**
     * Contains magnetometer measurement error.
     */
    private val magnetometerMeasurementError = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Contains the most recent residual in the magnetometer measurement update.
     */
    private val magnetometerResidual = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Temporary value to compute [sm].
     */
    private val smTmp1 =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Magnetometer residual covariance matrix
     */
    private val sm =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Inverse of [sm].
     */
    private val invSm =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Covariance measurement matrix of magnetometer measurements expressed in (T)^2.
     */
    private val rm =
        Matrix.identity(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)
            .multiplyByScalarAndReturnNew(magnetometerNoiseStandardDeviation.pow(2.0))

    /**
     * Matrix containing [0, 0, 1].
     */
    private val one = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Third column of rotation matrix.
     */
    private val r3 = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Transposed of [r3].
     */
    private val r3Transposed = Matrix(1, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Temporary 3x3 sub-matrix of [suhMatrix].
     */
    private val suhTmp =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Matrix to limit the effect of magnetometer corrector.
     */
    private val suhMatrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary matrix to compute [km].
     */
    private val kmTmp = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Magnetometer Kalman gain.
     */
    private val km = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Transpose of [km]
     */
    private val kmTransposed =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // temp = eye(9) - K_m * H_m;
    /**
     * Temporary value to update accelerometer error covariance matrix.
     */
    private val mTemp = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * 9x9 identity matrix.
     */
    private val identity9 =
        Matrix.identity(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Transposed of [mTemp].
     */
    private val mTempTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary value to compute magnetometer corrected error covariance.
     */
    private val errorCovMagnetometerCorrectedTmp1 =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary value to compute magnetometer corrected error covariance.
     */
    private val errorCovMagnetometerCorrectedTmp2 =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Temporary value to compute magnetometer corrected error covariance.
     */
    private val errorCovMagnetometerCorrectedTmp3 =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Computes magnetometer measurement correction.
     */
    fun correct() {
        // compute magnetometer measurement matrix
        correctedBodyQ.normalize()
//        correctedNedQ.normalize()
        correctedBodyQ.inverse(correctedNedQ)
//        correctedNedQ.inverse(invCorrectedNedQ)
        correctedNedQ.toMatrixRotation(correctedNedRotationMatrix)
//        invCorrectedNedQ.toMatrixRotation(invCorrectedNedRotationMatrix)

        magneticFluxDensity.getValuesAsMatrix(magneticFluxDensityMatrix)
        correctedNedRotationMatrix.multiply(
            magneticFluxDensityMatrix,
            expectedBodyMagneticFluxDensityMatrix
        )
/*        invCorrectedNedRotationMatrix.multiply(
            magneticFluxDensityMatrix,
            expectedBodyMagneticFluxDensityMatrix
        )*/

        Utils.skewMatrix(expectedBodyMagneticFluxDensityMatrix, skewBodyMagneticFluxDensityMatrix)
        skewBodyMagneticFluxDensityMatrix.multiplyByScalar(2.0)

        // H_m = [2 * skewSymmetric(C_hat_n_b * m_tilde), zeros(3), zeros(3)];
        hm.initialize(0.0)
        hm.setSubmatrix(0, 0, 2, 2, skewBodyMagneticFluxDensityMatrix)

        // compute magnetometer measurement error (zm)
        // z_m = y_m__next - C_hat_n_b * m_tilde;
        nedBodyMagneticFluxDensityTriad.getValuesAsMatrix(measuredBodyMagneticFluxDensityMatrix)
        measuredBodyMagneticFluxDensityMatrix.subtract(
            expectedBodyMagneticFluxDensityMatrix,
            magnetometerMeasurementError
        )

        // update the covariance matrix
        // Suh proposes to apply this update only to the quaternion and not to the
        // gyroscope bias by simplifying the state covariance matrix:
        // P_m__next_prev = [P_a__next_next(1:3,1:3),  zeros(3,6)]
        //                  [zeros(6,3),               zeros(6,6)];
        kalmanErrorCovMagnetometerCorrected.initialize(0.0)
        kalmanErrorCovMagnetometerCorrected.setSubmatrix(
            0,
            0,
            2,
            2,
            kalmanErrorCovAccelerometerCorrected,
            0,
            0,
            2,
            2
        )

        // compute magnetometer residual covariance matrix:
        // S_m = H_m * P_m__next_prev * H_m' + R_m;
        hm.transpose(hmTransposed)

        // H_m * P_m__next_prev
        hm.multiply(kalmanErrorCovMagnetometerCorrected, smTmp1)
        // H_m * P_m__next_prev * H_m'
        smTmp1.multiply(hmTransposed, sm)
        // H_m * P_m__next_prev * H_m' + R_m
        sm.add(rm)

        // Compute the magnetometer Kalman gain:
        /*
        % To limit the effect of the correction only to the yaw component, Suh
        % proposed to modify the computation of the gain as following:
        % [Eq. 22a Suh]
        r_3 = C_hat_n_b * [0; 0; 1];
        % and then:
        % [Eq. 21b (partial) Suh]
        % Suh_matrix: A fictitious matrix proposed by Suh
        Suh_matrix = [r_3 * r_3' 	zeros(3,6);
                      zeros(6,3)    zeros(6,6)];

        % [Eq. 21b Suh]
        % K_m: Magnetometer Kalman gain
        K_m = Suh_matrix * P_m__next_prev * H_m' / S_m;
         */
        correctedNedRotationMatrix.multiply(one, r3)
//        invCorrectedNedRotationMatrix.multiply(one, r3)
        r3.transpose(r3Transposed)
        r3.multiply(r3Transposed, suhTmp)

        suhMatrix.initialize(0.0)
        suhMatrix.setSubmatrix(0, 0, 2, 2, suhTmp)

        suhMatrix.multiply(kalmanErrorCovMagnetometerCorrected, kmTmp)
        kmTmp.multiply(hmTransposed, km)

        Utils.inverse(sm, invSm)
        km.multiply(invSm)

        // Compute residual in the magnetometer measurement update
        // r_m = z_m - H_m * x_hat_a__next_next;
        hm.multiply(kalmanStateAccelerometerCorrected, magnetometerResidual)
        magnetometerResidual.multiplyByScalar(-1.0)
        magnetometerResidual.add(magnetometerMeasurementError)

        // Update the state vector
        // x_hat__next_next: Updated state
        // x_hat__next_next = x_hat_a__next_next + K_m * (r_m);
        km.multiply(magnetometerResidual, kalmanStateMagnetometerCorrected)
        kalmanStateMagnetometerCorrected.add(kalmanStateAccelerometerCorrected)

        /*
        %% 8. Update the covariance matrix:
        % [Eq. 21d Suh]
        % P__next_next: State covariance matrix
        temp = eye(9) - K_m * H_m;
        */
        km.multiply(hm, mTemp)
        mTemp.multiplyByScalar(-1.0)
        mTemp.add(identity9)

        /*
        P__next_next = temp * P_a__next_next * temp' + K_m * R_m * K_m';
         */
        mTemp.transpose(mTempTransposed)
        km.transpose(kmTransposed)

        // K_m * R_m * K_m'
        rm.multiply(kmTransposed, errorCovMagnetometerCorrectedTmp1)
        km.multiply(errorCovMagnetometerCorrectedTmp1, errorCovMagnetometerCorrectedTmp2)

        // temp * P_a__next_next * temp'
        kalmanErrorCovAccelerometerCorrected.multiply(
            mTempTransposed,
            errorCovMagnetometerCorrectedTmp3
        )

        mTemp.multiply(errorCovMagnetometerCorrectedTmp3, kalmanErrorCovMagnetometerCorrected)
        kalmanErrorCovMagnetometerCorrected.add(errorCovMagnetometerCorrectedTmp2)
    }

    init {
        one.setElementAtIndex(2, 1.0)
    }

    companion object {
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
         * Default standard deviation for magnetometer noise expressed in Teslas (T).
         */
        const val DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION = 0.5e-6
    }
}