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
package com.irurueta.android.navigation.inertial.numerical;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.FrobeniusNormComputer;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;

public class SuhQuaternionStepIntegrator extends QuaternionStepIntegrator {

    private static final double THREE_FOURTHS = 3.0 / 4.0;

    private Matrix omega0;

    private Matrix omega1;

    private Matrix quat;

    private Matrix omegaSkew0;

    private Matrix omegaSkew1;

    private Matrix constant;

    private Matrix omegaSkew10;

    private Matrix identity;

    private Matrix omegaSkew1A;

    private Matrix omegaSkew0A;

    private Matrix omegaSkew1B;

    private Matrix tmp;

    private Matrix quatResult;

    public SuhQuaternionStepIntegrator() {
        try {
            omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            constant = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew10 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew0A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1B = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            quatResult = new Matrix(Quaternion.N_PARAMS, 1);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    @Override
    public QuaternionStepIntegratorType getType() {
        return null;
    }

    @Override
    public void integrate(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result) throws RotationException {
        integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                currentWx, currentWy, currentWz, dt, result, omega0, omega1, quat, omegaSkew0,
                omegaSkew1, constant, omegaSkew10, identity, omegaSkew1A, omegaSkew0A,
                omegaSkew1B, tmp, quatResult);
    }

    public static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result) throws RotationException {
        try {
            final Matrix omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix quat = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix constant = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew10 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew1A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew0A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew1B = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, omega0, omega1, quat, omegaSkew0,
                    omegaSkew1, constant, omegaSkew10, identity, omegaSkew1A, omegaSkew0A,
                    omegaSkew1B, tmp, quatResult);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    private static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result, final Matrix omega0, final Matrix omega1,
            final Matrix quat, final Matrix omegaSkew0, final Matrix omegaSkew1,
            final Matrix constant, final Matrix omegaSkew10, final Matrix identity,
            final Matrix omegaSkew1A, final Matrix omegaSkew0A, final Matrix omegaSkew1B,
            final Matrix tmp, final Matrix quatResult) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // angular speed at end timestamp t1
            copyAngularSpeedToMatrix(currentWx, currentWy, currentWz, omega1);

            final double norm1 = FrobeniusNormComputer.norm(omega1);
            final double sqrNorm1 = norm1 * norm1;

            computeOmegaSkew(omega0, omegaSkew0);
            computeOmegaSkew(omega1, omegaSkew1);

            final double dt2 = dt * dt;
            final double dt3 = dt * dt2;

            constant.initialize(sqrNorm1 * dt2 / 6.0);

            omegaSkew1.multiply(omegaSkew0, omegaSkew10);
            omegaSkew10.multiplyByScalar(dt2 / 24.0);

            omegaSkew1A.copyFrom(omegaSkew1);
            omegaSkew1A.multiplyByScalar(THREE_FOURTHS * dt);

            omegaSkew0A.copyFrom(omegaSkew0);
            omegaSkew0A.multiplyByScalar(dt / 4.0);

            omegaSkew1B.copyFrom(omegaSkew1);
            omegaSkew1B.multiplyByScalar(sqrNorm1 * dt3 / 48.0);

            tmp.copyFrom(identity);
            tmp.add(omegaSkew1A);
            tmp.subtract(omegaSkew0A);
            tmp.subtract(constant);
            tmp.subtract(omegaSkew10);
            tmp.subtract(omegaSkew1B);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }
}
