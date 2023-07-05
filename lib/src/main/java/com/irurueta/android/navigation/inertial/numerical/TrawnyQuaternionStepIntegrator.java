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
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;

public class TrawnyQuaternionStepIntegrator extends QuaternionStepIntegrator {

    private final ExponentialMatrixEstimator exponentialMatrixEstimator =
            new ExponentialMatrixEstimator();

    private Matrix quat;
    private Matrix omega0;
    private Matrix omega1;
    private Matrix omegaSkew0;
    private Matrix omegaSkew1;
    private Matrix omegaAvgOmega;
    private Matrix dotOmega;
    private Matrix omegaSkewDotOmega;
    private Matrix a;
    private Matrix b;
    private Matrix tmp;
    private Matrix expA;
    private Matrix quatResult;

    public TrawnyQuaternionStepIntegrator() {
        try {
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaAvgOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            dotOmega = new Matrix(Quaternion.INHOM_COORDS, 1);
            omegaSkewDotOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            b = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            expA = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result) throws RotationException {
        integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                currentWx, currentWy, currentWz, dt, result, exponentialMatrixEstimator, quat,
                omega0, omega1, omegaSkew0, omegaSkew1, omegaAvgOmega, dotOmega,
                omegaSkewDotOmega, a, b, tmp, expA, quatResult);
    }

    public static void integrationStep(
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result) throws RotationException {
        try {
            final ExponentialMatrixEstimator exponentialMatrixEstimator =
                    new ExponentialMatrixEstimator();
            final Matrix quat = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaAvgOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix dotOmega = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omegaSkewDotOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix b = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix expA = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, exponentialMatrixEstimator, quat,
                    omega0, omega1, omegaSkew0, omegaSkew1, omegaAvgOmega, dotOmega,
                    omegaSkewDotOmega, a, b, tmp, expA, quatResult);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    private static void integrationStep(
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result,
            final ExponentialMatrixEstimator exponentialMatrixEstimator, final Matrix quat,
            final Matrix omega0, final Matrix omega1, final Matrix omegaSkew0,
            final Matrix omegaSkew1, final Matrix omegaAvgOmega, final Matrix dotOmega,
            final Matrix omegaSkewDotOmega, final Matrix a, final Matrix b, final Matrix tmp,
            final Matrix expA, final Matrix quatResult) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // angular speed at end timestamp t1
            copyAngularSpeedToMatrix(currentWx, currentWy, currentWz, omega1);

            computeOmegaSkew(omega0, omegaSkew0);
            computeOmegaSkew(omega1, omegaSkew1);

            computeOmegaAvgOmega(omega0, omega1, dt, omegaSkew1, omegaAvgOmega, dotOmega,
                    omegaSkewDotOmega);

            a.copyFrom(omegaAvgOmega);
            a.multiplyByScalar(0.5 * dt);

            omegaSkew1.multiply(omegaSkew0, b);
            omegaSkew0.multiply(omegaSkew1, tmp);
            b.subtract(tmp);

            exponentialMatrixEstimator.exponential(a, expA);

            b.multiplyByScalar(dt * dt / 48.0);

            tmp.copyFrom(expA);
            tmp.add(b);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }

    private static void computeOmegaAvgOmega(
            final Matrix omega0, final Matrix omega1, final double dt, final Matrix omegaSkew1,
            final Matrix result, final Matrix dotOmega, final Matrix omegaSkewDotOmega)
            throws AlgebraException {
        omega1.subtract(omega0, dotOmega);
        dotOmega.multiplyByScalar(1.0 / dt);

        computeOmegaSkew(dotOmega, omegaSkewDotOmega);

        omegaSkewDotOmega.multiplyByScalar(0.5 * dt);

        omegaSkew1.add(omegaSkewDotOmega, result);
    }
}
