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

public class YuanQuaternionStepIntegrator extends QuaternionStepIntegrator {

    private static final double EPSILON = 1e-15;

    private Matrix quat;
    private Matrix omega;
    private Matrix a;
    private Matrix identity;
    private Matrix tmp;
    private Matrix quatResult;

    public YuanQuaternionStepIntegrator() {
        try {
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omega = new Matrix(Quaternion.N_ANGLES, 1);
            a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
                currentWx, currentWy, currentWz, dt, result, quat, omega, a, identity, tmp,
                quatResult);
    }

    public static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result) throws RotationException {
        try {
            final Matrix quat = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix omega = new Matrix(Quaternion.N_ANGLES, 1);
            final Matrix a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, quat, omega, a, identity, tmp,
                    quatResult);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    private static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result, final Matrix quat, final Matrix omega,
            final Matrix a, final Matrix identity, final Matrix tmp, final Matrix quatResult)
            throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            double w1 = (initialWx + currentWx) / 2.0;
            double w2 = (initialWy + currentWy) / 2.0;
            double w3 = (initialWz + currentWz) / 2.0;

            omega.setElementAtIndex(0, w1);
            omega.setElementAtIndex(1, w2);
            omega.setElementAtIndex(2, w3);

            computeOmegaSkew(omega, a);

            final double w1dt = w1 * dt;
            final double w2dt = w2 * dt;
            final double w3dt = w3 * dt;

            final double w1dt2 = w1dt * w1dt;
            final double w2dt2 = w2dt * w2dt;
            final double w3dt2 = w3dt * w3dt;

            final double theta = Math.sqrt(w1dt2 + w2dt2 + w3dt2);
            final double halfTheta = theta / 2;
            final double sinc;
            if (theta > EPSILON) {
                sinc = Math.sin(halfTheta) / halfTheta;
            } else {
                // notice that sin(x) / x --> 1 for x --> 0
                sinc = 1.0;
            }

            a.multiplyByScalar(0.5 * sinc * dt);

            tmp.copyFrom(identity);
            tmp.multiplyByScalar(Math.cos(halfTheta));
            tmp.add(a);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException ex) {
            throw new RotationException(ex);
        }
    }

}
