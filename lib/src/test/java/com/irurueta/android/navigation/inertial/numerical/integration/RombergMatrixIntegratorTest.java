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
package com.irurueta.android.navigation.inertial.numerical.integration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class RombergMatrixIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double EPS = 1e-6;

    @Test
    public void create_whenAccuracyAndQuadratureType_returnsExpectedIntegrator()
            throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        RombergMatrixIntegrator<?> integrator = RombergMatrixIntegrator.create(a, b, listener,
                EPS, QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, EPS,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndQuadratureType_returnsExpectedIntegrator()
            throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        RombergMatrixIntegrator<?> integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener, QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());
    }

    @Test
    public void create_whenNoQuadratureType_returnsExpectedIntegrator() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        RombergMatrixIntegrator<?> integrator = RombergMatrixIntegrator.create(a, b, listener,
                EPS);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergMatrixIntegrator.create(a, b, listener);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }
}