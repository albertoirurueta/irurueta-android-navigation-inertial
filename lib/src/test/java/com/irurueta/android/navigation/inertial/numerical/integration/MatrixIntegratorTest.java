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
import static org.junit.Assert.fail;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.logging.Level;
import java.util.logging.Logger;

public class MatrixIntegratorTest {
    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double MIN_LAMBDA = -1.0;

    private static final double MAX_LAMBDA = 1.0;

    private static final double EPS = 1e-6;

    @Test
    public void create_whenAccuracyIntegratorAndQuadratureTypes_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener, EPS,
                IntegratorType.QUADRATURE, QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof TrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof MidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof InfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof LowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof UpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof DoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof SimpsonMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof SimpsonInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof SimpsonDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyIntegratorAndQuadratureTypes_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener,
                IntegratorType.QUADRATURE, QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof TrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof MidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof InfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof LowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof UpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof DoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            MatrixIntegrator.create(a, b, listener, IntegratorType.QUADRATURE,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof SimpsonMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof SimpsonInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof SimpsonDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenAccuracyAndIntegratorType_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener, EPS,
                IntegratorType.QUADRATURE);
        assertTrue(integrator instanceof TrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.SIMPSON);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, EPS, IntegratorType.ROMBERG);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndIntegratorType_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener,
                IntegratorType.QUADRATURE);
        assertTrue(integrator instanceof TrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.SIMPSON);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = MatrixIntegrator.create(a, b, listener, IntegratorType.ROMBERG);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenAccuracyAndDefaultIntegratorType_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener, EPS);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndDefaultIntegratorType_returnsExpectedIntegrator()
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

        MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureMatrixIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void comparePerformance() throws IntegrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double lambda = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

        final int[] evaluations = new int[1];
        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        evaluations[0] += 1;
                        result.setElementAtIndex(0, Math.exp(lambda * point));
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

        final double expected = 1.0 / lambda * (Math.exp(lambda * b) - Math.exp(lambda * a));

        int bestEvaluations = Integer.MAX_VALUE;
        IntegratorType bestEvaluationsIntegratorType = null;
        QuadratureType bestEvaluationsQuadratureType = null;

        long bestDuration = Long.MAX_VALUE;
        IntegratorType bestDurationIntegratorType = null;
        QuadratureType bestDurationQuadratureType = null;

        double bestError = Double.MAX_VALUE;
        IntegratorType bestErrorIntegratorType = null;
        QuadratureType bestErrorQuadratureType = null;

        for (final IntegratorType integratorType : IntegratorType.values()) {
            for (final QuadratureType quadratureType : QuadratureType.values()) {
                final MatrixIntegrator integrator = MatrixIntegrator.create(a, b, listener);
                evaluations[0] = 0;
                final long start = System.nanoTime();
                final Matrix result = new Matrix(1, 1);
                integrator.integrate(result);
                final long end = System.nanoTime();
                final long duration = end - start;
                final double error = Math.abs(expected - result.getElementAtIndex(0));

                Logger.getGlobal().log(Level.INFO, "Integrator type: " + integratorType
                        + ", Quadrature type: " + quadratureType
                        + ", evaluations: " + evaluations[0]
                        + ", duration: " + duration + "ns, error: " + error);

                if (evaluations[0] < bestEvaluations) {
                    bestEvaluations = evaluations[0];
                    bestEvaluationsIntegratorType = integratorType;
                    bestEvaluationsQuadratureType = quadratureType;
                }

                if (duration < bestDuration) {
                    bestDuration = duration;
                    bestDurationIntegratorType = integratorType;
                    bestDurationQuadratureType = quadratureType;
                }

                if (error < bestError) {
                    bestError = error;
                    bestErrorIntegratorType = integratorType;
                    bestErrorQuadratureType = quadratureType;
                }
            }
        }

        Logger.getGlobal().log(Level.INFO,
                "Best evaluations - Integrator type: " + bestEvaluationsIntegratorType
                        + ", Quadrature type: " + bestEvaluationsQuadratureType
                        + ", evaluations: " + bestEvaluations);

        Logger.getGlobal().log(Level.INFO,
                "Best duration - Integrator type: " + bestDurationIntegratorType
                        + ", Quadrature type: " + bestDurationQuadratureType
                        + ", duration: " + bestDuration + "ns");

        Logger.getGlobal().log(Level.INFO,
                "Best error - Integrator type: " + bestErrorIntegratorType
                        + ", Quadrature type: " + bestErrorQuadratureType
                        + ", error: " + bestError);
    }
}