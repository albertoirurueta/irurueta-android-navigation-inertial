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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.EvaluationException;

/**
 * Integrates matrix (multivariate) single dimension functions given a quadrature implementation up
 * to desired accuracy.
 * If assumptions can be made about the smoothness of a function other implementations such as
 * Simpon's or Romberg's are more efficient and require less function evaluations. Otherwise, this
 * is the simplest integrator that can be used for general purpose integrations when no assumptions
 * can be made.
 *
 * @param <T> a quadrature.
 */
public abstract class QuadratureMatrixIntegrator<T extends MatrixQuadrature>
        extends MatrixIntegrator {
    /**
     * Default accuracy.
     */
    public static final double EPS = 1e-10;

    /**
     * Minimum required number of steps.
     */
    private static final int JMIN = 5;

    /**
     * Maximum number of allowed steps.
     */
    private static final int JMAX = 35;

    /**
     * Quadrature used for integration.
     */
    private final T q;

    /**
     * Required accuracy.
     */
    private final double eps;

    /**
     * Constructor.
     *
     * @param q   Quadrature used for integration.
     * @param eps Required accuracy.
     */
    public QuadratureMatrixIntegrator(final T q, final double eps) {
        this.q = q;
        this.eps = eps;
    }

    /**
     * Integrates function between provided lower and upper limits.
     *
     * @param result instance where result of integration will be stored.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    @Override
    public void integrate(final Matrix result) throws IntegrationException {
        try {
            final int rows = q.getRows();
            final int columns = q.getColumns();
            final Matrix s = new Matrix(rows, columns);

            // Initial value of olds is arbitrary.
            final Matrix olds = new Matrix(rows, columns);

            for (int j = 0; j < JMAX; j++) {
                q.next(s);
                if (j > JMIN) {
                    // Avoid spurious early convergence.
                    if (Math.abs(normMin(s) - normMin(olds)) < eps * normMin(olds)
                            || (normMin(s) == 0.0 && normMin(olds) == 0.0)) {
                        result.copyFrom(s);
                        return;
                    }
                }
                olds.copyFrom(s);
            }
        } catch (final EvaluationException | AlgebraException e) {
            throw new IntegrationException(e);
        }

        // Too many steps
        throw new IntegrationException();
    }

    /**
     * Gets type of integrator.
     *
     * @return type of integrator.
     */
    @Override
    public IntegratorType getIntegratorType() {
        return IntegratorType.QUADRATURE;
    }

    /**
     * Creates a quadrature integrator.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param eps            required accuracy.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static QuadratureMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case TRAPEZOIDAL -> new TrapezoidalQuadratureMatrixIntegrator(a, b, listener, eps);
            case MID_POINT -> new MidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT ->
                    new InfinityMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new LowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new UpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new DoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener, eps);
            default -> throw new IllegalArgumentException();
        };
    }

    /**
     * Creates a quadrature integrator with default accuracy.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static QuadratureMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case TRAPEZOIDAL -> new TrapezoidalQuadratureMatrixIntegrator(a, b, listener);
            case MID_POINT -> new MidPointQuadratureMatrixIntegrator(a, b, listener);
            case INFINITY_MID_POINT ->
                    new InfinityMidPointQuadratureMatrixIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new LowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new UpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new DoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener);
            default -> throw new IllegalArgumentException();
        };
    }

    /**
     * Creates a quadrature integrator using default quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static QuadratureMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps)
            throws WrongSizeException {
        return create(a, b, listener, eps, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Creates a quadrature integrator using default accuracy and quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static QuadratureMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        return create(a, b, listener, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Estimates smallest norm of provided matrix.
     * Smallest norm is used to ensure convergence of all elements in matrix.
     *
     * @param a matrix to compute min norm for.
     * @return estimated min norm.
     */
    private static double normMin(final Matrix a) {
        double min = Double.MAX_VALUE;
        double[] buffer = a.getBuffer();
        for (double v : buffer) {
            double value = Math.abs(v);
            if (Double.isNaN(value)) {
                return value;
            }

            if (value < min) {
                min = value;
            }
        }
        return min;
    }
}
