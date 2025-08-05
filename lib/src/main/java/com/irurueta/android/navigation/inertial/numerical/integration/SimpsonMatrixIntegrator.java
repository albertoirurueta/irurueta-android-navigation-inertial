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
 * Base integrator for implementations based on Simpson's method.
 * Simpson's method is an optimization of Trapezoidal quadrature integrator.
 * Implementations of this class will in general be more efficient than
 * Trapezoidal quadrature matrix integrators (i.e., require fewer function evaluations) when the
 * matrix function to be integrated has a finite fourth derivative (i.e., a continuous third
 * derivative).
 *
 * @param <T> a quadrature.
 */
public abstract class SimpsonMatrixIntegrator<T extends MatrixQuadrature> extends MatrixIntegrator {

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
    private static final int JMAX = 20;

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
    public SimpsonMatrixIntegrator(
            final T q,
            final double eps) {
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
            final Matrix st = new Matrix(rows, columns);
            final Matrix s = new Matrix(rows, columns);
            final Matrix ost = new Matrix(rows, columns);
            final Matrix os = new Matrix(rows, columns);
            final Matrix tmp = new Matrix(rows, columns);
            for (int j = 0; j < JMAX; j++) {
                q.next(st);

                // s = (4.0 * st - ost) / 3.0;
                tmp.copyFrom(st);
                tmp.multiplyByScalar(4.0);
                tmp.subtract(ost);
                tmp.multiplyByScalar(1.0 / 3.0);
                s.copyFrom(tmp);

                if (j > JMIN) {
                    if (Math.abs(normMin(s) - normMin(os)) < eps * normMin(os)
                            || (normMin(s) == 0.0 && normMin(os) == 0.0)) {
                        result.copyFrom(s);
                        return;
                    }
                }
                os.copyFrom(s);
                ost.copyFrom(st);
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
        return IntegratorType.SIMPSON;
    }

    /**
     * Creates an integrator using Simpson's method.
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
    public static SimpsonMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case TRAPEZOIDAL ->
                    new SimpsonTrapezoidalQuadratureMatrixIntegrator(a, b, listener, eps);
            case MID_POINT -> new SimpsonMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT ->
                    new SimpsonInfinityMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonLowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new SimpsonDoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            default -> throw new IllegalArgumentException();
        };
    }

    /**
     * Creates an integrator using Simpson's method and having default accuracy.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static SimpsonMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case TRAPEZOIDAL -> new SimpsonTrapezoidalQuadratureMatrixIntegrator(a, b, listener);
            case MID_POINT -> new SimpsonMidPointQuadratureMatrixIntegrator(a, b, listener);
            case INFINITY_MID_POINT ->
                    new SimpsonInfinityMidPointQuadratureMatrixIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonLowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new SimpsonDoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener);
            default -> throw new IllegalArgumentException();
        };
    }

    /**
     * Creates an integrator using Simpson's method and default quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static SimpsonMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps)
            throws WrongSizeException {
        return create(a, b, listener, eps, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Creates an integrator using Simpson's method and having default accuracy and quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static SimpsonMatrixIntegrator<?> create(
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
