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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.android.navigation.inertial.numerical.interpolation.InterpolationException;
import com.irurueta.android.navigation.inertial.numerical.interpolation.PolynomialInterpolator;
import com.irurueta.numerical.EvaluationException;

/**
 * Base integrator for implementations based on Romberg's method.
 * Romberg's method is a generalization of Simpson's method for higher order integration schemes.
 * This can be used to computed integration with less function evaluations for the same level of
 * accuracy when more assumptions of function "smoothness" can be made.
 * Implementations of Romberg's method are quite powerful for sufficiently smooth (e.g., analytic)
 * integrands, integrated over intervals that contain no singularities, and where the endpoints are
 * also non-singular. In such circumstances, Romberg's method, takes many, many fewer function
 * evaluations than other method's such as Simpson's.
 *
 * @param <T> instance of a quadrature to be used for Romber'gs method integration.
 */
public abstract class RombergMatrixIntegrator<T extends MatrixQuadrature> extends MatrixIntegrator {

    /**
     * Default accuracy.
     */
    public static final double EPS = 3.0e-9;

    /**
     * Maximum number of allowed steps.
     */
    private static final int JMAX = 14;

    /**
     * Maximum number of allowed steps + 1.
     */
    private static final int JMAXP = JMAX + 1;

    /**
     * Minimum required number of steps.
     */
    private static final int K = 5;

    /**
     * Quadrature used for integration.
     */
    protected final T q;

    /**
     * Required accuracy.
     */
    protected final double eps;

    /**
     * Successive trapezoidal approximations.
     */
    private final Matrix[] s = new Matrix[JMAX];

    /**
     * Successive trapezoidal step sizes.
     */
    private final double[] h = new double[JMAXP];

    /**
     * Constructor.
     *
     * @param q   Quadrature used for integration.
     * @param eps Required accuracy.
     */
    public RombergMatrixIntegrator(
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
            final int elems = rows * columns;
            for (int i = 0; i < JMAX; i++) {
                s[i] = new Matrix(rows, columns);
            }

            final PolynomialInterpolator[] interpolators = new PolynomialInterpolator[elems];
            final double[][] sInterp = new double[elems][JMAX];
            for (int i = 0; i < elems; i++) {
                sInterp[i] = new double[JMAX];
                interpolators[i] = new PolynomialInterpolator(h, sInterp[i], K, false);
            }

            h[0] = 1.0;
            for (int j = 1; j <= JMAX; j++) {
                q.next(s[j - 1]);
                // update sInterp
                for (int i = 0; i < elems; i++) {
                    sInterp[i][j - 1] = s[j - 1].getElementAtIndex(i);
                }
                if (j >= K) {
                    boolean finished = true;
                    for (int i = 0; i < elems; i++) {
                        final double ss = interpolators[i].rawinterp(j - K, 0.0);
                        if (Double.isNaN(ss)) {
                            throw new IntegrationException("NaN was found");
                        }
                        result.setElementAtIndex(i, ss);
                        if (Math.abs(interpolators[i].getDy()) > eps * Math.abs(ss)) {
                            finished = false;
                        }
                    }

                    if (finished) {
                        return;
                    }
                }
                h[j] = h[j - 1] / 9.0;
            }
        } catch (final EvaluationException | InterpolationException | WrongSizeException e) {
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
        return IntegratorType.ROMBERG;
    }

    /**
     * Creates an integrator using Romberg's method.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type.
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
    public static RombergMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case MID_POINT -> new RombergMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT ->
                    new RombergInfinityMidPointQuadratureMatrixIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new RombergLowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new RombergUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            case EXPONENTIAL_MID_POINT ->
                    new RombergExponentialMidPointQuadratureMatrixIntegrator(a, listener, eps);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new RombergDoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener,
                            eps);
            default -> new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener, eps);
        };
    }

    /**
     * Creates an integrator using Romberg's method and having default accuracy.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @param quadratureType quadrature type.
     * @return created integrator.
     * @throws IllegalArgumentException if provided quadrature type is not supported.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static RombergMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) throws WrongSizeException {
        return switch (quadratureType) {
            case MID_POINT -> new RombergMidPointQuadratureMatrixIntegrator(a, b, listener);
            case INFINITY_MID_POINT ->
                    new RombergInfinityMidPointQuadratureMatrixIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new RombergLowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new RombergUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);
            case EXPONENTIAL_MID_POINT ->
                    new RombergExponentialMidPointQuadratureMatrixIntegrator(a, listener);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new RombergDoubleExponentialRuleQuadratureMatrixIntegrator(a, b, listener);
            default -> new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);
        };
    }

    /**
     * Creates an integrator using Romberg's method and default quadrature type.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static RombergMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener, final double eps)
            throws WrongSizeException {
        return create(a, b, listener, eps, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Creates an integrator using Romberg's method and having default accuracy and quadrature type.
     * It must be noticed that upper limit of integration is ignored when using exponential mid
     * point quadrature type.
     *
     * @param a              Lower limit of integration.
     * @param b              Upper limit of integration.
     * @param listener       listener to evaluate a single dimension function at required points.
     * @return created integrator.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public static RombergMatrixIntegrator<?> create(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        return create(a, b, listener, DEFAULT_QUADRATURE_TYPE);
    }
}
