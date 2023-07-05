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

import com.irurueta.android.navigation.inertial.numerical.interpolation.InterpolationException;
import com.irurueta.android.navigation.inertial.numerical.interpolation.PolynomialInterpolator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;

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
public abstract class RombergIntegrator<T extends Quadrature> extends Integrator {

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
    private final double[] s = new double[JMAX];

    /**
     * Successive trapezoidal step sizes.
     */
    private final double[] h = new double[JMAXP];

    /**
     * Polynomial interpolator.
     */
    private final PolynomialInterpolator interpolator = new PolynomialInterpolator(h, s, K, false);

    /**
     * Constructor.
     *
     * @param q   Quadrature used for integration.
     * @param eps Required accuracy.
     */
    public RombergIntegrator(
            final T q,
            final double eps) {
        this.q = q;
        this.eps = eps;
    }

    /**
     * Integrates function between lower and upper limits defined by provided quadrature.
     * This implementation is suitable for open intervals.
     *
     * @return result of integration.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    @Override
    public double integrate() throws IntegrationException {
        try {
            h[0] = 1.0;
            for (int j = 1; j <= JMAX; j++) {
                s[j - 1] = q.next();
                if (j >= K) {
                    final double ss = interpolator.rawinterp(j - K, 0.0);
                    if (Math.abs(interpolator.getDy()) <= eps * Math.abs(ss)) {
                        return ss;
                    }
                }
                h[j] = h[j - 1] / 9.0;
            }
        } catch (final EvaluationException | InterpolationException e) {
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
     */
    public static RombergIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) {
        switch (quadratureType) {
            case MID_POINT:
                return new RombergMidPointQuadratureIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT:
                return new RombergInfinityMidPointQuadratureIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT:
                return new RombergLowerSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case UPPER_SQUARE_ROOT_MID_POINT:
                return new RombergUpperSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case EXPONENTIAL_MID_POINT:
                return new RombergExponentialMidPointQuadratureIntegrator(a, listener, eps);
            case DOUBLE_EXPONENTIAL_RULE:
                return new RombergDoubleExponentialRuleQuadratureIntegrator(a, b, listener, eps);
            case TRAPEZOIDAL:
            default:
                return new RombergTrapezoidalQuadratureIntegrator(a, b, listener, eps);
        }
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
     */
    public static RombergIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) {
        switch (quadratureType) {
            case MID_POINT:
                return new RombergMidPointQuadratureIntegrator(a, b, listener);
            case INFINITY_MID_POINT:
                return new RombergInfinityMidPointQuadratureIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT:
                return new RombergLowerSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT:
                return new RombergUpperSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case EXPONENTIAL_MID_POINT:
                return new RombergExponentialMidPointQuadratureIntegrator(a, listener);
            case DOUBLE_EXPONENTIAL_RULE:
                return new RombergDoubleExponentialRuleQuadratureIntegrator(a, b, listener);
            case TRAPEZOIDAL:
            default:
                return new RombergTrapezoidalQuadratureIntegrator(a, b, listener);
        }
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
     */
    public static RombergIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps) {
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
     */
    public static RombergIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        return create(a, b, listener, DEFAULT_QUADRATURE_TYPE);
    }
}
