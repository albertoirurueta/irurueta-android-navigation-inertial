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

import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;

/**
 * Integrates functions given a quadrature implementation up to desired accuracy.
 * If assumptions can be made about the smoothness of a function other implementations such as
 * Simpon's or Romberg's are more efficient and require less function evaluations. Otherwise, this
 * is the simplest integrator that can be used for general purpose integrations when no assumptions
 * can be made.
 *
 * @param <T> a quadrature.
 */
public abstract class QuadratureIntegrator<T extends Quadrature> extends Integrator {

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
    public QuadratureIntegrator(final T q, final double eps) {
        this.q = q;
        this.eps = eps;
    }

    /**
     * Integrates function between provided lower and upper limits.
     *
     * @return result of integration.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    @Override
    public double integrate() throws IntegrationException {
        try {
            double s;
            // Initial value of olds is arbitrary.
            double olds = 0.0;

            for (int j = 0; j < JMAX; j++) {
                s = q.next();
                if (j > JMIN) {
                    // Avoid spurious early convergence.
                    if (Math.abs(s - olds) < eps * Math.abs(olds) || (s == 0.0 && olds == 0.0)) {
                        return s;
                    }
                }
                olds = s;
            }
        } catch (final EvaluationException e) {
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
     */
    public static QuadratureIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) {
        switch (quadratureType) {
            case TRAPEZOIDAL:
                return new TrapezoidalQuadratureIntegrator(a, b, listener, eps);
            case MID_POINT:
                return new MidPointQuadratureIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT:
                return new InfinityMidPointQuadratureIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT:
                return new LowerSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case UPPER_SQUARE_ROOT_MID_POINT:
                return new UpperSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case DOUBLE_EXPONENTIAL_RULE:
                return new DoubleExponentialRuleQuadratureIntegrator(a, b, listener, eps);
            case EXPONENTIAL_MID_POINT:
            default:
                throw new IllegalArgumentException();
        }
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
     */
    public static QuadratureIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) {
        switch (quadratureType) {
            case TRAPEZOIDAL:
                return new TrapezoidalQuadratureIntegrator(a, b, listener);
            case MID_POINT:
                return new MidPointQuadratureIntegrator(a, b, listener);
            case INFINITY_MID_POINT:
                return new InfinityMidPointQuadratureIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT:
                return new LowerSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT:
                return new UpperSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case DOUBLE_EXPONENTIAL_RULE:
                return new DoubleExponentialRuleQuadratureIntegrator(a, b, listener);
            case EXPONENTIAL_MID_POINT:
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Creates a quadrature integrator using default quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     * @return created integrator.
     */
    public static QuadratureIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps) {
        return create(a, b, listener, eps, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Creates a quadrature integrator using default accuracy and quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @return created integrator.
     */
    public static QuadratureIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        return create(a, b, listener, DEFAULT_QUADRATURE_TYPE);
    }
}
