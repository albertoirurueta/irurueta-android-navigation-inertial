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
 * Base integrator for implementations based on Simpson's method.
 * Simpson's method is an optimization of Trapezoidal quadrature integrator.
 * Implementations of this class will in general be more efficient than
 * Trapezoidal quadrature integrators (i.e., require fewer function evaluations) when the function
 * to be integrated has a finite fourth derivative (i.e., a continuous third derivative).
 *
 * @param <T> a quadrature.
 */
public abstract class SimpsonIntegrator<T extends Quadrature> extends Integrator {

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
    public SimpsonIntegrator(
            final T q,
            final double eps) {
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
            double st;
            double ost = 0.0;
            double os = 0.0;
            for (int j = 0; j < JMAX; j++) {
                st = q.next();
                s = (4.0 * st - ost) / 3.0;
                if (j > JMIN) {
                    if (Math.abs(s - os) < eps * Math.abs(os) || (s == 0.0 && os == 0.0)) {
                        return s;
                    }
                }
                os = s;
                ost = st;
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
     */
    public static SimpsonIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps,
            final QuadratureType quadratureType) {
        return switch (quadratureType) {
            case TRAPEZOIDAL -> new SimpsonTrapezoidalQuadratureIntegrator(a, b, listener, eps);
            case MID_POINT -> new SimpsonMidPointQuadratureIntegrator(a, b, listener, eps);
            case INFINITY_MID_POINT ->
                    new SimpsonInfinityMidPointQuadratureIntegrator(a, b, listener, eps);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonLowerSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonUpperSquareRootMidPointQuadratureIntegrator(a, b, listener, eps);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new SimpsonDoubleExponentialRuleQuadratureIntegrator(a, b, listener, eps);
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
     */
    public static SimpsonIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final QuadratureType quadratureType) {
        return switch (quadratureType) {
            case TRAPEZOIDAL -> new SimpsonTrapezoidalQuadratureIntegrator(a, b, listener);
            case MID_POINT -> new SimpsonMidPointQuadratureIntegrator(a, b, listener);
            case INFINITY_MID_POINT ->
                    new SimpsonInfinityMidPointQuadratureIntegrator(a, b, listener);
            case LOWER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonLowerSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case UPPER_SQUARE_ROOT_MID_POINT ->
                    new SimpsonUpperSquareRootMidPointQuadratureIntegrator(a, b, listener);
            case DOUBLE_EXPONENTIAL_RULE ->
                    new SimpsonDoubleExponentialRuleQuadratureIntegrator(a, b, listener);
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
     */
    public static SimpsonIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener, final double eps) {
        return create(a, b, listener, eps, DEFAULT_QUADRATURE_TYPE);
    }

    /**
     * Creates an integrator using Simpson's method and having default accuracy and quadrature type.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @return created integrator.
     */
    public static SimpsonIntegrator<?> create(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        return create(a, b, listener, DEFAULT_QUADRATURE_TYPE);
    }
}
