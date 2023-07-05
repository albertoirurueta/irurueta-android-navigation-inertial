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

import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;

/**
 * Computes function integration by using Simpson's method and double exponential quadrature.
 * Double exponential quadrature allows improper integrands containing singularities to be
 * integrated.
 *
 * @see DoubleExponentialRuleQuadrature
 */
public class SimpsonDoubleExponentialRuleQuadratureIntegrator
        extends SimpsonIntegrator<DoubleExponentialRuleQuadrature> {

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b, final double hmax,
            final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new DoubleExponentialRuleQuadrature(listener, a, b, hmax), eps);
    }

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b, final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new DoubleExponentialRuleQuadrature(listener, a, b), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b, final double hmax,
            final SingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, hmax, listener, EPS);
    }

    /**
     * Constructor with default accuracy and default maximum step size.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, listener, EPS);
    }

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @param listener listener to evaluate a single dimension function at required points for
     *                 double exponential quadrature to take into account any non-mild
     *                 singularities.
     * @param eps      required accuracy.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b, final double hmax,
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new DoubleExponentialRuleQuadrature(listener, a, b, hmax), eps);
    }

    /**
     * Constructor with default maximum step size.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points for
     *                 double exponential quadrature to take into account any non-mild
     *                 singularities.
     * @param eps      required accuracy.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b,
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new DoubleExponentialRuleQuadrature(listener, a, b), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @param listener listener to evaluate a single dimension function at required points for
     *                 double exponential quadrature to take into account any non-mild
     *                 singularities.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b, final double hmax,
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, hmax, listener, EPS);
    }

    /**
     * Constructor with default accuracy and default maximum step size.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points for
     *                 double exponential quadrature to take into account any non-mild
     *                 singularities.
     */
    public SimpsonDoubleExponentialRuleQuadratureIntegrator(
            final double a, final double b,
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, listener, EPS);
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getQuadratureType() {
        return QuadratureType.DOUBLE_EXPONENTIAL_RULE;
    }
}
