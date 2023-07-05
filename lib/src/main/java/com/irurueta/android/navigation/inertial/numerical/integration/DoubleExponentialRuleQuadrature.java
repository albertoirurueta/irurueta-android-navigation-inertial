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
 * Implementation of quadrature using double exponential, which allows integration with a variable
 * transformation.
 * This implementation is suitable for improper integrands containing singularities.
 */
public class DoubleExponentialRuleQuadrature extends Quadrature {

    /**
     * Default transformation of range of integration.
     */
    public static final double DEFAULT_HMAX = 3.7;

    /**
     * Lower limit of integration.
     */
    private final double a;

    /**
     * Upper limit of integration.
     */
    private final double b;

    /**
     * Maximum step size. Determines transformation of range of integration.
     */
    private final double hmax;

    /**
     * Value of the next stage of refinement.
     */
    private double s;

    /**
     * Listener to evaluate single dimension functions at required points.
     */
    private final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener;

    /**
     * Constructor.
     *
     * @param listener listener to evaluate function and handle singularities.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     */
    public DoubleExponentialRuleQuadrature(
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b, final double hmax) {
        this.listener = listener;
        this.a = a;
        this.b = b;
        this.hmax = hmax;
        n = 0;
    }

    /**
     * Constructor with default maximum step size.
     *
     * @param listener listener to evaluate function and handle singularities.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     */
    public DoubleExponentialRuleQuadrature(
            final DoubleExponentialSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b) {
        this(listener, a, b, DEFAULT_HMAX);
    }

    /**
     * Constructor.
     *
     * @param listener listener to evaluate function if function has mild singularities.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     */
    public DoubleExponentialRuleQuadrature(
            final SingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b, final double hmax) {
        this(new DoubleExponentialSingleDimensionFunctionEvaluatorListener() {
            @Override
            public double evaluate(double x, double delta) throws EvaluationException {
                return listener.evaluate(x);
            }
        }, a, b, hmax);
    }

    /**
     * Constructor with default maximum step size.
     *
     * @param listener listener to evaluate function if function has mild singularities.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     */
    public DoubleExponentialRuleQuadrature(
            final SingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b) {
        this(listener, a, b, DEFAULT_HMAX);
    }

    /**
     * Returns the value of the integral at the nth stage of refinement.
     *
     * @return the value of the integral at the nth stage of refinement.
     * @throws EvaluationException Raised if something failed during the evaluation.
     */
    @Override
    public double next() throws EvaluationException {
        // On the first call to the function next (n = 1), the routine returns the crudest estimate
        // of integral between a and b of f(x). Subsequent calls to next (n = 2, 3, ...) will
        // improve the accuracy by adding 2^(n-1) additional interior points.
        double del;
        double fact;
        double q;
        double sum;
        double t;
        final double twoh;
        int it;
        int j;
        n++;
        if (n == 1) {
            fact = 0.25;
            s = hmax * 2.0 * (b - a) * fact * listener.evaluate(0.5 * (b + a), 0.5 * (b - a));
        } else {
            for (it = 1, j = 1; j < n - 1; j++) {
                it <<= 1;
            }

            // Twice the spacing of the points to be added
            twoh = hmax / it;
            t = 0.5 * twoh;
            for (sum = 0.0, j = 0; j < it; j++) {
                q = Math.exp(-2.0 * Math.sinh(t));
                del = (b - a) * q / (1.0 + q);
                final double value = 1.0 + q;
                fact = q / (value * value) * Math.cosh(t);
                sum += fact * (listener.evaluate(a + del, del) + listener.evaluate(b - del, del));
                t += twoh;
            }

            // Replace s by its refined value and return.
            s = 0.5 * s + (b - a) * twoh * sum;
        }
        return s;
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.DOUBLE_EXPONENTIAL_RULE;
    }
}