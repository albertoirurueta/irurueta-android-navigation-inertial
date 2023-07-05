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
 * Implementation of quadrature using mid-point algorithm.
 * Mid-point algorithm is suitable for improper integrations, which consists of any of the following
 * situations:
 * - integrand goes to a finite limiting value at finite upper and lower limits, but cannot be
 * evaluated right on one of those limits (e.g. sin(x)/x at x = 0)
 * - the upper limit is positive infinity, or the lower limit is negative infinity.
 * - integrand has an integrable singularity at either limit (e.g. x^-0.5 at x = 0)
 * - integrand has an integrable singularity at a known place between its upper and lower limits.
 * - integrand has an integrable singularity at an unknown place between its upper and lower limits.
 * <p>
 * Impossible integrals, such as those being infinite (e.g. integral from 1 to infinity of x^-1) or
 * those that have no limiting sense (e.g. integral from -infinity to infinity of cos(x)) cannot
 * be handled by this implementation.
 */
public class MidPointQuadrature extends Quadrature {

    /**
     * Lower limit of integration.
     */
    private final double a;

    /**
     * Upper limit of integration.
     */
    private final double b;

    /**
     * Current value of integral.
     */
    private double s;

    /**
     * Listener to evaluate single dimension functions at required points.
     */
    protected final SingleDimensionFunctionEvaluatorListener listener;

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public MidPointQuadrature(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        this.n = 0;
        this.a = a;
        this.b = b;
        this.s = 0;
        this.listener = listener;
    }

    /**
     * Gets lower limit of integration.
     *
     * @return lower limit of integration.
     */
    public double getA() {
        return a;
    }

    /**
     * Gets upper limit of integration.
     *
     * @return upper limit of integration.
     */
    public double getB() {
        return b;
    }

    /**
     * Gets current value of integral.
     *
     * @return current value of integral.
     */
    public double getS() {
        return s;
    }

    /**
     * Returns the value of the integral at the nth stage of refinement.
     *
     * @return the value of the integral at the nth stage of refinement.
     * @throws EvaluationException Raised if something failed during the evaluation.
     */
    @Override
    public double next() throws EvaluationException {
        int it;
        int j;
        double x;
        double tnm;
        double sum;
        double del;
        double ddel;
        n++;
        if (n == 1) {
            return (s = (b - a) * func(0.5 * (a + b)));
        } else {
            for (it = 1, j = 1; j < n - 1; j++) {
                it *= 3;
            }
            tnm = it;
            del = (b - a) / (3.0 * tnm);
            // The added points alternate in spacing between del and ddel
            ddel = del + del;
            x = a + 0.5 * del;
            sum = 0.0;
            for (j = 0; j < it; j++) {
                sum += func(x);
                x += ddel;
                sum += func(x);
                x += del;
            }
            // The new sum is combined with the old integral to give a refined integral
            s = (s + (b - a) * sum / tnm) / 3.0;
            return s;
        }
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.MID_POINT;
    }

    /**
     * Evaluates function at x.
     *
     * @param x point where function is evaluated.
     * @return result of evaluation.
     * @throws EvaluationException if evaluation fails.
     */
    protected double func(final double x) throws EvaluationException {
        return listener.evaluate(x);
    }
}
