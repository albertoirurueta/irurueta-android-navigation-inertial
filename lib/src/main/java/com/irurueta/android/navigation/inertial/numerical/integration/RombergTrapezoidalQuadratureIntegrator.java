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
 * Computes function integration by using Romberg integration.
 */
public class RombergTrapezoidalQuadratureIntegrator
        extends RombergIntegrator<TrapezoidalQuadrature> {

    /**
     * Default accuracy.
     */
    public static final double EPS = 1e-10;

    /**
     * Maximum number of allowed steps.
     */
    private static final int JMAX = 20;

    /**
     * Maximum number of allowed steps + 1.
     */
    private static final int JMAXP = JMAX + 1;

    /**
     * Minimum required number of steps.
     */
    private static final int K = 5;

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
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public RombergTrapezoidalQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new TrapezoidalQuadrature(a, b, listener), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public RombergTrapezoidalQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, listener, EPS);
    }

    /**
     * Integrates function between lower and upper limits defined by provided quadrature.
     * This implementation is suitable for closed intervals.
     *
     * @return result of integration.
     * @throws IntegrationException if integration fails for numerical reasons.
     */
    public double integrate() throws IntegrationException {
        try {
            h[0] = 1.0;
            for (int j = 1; j <= JMAX; j++) {
                s[j - 1] = q.next();
                if (j >= K) {
                    double ss = interpolator.rawinterp(j - K, 0.0);
                    if (Math.abs(interpolator.getDy()) <= eps * Math.abs(ss)) {
                        return ss;
                    }
                }
                h[j] = 0.25 * h[j - 1];
            }
        } catch (final EvaluationException | InterpolationException e) {
            throw new IntegrationException(e);
        }

        // Too many steps
        throw new IntegrationException();
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getQuadratureType() {
        return QuadratureType.TRAPEZOIDAL;
    }
}
