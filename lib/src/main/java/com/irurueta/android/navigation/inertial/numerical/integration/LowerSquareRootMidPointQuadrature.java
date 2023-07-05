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
 * This is an exact replacement for MidPointQuadrature, except that it allows for an inverse
 * square-root singularity in the integrand at the lower limit a.
 */
public class LowerSquareRootMidPointQuadrature extends MidPointQuadrature {

    /**
     * Original lower bound of integration.
     */
    private final double aorig;

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public LowerSquareRootMidPointQuadrature(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        super(0.0, Math.sqrt(b - a), listener);
        aorig = a;
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.LOWER_SQUARE_ROOT_MID_POINT;
    }

    /**
     * Evaluates function at 2*x*f(a0+x^2).
     *
     * @param x point where function is evaluated.
     * @return result of evaluation.
     * @throws EvaluationException if evaluation fails.
     */
    @Override
    protected double func(final double x) throws EvaluationException {
        return 2.0 * x * listener.evaluate(aorig + x * x);
    }
}
