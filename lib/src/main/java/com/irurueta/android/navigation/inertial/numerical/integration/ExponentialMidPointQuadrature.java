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
 * This is an exact replacement for MidPointQuadrature, except that upper limit is assumed to be
 * infinite. It is assumed that the function decreases exponentially rapidly at infinity.
 */
public class ExponentialMidPointQuadrature extends MidPointQuadrature {
    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public ExponentialMidPointQuadrature(
            final double a,
            final SingleDimensionFunctionEvaluatorListener listener) {
        super(0.0, Math.exp(-a), listener);
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.EXPONENTIAL_MID_POINT;
    }

    /**
     * Evaluates function at f(-log(x))/x.
     *
     * @param x point where function is evaluated.
     * @return result of evaluation.
     * @throws EvaluationException if evaluation fails.
     */
    @Override
    protected double func(final double x) throws EvaluationException {
        return listener.evaluate(-Math.log(x)) / x;
    }
}
