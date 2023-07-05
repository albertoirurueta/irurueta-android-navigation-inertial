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
 * Computes function integration by using Romberg's method and exponential quadrature.
 * Exponential quadrature allows improper integrations when upper bound is at infinity.
 *
 * @see ExponentialMidPointQuadrature
 */
public class RombergExponentialMidPointQuadratureIntegrator
        extends RombergIntegrator<ExponentialMidPointQuadrature> {

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public RombergExponentialMidPointQuadratureIntegrator(
            final double a, final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new ExponentialMidPointQuadrature(a, listener), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a        Lower limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public RombergExponentialMidPointQuadratureIntegrator(
            final double a, final SingleDimensionFunctionEvaluatorListener listener) {
        this(a, listener, EPS);
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getQuadratureType() {
        return QuadratureType.EXPONENTIAL_MID_POINT;
    }
}
