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
 * Computes function integration by using Simpson's rule and infinity mid point quadrature.
 * This type of integrator will in general be more efficient than Trapezoidal quadrature
 * integrators (i.e., require fewer function evaluations) when the function
 * to be integrated has a finite fourth derivative (i.e., a continuous third derivative).
 * If you need to integrate from a negative lower limit to positive infinity, you do this by
 * breaking the integral into two pieces at some positive value:
 * SimpsonInfinityMidPointQuadratureIntegrator integrator1 =
 * new SimpsonInfinityMidPointQuadratureIntegrator(-5.0, 2.0, listener);
 * SimpsonInfinityMidPointQuadratureIntegrator integrator2 =
 * new SimpsonInfinityMidPointQuadratureIntegrator(2.0, 1e99, listener);
 * double integral = integrator1.integrate() + integrator2.integrate();
 */
public class SimpsonInfinityMidPointQuadratureIntegrator
        extends SimpsonIntegrator<InfinityMidPointQuadrature> {

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public SimpsonInfinityMidPointQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new InfinityMidPointQuadrature(a, b, listener), eps);
    }

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public SimpsonInfinityMidPointQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener) {
        this(a, b, listener, EPS);
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getQuadratureType() {
        return QuadratureType.INFINITY_MID_POINT;
    }
}
