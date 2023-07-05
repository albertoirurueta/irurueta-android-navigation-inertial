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
 * Computes function integration by using Simpson's rule and trapezoidal quadrature.
 * Simpson's method is an optimization of Trapezoidal quadrature integrator.
 * Implementations of this class will in general be more efficient than
 * Trapezoidal quadrature integrators (i.e., require fewer function evaluations) when the function
 * to be integrated has a finite fourth derivative (i.e., a continuous third derivative).
 * {@link SimpsonTrapezoidalQuadratureIntegrator} will in general be more efficient than
 * {@link TrapezoidalQuadratureIntegrator} (i.e. require fewer function evaluations) when the
 * function to be integrated has a finite fourth derivative (i.e. a continuous third derivative,
 * which means that the function is sufficiently smooth).
 */
public class SimpsonTrapezoidalQuadratureIntegrator
        extends SimpsonIntegrator<TrapezoidalQuadrature> {

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @param eps      required accuracy.
     */
    public SimpsonTrapezoidalQuadratureIntegrator(
            final double a, final double b,
            final SingleDimensionFunctionEvaluatorListener listener,
            final double eps) {
        super(new TrapezoidalQuadrature(a, b, listener), eps);
    }

    /**
     * Constructor with default accuracy.
     *
     * @param a Lower limit of integration.
     * @param b Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     */
    public SimpsonTrapezoidalQuadratureIntegrator(
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
        return QuadratureType.TRAPEZOIDAL;
    }
}
