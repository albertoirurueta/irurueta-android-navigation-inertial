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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.EvaluationException;

/**
 * This is an exact replacement for MidPointQuadrature i.e., returns the nth stage of refinement of
 * the integral of a function from a to b, except that the function is evaluated at evenly spaced
 * points in 1=x rather than in x. This allows the upper limit b to be as large and positive as the
 * computer allows, or the lower limit a to be as large and negative, but not both. a and b must
 * have the same sign.
 */
public class InfinityMidPointMatrixQuadrature extends MidPointMatrixQuadrature {

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public InfinityMidPointMatrixQuadrature(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        super(1.0 / b, 1.0 / a, listener);
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.INFINITY_MID_POINT;
    }

    /**
     * Evaluates function at 1/x.
     *
     * @param x      point where function is evaluated.
     * @param result instance where result of evaluation is stored.
     * @throws EvaluationException if evaluation fails.
     */
    @Override
    protected void func(final double x, final Matrix result) throws EvaluationException {
        listener.evaluate(1.0 / x, result);
        result.multiplyByScalar(1.0 / (x * x));
    }
}
