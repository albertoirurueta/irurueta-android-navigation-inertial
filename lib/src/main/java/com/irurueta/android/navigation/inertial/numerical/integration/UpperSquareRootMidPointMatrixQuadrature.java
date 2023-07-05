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
 * This is an exact replacement for MidPointMatrixQuadrature, except that it allows for an inverse
 * square-root singularity in the integrand at the upper limit b.
 */
public class UpperSquareRootMidPointMatrixQuadrature extends MidPointMatrixQuadrature {

    /**
     * Original upper bound of integration.
     */
    private final double borig;

    /**
     * Constructor.
     *
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public UpperSquareRootMidPointMatrixQuadrature(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        super(0.0, Math.sqrt(b - a), listener);
        borig = b;
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.UPPER_SQUARE_ROOT_MID_POINT;
    }

    /**
     * Evaluates function at 2*x*f(a0+x^2).
     *
     * @param x point where function is evaluated.
     * @param result instance where result of evaluation is stored.
     * @throws EvaluationException if evaluation fails.
     */
    @Override
    protected void func(final double x, final Matrix result) throws EvaluationException {
        listener.evaluate(borig - x * x, result);
        result.multiplyByScalar(2.0 * x);
    }
}
