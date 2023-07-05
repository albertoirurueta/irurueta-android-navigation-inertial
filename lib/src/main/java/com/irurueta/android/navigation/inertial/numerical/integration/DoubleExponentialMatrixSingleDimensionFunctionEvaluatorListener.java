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
import com.irurueta.numerical.EvaluationException;

/**
 * Interface to define how matrix (multivariate) single dimension functions can be evaluated in
 * Double Exponential Rule Quadrature function integrators.
 */
interface DoubleExponentialMatrixSingleDimensionFunctionEvaluatorListener {

    /**
     * Evaluates a single dimension function such as f(x) at provided point and returns the result.
     *
     * @param x      point where function will be evaluated.
     * @param delta  value to handle singularities. If the function has no singularities, or the
     *               singularities are “mild” (e.g., no worse than logarithmic), this can be ignored.
     * @param result matrix where function evaluation will be stored.
     * @throws EvaluationException raised if something failed during the evaluation.
     */
    void evaluate(final double x, final double delta, final Matrix result)
            throws EvaluationException;

    /**
     * Gets number of rows of matrix result of function f.
     *
     * @return number of rows.
     */
    int getRows();

    /**
     * Gets number of columns of matrix result of function f.
     *
     * @return number of columns.
     */
    int getColumns();
}
