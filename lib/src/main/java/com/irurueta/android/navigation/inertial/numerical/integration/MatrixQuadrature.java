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
 * Abstract base class for elementary matrix quadrature algorithms used for matrix (multivariate)
 * single dimension function integration.
 */
public abstract class MatrixQuadrature {
    /**
     * Current level of refinement.
     */
    protected int n;

    /**
     * Gets current level of refinement.
     *
     * @return current level of refinement.
     */
    public int getN() {
        return n;
    }

    /**
     * Returns the value of the integrat at the nth stage of refinement.
     *
     * @param result instance where the value of the integral at the nth stage of refinement will
     *               be stored.
     * @throws EvaluationException Raised if something failed during the evaluation.
     */
    public abstract void next(final Matrix result) throws EvaluationException;

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    public abstract QuadratureType getType();

    /**
     * Gets number of rows of quadrature result.
     *
     * @return number of rows of quadrature result.
     */
    protected abstract int getRows();

    /**
     * Gets number of columns of quadrature result.
     *
     * @return number of columns of quadrature result.
     */
    protected abstract int getColumns();
}
