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
 * Implementation of matrix quadrature using trapezoidal algorithm.
 * This implementation is suitable for non-improper integrands, which consist
 * of functions with not known singularities that can be evaluated on all the integration
 * interval, which must be finite.
 */
public class TrapezoidalMatrixQuadrature extends MatrixQuadrature {

    /**
     * Lower limit of integration.
     */
    private final double a;

    /**
     * Upper limit of integration.
     */
    private final double b;

    /**
     * Number of rows of quadrature result.
     */
    private final int rows;

    /**
     * Number of columns of quadrature result.
     */
    private final int columns;

    /**
     * Current value of integral.
     */
    private final Matrix s;

    /**
     * Listener to evaluate single dimension matrix functions at required points.
     */
    private final MatrixSingleDimensionFunctionEvaluatorListener listener;

    /**
     * Temporary value storing evaluation at point a.
     */
    private final Matrix tmpA;

    /**
     * Temporary value storing evaluation at point b.
     */
    private final Matrix tmpB;

    /**
     * Temporary value storing summation of evaluations.
     */
    private final Matrix sum;

    /**
     * Temporary value storing evaluation at point x.
     */
    private final Matrix tmpX;

    /**
     * Constructor.
     *
     * @param a Lower limit of integration.
     * @param b Upper limit of integration.
     * @param listener listener to evaluate a single dimension function at required points.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public TrapezoidalMatrixQuadrature(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        this.n = 0;
        this.a = a;
        this.b = b;

        rows = listener.getRows();
        columns = listener.getColumns();
        s = new Matrix(rows, columns);
        tmpA = new Matrix(rows, columns);
        tmpB = new Matrix(rows, columns);
        sum = new Matrix(rows, columns);
        tmpX = new Matrix(rows, columns);

        this.listener = listener;
    }

    /**
     * Gets lower limit of integration.
     *
     * @return lower limit of integration.
     */
    public double getA() {
        return a;
    }

    /**
     * Gets upper limit of integration.
     *
     * @return upper limit of integration.
     */
    public double getB() {
        return b;
    }

    /**
     * Gets current value of integral.
     *
     * @return current value of integral.
     */
    public Matrix getS() {
        return s;
    }

    /**
     * Returns the value of the integral at the nth stage of refinement.
     *
     * @param result instance where the value of the integral at the nth stage of refinement will
     *               be stored.
     * @throws EvaluationException Raised if something failed during the evaluation.
     */
    @Override
    public void next(final Matrix result) throws EvaluationException {
        try {
            double x;
            double tnm;
            double del;
            int it;
            int j;
            n++;
            if (n == 1) {
                // (s = 0.5 * (b - a) * (listener.evaluate(a) + listener.evaluate(b))
                listener.evaluate(a, tmpA);
                listener.evaluate(b, tmpB);
                s.copyFrom(tmpA);
                s.add(tmpB);
                s.multiplyByScalar(0.5 * (b - a));
                result.copyFrom(s);
            } else {
                for (it = 1, j = 1; j < n - 1; j++) {
                    it <<= 1;
                }
                tnm = it;
                // This is the spacing of the points to be added
                del = (b - a) / tnm;
                x = a + 0.5 * del;
                sum.initialize(0.0);
                for (j = 0; j < it; j++, x += del) {
                    listener.evaluate(x, tmpX);
                    sum.add(tmpX);
                }
                // This replaces s by its refined value
                // s = 0.5 * (s + (b-a) * sum / tnm);
                sum.multiplyByScalar((b - a) / tnm);
                s.add(sum);
                s.multiplyByScalar(0.5);
                result.copyFrom(s);
            }
        } catch (final WrongSizeException ex) {
            throw new EvaluationException(ex);
        }
    }

    /**
     * Gets type of quadrature.
     *
     * @return type of quadrature.
     */
    @Override
    public QuadratureType getType() {
        return QuadratureType.TRAPEZOIDAL;
    }

    /**
     * Gets number of rows of quadrature result.
     *
     * @return number of rows of quadrature result.
     */
    @Override
    protected int getRows() {
        return rows;
    }

    /**
     * Gets number of columns of quadrature result.
     *
     * @return number of columns of quadrature result.
     */
    @Override
    protected int getColumns() {
        return columns;
    }
}
