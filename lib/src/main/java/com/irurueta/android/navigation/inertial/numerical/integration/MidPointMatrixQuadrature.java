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
 * Implementation of matrix quadrature using mid-point algorithm.
 * Mid-point algorithm is suitable for improper integrations, which consists of any of the following
 * situations:
 * - integrand goes to a finite limiting value at finite upper and lower limits, but cannot be
 * evaluated right on one of those limits (e.g. sin(x)/x at x = 0)
 * - the upper limit is positive infinity, or the lower limit is negative infinity.
 * - integrand has an integrable singularity at either limit (e.g. x^-0.5 at x = 0)
 * - integrand has an integrable singularity at a known place between its upper and lower limits.
 * - integrand has an integrable singularity at an unknown place between its upper and lower limits.
 * <p>
 * Impossible integrals, such as those being infinite (e.g. integral from 1 to infinity of x^-1) or
 * those that have no limiting sense (e.g. integral from -infinity to infinity of cos(x)) cannot
 * be handled by this implementation.
 */
public class MidPointMatrixQuadrature extends MatrixQuadrature {
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
    protected final MatrixSingleDimensionFunctionEvaluatorListener listener;

    /**
     * Temporary value storing evaluation at mid point.
     */
    private final Matrix tmpMid;

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
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public MidPointMatrixQuadrature(
            final double a, final double b,
            final MatrixSingleDimensionFunctionEvaluatorListener listener)
            throws WrongSizeException {
        this.n = 0;
        this.a = a;
        this.b = b;

        rows = listener.getRows();
        columns = listener.getColumns();
        s = new Matrix(rows, columns);
        tmpMid = new Matrix(rows, columns);
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
     * @return current vlaue of integral.
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
    public void next(Matrix result) throws EvaluationException {
        try {
            int it;
            int j;
            double x;
            double tnm;
            double del;
            double ddel;
            n++;
            if (n == 1) {
                // (s = (b - a) * func(0.5 * (a + b)))
                func(0.5 * (a + b), tmpMid);
                s.copyFrom(tmpMid);
                s.multiplyByScalar(b - a);
                result.copyFrom(s);
            } else {
                for (it = 1, j = 1; j < n - 1; j++) {
                    it *= 3;
                }
                tnm = it;
                del = (b - a) / (3.0 * tnm);
                // The added points alternate in spacing between del and ddel
                ddel = del + del;
                x = a + 0.5 * del;
                sum.initialize(0.0);
                for (j = 0; j < it; j++) {
                    func(x, tmpX);
                    sum.add(tmpX);

                    x += ddel;
                    func(x, tmpX);
                    sum.add(tmpX);
                    x += del;
                }
                // The new sum is combined with the old integral to give a refined integral
                // s = (s + (b - a) * sum / tnm) / 3.0;
                sum.multiplyByScalar((b - a) / tnm);
                s.add(sum);
                s.multiplyByScalar(1.0 / 3.0);
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
        return QuadratureType.MID_POINT;
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

    /**
     * Evaluates matrix function at x.
     *
     * @param x      point where function is evaluated.
     * @param result instance where result of evaluation is stored.
     * @throws EvaluationException if evaluation fails.
     */
    protected void func(final double x, final Matrix result) throws EvaluationException {
        listener.evaluate(x, result);
    }
}
