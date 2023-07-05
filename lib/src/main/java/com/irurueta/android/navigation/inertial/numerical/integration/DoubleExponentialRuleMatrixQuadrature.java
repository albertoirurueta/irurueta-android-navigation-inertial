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
 * Implementation of quadrature using double exponential, which allows integration with a variable
 * transformation.
 * This implementation is suitable for improper integrands containing singularities.
 */
public class DoubleExponentialRuleMatrixQuadrature extends MatrixQuadrature {

    /**
     * Default transformation of range of integration.
     */
    public static final double DEFAULT_HMAX = 3.7;

    /**
     * Lower limit of integration.
     */
    private final double a;

    /**
     * Upper limit of integration.
     */
    private final double b;

    /**
     * Maximum step size. Determines transformation of range of integration.
     */
    private final double hmax;

    /**
     * Number of rows of quadrature result.
     */
    private final int rows;

    /**
     * Number of columns of quadrature result.
     */
    private final int columns;

    /**
     * Value of the next stage of refinement.
     */
    private final Matrix s;

    /**
     * Temporary value storing summation of evaluations.
     */
    private final Matrix sum;

    /**
     * Temporary value storing evaluation at mid point.
     */
    private final Matrix tmpMid;

    /**
     * Temporary value storing evaluation at lower bound.
     */
    private final Matrix tmpA;

    /**
     * Temporary value storing evaluation at upper bound.
     */
    private final Matrix tmpB;

    /**
     * Temporary value storing evaluation at point x.
     */
    private final Matrix tmpX;

    /**
     * Listener to evaluate single dimension functions at required points.
     */
    private final DoubleExponentialMatrixSingleDimensionFunctionEvaluatorListener listener;

    /**
     * Constructor.
     *
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public DoubleExponentialRuleMatrixQuadrature(
            final DoubleExponentialMatrixSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b, final double hmax) throws WrongSizeException {
        this.listener = listener;
        this.a = a;
        this.b = b;
        this.hmax = hmax;
        n = 0;

        rows = listener.getRows();
        columns = listener.getColumns();
        s = new Matrix(rows, columns);
        sum = new Matrix(rows, columns);
        tmpMid = new Matrix(rows, columns);
        tmpA = new Matrix(rows, columns);
        tmpB = new Matrix(rows, columns);
        tmpX = new Matrix(rows, columns);
    }

    /**
     * Constructor.
     *
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public DoubleExponentialRuleMatrixQuadrature(
            final DoubleExponentialMatrixSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b) throws WrongSizeException {
        this(listener, a, b, DEFAULT_HMAX);
    }

    /**
     * Constructor.
     *
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @param hmax     Maximum step size. This quadrature transforms the range of integration to
     *                 [-hmax, hmax].
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public DoubleExponentialRuleMatrixQuadrature(
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b, final double hmax) throws WrongSizeException {
        this(new DoubleExponentialMatrixSingleDimensionFunctionEvaluatorListener() {
            @Override
            public void evaluate(double x, double delta, Matrix result) throws EvaluationException {
                listener.evaluate(x, result);
            }

            @Override
            public int getRows() {
                return listener.getRows();
            }

            @Override
            public int getColumns() {
                return listener.getColumns();
            }
        }, a, b, hmax);
    }

    /**
     * Constructor.
     *
     * @param listener listener to evaluate a single dimension matrix function at required points.
     * @param a        Lower limit of integration.
     * @param b        Upper limit of integration.
     * @throws WrongSizeException if size notified by provided listener is invalid.
     */
    public DoubleExponentialRuleMatrixQuadrature(
            final MatrixSingleDimensionFunctionEvaluatorListener listener,
            final double a, final double b) throws WrongSizeException {
        this(listener, a, b, DEFAULT_HMAX);
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
            // On the first call to the function next (n = 1), the routine returns the crudest estimate
            // of integral between a and b of f(x). Subsequent calls to next (n = 2, 3, ...) will
            // improve the accuracy by adding 2^(n-1) additional interior points.
            double del;
            double fact;
            double q;
            double t;
            final double twoh;
            int it;
            int j;
            n++;
            if (n == 1) {
                fact = 0.25;
                // s = hmax * 2.0 * (b - a) * fact * listener.evaluate(0.5 * (b + a), 0.5 * (b - a))
                listener.evaluate(0.5 * (b + a), 0.5 * (b - a), tmpMid);
                s.copyFrom(tmpMid);
                s.multiplyByScalar(hmax * 2.0 * (b - a) * fact);
                result.copyFrom(s);
            } else {
                for (it = 1, j = 1; j < n - 1; j++) {
                    it <<= 1;
                }

                // Twice the spacing of the points to be added
                twoh = hmax / it;
                t = 0.5 * twoh;
                sum.initialize(0.0);
                for (j = 0; j < it; j++) {
                    q = Math.exp(-2.0 * Math.sinh(t));
                    del = (b - a) * q / (1.0 + q);
                    final double value = 1.0 + q;
                    fact = q / (value * value) * Math.cosh(t);
                    listener.evaluate(a + del, del, tmpA);
                    listener.evaluate(b - del, del, tmpB);
                    tmpX.copyFrom(tmpA);
                    tmpX.add(tmpB);
                    tmpX.multiplyByScalar(fact);
                    sum.add(tmpX);
                    t += twoh;
                }

                // Replace s by its refined value and return.
                // s = 0.5 * s + (b - a) * twoh * sum;
                sum.multiplyByScalar((b - a) * twoh);
                s.multiplyByScalar(0.5);
                s.add(sum);
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
        return QuadratureType.DOUBLE_EXPONENTIAL_RULE;
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
