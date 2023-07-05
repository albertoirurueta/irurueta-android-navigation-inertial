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
package com.irurueta.android.navigation.inertial.numerical;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;

/**
 * Estimates exponential of a square matrix.
 * This is based on Gene H. Golub and Charles F. Van Loan. Matrix Computations. 3rd ed. 1996. p. 572
 */
public class ExponentialMatrixEstimator {

    /**
     * Default error tolerance of estimated result element-wise.
     * When this tolerance is used, algorithm achieves an error close to machine precision, but
     * might be slightly larger than this value (which is about 1.1e-16).
     */
    public static final double TOLERANCE = Math.pow(2.0, -53.0);

    /**
     * Estimates factorial values.
     */
    private final DoubleFactorialEstimator factorialEstimator = new DoubleFactorialEstimator();

    /**
     * Number of rows of matrix to be estimated. Every time the number of rows change, reused
     * matrices are re-instantiated for computational efficiency.
     */
    private int rows = 0;

    /**
     * Scaled version of provided input matrix. This instance is reused as long as provided input
     * matrices keep the same size.
     */
    private Matrix as;

    /**
     * Denominator of Padé approximant. This is reused for efficiency while provided input matrices
     * keep the same size.
     */
    private Matrix d;

    /**
     * Numerator of Padé approximant. This is reused for efficiency while provided input matrices
     * keep the same size.
     */
    private Matrix n;

    /**
     * Internal matrix reused for efficiency while provided input matrices keep the same size.
     */
    private Matrix x;

    /**
     * Internal matrix reused for efficiency while provided input matrices keep the same size.
     */
    private Matrix tmp;

    /**
     * Internal matrix reused for efficiency while provided input matrices keep the same size.
     */
    private Matrix cX;

    /**
     * Contains copy of estimated result. This is reused for efficiency while provided input
     * matrices keep the same size.
     */
    private Matrix f;

    /**
     * Estimates exponential of provided matrix with default error tolerance.
     * When this tolerance is used, algorithm achieves an error close to machine precision, but
     * might be slightly larger than this value (which is about 1.1e-6).
     *
     * @param a matrix to be used for exponential estimation.
     * @return estimated exponential matrix.
     * @throws AlgebraException if there are numerical errors.
     */
    public Matrix exponential(final Matrix a) throws AlgebraException {
        return exponential(a, TOLERANCE);
    }

    /**
     * Estimates exponential of provided matrix.
     * Larger tolerance than default one can be used to reduce computational complexity if less
     * accuracy is required.
     *
     * @param a         matrix to be used for exponential estimation.
     * @param tolerance maximum allowed absolute error tolerance element-wise.
     * @return estimated exponential matrix.
     * @throws AlgebraException if there are numerical errors.
     */
    public Matrix exponential(final Matrix a, final double tolerance) throws AlgebraException {
        final Matrix result = new Matrix(a.getRows(), a.getColumns());
        exponential(a, result, tolerance);
        return result;
    }

    /**
     * Estimates exponential of provided matrix with default error tolerance.
     * When this tolerance is used, algorithm achieves an error close to machine precision, but
     * might be slightly larger than this value (which is about 1.1e-6).
     *
     * @param a      matrix to be used for exponential estimation.
     * @param result instance where result will be stored.
     * @throws AlgebraException if there are numerical errors.
     */
    public void exponential(final Matrix a, final Matrix result) throws AlgebraException {
        exponential(a, result, TOLERANCE);
    }

    /**
     * Estimates exponential of provided matrix.
     * Larger tolerance than default one can be used to reduce computational complexity if less
     * accuracy is required.
     *
     * @param a         matrix to be used for exponential estimation.
     * @param result    instance where result will be stored.
     * @param tolerance maximum allowed absolute error tolerance element-wise.
     * @throws AlgebraException if there are numerical errors.
     */
    public void exponential(final Matrix a, final Matrix result, final double tolerance)
            throws AlgebraException {
        final int rows = a.getRows();

        if (rows != a.getColumns()) {
            throw new IllegalArgumentException("Matrix must be squared");
        }
        if (result.getRows() != rows || result.getColumns() != rows) {
            throw new IllegalArgumentException(
                    "Result and provided matrix must have the same size");
        }
        if (tolerance < 0.0) {
            throw new IllegalArgumentException("Tolerance must be zero or greater");
        }

        final double normmax = normmax(a);
        final int j = Math.max(0, 1 + (int) Math.floor(Math.log(normmax) / Math.log(2.0)));

        initialize(rows);

        // scaled version of A
        as.copyFrom(a);
        as.multiplyByScalar(1.0 / Math.pow(2.0, j));

        // find order for required tolerance
        final int q = findQForTolerance(tolerance, normmax);

        double c = 1.0;
        for (int k = 1; k <= q; k++) {
            c = c * (q - k + 1) / ((2 * q - k + 1) * k);

            // X = As * X
            tmp.copyFrom(x);
            as.multiply(tmp, x);

            // c * X
            cX.copyFrom(x);
            cX.multiplyByScalar(c);

            // N = N + c * X
            n.add(cX);

            // D = D + (-1)^k * cX
            if (k % 2 != 0) {
                // k is odd --> (-1)^k = -1
                cX.multiplyByScalar(-1.0);
            }
            // when k is even --> (-1)^k = 1 and there is no sign multiplication needed
            d.add(cX);
        }

        // Solve DF = N for F using Gaussian elimination.
        Utils.solve(d, n, f);

        // now square j times
        for (int k = 0; k < j; k++) {
            // F = F^2 = F * F
            tmp.copyFrom(f);
            tmp.multiply(f);
            f.copyFrom(tmp);
        }

        result.copyFrom(f);
    }

    /**
     * Initializes matrices being reused as long as number of rows is preserved for multiple
     * provided input matrices for efficiency purposes.
     *
     * @param rows number of rows.
     * @throws AlgebraException if an error occurs during instantiation.
     */
    private void initialize(final int rows) throws AlgebraException {
        if (rows != this.rows) {
            as = new Matrix(rows, rows);
            d = Matrix.identity(rows, rows);
            n = Matrix.identity(rows, rows);
            x = Matrix.identity(rows, rows);

            tmp = new Matrix(rows, rows);
            cX = new Matrix(rows, rows);

            f = new Matrix(rows, rows);

            this.rows = rows;
        } else {
            as.initialize(0.0);
            Matrix.identity(d);
            Matrix.identity(n);
            Matrix.identity(x);

            tmp.initialize(0.0);
            cX.initialize(0.0);

            f.initialize(0.0);
        }
    }

    /**
     * Estimates relative error achieved by this algorithm for provided input values.
     *
     * @param p Padé approximant order of numerator.
     * @param q Padé approximant order of denominator.
     * @return estimated relative error.
     */
    private double relativeError(final int p, final int q) {
        final double pFact = factorialEstimator.factorial(p);
        final double qFact = factorialEstimator.factorial(q);
        final double pPlusQFact = factorialEstimator.factorial(p + q);
        final double pPlusQPlusOneFact = factorialEstimator.factorial(p + q + 1);

        return Math.pow(2.0, 3.0 - (p + q)) * pFact / pPlusQFact * qFact / pPlusQPlusOneFact;
    }

    /**
     * Finds required order of Padé approximant for provided maximum allowed relative error to be
     * achieved.
     *
     * @param maxRelativeError maximum allowed relative error.
     * @return Padé approximant order.
     */
    private int findQForRelativeError(final double maxRelativeError) {
        for (int q = 0; ; q++) {
            final double relativeError = relativeError(q, q);
            if (relativeError <= maxRelativeError) {
                return q;
            }
        }
    }

    /**
     * Finds required order of Padé approximant for provided absolute error tolerance to be
     * achieved.
     *
     * @param tolerance maximum allowed absolute error tolerance.
     * @param normA     infinite norm (maximum absolute value) of provided matrix.
     * @return Padé approximant order.
     */
    private int findQForTolerance(final double tolerance, final double normA) {
        return findQForRelativeError(tolerance / normA);
    }

    /**
     * Estimates infinite norm of provided matrix.
     * Infinite norm is equivalent to the maximum absolute value of all matrix elements.
     *
     * @param a matrix to compute infinite norm for.
     * @return estimated infinite norm.
     */
    private static double normmax(final Matrix a) {
        double max = 0.0;
        double[] buffer = a.getBuffer();
        for (double v : buffer) {
            double value = Math.abs(v);
            if (value > max) {
                max = value;
            }
        }
        return max;
    }
}
