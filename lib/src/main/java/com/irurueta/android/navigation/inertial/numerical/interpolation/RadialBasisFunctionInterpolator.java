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
package com.irurueta.android.navigation.inertial.numerical.interpolation;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.LUDecomposer;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Interpolates sparsely defined points of dimension "dim" using a Radial Basis Function.
 */
public class RadialBasisFunctionInterpolator extends BaseRadialBasisFunctionInterpolator {
    /**
     * Computed weights to compute interpolation from provided points.
     */
    private final Matrix w;

    /**
     * Radial basis function defining a value based on the distance of two points.
     */
    private final RadialBasisFunction fn;

    /**
     * Indicates whether normalized Radial Basis Function (RBF) must be used or not.
     */
    private final boolean norm;

    /**
     * Constructor.
     *
     * @param ptss  Matrix containing points to interpolate from. Each row contains one point.
     *              Matrix will have n points (rows) having a dimension (columns) equal to dim.
     * @param valss values of function at provided points. Must have the same length as the number
     *              of rows of provided points matrix.
     * @param func  function to be used as Radial Basis Function (RBF).
     * @param nrbf  true to normalize RBF, false otherwise.
     * @throws InterpolationException   if provided points are redundant and result in a degenerate
     *                                  solution.
     * @throws IllegalArgumentException if provided values array does not match the number of
     *                                  points (rows) in provided matrix.
     */
    public RadialBasisFunctionInterpolator(
            final Matrix ptss, final double[] valss, final RadialBasisFunction func,
            final boolean nrbf) throws InterpolationException {
        super(ptss);

        if (valss.length != n) {
            throw new IllegalArgumentException("wrong length of values");
        }

        try {
            w = new Matrix(n, 1);
            fn = func;
            norm = nrbf;

            final double[] pj = new double[dim];

            int i;
            int j;
            double sum;
            final Matrix rbf = new Matrix(n, n);
            final Matrix rhs = new Matrix(n, 1);
            for (i = 0; i < n; i++) {
                // Fill the matrix phi(|ri - rj|) and the right hand sisde (rhs) vector
                sum = 0.;
                for (j = 0; j < n; j++) {
                    final int endCol = dim - 1;
                    pts.getSubmatrixAsArray(i, 0, i, endCol, pi);
                    pts.getSubmatrixAsArray(j, 0, j, endCol, pj);
                    final double value = fn.evaluate(rad(pi, pj));
                    rbf.setElementAt(i, j, value);
                    sum += value;
                }

                if (norm) {
                    rhs.setElementAtIndex(i, sum * valss[i]);
                } else {
                    rhs.setElementAtIndex(i, valss[i]);
                }
            }

            // Solve the set of linear equations
            final LUDecomposer lu = new LUDecomposer(rbf);
            lu.decompose();
            lu.solve(rhs, w);
        } catch (final AlgebraException e) {
            throw new InterpolationException(e);
        }
    }

    /**
     * Constructor.
     *
     * @param ptss  Matrix containing points to interpolate from. Each row contains one point.
     *              Matrix will have n points (rows) having a dimension (columns) equal to dim.
     * @param valss values of function at provided points. Must have the same length as the number
     *              of rows of provided points matrix.
     * @param func  function to be used as Radial Basis Function (RBF).
     * @throws InterpolationException   if provided points are redundant and result in a degenerate
     *                                  solution.
     * @throws IllegalArgumentException if provided values array does not match the number of
     *                                  points (rows) in provided matrix.
     */
    public RadialBasisFunctionInterpolator(
            final Matrix ptss, final double[] valss, final RadialBasisFunction func)
            throws InterpolationException {
        this(ptss, valss, func, false);
    }

    /**
     * Returns the interpolated function value at a dim-dimensional point pt.
     *
     * @param pt dim-dimentional point where interpolation must be computed.
     * @return result of interpolation.
     * @throws IllegalArgumentException if provided point has an invalid length.
     */
    @Override
    public double interpolate(final double[] pt) {
        if (pt.length != dim) {
            throw new IllegalArgumentException("Wrong point length");
        }

        double fval;
        double sum = 0.0;
        double sumw = 0.0;

        try {
            for (int i = 0; i < n; i++) {
                final int endCol = dim - 1;
                pts.getSubmatrixAsArray(i, 0, i, endCol, pi);
                fval = fn.evaluate(rad(pt, pi));
                sumw += w.getElementAtIndex(i) * fval;
                sum += fval;
            }
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return norm ? sumw / sum : sumw;
    }
}
