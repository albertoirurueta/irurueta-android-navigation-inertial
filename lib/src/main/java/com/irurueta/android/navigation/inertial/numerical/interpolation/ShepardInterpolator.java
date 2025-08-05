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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Interpolates sparsely defined points of dimension "dim" using Shepard interpolation, which is
 * a simplification of Radial Basis Function interpolation that achieves less accurate results but
 * having less computational cost.
 */
public class ShepardInterpolator extends BaseRadialBasisFunctionInterpolator {
    /**
     * Values of function at provided points. Must have the same length as the number of rows of
     * provided points matrix.
     */
    private final double[] vals;

    /**
     * Negative value of p parameter controlling Shepard power-law function phi(r) = r^-p.
     * Typically p lies between 1 and 3.
     */
    private final double pneg;

    /**
     * Constructor.
     *
     * @param ptss  Matrix containing points to interpolate from. Each row contains one point.
     *              Matrix will have n points (rows) having a dimension (columns) equal to dim.
     * @param valss values of function at provided points. Must have the same length as the number
     *              of rows of provided points matrix.
     * @param p     p parameter controlling Shepard power-law function phi(r) = r^-p. Typically p
     *              lies between 1 and 3.
     * @throws IllegalArgumentException if provided values array does not match the number of
     *                                  points (rows) in provided matrix.
     */
    public ShepardInterpolator(final Matrix ptss, final double[] valss, final double p) {
        super(ptss);

        if (valss.length != n) {
            throw new IllegalArgumentException("wrong length of values");
        }

        vals = valss;
        pneg = -p;
    }

    /**
     * Constructor using default p parameter, which is equal to 2.0.
     *
     * @param ptss  Matrix containing points to interpolate from. Each row contains one point.
     *              Matrix will have n points (rows) having a dimension (columns) equal to dim.
     * @param valss values of function at provided points. Must have the same length as the number
     *              of rows of provided points matrix.
     * @throws IllegalArgumentException if provided values array does not match the number of
     *                                  points (rows) in provided matrix.
     */
    public ShepardInterpolator(final Matrix ptss, final double[] valss) {
        this(ptss, valss, 2.0);
    }

    /**
     * Returns the interpolated function value at a dim-dimensional point pt.
     *
     * @param pt dim-dimensional point where interpolation must be computed.
     * @return result of interpolation.
     * @throws IllegalArgumentException if provided point has an invalid length.
     */
    public double interpolate(final double[] pt) {
        double r, w, sum = 0., sumw = 0.;
        if (pt.length != dim) {
            throw new IllegalArgumentException("Wrong point length");
        }

        try {
            for (int i = 0; i < n; i++) {
                final int endCol = dim - 1;
                pts.getSubmatrixAsArray(i, 0, i, endCol, pi);
                if ((r = rad(pt, pi)) == 0.0) return vals[i];
                sum += (w = Math.pow(r, pneg));
                sumw += w * vals[i];
            }
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return sumw / sum;
    }
}