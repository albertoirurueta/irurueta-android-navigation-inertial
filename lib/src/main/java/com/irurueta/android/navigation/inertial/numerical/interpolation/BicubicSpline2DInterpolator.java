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

/**
 * Computes bicubic spline interpolation in two dimensions.
 */
public class BicubicSpline2DInterpolator {

    /**
     * Length of x1v array.
     */
    private final int m;

    /**
     * Length of x2v array.
     */
    private final int n;

    /**
     * Array of x1v.
     */
    private final double[] x1;

    /**
     * Array of x2v.
     */
    private final double[] yv;

    /**
     * Array of one dimensional cubic spline interpolators.
     */
    private final CubicSplineInterpolator[] srp;

    /**
     * Constructor.
     *
     * @param x1v array of x1v.
     * @param x2v array of x2v.
     * @param ym  matrix of tabulated function values yij.
     */
    public BicubicSpline2DInterpolator(final double[] x1v, final double[] x2v, final Matrix ym) {
        m = x1v.length;
        n = x2v.length;
        yv = new double[m];
        x1 = x1v;
        srp = new CubicSplineInterpolator[m];

        for (int i = 0; i < m; i++) {
            // get i-th row of y
            final double[] yi = ym.getSubmatrixAsArray(i, 0, i, ym.getColumns() - 1);
            srp[i] = new CubicSplineInterpolator(x2v, yi);
        }
    }

    /**
     * Gets length of x1v array.
     *
     * @return length of x1v array.
     */
    public int getM() {
        return m;
    }

    /**
     * Gets length of x2v array.
     *
     * @return length of x2v array.
     */
    public int getN() {
        return n;
    }

    /**
     * Given values x1p an x2p, returns an interpolated value.
     *
     * @param x1p x1p value where interpolation is estimated.
     * @param x2p x2p value where interpolation is estimated.
     * @return interpolated value.
     * @throws InterpolationException if interpolation fails.
     */
    public double interpolate(final double x1p, final double x2p) throws InterpolationException {
        for (int i = 0; i < m; i++) {
            yv[i] = srp[i].interpolate(x2p);
        }

        final CubicSplineInterpolator scol = new CubicSplineInterpolator(x1, yv);
        return scol.interpolate(x1p);
    }
}
