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
 * Interpolation in two dimensions.
 * This implementation uses a higher order than {@link BilinearInterpolator} for accuracy reasons.
 */
public class Polynomial2DInterpolator {

    /**
     * Length of x1v array.
     */
    private final int m;

    /**
     * Length of x2v array.
     */
    private final int n;

    /**
     * Number of rows of sub-block of ym values to be processed.
     */
    private final int mm;

    /**
     * Number of columns of sub-block of ym values to be processed.
     */
    private final int nn;

    /**
     * Matrix of tabulated function values yij.
     */
    private final Matrix y;

    /**
     * Temporary array containing interpolated values in one direction.
     */
    private final double[] yv;

    /**
     * One dimensional interpolator for x1v.
     */
    private final PolynomialInterpolator x1terp;

    /**
     * One dimensional interpolator for x2v.
     */
    private final PolynomialInterpolator x2terp;

    /**
     * Constructor.
     *
     * @param x1v array of x1v.
     * @param x2v array of x2v.
     * @param ym  matrix of tabulated function values yij.
     * @param mp  defines number of rows of sub-block of ym values to be processed.
     * @param np  defined number of columns of sub-block of ym values to be processed.
     */
    public Polynomial2DInterpolator(
            final double[] x1v, final double[] x2v, final Matrix ym, final int mp, final int np) {
        m = x1v.length;
        n = x2v.length;
        mm = mp;
        nn = np;
        y = ym;
        yv = new double[m];
        // Dummy 1-dim interpolations for their locate and hunt functions
        x1terp = new PolynomialInterpolator(x1v, yv, mm);
        x2terp = new PolynomialInterpolator(x2v, new double[n], nn);
    }

    /**
     * Constructor.
     *
     * @param x1v array of x1v.
     * @param x2v array of x2v.
     * @param ym  matrix of tabulated function values yij.
     */
    public Polynomial2DInterpolator(final double[] x1v, final double[] x2v, final Matrix ym) {
        this(x1v, x2v, ym, x1v.length, x2v.length);
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
     * Gets number of rows of sub-block of ym values to be processed.
     *
     * @return number of rows of sub-block of ym values to be processed.
     */
    public int getMm() {
        return mm;
    }

    /**
     * Gets number of columns of sub-block of ym values to be processed.
     *
     * @return number of columns of sub-block of ym values to be processed.
     */
    public int getNn() {
        return nn;
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
        try {
            final int i = x1terp.cor != 0 ? x1terp.hunt(x1p) : x1terp.locate(x1p);
            final int j = x2terp.cor != 0 ? x2terp.hunt(x2p) : x2terp.locate(x2p);
            int k;

            // Find grid block
            for (k = i; k < i + mm; k++) {
                // mm interpolations in the x2 direction.
                // copy k-row of matrix y
                y.getSubmatrixAsArray(k, 0, k, y.getColumns() - 1, x2terp.yy);
                yv[k] = x2terp.rawinterp(j, x2p);
            }

            // A final interpolation in the x1 direction.
            return x1terp.rawinterp(i, x1p);
        } catch (final WrongSizeException e) {
            throw new InterpolationException(e);
        }
    }
}
