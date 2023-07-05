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
 * Interpolation in two dimensions.
 * This is the simplest implementation.
 */
public class BilinearInterpolator {

    /**
     * Length of x1v array.
     */
    private final int m;

    /**
     * Length of x2v array.
     */
    private final int n;

    /**
     * Matrix of tabulated function values yij.
     */
    private final Matrix y;

    /**
     * One dimensional interpolator for x1v.
     */
    private final LinearInterpolator x1terp;

    /**
     * One dimensional interpolator for x2v.
     */
    private final LinearInterpolator x2terp;

    /**
     * Constructor.
     *
     * @param x1v array of x1v.
     * @param x2v array of x2v.
     * @param ym matrix of tabulated function values yij.
     */
    @SuppressWarnings("SuspiciousNameCombination")
    public BilinearInterpolator(final double[] x1v, final double[] x2v, final Matrix ym) {
        m = x1v.length;
        n = x2v.length;
        y = ym;
        // Construct dummy 1-dim interpolators for their locate and hunt methods
        x1terp = new LinearInterpolator(x1v, x1v);
        x2terp = new LinearInterpolator(x2v, x2v);
    }

    /**
     * Gets length of x1v array.
     * @return length of x1v array.
     */
    public int getM() {
        return m;
    }

    /**
     * Gets length of x2v array.
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
     */
    public double interpolate(final double x1p, final double x2p) {
        final int i = x1terp.cor != 0 ? x1terp.hunt(x1p) : x1terp.locate(x1p);
        final int j = x2terp.cor != 0 ? x2terp.hunt(x2p) : x2terp.locate(x2p);

        // Find the grid square
        final double t = (x1p - x1terp.xx[i]) / (x1terp.xx[i + 1] - x1terp.xx[i]);
        final double u = (x2p - x2terp.xx[j]) / (x2terp.xx[j + 1] - x2terp.xx[j]);

        // Interpolate
        return (1. - t) * (1. - u) * y.getElementAt(i, j)
                + t * (1. - u) * y.getElementAt(i + 1, j)
                + (1. - t) * u * y.getElementAt(i, j + 1)
                + t * u * y.getElementAt(i + 1, j + 1);
    }

}
