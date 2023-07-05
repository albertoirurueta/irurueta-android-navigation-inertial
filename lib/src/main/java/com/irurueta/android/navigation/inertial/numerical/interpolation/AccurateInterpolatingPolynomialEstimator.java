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

/**
 * Estimates coefficients of a polynomial passing through provided set of x and y points.
 * This implementation is more accurate than other implementations such as implementations of
 * {@link com.irurueta.numerical.polynomials.estimators.PolynomialEstimator}.
 * Those implementations are based on curve fitting a polynomial of a given degree based on a number
 * of points that can be much larger than the number of points, which results in a polynomial that
 * does not exactly pass through provided points, but provides a minimum square error result.
 * On the other hand, this implementation builds a polynomial of order equal to the length of
 * provided x and y points.
 * Ths implementation is more accurate than {@link SimpleInterpolatingPolynomialEstimator}, at the
 * expense of greater computational cost (N^3 of this method vs N^2 of the simple one).
 */
public class AccurateInterpolatingPolynomialEstimator extends InterpolatingPolynomialEstimator {

    /**
     * Estimates polynomial coefficients from provided x and y points.
     *
     * @param xa  x points the estimated polynomial passes through.
     * @param ya  y points the estimated polynomial passes through.
     * @param cof instance where coefficients of estimated polynomial will be stored.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length.
     * @throws InterpolationException   if interpolation fails for numerical reasons.
     */
    @Override
    public void estimate(final double[] xa, final double[] ya, final double[] cof)
            throws InterpolationException {
        int k;
        int j;
        int i;
        double xmin;
        final int n = xa.length;
        final double[] x = new double[n];
        final double[] y = new double[n];

        for (j = 0; j < n; j++) {
            x[j] = xa[j];
            y[j] = ya[j];
        }
        for (j = 0; j < n; j++) {
            final PolynomialInterpolator interp = new PolynomialInterpolator(x, y, n - j, false);
            // extrapolate to x = 0
            cof[j] = interp.rawinterp(0, 0.);
            xmin = 1.0e99;
            k = -1;
            for (i = 0; i < n - j; i++) {
                // Find the remaining xi of smallest absolute value
                if (Math.abs(x[i]) < xmin) {
                    xmin = Math.abs(x[i]);
                    k = i;
                }
                if (x[i] != 0.0)
                    // (meanwhile reducing all the terms)
                    y[i] = (y[i] - cof[j]) / x[i];
            }
            // and eliminate it
            for (i = k + 1; i < n - j; i++) {
                y[i - 1] = y[i];
                x[i - 1] = x[i];
            }
        }
    }
}
