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
 * This implementation is faster and more accurate than other implementations such as
 * implementations of {@link com.irurueta.numerical.polynomials.estimators.PolynomialEstimator}.
 * Those implementations are based on curve fitting a polynomial of a given degree based on a number
 * of points that can be much larger than the number of points, which results in a polynomial that
 * does not exactly pass through provided points, but provides a minimum square error result.
 * On the other hand, this implementation builds a polynomial of order equal to the length of
 * provided x and y points.
 */
public class SimpleInterpolatingPolynomialEstimator extends InterpolatingPolynomialEstimator {

    /**
     * Estimates polynomial coefficients from provided x and y points.
     *
     * @param x   x points the estimated polynomial passes through.
     * @param y   y points the estimated polynomial passes through.
     * @param cof instance where coefficients of estimated polynomial will be stored.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length.
     */
    @Override
    public void estimate(final double[] x, final double[] y, final double[] cof) {
        if (x.length != y.length || x.length != cof.length) {
            throw new IllegalArgumentException("Wrong length of points or polynomial order");
        }

        int k;
        int j;
        int i;
        double phi;
        double ff;
        double b;
        final int n = x.length;
        final double[] s = new double[n];

        for (i = 0; i < n; i++) {
            s[i] = cof[i] = 0.0;
        }
        s[n - 1] = -x[0];
        for (i = 1; i < n; i++) {
            // Coefficients si of the master polynomial P(x) are found by recurrence
            for (j = n - 1 - i; j < n - 1; j++) {
                s[j] -= x[i] * s[j + 1];
            }
            s[n - 1] -= x[i];
        }
        for (j = 0; j < n; j++) {
            phi = n;
            for (k = n - 1; k > 0; k--) {
                // The quantity phi = is found as a derivative of P(xj)
                phi = k * s[k] + x[j] * phi;
            }
            ff = y[j] / phi;
            b = 1.0;
            // Coefficients of polynomials in each term of the Lagrange formula are found by
            // synthetic division of P(x) by (x - xj). The solution ck is accumulated.
            for (k = n - 1; k >= 0; k--) {
                cof[k] += b * ff;
                b = s[k] + x[j] * b;
            }
        }
    }
}
