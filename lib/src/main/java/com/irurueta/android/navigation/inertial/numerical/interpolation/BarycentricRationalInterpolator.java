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
 * Computes barycentric rational interpolation.
 */
public class BarycentricRationalInterpolator extends BaseInterpolator {

    /**
     * Weights for barycentric rational interpolation.
     */
    private final double[] w;

    /**
     * Order of desired approximation.
     */
    private final int d;

    /**
     * Constructor.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     * @param d order of desired approximation.
     */
    public BarycentricRationalInterpolator(final double[] x, final double[] y, final int d) {
        super(x, y, x.length);
        w = new double[n];
        this.d = d;

        if (n <= d) {
            throw new IllegalArgumentException("d too large for number of points");
        }

        for (int k = 0; k < n; k++) {
            int imin = Math.max(k - d, 0);
            int imax = k >= n - d ? n - d - 1 : k;
            double temp = (imin & 1) != 0 ? -1.0 : 1.0;
            double sum = 0.0;
            for (int i = imin; i <= imax; i++) {
                int jmax = Math.min(i + d, n - 1);
                double term = 1.0;
                for (int j = i; j <= jmax; j++) {
                    if (j == k) {
                        continue;
                    }
                    term *= (xx[k] - xx[j]);
                }
                term = temp / term;
                temp = -temp;
                sum += term;
            }
            w[k] = sum;
        }
    }

    /**
     * Gets order of desired approximation.
     *
     * @return order of desired approximation.
     */
    public int getD() {
        return d;
    }

    /**
     * Given a value x, returns an interpolated value, using data pointed to by {@link #xx} and
     * {@link #yy}.
     *
     * @param x value to obtain interpolation for.
     * @return interpolated value.
     * @throws InterpolationException if interpolation fails.
     */
    public double interpolate(final double x) throws InterpolationException {
        return rawinterp(1, x);
    }

    /**
     * Actual interpolation method.
     *
     * @param jlo index where value x to be interpolated in located in the array of xx.
     * @param x   value to obtain interpolation for.
     * @return interpolated value.
     */
    @Override
    public double rawinterp(int jlo, double x) {
        double num = 0, den = 0;
        for (int i = 0; i < n; i++) {
            double h = x - xx[i];
            if (h == 0.0) {
                return yy[i];
            } else {
                double temp = w[i] / h;
                num += temp * yy[i];
                den += temp;
            }
        }
        return num / den;
    }
}
