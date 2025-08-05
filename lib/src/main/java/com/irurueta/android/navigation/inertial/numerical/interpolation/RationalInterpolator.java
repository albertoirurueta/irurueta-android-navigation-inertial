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
 * Computes rational interpolation.
 */
public class RationalInterpolator extends BaseInterpolator {

    /**
     * A small number.
     */
    private static final double TINY = 1.0e-99;

    /**
     * An indication of interpolation error on the y values of the last call to
     * {@link #interpolate(double)}.
     */
    private double dy;

    /**
     * Constructor.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     * @param m length of x's and y's to take into account. Must be less or equal than x or y
     *          length.
     * @throws IllegalArgumentException if x or y have invalid length or m exceeds length of x or y.
     */
    public RationalInterpolator(final double[] x, final double[] y, final int m) {
        super(x, y, m);
        dy = 0.0;
    }

    /**
     * Constructor.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     * @throws IllegalArgumentException if x or y have invalid length or m exceeds length of x or y.
     */
    public RationalInterpolator(final double[] x, final double[] y) {
        this(x, y, x.length);
    }

    /**
     * Gets an indication of the error of interpolation on the y values.
     *
     * @return indication of error of interpolation.
     */
    public double getDy() {
        return dy;
    }

    /**
     * Actual interpolation method.
     *
     * @param jl index where value x to be interpolated in located in the array of xx.
     * @param x  value to obtain interpolation for.
     * @return interpolated value.
     * @throws InterpolationException if interpolation fails.
     */
    @Override
    public double rawinterp(int jl, double x) throws InterpolationException {
        int m;
        int i;
        int ns = 0;
        double y;
        double w;
        double t;
        double hh;
        double h;
        double dd;
        final double[] xa = xx;
        final double[] ya = yy;
        final double[] c = new double[mm];
        final double[] d = new double[mm];
        hh = Math.abs(x - xa[jl]);
        for (i = 0; i < mm; i++) {
            h = Math.abs(x - xa[jl + i]);
            if (h == 0.0) {
                dy = 0.0;
                return ya[jl + i];
            } else if (h < hh) {
                ns = i;
                hh = h;
            }
            c[i] = ya[jl + i];
            // The TINY part is needed to prevent a rare zero-over-zero condition
            d[i] = ya[jl + i] + TINY;
        }
        y = ya[jl + ns--];
        for (m = 1; m < mm; m++) {
            for (i = 0; i < mm - m; i++) {
                w = c[i + 1] - d[i];
                // h will never be zero, since this was tested in the initializing loop
                h = xa[jl + i + m] - x;
                t = (xa[jl + i] - x) * d[i] / h;
                dd = t - c[i + 1];
                if (dd == 0.0) {
                    // This error condition indicates that the interpolating function has a pole at
                    // the requested value of x
                    throw new InterpolationException();
                }
                dd = w / dd;
                d[i] = c[i + 1] * dd;
                c[i] = t * dd;
            }

            dy = 2 * (ns + 1) < (mm - m) ? c[ns + 1] : d[ns--];
            y += dy;
        }
        return y;
    }
}
