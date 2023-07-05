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
 * Computes polynomial interpolation.
 * For large sets of ata, this interpolator might return inaccurate results.
 * Additionally, accuracy worsens as the polynomial degree to interpolate increases.
 */
public class PolynomialInterpolator extends BaseInterpolator {

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
    public PolynomialInterpolator(final double[] x, final double[] y, final int m) {
        this(x, y, m, true);
    }

    /**
     * Constructor.
     *
     * @param x     x values to interpolate to. Values in x must be monotonic (either increasing or
     *              decreasing)
     * @param y     y values to interpolate to.
     * @param m     length of x's and y's to take into account. Must be less or equal than x or y
     *              length.
     * @param check true to make validations, false otherwise.
     * @throws IllegalArgumentException if x or y have invalid length or m exceeds length of x or y.
     */
    public PolynomialInterpolator(final double[] x, final double[] y, final int m,
                                     final boolean check) {
        super(x, y, m, check);
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
    public PolynomialInterpolator(final double[] x, final double[] y) {
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
    public double rawinterp(final int jl, final double x) throws InterpolationException {
        int i;
        int m;
        int ns = 0;
        double y;
        double den;
        double dif;
        double dift;
        double ho;
        double hp;
        double w;
        final double[] xa = xx;
        final double[] ya = yy;
        final double[] c = new double[mm];
        final double[] d = new double[mm];
        dif = Math.abs(x - xa[jl]);
        for (i = 0; i < mm; i++) {
            // Here we find the index ns of the closest table entry
            if ((dift = Math.abs(x - xa[jl + i])) < dif) {
                ns = i;
                dif = dift;
            }
            // and initialize the tableau of c's and d's
            c[i] = ya[jl + i];
            d[i] = ya[jl + i];
        }
        // This is the initial approximation to y
        y = ya[jl + ns--];
        for (m = 1; m < mm; m++) {
            // For each column of the tableau
            for (i = 0; i < mm - m; i++) {
                // we loop over the current c's and d's and update them
                ho = xa[jl + i] - x;
                hp = xa[jl + i + m] - x;
                w = c[i + 1] - d[i];
                den = ho - hp;
                if (den == 0.0) {
                    // This error can occur only if two input xa's are (to within roundoff)
                    // identical
                    throw new InterpolationException();
                }
                den = w / den;
                // Here the c’s and d’s are updated.
                d[i] = hp * den;
                c[i] = ho * den;
            }
            dy = 2 * (ns + 1) < (mm - m) ? c[ns + 1] : d[ns--];
            y += dy;
            // After each column in the tableau is completed, we decide which correction, c or d,
            // we want to add to our accumulating value of y, i.e., which path to take through the
            // tableau — forking up or down. We do this in such a way as to take the most “straight
            // line” route through the tableau to its apex, updating ns accordingly to keep track
            // of where we are. This route keeps the partial approximations centered (insofar as
            // possible) on the target x. The last dy added is thus the error indication.
        }
        return y;
    }
}
