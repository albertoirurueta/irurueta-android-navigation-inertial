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
 * Abstract base class used by all interpolation implementations.
 */
public abstract class BaseInterpolator {
    /**
     * Length of x an y values to be interpolated.
     */
    protected final int n;

    /**
     * Length of data to be taken into account on x's and y's.
     */
    protected final int mm;

    protected int cor = 0;

    private int jsav = 0;

    private final int dj;

    /**
     * X values to be used for interpolation estimation. Must be monotonic (either increasing or
     * decreasing).
     */
    protected final double[] xx;

    /**
     * Y values to be used for interpolation estimation.
     */
    protected final double[] yy;

    /**
     * Constructor.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     * @param m length of x's and y's to take into account. Must be less or equal than x or y
     *          length.
     * @param check true to make validations, false otherwise.
     * @throws IllegalArgumentException if x or y have invalid length or m exceeds length of x or y.
     */
    BaseInterpolator(final double[] x, final double[] y, final int m, final boolean check) {
        n = x.length;
        mm = m;

        if (check) {
            if (n != y.length) {
                throw new IllegalArgumentException("mismatched x and y length");
            }
            if (n < 2 || mm < 2) {
                throw new IllegalArgumentException("x length is too small");
            }
            if (mm > n) {
                throw new IllegalArgumentException("m exceeds length of x or y");
            }
        }

        xx = x;
        yy = y;

        dj = Math.min(1, (int) Math.pow(n, 0.25));
    }

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
    BaseInterpolator(final double[] x, final double[] y, final int m) {
        this(x, y, m, true);
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
        final int jlo = cor != 0 ? hunt(x) : locate(x);
        return rawinterp(jlo, x);
    }

    /**
     * Actual interpolation method to be implemented by subclasses.
     *
     * @param jlo index where value x to be interpolated in located in the array of xx.
     * @param x value to obtain interpolation for.
     * @return interpolated value.
     * @throws InterpolationException if interpolation fails.
     */
    public abstract double rawinterp(final int jlo, final double x)
            throws InterpolationException;

    /**
     * Given a value x, returns a value j such that x is (insofar as possible) centered in the
     * subrange xx[j..j+mm-1], where xx is the stored array. The value in xx must be monotonic,
     * either increasing or decreasing. The returned value is not less than 0, nor greater than
     * n - 1.
     * @param x value to obtain interpolation for.
     * @return position where value to obtain interpolation lies in the array of {@link #xx}.
     */
    protected int locate(final double x) {
        int ju;
        int jm;
        int jl;
        // True if ascending order of table, false otherwise.
        final boolean ascnd = (xx[n - 1] >= xx[0]);

        // Initialize lower and upper limits.
        jl = 0;
        ju = n - 1;

        while (ju - jl > 1) {
            // If we are not yet done, compute a midpoint
            jm = (ju + jl) >> 1;
            if (x >= xx[jm] == ascnd) {
                // and replace either the lower limit
                jl = jm;
            } else {
                // or the upper limit, as appropriate
                ju = jm;
            }

            // Repeat until the test condition is satisfied
        }

        // Decide whether to use hunt or locate next time
        cor = Math.abs(jl - jsav) > dj ? 0 : 1;
        jsav = jl;
        return Math.max(0, Math.min(n - mm, jl - ((mm - 2) >> 1)));
    }

    /**
     * Given a value x, returns a value j such that x is (insofar as possible) centered in the
     * subrange xx[j..j+mm-1], where xx is the stored array. The value in xx must be monotonic,
     * either increasing or decreasing. The returned value is not less than 0, nor greater than
     * n - 1.
     * This method starts with a guessed position in the table. It first "hunts" either up or own,
     * in increments of 1, then 2, then 3, etc. until the desired value is bracketed. It then
     * bisects in the bracketed interval. At wors, this routine is about a factor of 2 slower than
     * {@link #locate(double)} (if the hunt phase expands to include the whole table). At best, it
     * can be a factor of log(n)/log(2) faster than {@link #locate(double)} if the desired point is
     * usually quite close to the input guess.
     *
     * @param x value to obtain interpolation for.
     * @return position where value to obtain interpolation lies in the array of {@link #xx}.
     */
    protected int hunt(final double x) {
        int jl = jsav;
        int jm;
        int ju;
        int inc = 1;

        // Ture if ascending order of table, false otherwise
        final boolean ascnd = (xx[n - 1] >= xx[0]);
        if (jl < 0 || jl > n - 1) {
            // Input guess not useful. Go immediately to bisection
            jl = 0;
            ju = n - 1;
        } else {
            if (x >= xx[jl] == ascnd) {
                // Hunt up
                for (; ; ) {
                    ju = jl + inc;
                    if (ju >= n - 1) {
                        // Off end of table
                        ju = n - 1;
                        break;
                    } else if (x < xx[ju] == ascnd) {
                        // Found bracket
                        break;
                    } else {
                        // Not done, so double the increment and try again
                        jl = ju;
                        inc += inc;
                    }
                }
            } else {
                // Hunt down
                ju = jl;
                for (; ; ) {
                    jl = jl - inc;
                    if (jl <= 0) {
                        // Off end of table
                        jl = 0;
                        break;
                    } else if (x >= xx[jl] == ascnd) {
                        // Found bracket
                        break;
                    } else {
                        // Not done, so double the increment and try again
                        ju = jl;
                        inc += inc;
                    }
                }
            }
        }

        // Hunt is done, so begin the final bisection phase
        while (ju - jl > 1) {
            jm = (ju + jl) >> 1;
            if (x >= xx[jm] == ascnd) {
                jl = jm;
            } else {
                ju = jm;
            }
        }

        // Decide whether to use hunt or locate next time
        cor = Math.abs(jl - jsav) > dj ? 0 : 1;
        jsav = jl;
        return Math.max(0, Math.min(n - mm, jl - ((mm - 2) >> 1)));
    }
}
