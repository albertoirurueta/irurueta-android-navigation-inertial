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
 * Computes cubic spline interpolation.
 * Accuracy of this interpolator worsens as the polynomial degree to interpolate increases.
 * Typically, up to degree 3 results are reasonable.
 */
public class CubicSplineInterpolator extends BaseInterpolator {

    /**
     * Length of x's and y's to take into account.
     */
    private static final int M = 2;

    private static final double YP1 = 1.e99;

    private static final double YPN = 1.e99;

    private final double[] y2;

    /**
     * Constructor with x and y vectors, and values of the first derivative at the endpoints.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     * @param yp1 1st derivative at lowest endpoint.
     * @param ypn 1st derivative at highest endpoint
     */
    public CubicSplineInterpolator(
            final double[] x, final double[] y, final double yp1, final double ypn) {
        super(x, y, M);
        y2 = new double[x.length];
        setY2(x, y, yp1, ypn);
    }

    /**
     * Constructor with x and y vectors and default values for first derivative at the endpoints.
     *
     * @param x x values to interpolate to. Values in x must be monotonic (either increasing or
     *          decreasing)
     * @param y y values to interpolate to.
     */
    public CubicSplineInterpolator(final double[] x, final double[] y) {
        this(x, y, YP1, YPN);
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
        // Given a value x, and using pointers to data xx and yy, and the stored vector of second
        // derivatives y2, this routine this cubic spline interpolated value y
        final int khi = jl + 1;

        final double h = xx[khi] - xx[jl];
        if (h == 0.0) {
            // The xa's must be istinct
            throw new InterpolationException();
        }

        // Cubic spline polynomial is now evaluated
        final double a = (xx[khi] - x) / h;
        final double b = (x - xx[jl]) / h;
        return a * yy[jl] + b * yy[khi] + ((a * a * a - a) * y2[jl]
                + (b * b * b - b) * y2[khi]) * (h * h) / 6.0;
    }

    /**
     * This method stores an array y2[0..n-1] with second derivatives of the interpolating function
     * at the tabulated points pointed to by xv, using function values pointed to by yv. If yp1
     * and/or ypn are equal to 1e99 or larger, the routine is signaled to set the corresponding
     * boundary condition for a natural spline, with zero second derivative on that boundary;
     * otherwise, they are the values of the first derivatives at the endpoints.
     *
     * @param xv x values to interpolate to. Values in x must be monotonic (either increasing or
     *           decreasing)
     * @param yv y values to interpolate to.
     * @param yp1 1st derivative at lowest endpoint.
     * @param ypn 1st derivative at highest endpoint
     */
    private void setY2(final double[] xv, final double[] yv, final double yp1, final double ypn) {
        int i;
        int k;
        double p;
        final double qn;
        double sig;
        final double un;
        final int n = y2.length;
        final double[] u = new double[n - 1];

        if (yp1 > 0.99e99)
            // The lower boundary condition is set either to be "natural"
            y2[0] = u[0] = 0.0;
        else {
            // or else to have a specified first derivative
            y2[0] = -0.5;
            u[0] = (3.0 / (xv[1] - xv[0])) * ((yv[1] - yv[0]) / (xv[1] - xv[0]) - yp1);
        }

        for (i = 1; i < n - 1; i++) {
            // This is the decomposition loop of the tri-diagonal algorithm. y2 and u are used for
            // temporary storage of the decomposed factors
            sig = (xv[i] - xv[i - 1]) / (xv[i + 1] - xv[i - 1]);
            p = sig * y2[i - 1] + 2.0;
            y2[i] = (sig - 1.0) / p;
            u[i] = (yv[i + 1] - yv[i]) / (xv[i + 1] - xv[i]) - (yv[i] - yv[i - 1]) / (xv[i] - xv[i - 1]);
            u[i] = (6.0 * u[i] / (xv[i + 1] - xv[i - 1]) - sig * u[i - 1]) / p;
        }

        if (ypn > 0.99e99)
            // The upper boundary condition is set either to be "natural"
            qn = un = 0.0;
        else {
            // or else to have a specified first derivative
            qn = 0.5;
            un = (3.0 / (xv[n - 1] - xv[n - 2])) * (ypn - (yv[n - 1] - yv[n - 2]) / (xv[n - 1] - xv[n - 2]));
        }
        y2[n - 1] = (un - qn * u[n - 2]) / (qn * y2[n - 2] + 1.0);
        for (k = n - 2; k >= 0; k--) {
            // This is the back-substitution loop of the tri-diagonal algorithm
            y2[k] = y2[k] * y2[k + 1] + u[k];
        }
    }
}
