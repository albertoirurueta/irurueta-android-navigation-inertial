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
 * Computes curve interpolation of multidimensional points using cubic splines.
 * This interpolator uses an ordered set of N tabulated points in dim dimensions that lie on
 * a one-dimensional curve, x0... xn-1, and interpolates values along the curve.
 */
public class CurveInterpolator {

    /**
     * Number of points dimensions.
     */
    private final int dim;

    /**
     * True indicates a closed curve, false indicates an open one.
     */
    private final boolean cls;

    /**
     * Result of interpolation.
     */
    private final double[] ans;

    /**
     * Array of one dimensional cubic spline interpolators.
     */
    private final CubicSplineInterpolator[] srp;

    /**
     * Constructor.
     *
     * @param ptsin matrix containing points on each row. Number of columns determines number of
     *              dimensions of each point.
     * @param close true indicates that points lie in a closed curve, false indicates they lie on
     *              an open one.
     * @throws InterpolationException if a numerical exception occurs.
     */
    public CurveInterpolator(final Matrix ptsin, final boolean close)
            throws InterpolationException {
        try {
            final int n = ptsin.getRows();
            dim = ptsin.getColumns();
            // The trick for closed curves is to duplicate half a period at the beginning and end,
            // and then use the middle half of the resulting spline
            final int in = close ? 2 * n : n;
            cls = close;
            final Matrix pts = new Matrix(dim, in);
            final double[] s = new double[in];
            ans = new double[dim];
            srp = new CubicSplineInterpolator[dim];

            final double[] p1 = new double[dim];
            final double[] p2 = new double[dim];
            final double[] p = new double[in];
            final int end = dim - 1;

            int i;
            int ii;
            int im;
            int j;
            int ofs;
            double ss;
            double soff;
            double db;
            double de;
            ofs = close ? n / 2 : 0;
            s[0] = 0.;
            for (i = 0; i < in; i++) {
                ii = (i - ofs + n) % n;
                im = (ii - 1 + n) % n;
                for (j = 0; j < dim; j++) {
                    // store transpose
                    pts.setElementAt(j, i, ptsin.getElementAt(ii, j));
                }

                if (i > 0) {
                    // Accumulate arc length
                    ptsin.getSubmatrixAsArray(ii, 0, ii, end, p1);
                    ptsin.getSubmatrixAsArray(im, 0, im, end, p2);
                    s[i] = s[i - 1] + rad(p1, p2);
                    if (s[i] == s[i - 1]) {
                        // Consecutive points may not be identical. For a close curve, the last
                        // data point should not duplicate the first.
                        throw new InterpolationException();
                    }
                }
            }

            // Rescale parameter so that the interval [0,1] is the whole curve (or one period)
            ss = close ? s[ofs + n] - s[ofs] : s[n - 1] - s[0];
            soff = s[ofs];
            for (i = 0; i < in; i++) {
                s[i] = (s[i] - soff) / ss;
            }

            // Construct the splines using endpoint derivatives
            final int endPts = in - 1;
            for (j = 0; j < dim; j++) {
                if (in < 4) {
                    db = 1.e99;
                    de = 1.e99;
                } else {
                    pts.getSubmatrixAsArray(j, 0, j, endPts, p);
                    db = fprime(s, p, 1, 0, 0);
                    de = fprime(s, p, -1, in - 1, in - 1);
                }
                pts.getSubmatrixAsArray(j, 0, j, endPts, p);
                srp[j] = new CubicSplineInterpolator(s, p, db, de);
            }
        } catch (final WrongSizeException e) {
            throw new InterpolationException(e);
        }
    }

    /**
     * Constructor assuming that curve is NOT closed.
     *
     * @param ptsin matrix containing points on each row. Number of columns determines number of
     *              dimensions of each point.
     * @throws InterpolationException if a numerical exception occurs.
     */
    public CurveInterpolator(final Matrix ptsin) throws InterpolationException {
        this(ptsin, false);
    }

    /**
     * Interpolates a point on the stored curve. The point is parameterized by t, in the range
     * [0,1].
     * For open curves, values of t outside this range will return extrapolations (dangerous!).
     * For closed curves t is periodic with period 1.
     *
     * @param t position in the curve to be interpolated.
     * @return result of interpolation.
     * @throws InterpolationException if interpolation fails for numerical reasons.
     */
    public double[] interpolate(double t) throws InterpolationException {
        if (cls) {
            t = t - Math.floor(t);
        }

        for (int j = 0; j < dim; j++) {
            ans[j] = srp[j].interpolate(t);
        }
        return ans;
    }

    /**
     * Utility for estimating the derivatives at the endpoints, x and y point to the abscissa and
     * ordinate of the endpoint. If pm is +1, points to the right will be used (left endpoint): if it
     * is -1, points to the left will be used (right endpoint).
     */
    private double fprime(final double[] x, final double[] y, final int pm, final int xoff,
                          final int yoff) {
        final double s1 = x[xoff] - x[xoff + pm];
        final double s2 = x[xoff] - x[xoff + pm * 2];
        final double s3 = x[xoff] - x[xoff + pm * 3];
        final double s12 = s1 - s2, s13 = s1 - s3, s23 = s2 - s3;
        return -(s1 * s2 / (s13 * s23 * s3)) * y[yoff + pm * 3]
                + (s1 * s3 / (s12 * s2 * s23)) * y[yoff + pm * 2]
                - (s2 * s3 / (s1 * s12 * s13)) * y[yoff + pm]
                + (1.0 / s1 + 1.0 / s2 + 1.0 / s3) * y[yoff];
    }

    /**
     * Computes the euclidean distance between two points.
     *
     * @param p1 first point.
     * @param p2 second point.
     * @return euclidean distance.
     */
    private double rad(final double[] p1, final double[] p2) {
        double sum = 0.0;
        for (int i = 0; i < dim; i++) {
            final double value = p1[i] - p2[i];
            sum += value * value;
        }
        return Math.sqrt(sum);
    }
}
