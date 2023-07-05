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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.LUDecomposer;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

/**
 * Interpolates sparsely defined points using D.G. Krige method.
 */
public class KrigingInterpolator {
    /**
     * Data to compute interpolations from.
     * Each row corresponds to a point.
     * The number of columns determines the number of dimensions of provided points.
     */
    private final Matrix x;

    /**
     * Variogram of provided data.
     */
    private final Variogram vgram;

    /**
     * Number of dimensions of each point.
     */
    private final int ndim;

    /**
     * Number of provided points.
     */
    private final int npt;

    /**
     * Most recently computed value.
     */
    private double lastval;

    /**
     * Most recently computed error.
     */
    private double lasterr;

    private final Matrix dstar;

    private final Matrix vstar;

    private final Matrix yvi;

    /**
     * LU decomposer.
     */
    private final LUDecomposer vi;

    /**
     * Values of i-th point.
     */
    private final double[] xi;

    /**
     * Constructor.
     *
     * @param xx      Data to compute interpolations from. Each row corresponds to a point. The
     *                number of columns determines the number of dimensions of provided points.
     * @param yy      Function values for each provided point.
     * @param vargram Variogram to use.
     * @param err     array containing estimated error of function values.
     * @throws InterpolationException if initialization fails for numerical reasons.
     */
    public KrigingInterpolator(
            final Matrix xx, final double[] yy, final Variogram vargram, final double[] err)
            throws InterpolationException {
        try {
            x = xx;
            vgram = vargram;
            npt = xx.getRows();
            ndim = xx.getColumns();
            final int nptPlus1 = npt + 1;
            dstar = new Matrix(nptPlus1, 1);
            vstar = new Matrix(nptPlus1, 1);
            final Matrix v = new Matrix(nptPlus1, nptPlus1);
            final Matrix y = new Matrix(nptPlus1, 1);
            yvi = new Matrix(nptPlus1, 1);

            xi = new double[ndim];
            final double[] xj = new double[ndim];
            final int end = ndim - 1;

            int i;
            int j;
            // Fill Y and V
            for (i = 0; i < npt; i++) {
                y.setElementAtIndex(i, yy[1]);
                for (j = i; j < npt; j++) {
                    x.getSubmatrixAsArray(i, 0, i, end, xi);
                    x.getSubmatrixAsArray(j, 0, j, end, xj);
                    final double evaluation = vgram.evaluate(rdist(xi, xj));
                    v.setElementAt(i, j, evaluation);
                    v.setElementAt(j, i, evaluation);
                }

                v.setElementAt(i, npt, 1.0);
                v.setElementAt(npt, i, 1.0);
            }
            v.setElementAt(npt, npt, 0.0);
            y.setElementAtIndex(npt, 0.0);

            if (err != null) {
                for (i = 0; i < npt; i++) {
                    v.setElementAt(i, i, v.getElementAt(i, i) - sqr(err[i]));
                }
            }

            vi = new LUDecomposer(v);
            vi.decompose();
            vi.solve(y, yvi);
        } catch (final AlgebraException e) {
            throw new InterpolationException(e);
        }
    }

    /**
     * Constructor.
     *
     * @param xx      Data to compute interpolations from. Each row corresponds to a point. The
     *                number of columns determines the number of dimensions of provided points.
     * @param yy      Function values for each provided point.
     * @param vargram Variogram to use.
     * @throws InterpolationException if initialization fails for numerical reasons.
     */
    public KrigingInterpolator(final Matrix xx, final double[] yy, final Variogram vargram)
            throws InterpolationException {
        this(xx, yy, vargram, null);
    }

    /**
     * Constructor.
     *
     * @param xx Data to compute interpolations from. Each row corresponds to a point. The
     *           number of columns determines the number of dimensions of provided points.
     * @param yy Function values for each provided point.
     * @throws InterpolationException if initialization fails for numerical reasons.
     */
    public KrigingInterpolator(final Matrix xx, final double[] yy) throws InterpolationException {
        this(xx, yy, new Variogram(xx, yy));
    }

    /**
     * Gets number of dimensions of each point.
     *
     * @return number of dimensions of each point.
     */
    public int getNdim() {
        return ndim;
    }

    /**
     * Gets number of provided points.
     *
     * @return number of provided points.
     */
    public int getNpt() {
        return npt;
    }

    /**
     * Gets most recently computed interpolated value.
     *
     * @return most recently computed interpolatd value.
     */
    public double getLastVal() {
        return lastval;
    }

    /**
     * Gets most recently computed error.
     *
     * @return most recently computed error.
     */
    public double getLastErr() {
        return lasterr;
    }

    /**
     * Returns interpolated value at provided point.
     *
     * @param xstar point where interpolation is computed.
     * @return computed interpolation.
     * @throws InterpolationException if interpolation fails.
     */
    public double interpolate(final double[] xstar) throws InterpolationException {
        try {
            int i;
            final int end = ndim - 1;
            for (i = 0; i < npt; i++) {
                x.getSubmatrixAsArray(i, 0, i, end, xi);
                vstar.setElementAtIndex(i, vgram.evaluate(rdist(xstar, xi)));
            }

            vstar.setElementAtIndex(npt, 1.0);
            lastval = 0.;
            for (i = 0; i <= npt; i++) {
                lastval += yvi.getElementAtIndex(i) * vstar.getElementAtIndex(i);
            }
            return lastval;

        } catch (WrongSizeException e) {
            throw new InterpolationException(e);
        }
    }

    /**
     * Returns interpolated value at provided point, and estimated error.
     *
     * @param xstar  point where interpolation is computed.
     * @param esterr array where estimated error will be stored on its first position.
     * @return computed interpolation.
     * @throws InterpolationException if interpolation fails.
     */
    public double interpolate(final double[] xstar, final double[] esterr)
            throws InterpolationException {
        try {
            lastval = interpolate(xstar);
            vi.solve(vstar, dstar);
            lasterr = 0;
            for (int i = 0; i <= npt; i++) {
                lasterr += dstar.getElementAtIndex(i) * vstar.getElementAtIndex(i);
            }

            lasterr = Math.sqrt(Math.max(0.0, lasterr));
            esterr[0] = lasterr;
            return lastval;
        } catch (final AlgebraException e) {
            throw new InterpolationException(e);
        }
    }

    /**
     * Computes euclidean distance between two points.
     *
     * @param x1 1st point.
     * @param x2 2nd point.
     * @return euclidean distance.
     */
    private double rdist(final double[] x1, final double[] x2) {
        double d = 0.;
        for (int i = 0; i < ndim; i++) {
            d += sqr(x1[i] - x2[i]);
        }
        return Math.sqrt(d);
    }

    /**
     * Computes squared value.
     *
     * @param value a value.
     * @return computed square value.
     */
    private static double sqr(final double value) {
        return value * value;
    }

    /**
     * Variogram function.
     * Follows expression: v(r) = alpha * r^beta
     * where beta is considered fixed and alpha is fitted by unweighted least squares over all pairs
     * of data points i an j.
     * The value of beta should be in the range 1 <= beta < 2.
     * A good general choice is 1.5, but for functions with a strong linear trend, you may want to
     * experiment with values as large as 1.99 (The value 2 gives a degenerate matrix an meaningless
     * results).
     */
    public static class Variogram {

        /**
         * Default Beta value to be used.
         */
        public static final double DEFAULT_BETA = 1.5;

        /**
         * Default offset to use.
         */
        public static final double DEFAULT_NUG = 0.0;

        /**
         * Estimated alpha of variogram.
         */
        private final double alph;

        /**
         * Beta of variogram.
         */
        private final double bet;

        /**
         * Squared offset.
         */
        private final double nugsq;

        /**
         * Constructor.
         *
         * @param x    Data to compute interpolations from. Each row corresponds to a point. The
         *             number of columns determines the number of dimensions of provided points.
         * @param y    Function values for each provided point.
         * @param beta Beta to be used for variogram. The value of beta should be in the range
         *             1 <= beta < 2.
         * @param nug  Offset of variogram.
         */
        public Variogram(final Matrix x, final double[] y, final double beta, final double nug) {
            bet = beta;
            nugsq = nug * nug;

            int i;
            int j;
            int k;
            final int npt = x.getRows();
            final int ndim = x.getColumns();
            double rb;
            double num = 0.0;
            double denom = 0.0;
            for (i = 0; i < npt; i++) {
                for (j = i + 1; j < npt; j++) {
                    rb = 0.;
                    for (k = 0; k < ndim; k++) {
                        rb += sqr(x.getElementAt(i, k) - x.getElementAt(j, k));
                    }
                    rb = Math.pow(rb, 0.5 * beta);
                    num += rb * (0.5 * sqr(y[i] - y[j]) - nugsq);
                    denom += sqr(rb);
                }
            }
            alph = num / denom;
        }

        /**
         * Constructor using default zero offset.
         *
         * @param x    Data to compute interpolations from. Each row corresponds to a point. The
         *             number of columns determines the number of dimensions of provided points.
         * @param y    Function values for each provided point.
         * @param beta Beta to be used for variogram. The value of beta should be in the range
         *             1 <= beta < 2.
         */
        public Variogram(final Matrix x, final double[] y, final double beta) {
            this(x, y, beta, DEFAULT_NUG);
        }

        /**
         * Constructor using default beta = 1.5 and zero offset.
         *
         * @param x Data to compute interpolations from. Each row corresponds to a point. The
         *          number of columns determines the number of dimensions of provided points.
         * @param y Function values for each provided point.
         */
        public Variogram(final Matrix x, final double[] y) {
            this(x, y, DEFAULT_BETA);
        }

        /**
         * Evaluates variogram at provided distance.
         *
         * @param r a distance.
         * @return variogram evaluation.
         */
        public double evaluate(final double r) {
            return nugsq + alph * Math.pow(r, bet);
        }
    }
}
