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

import com.irurueta.numerical.polynomials.Polynomial;

/**
 * Base class for interpolating polynomial estimators.
 */
public abstract class InterpolatingPolynomialEstimator {
    /**
     * Estimates polynomial from provided x and y points.
     *
     * @param x x points the estimated polynomial passes through.
     * @param y y points the estimated polynomial passes through.
     * @return estimated polynomial.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length.
     * @throws InterpolationException   if interpolation fails for numerical reasons.
     */
    public Polynomial estimate(final double[] x, final double[] y) throws InterpolationException {
        final Polynomial polynomial = new Polynomial(x.length);
        estimate(x, y, polynomial);
        return polynomial;
    }

    /**
     * Estimates polynomial coefficients from provided x and y points.
     *
     * @param x x points the estimated polynomial passes through.
     * @param y y points the estimated polynomial passes through.
     * @return coefficients of estimated polynomial.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length.
     * @throws InterpolationException   if interpolation fails for numerical reasons.
     */
    public double[] estimateCoefficients(final double[] x, final double[] y)
            throws InterpolationException {
        final double[] result = new double[x.length];
        estimate(x, y, result);
        return result;
    }

    /**
     * Estimates polynomial from provided x and y points.
     *
     * @param x      x points the estimated polynomial passes through.
     * @param y      y points the estimated polynomial passes through.
     * @param result instance where estimated polynomial will be stored.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length
     *                                  or polynomial doesn't have expected order.
     * @throws InterpolationException   if interpolation fails for numerical reasons.
     */
    public void estimate(final double[] x, final double[] y, final Polynomial result)
            throws InterpolationException {
        estimate(x, y, result.getPolyParams());
    }

    /**
     * Estimates polynomial coefficients from provided x and y points.
     *
     * @param x   x points the estimated polynomial passes through.
     * @param y   y points the estimated polynomial passes through.
     * @param cof instance where coefficients of estimated polynomial will be stored.
     * @throws IllegalArgumentException if any of the provided values doesn't have the same length.
     * @throws InterpolationException   if interpolation fails for numerical reasons.
     */
    public abstract void estimate(final double[] x, final double[] y, final double[] cof)
            throws InterpolationException;
}
