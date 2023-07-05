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

/**
 * Base class for interpolation methods based on Radial Basis Functions to interpolate sparse
 * points.
 */
public abstract class BaseRadialBasisFunctionInterpolator {
    /**
     * Dimension of points to be interpolated.
     */
    protected final int dim;

    /**
     * Number of points provided in a matrix as a basis for interpolation.
     */
    protected final int n;

    /**
     * Matrix containing points to interpolate from.
     * Each row contains one point.
     * Matrix will have n points (rows) having a dimension (columns) equal to dim.
     */
    protected final Matrix pts;

    /**
     * ith point to make comparisons.
     */
    protected final double[] pi;

    /**
     * Constructor.
     * @param ptss  Matrix containing points to interpolate from. Each row contains one point.
     *              Matrix will have n points (rows) having a dimension (columns) equal to dim.
     */
    protected BaseRadialBasisFunctionInterpolator(final Matrix ptss) {
        this.dim = ptss.getColumns();
        n = ptss.getRows();
        pts = ptss;

        pi = new double[dim];
    }

    /**
     * Returns the interpolated function value at a dim-dimensional point pt.
     *
     * @param pt dim-dimentional point where interpolation must be computed.
     * @return result of interpolation.
     * @throws IllegalArgumentException if provided point has an invalid length.
     */
    public abstract double interpolate(final double[] pt);

    /**
     * Computes the euclidean distance between two points.
     *
     * @param p1 first point.
     * @param p2 second point.
     * @return euclidean distance.
     */
    protected double rad(final double[] p1, final double[] p2) {
        double sum = 0.0;
        for (int i = 0; i < dim; i++) {
            final double value = p1[i] - p2[i];
            sum += value * value;
        }

        return Math.sqrt(sum);
    }
}
