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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class KrigingInterpolatorTest {

    private static final double MIN_VALUE = -1.0e-3;

    private static final double MAX_VALUE = 1.0e-3;

    private static final int SAMPLES = 100;

    private static final double ABSOLUTE_ERROR_1 = 1e-2;

    private static final double ABSOLUTE_ERROR_2 = 1e-1;

    @Test
    public void interpolate_dim1_returnsExpectedResult()
            throws WrongSizeException, InterpolationException {
        assertInterpolation(1, ABSOLUTE_ERROR_1);
    }

    @Test
    public void interpolate_dim2_returnsExpectedResult()
            throws WrongSizeException, InterpolationException {
        assertInterpolation(2, ABSOLUTE_ERROR_2);
    }

    private void assertInterpolation(final int dim, final double error)
            throws WrongSizeException, InterpolationException {
        final double[] roots = new double[dim];
        final Polynomial[] polynomials = buildPolynomials(dim, roots);

        for (int i = 0; i < dim; i++) {
            assertEquals(0.0, polynomials[i].evaluate(roots[i]), 0.0);
        }
        assertEquals(0.0, evaluate(polynomials, roots), 0.0);

        // create multiple samples and evaluations
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double[] point = new double[dim];
        final Matrix pts = new Matrix(SAMPLES, dim);
        final double[] values = new double[SAMPLES];
        for (int i = 0; i < SAMPLES; i++) {
            randomizer.fill(point, MIN_VALUE, MAX_VALUE);
            pts.setSubmatrix(i, 0, i, dim - 1, point);
            values[i] = evaluate(polynomials, point);
        }

        // check data
        for (int i = 0; i < SAMPLES; i++) {
            pts.getSubmatrixAsArray(i, 0, i, dim - 1, point);
            assertEquals(values[i], evaluate(polynomials, point), 0.0);
        }

        final KrigingInterpolator interpolator = new KrigingInterpolator(pts, values);

        assertEquals(dim, interpolator.getNdim());
        assertEquals(SAMPLES, interpolator.getNpt());

        // check that interpolator at provided points
        for (int i = 0; i < SAMPLES; i++) {
            pts.getSubmatrixAsArray(i, 0, i, dim - 1, point);
            final double value = interpolator.interpolate(point);
            assertEquals(value, interpolator.getLastVal(), 0.0);
            assertTrue(interpolator.getLastErr() >= 0.0);
            assertEquals(values[i], value, error);
        }

        // check random values
        final double[] lastError = new double[1];
        for (int i = 0; i < SAMPLES; i++) {
            randomizer.fill(point, MIN_VALUE, MAX_VALUE);
            final double value = interpolator.interpolate(point);
            assertEquals(evaluate(polynomials, point), value, error);
            assertEquals(value, interpolator.interpolate(point, lastError), 0.0);
            final double lastErr = interpolator.getLastErr();
            assertTrue(lastErr >= 0.0);
            assertEquals(lastErr, lastError[0], 0.0);
        }
    }

    private double evaluate(final Polynomial[] polynomials, double[] point) {
        final int dim = polynomials.length;
        double result = 1.0;
        for (int i = 0; i < dim; i++) {
            result *= polynomials[i].evaluate(point[i]);
        }
        return result;
    }

    private Polynomial[] buildPolynomials(int dim, final double[] roots) {
        final double[] r = new double[1];
        final Polynomial[] result = new Polynomial[dim];
        for (int i = 0; i < dim; i++) {
            result[i] = buildPolynomial(r);
            roots[i] = r[0];
        }
        return result;
    }

    private Polynomial buildPolynomial(final double[] roots) {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Polynomial result = new Polynomial(-root, 1.0);
        roots[0] = root;

        return result;
    }
}