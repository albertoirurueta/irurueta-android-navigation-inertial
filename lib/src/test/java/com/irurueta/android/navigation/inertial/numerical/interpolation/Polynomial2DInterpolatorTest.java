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

import static org.junit.Assert.*;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.sorting.Sorter;
import com.irurueta.sorting.SortingException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.Arrays;

public class Polynomial2DInterpolatorTest {

    private static final double MIN_VALUE = -1.0;

    private static final double MAX_VALUE = 1.0;

    private static final int SAMPLES = 10;

    private static final double ABSOLUTE_ERROR_1 = 1e-2;

    private static final double ABSOLUTE_ERROR_2 = 1e-1;

    @Test
    public void interpolate_whenFirstDegree2DPolynomial_returnsExpectedResult()
            throws SortingException, WrongSizeException, InterpolationException {
        assertInterpolation(1, SAMPLES, ABSOLUTE_ERROR_1);
    }

    @Test
    public void interpolate_whenFirstDegree2DPolynomialMinimumSamples_returnsExpectedResult()
            throws SortingException, WrongSizeException, InterpolationException {
        assertInterpolation(1, 2, ABSOLUTE_ERROR_1);
    }

    @Test
    public void interpolate_whenSecondDegree2DPolynomial_returnsExpectedResult()
            throws SortingException, WrongSizeException, InterpolationException {
        assertInterpolation(2, SAMPLES, ABSOLUTE_ERROR_2);
    }

    @Test
    public void interpolate_whenSecondDegree2DPolynomialMinimumSamples_returnsExpectedResult()
            throws SortingException, WrongSizeException, InterpolationException {
        assertInterpolation(2, 3, ABSOLUTE_ERROR_2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenNotEnoughSamples_throwsIllegalArgumentException()
            throws WrongSizeException {
        final double[] x1 = new double[1];
        final double[] x2 = new double[1];
        final Matrix y = new Matrix(SAMPLES, SAMPLES);
        new Polynomial2DInterpolator(x1, x2, y);
    }

    @Test
    public void getM_returnsExpectedValue() throws WrongSizeException {
        final double[] x1 = new double[SAMPLES];
        final double[] x2 = new double[SAMPLES + 1];
        final Matrix y = new Matrix(SAMPLES, SAMPLES + 1);
        final Polynomial2DInterpolator interpolator = new Polynomial2DInterpolator(x1, x2, y);

        assertEquals(SAMPLES, interpolator.getM());
    }

    @Test
    public void getN_returnsExpectedValue() throws WrongSizeException {
        final double[] x1 = new double[SAMPLES];
        final double[] x2 = new double[SAMPLES + 1];
        final Matrix y = new Matrix(SAMPLES, SAMPLES + 1);
        final Polynomial2DInterpolator interpolator = new Polynomial2DInterpolator(x1, x2, y);

        assertEquals(SAMPLES + 1, interpolator.getN());
    }

    private void assertInterpolation(final int degree, final int samples, final double error)
            throws SortingException, WrongSizeException, InterpolationException {
        final double[] roots1 = new double[degree];
        final Polynomial polynomial1 = buildPolynomial(degree, roots1);

        final double[] roots2 = new double[degree];
        final Polynomial polynomial2 = buildPolynomial(degree, roots2);

        for (int i = 0; i < degree; i++) {
            assertEquals(0.0, polynomial1.evaluate(roots1[i]), error);
            assertEquals(0.0, polynomial2.evaluate(roots2[i]), error);
        }

        // create multiple samples and evaluations
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double[] unorderedX1 = new double[samples];
        final double[] unorderedX2 = new double[samples];
        for (int i = 0; i < samples; i++) {
            unorderedX1[i] = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            unorderedX2[i] = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        }

        // order values by ascending x and create evaluations
        final double[] x1 = Arrays.copyOf(unorderedX1, samples);
        final double[] x2 = Arrays.copyOf(unorderedX2, samples);
        final Matrix y = new Matrix(samples, samples);
        final Sorter<Double> sorter = Sorter.create();
        sorter.sort(x1);
        sorter.sort(x2);
        for (int i = 0; i < samples; i++) {
            for (int j = 0; j < samples; j++) {
                y.setElementAt(i, j, polynomial1.evaluate(x1[i]) * polynomial2.evaluate(x2[j]));
            }
        }

        // check data
        for (int i = 0; i < samples; i++) {
            for (int j = 0; j < samples; j++) {
                assertEquals(y.getElementAt(i, j),
                        polynomial1.evaluate(x1[i]) * polynomial2.evaluate(x2[j]), 0.0);
            }
        }

        final Polynomial2DInterpolator interpolator = new Polynomial2DInterpolator(x1, x2, y);

        assertEquals(samples, interpolator.getM());
        assertEquals(samples, interpolator.getN());

        // check that interpolator passes through provided points
        for (int i = 0; i < samples; i++) {
            for (int j = 0; j < samples; j++) {
                assertEquals(y.getElementAt(i, j), interpolator.interpolate(x1[i], x2[j]), 0.0);
            }
        }

        // check random values
        for (int i = 0; i < SAMPLES; i++) {
            final double xi = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            for (int j = 0; j < SAMPLES; j++) {
                final double xj = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
                assertEquals(polynomial1.evaluate(xi) * polynomial2.evaluate(xj),
                        interpolator.interpolate(xi, xj), error);
            }
        }
    }

    private Polynomial buildPolynomial(final int degree, final double[] roots) {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final Polynomial result = new Polynomial(1.0);
        for (int i = 0; i < degree; i++) {
            final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final Polynomial poly = new Polynomial(-root, 1.0);
            result.multiply(poly);

            roots[i] = root;
        }

        return result;
    }
}