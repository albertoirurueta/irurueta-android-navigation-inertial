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

import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.sorting.Sorter;
import com.irurueta.sorting.SortingException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.Arrays;

public class LinearInterpolatorTest {

    private static final double MIN_VALUE = -100.0;

    private static final double MAX_VALUE = 100.0;

    private static final int SAMPLES = 1000;

    private static final int DEGREE = 1;

    private static final int MIN_SAMPLES = 2;

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void interpolate_whenFirstDegreePolynomial_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(SAMPLES);
    }

    @Test
    public void interpolate_whenFirstDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(MIN_SAMPLES);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenMismatchedLength_throwsIllegalArgumentException() {
        final double[] x = new double[SAMPLES];
        final double[] y = new double[SAMPLES + 1];
        new LinearInterpolator(x, y);

    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenNotEnoughSamples_throwsIllegalArgumentException() {
        final double[] x = new double[1];
        final double[] y = new double[1];
        new LinearInterpolator(x, y);
    }

    private void assertInterpolation(final int samples)
            throws InterpolationException, SortingException {
        final double[] roots = new double[DEGREE];
        final Polynomial polynomial = buildPolynomial(roots);

        for (int i = 0; i < DEGREE; i++) {
            assertEquals(0.0, polynomial.evaluate(roots[i]), 0.0);
        }

        // create multiple samples and evaluations
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double[] unorderedX = new double[samples];
        final double[] unorderedY = new double[samples];
        for (int i = 0; i < samples; i++) {
            unorderedX[i] = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            unorderedY[i] = polynomial.evaluate(unorderedX[i]);
        }

        // order values by ascending x
        final double[] x = Arrays.copyOf(unorderedX, samples);
        final double[] y = new double[samples];
        final int[] indices = Sorter.create().sortWithIndices(x);
        for (int i = 0; i < samples; i++) {
            y[i] = unorderedY[indices[i]];
        }

        // check data
        for (int i = 0; i < samples; i++) {
            assertEquals(y[i], polynomial.evaluate(x[i]), 0.0);
        }

        final LinearInterpolator interpolator = new LinearInterpolator(x, y);

        // check that interpolator passes through provided points
        for (int i = 0; i < samples; i++) {
            assertEquals(y[i], interpolator.interpolate(x[i]), ABSOLUTE_ERROR);
        }

        // check random values
        for (int i = 0; i < SAMPLES; i++) {
            final double xi = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            assertEquals(polynomial.evaluate(xi), interpolator.interpolate(xi), ABSOLUTE_ERROR);
        }
    }

    private Polynomial buildPolynomial(final double[] roots) {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final Polynomial result = new Polynomial(1.0);
        for (int i = 0; i < DEGREE; i++) {
            final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final Polynomial poly = new Polynomial(-root, 1.0);
            result.multiply(poly);

            roots[i] = root;
        }

        return result;
    }
}