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
import java.util.logging.Level;
import java.util.logging.Logger;

public class PolynomialInterpolatorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final int SAMPLES = 10;

    private static final int INTERPOLATIONS = 1000;

    private static final double ABSOLUTE_ERROR_1 = 1e-5;

    private static final double ABSOLUTE_ERROR_2 = 1e-3;

    private static final double ABSOLUTE_ERROR_3 = 1.0;

    private static final double ABSOLUTE_ERROR_4 = 1.0;

    private static final double ABSOLUTE_ERROR_5 = 3.0;

    @Test
    public void interpolate_whenFirstDegreePolynomial_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(1, SAMPLES, ABSOLUTE_ERROR_1);
    }

    @Test
    public void interpolate_whenFirstDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(1, 2, ABSOLUTE_ERROR_1);
    }

    @Test
    public void interpolate_whenSecondDegreePolynomial_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(2, SAMPLES, ABSOLUTE_ERROR_2);
    }

    @Test
    public void interpolate_whenSecondDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(2, 3, ABSOLUTE_ERROR_2);
    }

    @Test
    public void interpolate_whenThirdDegreePolynomial_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(3, SAMPLES, ABSOLUTE_ERROR_3);
    }

    @Test
    public void interpolate_whenThirdDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(3, 4, ABSOLUTE_ERROR_3);
    }

    @Test
    public void interpolate_whenFourthDegreePolynomial_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(4, SAMPLES, ABSOLUTE_ERROR_4);
    }

    @Test
    public void interpolate_whenFourthDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(4, 5, ABSOLUTE_ERROR_4);
    }

    @Test
    public void interpolate_whenFifthDegreePolynomialMinimumSamples_returnsExpectedResult()
            throws InterpolationException, SortingException {
        assertInterpolation(5, 6, ABSOLUTE_ERROR_5);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenMismatchedLength_throwsIllegalArgumentException() {
        final double[] x = new double[SAMPLES];
        final double[] y = new double[SAMPLES + 1];
        new PolynomialInterpolator(x, y);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenNotEnoughSamples_throwsIllegalArgumentException() {
        final double[] x = new double[1];
        final double[] y = new double[1];
        new PolynomialInterpolator(x, y);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenNotEnoughSamplesToTakeIntoAccount_throwsIllegalArgumentException() {
        final double[] x = new double[SAMPLES];
        final double[] y = new double[SAMPLES];
        new PolynomialInterpolator(x, y, 1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void interpolate_whenTooManyPointsToTakeIntoAccount_throwsIllegalArgumentException() {
        final double[] x = new double[SAMPLES];
        final double[] y = new double[SAMPLES];
        new PolynomialInterpolator(x, y, SAMPLES + 1);
    }

    private void assertInterpolation(final int degree, final int samples, final double error)
            throws InterpolationException, SortingException {
        final double[] roots = new double[degree];
        final Polynomial polynomial = buildPolynomial(degree, roots);

        for (int i = 0; i < degree; i++) {
            assertEquals(0.0, polynomial.evaluate(roots[i]), error);
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

        final PolynomialInterpolator interpolator = new PolynomialInterpolator(x, y);

        // check that interpolator passes through provided points
        for (int i = 0; i < samples; i++) {
            assertEquals(y[i], interpolator.interpolate(x[i]), 0.0);
        }

        // check random values
        double accuracy = 0.0;
        for (int i = 0; i < INTERPOLATIONS; i++) {
            final double xi = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            assertEquals(polynomial.evaluate(xi), interpolator.interpolate(xi), error);
            final double dy = interpolator.getDy();
            accuracy += dy / INTERPOLATIONS;
        }

        Logger.getLogger(PolynomialInterpolatorTest.class.getName())
                .log(Level.INFO, "accuracy: " + accuracy + " degree: " + degree);
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