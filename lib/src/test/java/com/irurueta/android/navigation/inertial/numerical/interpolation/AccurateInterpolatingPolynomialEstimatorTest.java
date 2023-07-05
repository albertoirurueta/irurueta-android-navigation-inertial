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

import static org.junit.Assert.assertArrayEquals;

import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class AccurateInterpolatingPolynomialEstimatorTest {

    private static final double MIN_VALUE = -1.0;

    private static final double MAX_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void estimate_whenFirstDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(1);
    }

    @Test
    public void estimate_whenSecondDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(2);
    }

    @Test
    public void estimate_whenThirdDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(3);
    }

    @Test
    public void estimate_whenFourthDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(4);
    }

    @Test
    public void estimate_whenFifthDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(5);
    }

    @Test
    public void estimate_whenSixthDegree_returnsExpectedResult() throws InterpolationException {
        assertEstimation(6);
    }

    public void assertEstimation(final int degree) throws InterpolationException {
        final Polynomial polynomial = buildPolynomial(degree);

        final int samples = degree + 1;
        final double[] x = new double[samples];
        final double[] y = new double[samples];

        final UniformRandomizer randomizer = new UniformRandomizer();
        for (int i = 0; i < samples; i++) {
            x[i] = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            y[i] = polynomial.evaluate(x[i]);
        }

        final AccurateInterpolatingPolynomialEstimator estimator =
                new AccurateInterpolatingPolynomialEstimator();

        final double[] result = new double[samples];
        estimator.estimate(x, y, result);

        assertArrayEquals(polynomial.getPolyParams(), result, ABSOLUTE_ERROR);
    }

    private Polynomial buildPolynomial(final int degree) {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final Polynomial result = new Polynomial(1.0);
        for (int i = 0; i < degree; i++) {
            final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final Polynomial poly = new Polynomial(-root, 1.0);
            result.multiply(poly);
        }

        return result;
    }
}