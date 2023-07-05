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
package com.irurueta.android.navigation.inertial.numerical;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;

import com.irurueta.numerical.NumericalException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class PadeApproximantEstimatorTest {

    private static final double[] TAYLOR_COEFFICIENTS = {
            2.0, 1.0 / 9.0, 1.0 / 81.0, -49.0 / 8748.0, 175.0 / 78732
    };

    private static final double MIN_VALUE = 1.0;
    private static final double MAX_VALUE = 10.0;

    private static final double MIN_EXP_VALUE = -2.0;

    private static final double MAX_EXP_VALUE = 2.0;

    private static final int EXPONENTIAL_TAYLOR_ORDER = 14;

    @Test(expected = IllegalArgumentException.class)
    public void constructor_whenInvalidNumberOfTimes_throwsIllegalArgumentException() {
        new PadeApproximantEstimator(-1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void estimatePadeCoefficients_whenInvalidTaylorSeries_throwsIllegalArgumentException()
            throws NumericalException {
        final PadeApproximantEstimator estimator = new PadeApproximantEstimator();
        estimator.estimatePadeCoefficients(new double[1]);
    }

    @Test(expected = IllegalArgumentException.class)
    public void estimatePadeCoefficients_whenInvalidNumeratorCoefficients_throwsException()
            throws NumericalException {
        final PadeApproximantEstimator estimator = new PadeApproximantEstimator();
        estimator.estimatePadeCoefficients(TAYLOR_COEFFICIENTS, new double[1], new double[3]);
    }

    @Test(expected = IllegalArgumentException.class)
    public void estimatePadeCoefficients_whenInvalidDenominatorCoefficients_throwsException()
            throws NumericalException {
        final PadeApproximantEstimator estimator = new PadeApproximantEstimator();
        estimator.estimatePadeCoefficients(TAYLOR_COEFFICIENTS, new double[3], new double[1]);
    }

    @Test
    public void estimatePadeCoefficients_whenTestFunction_returnsExpectedValue()
            throws NumericalException {
        final PadeApproximantEstimator estimator = new PadeApproximantEstimator();
        final PadeApproximantEstimator.Result result = estimator.estimatePadeCoefficients(
                TAYLOR_COEFFICIENTS);
        final double[] numerators = new double[3];
        final double[] denominators = new double[3];
        estimator.estimatePadeCoefficients(TAYLOR_COEFFICIENTS, numerators, denominators);

        assertArrayEquals(numerators, result.getNumerators(), 0.0);
        assertArrayEquals(denominators, result.getDenominators(), 0.0);

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double exactValue = testFunction(x);
        final double taylorValue = testTaylorApproximation(x, TAYLOR_COEFFICIENTS);
        final double padeValue = padeApproximation(x, numerators, denominators);

        final double taylorError = Math.abs(taylorValue - exactValue);
        final double padeError = Math.abs(padeValue - exactValue);

        assertTrue(padeError < taylorError);
    }

    @Test
    public void estimatePadeCoefficients_whenExponentialFunction_returnsExpectedValue()
            throws NumericalException {
        final double[] taylorCoefficients = exponentialTaylorCoefficients();

        final PadeApproximantEstimator estimator = new PadeApproximantEstimator();
        final PadeApproximantEstimator.Result result = estimator.estimatePadeCoefficients(
                taylorCoefficients);

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double x = randomizer.nextDouble(MIN_EXP_VALUE, MAX_EXP_VALUE);
        final double exactValue = Math.exp(x);
        final double taylorValue = testTaylorApproximation(x, taylorCoefficients);
        final double padeValue = padeApproximation(x,
                result.getNumerators(), result.getDenominators());

        final double taylorError = Math.abs(taylorValue - exactValue);
        final double padeError = Math.abs(padeValue - exactValue);

        assertTrue(taylorError >= 0.0);
        assertTrue(padeError >= 0.0);
    }

    private double[] exponentialTaylorCoefficients() {
        final DoubleFactorialEstimator factorialEstimator = new DoubleFactorialEstimator();
        final double[] result = new double[EXPONENTIAL_TAYLOR_ORDER];
        for (int i = 0; i < EXPONENTIAL_TAYLOR_ORDER; i++) {
            result[i] = 1.0 / factorialEstimator.factorial(i);
        }

        return result;
    }

    private double testFunction(final double x) {
        return Math.pow(7.0 + Math.pow(1 + x, 4.0 / 3.0), 1.0 / 3.0);
    }

    private double testTaylorApproximation(final double x, final double[] taylorCoefficients) {
        double result = 0.0;
        double prevX = 1.0;
        for (final double taylorCoefficient : taylorCoefficients) {
            result += taylorCoefficient * prevX;
            prevX *= x;
        }

        return result;
    }

    private double padeApproximation(
            final double x, double[] numeratorCoefficients, double[] denominatorCoefficients) {
        double numerator = 0.0;
        double prevXNumerator = 1.0;
        for (final double numeratorCoefficient : numeratorCoefficients) {
            numerator += numeratorCoefficient * prevXNumerator;
            prevXNumerator *= x;
        }

        double denominator = 0.0;
        double prevXDenominator = 1.0;
        for (final double denominatorCoefficient : denominatorCoefficients) {
            denominator += denominatorCoefficient * prevXDenominator;
            prevXDenominator *= x;
        }

        return numerator / denominator;
    }
}