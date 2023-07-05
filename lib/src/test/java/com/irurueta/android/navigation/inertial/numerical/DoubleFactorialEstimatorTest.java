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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DoubleFactorialEstimatorTest {

    @Test
    public void defaultConstructor_setsExpectedCache() {
        final DoubleFactorialEstimator estimator = new DoubleFactorialEstimator();

        assertEquals(171, estimator.getCacheSize());
    }

    @Test(expected = IllegalArgumentException.class)
    public void constructor_whenSmallValue_throwsException() {
        new DoubleFactorialEstimator(0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void constructor_whenLargeValue_throwsException() {
        new DoubleFactorialEstimator(172);
    }

    @Test
    public void factorial_returnsExpectedValue() {
        final DoubleFactorialEstimator estimator = new DoubleFactorialEstimator();

        final int cacheSize = estimator.getCacheSize();
        double previous = 0.0;
        for (int i = 0; i < cacheSize; i++) {
            final double factorial = estimator.factorial(i);
            final double expected = fact(i);
            assertEquals(expected, factorial, 0.0);
            assertTrue(factorial >= previous);
            previous = factorial;
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void factorial_whenSmallValue_throwsException() {
        final DoubleFactorialEstimator estimator = new DoubleFactorialEstimator();

        estimator.factorial(-1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void factorial_whenLargeValue_throwsException() {
        final DoubleFactorialEstimator estimator = new DoubleFactorialEstimator();

        final int cacheSize = estimator.getCacheSize();
        estimator.factorial(cacheSize);
    }

    private static double fact(long value) {
        double result = 1.0;
        if (value > 0) {
            for (int i = 1; i <= value; i++) {
                result *= i;
            }
        }
        return result;
    }
}
