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
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class CurveInterpolatorTest {

    private static final double MIN_VALUE = -1.0;

    private static final double MAX_VALUE = 1.0;

    private static final int INTERPOLATIONS = 1000;

    private static final double ABSOLUTE_ERROR = 3.0;

    @Test
    public void interpolate_whenFirstDegreePolynomial1_returnsExpectedResult()
            throws InterpolationException, WrongSizeException {
        assertInterpolation(1);
    }

    @Test
    public void interpolate_whenFirstDegreePolynomial2_returnsExpectedResult()
            throws InterpolationException, WrongSizeException {
        assertInterpolation(2);
    }


    private void assertInterpolation(final int dims)
            throws WrongSizeException, InterpolationException {
        final Polynomial[] polynomials = new Polynomial[dims];
        for (int i = 0; i < dims; i++) {
            polynomials[i] = buildPolynomial();
        }

        // create multiple points
        final int nPoints = 1 + 1;
        final Matrix points = new Matrix(nPoints, dims);
        for (int i = 0; i < nPoints; i++) {
            final double x = (double) i / (double) nPoints;
            for (int j = 0; j < dims; j++) {
                points.setElementAt(i, j, polynomials[j].evaluate(x));
            }
        }

        final CurveInterpolator interpolator = new CurveInterpolator(points);

        // check random values
        final UniformRandomizer randomizer = new UniformRandomizer();
        for (int i = 0; i < INTERPOLATIONS; i++) {
            final double x = randomizer.nextDouble(0.0, 1.0);
            final double[] result = interpolator.interpolate(x);
            final double[] expected = new double[dims];
            for (int j = 0; j < dims; j++) {
                expected[j] = polynomials[j].evaluate(x);
            }

            assertArrayEquals(expected, result, ABSOLUTE_ERROR);
        }
    }

    private Polynomial buildPolynomial() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new Polynomial(-root, 1.0);
    }
}