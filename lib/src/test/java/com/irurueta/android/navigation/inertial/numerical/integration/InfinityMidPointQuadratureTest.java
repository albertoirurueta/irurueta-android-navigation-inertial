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
package com.irurueta.android.navigation.inertial.numerical.integration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class InfinityMidPointQuadratureTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    @Test
    public void next_returnsNotZeroValue() throws EvaluationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final Polynomial polynomial = buildPolynomial();

        final InfinityMidPointQuadrature quadrature = new InfinityMidPointQuadrature(a, b,
                new SingleDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(final double point) {
                        return polynomial.evaluate(point);
                    }
                });

        assertNotEquals(0.0, quadrature.next());
    }

    @Test
    public void getType_returnsExpectedValue() {
        final InfinityMidPointQuadrature quadrature =
                new InfinityMidPointQuadrature(0.0, 1.0, null);
        assertEquals(QuadratureType.INFINITY_MID_POINT, quadrature.getType());
    }

    private Polynomial buildPolynomial() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double root = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        return new Polynomial(-root, 1.0);
    }
}