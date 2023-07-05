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

import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class SimpsonInfinityMidPointQuadratureIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR_GAUSSIAN = 1e-10;

    @Test
    public void integrate_whenGaussian_returnsExpectedResult() throws IntegrationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        // Romberg Infinity Mid Point requires that a * b > 0.0
        // (either a and be are positive, or both a and be are negative)
        final double a = randomizer.nextDouble(0.0, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double mu = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sigma = ABSOLUTE_ERROR_GAUSSIAN
                + Math.abs(randomizer.nextDouble(a, MAX_VALUE));

        final double expected = NormalDist.cdf(b, mu, sigma) - NormalDist.cdf(a, mu, sigma);

        final SimpsonInfinityMidPointQuadratureIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureIntegrator(a, b,
                        new SingleDimensionFunctionEvaluatorListener() {
                            @Override
                            public double evaluate(double point) {
                                return NormalDist.p(point, mu, sigma);
                            }
                        });
        final double result = integrator.integrate();

        assertEquals(expected, result, ABSOLUTE_ERROR_GAUSSIAN);
    }

    @Test
    public void getIntegratorType_returnsExpectedValue() {
        final SimpsonInfinityMidPointQuadratureIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
    }

    @Test
    public void getQuadratureType_returnsExpectedValue() {
        final SimpsonInfinityMidPointQuadratureIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());
    }
}