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
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class SimpsonTrapezoidalQuadratureIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double MIN_LAMBDA = -1.0;

    private static final double MAX_LAMBDA = 1.0;

    private static final double ABSOLUTE_ERROR_1 = 1e-10;

    private static final double ABSOLUTE_ERROR_4 = 1e-7;

    private static final double ABSOLUTE_ERROR_5 = 1e-5;

    private static final double ABSOLUTE_ERROR_GAUSSIAN = 1e-10;

    private static final double ABSOLUTE_ERROR_EXPONENTIAL = 1e-9;

    private static final double ALMOST_INFINITY = 1e99;

    @Test
    public void integrate_whenFirstDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(1, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenSecondDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(2, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenThirdDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(3, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenFourthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(4, ABSOLUTE_ERROR_4);
    }

    @Test
    public void integrate_whenFifthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(5, ABSOLUTE_ERROR_5);
    }

    @Test
    public void integrate_whenSixthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException {
        assertPolynomialIntegration(6, ABSOLUTE_ERROR_5);
    }

    @Test
    public void integrate_whenGaussian_returnsExpectedResult() throws IntegrationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double mu = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sigma = ABSOLUTE_ERROR_GAUSSIAN
                + Math.abs(randomizer.nextDouble(a, MAX_VALUE));

        final double expected = NormalDist.cdf(b, mu, sigma) - NormalDist.cdf(a, mu, sigma);

        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(a, b,
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
    public void integrate_whenExponential_returnsExpectedResult() throws IntegrationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double lambda = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

        final double expected = 1.0 / lambda * (Math.exp(lambda * b) - Math.exp(lambda * a));

        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(a, b,
                        new SingleDimensionFunctionEvaluatorListener() {
                            @Override
                            public double evaluate(double point) {
                                return Math.exp(lambda * point);
                            }
                        });
        final double result = integrator.integrate();

        assertEquals(expected, result, ABSOLUTE_ERROR_EXPONENTIAL);
    }

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegrandWithSingularities_throwsIntegrationException()
            throws IntegrationException {

        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(0.0, 1.0,
                        new SingleDimensionFunctionEvaluatorListener() {
                            @Override
                            public double evaluate(double point) {
                                return Math.log(point) * Math.log(1 - point);
                            }
                        });
        integrator.integrate();
    }

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegralFromZeroToInfinity3_throwsIntegrationException()
            throws IntegrationException {

        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(0.0, ALMOST_INFINITY,
                        new SingleDimensionFunctionEvaluatorListener() {
                            @Override
                            public double evaluate(double point) {
                                return Math.pow(point, -2.0 / 7.0) * Math.exp(-point * point);
                            }
                        });
        integrator.integrate();
    }

    @Test
    public void getIntegratorType_returnsExpectedValue() {
        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
    }

    @Test
    public void getQuadratureType_returnsExpectedValue() {
        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    private void assertPolynomialIntegration(final int degree, final double error)
            throws IntegrationException {
        final Polynomial polynomial = buildPolynomial(degree);
        final Polynomial integrationPolynomial = polynomial.integrationAndReturnNew();

        // set integration interval
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final double expected = integrationPolynomial.evaluate(b)
                - integrationPolynomial.evaluate(a);

        final SimpsonTrapezoidalQuadratureIntegrator integrator =
                new SimpsonTrapezoidalQuadratureIntegrator(a, b,
                        new SingleDimensionFunctionEvaluatorListener() {
                            @Override
                            public double evaluate(final double point) {
                                return polynomial.evaluate(point);
                            }
                        });
        final double result = integrator.integrate();

        assertEquals(expected, result, error);
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