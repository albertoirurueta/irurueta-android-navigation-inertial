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
import static org.junit.Assert.assertTrue;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class LowerSquareRootMidPointQuadratureMatrixIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR_1 = 1e-7;

    private static final double ABSOLUTE_ERROR_3 = 1e-6;

    private static final double ABSOLUTE_ERROR_4 = 1e-5;

    private static final double ABSOLUTE_ERROR_6 = 1e-1;

    private static final double ABSOLUTE_ERROR_GAUSSIAN = 1e-9;

    private static final double ABSOLUTE_ERROR_IMPROPER_1 = 1e-5;

    @Test
    public void integrate_whenFirstDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(1, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenSecondDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(2, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenThirdDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(3, ABSOLUTE_ERROR_3);
    }

    @Test
    public void integrate_whenFourthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(4, ABSOLUTE_ERROR_4);
    }

    @Test
    public void integrate_whenSixthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(6, ABSOLUTE_ERROR_6);
    }

    @Test
    public void integrate_whenGaussian_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double mu = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sigma = ABSOLUTE_ERROR_GAUSSIAN
                + Math.abs(randomizer.nextDouble(MIN_VALUE, MAX_VALUE));

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        result.setElementAtIndex(0, NormalDist.p(point, mu, sigma));
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        final LowerSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new LowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);

        final double expected = NormalDist.cdf(b, mu, sigma) - NormalDist.cdf(a, mu, sigma);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_GAUSSIAN));
    }

    @Test
    public void integrate_whenImproperIntegrandWithSingularities_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        result.setElementAtIndex(0, Math.log(point) * Math.log(1 - point));
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        final LowerSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new LowerSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);

        final double expected = 2.0 - Math.PI * Math.PI / 6.0;

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_IMPROPER_1));
    }

    @Test
    public void getIntegratorType_returnsExpectedValue() throws WrongSizeException {
        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        final LowerSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new LowerSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
    }

    @Test
    public void getQuadratureType_returnsExpectedValue() throws WrongSizeException {
        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        final LowerSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new LowerSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());
    }

    private void assertPolynomialIntegration(final int degree, final double error)
            throws IntegrationException, WrongSizeException {
        final Polynomial polynomial = buildPolynomial(degree);
        final Polynomial integrationPolynomial = polynomial.integrationAndReturnNew();

        // set integration interval
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final double expected = integrationPolynomial.evaluate(b)
                - integrationPolynomial.evaluate(a);

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        result.setElementAtIndex(0, polynomial.evaluate(point));
                    }

                    @Override
                    public int getRows() {
                        return 1;
                    }

                    @Override
                    public int getColumns() {
                        return 1;
                    }
                };

        final LowerSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new LowerSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, error));
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