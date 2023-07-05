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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.android.navigation.inertial.numerical.ExponentialMatrixEstimator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class RombergTrapezoidalQuadratureMatrixIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double MIN_LAMBDA = -1.0;

    private static final double MAX_LAMBDA = 1.0;

    private static final double ABSOLUTE_ERROR_1 = 1e-9;

    private static final double ABSOLUTE_ERROR_6 = 1e-8;

    private static final double ABSOLUTE_ERROR_GAUSSIAN = 1e-10;

    private static final double ABSOLUTE_ERROR_EXPONENTIAL = 1e-10;

    private static final double ALMOST_INFINITY = 1e99;

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
        assertPolynomialIntegration(3, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenFourthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(4, ABSOLUTE_ERROR_1);
    }

    @Test
    public void integrate_whenFifthDegreePolynomial_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {
        assertPolynomialIntegration(5, ABSOLUTE_ERROR_1);
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
                + Math.abs(randomizer.nextDouble(a, MAX_VALUE));

        final double expected = NormalDist.cdf(b, mu, sigma) - NormalDist.cdf(a, mu, sigma);

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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_GAUSSIAN));
    }

    @Test
    public void integrate_whenExponential1_returnsExpectedResult()
            throws WrongSizeException, IntegrationException {

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double lambda = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double t, Matrix result) {
                        result.setElementAtIndex(0, Math.exp(lambda * t));
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);

        final double expected = 1.0 / lambda * (Math.exp(lambda * b) - Math.exp(lambda * a));

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_EXPONENTIAL));
    }

    @Test
    public void integrate_whenExponential2_returnsExpectedResult()
            throws WrongSizeException, IntegrationException {
        // for single dimension functions, integral of f(t) = e^(a*t) is f(t) = 1/a*e^(a*t)
        // Consequently, for matrix function f(t) = e^(A*t) where A is a matrix, should be:
        // f(t) = A^-1 * e^(A*t)
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        // for matrix A = [0    1]
        //                [-1   0]

        // f(t) = e^(A*t) = [cos(t)     sin(t)]
        //                  [-sin(t)    cos(t)]

        // matrix A is antisymmetric and orthonormal, consequently, its inverse is its transpose:
        // A^-1 = A^T = [0  -1]
        //              [1   0]

        // A * A^T = [0   1][0  -1] = [0*0 + 1*1    -1*0 + 1*0 ] = [1  0]
        //           [-1  0][1   0]   [-1*0 + 0*1   -1*-1 + 0*0]   [0  1]

        // Consequently, the integral of f(t) is:
        // F(t) = A^-1*e^(A*t) = [0  -1][cos(t)     sin(t)] = [sin(t)  -cos(t)]
        //                       [1   0][-sin(t)    cos(t)]   [cos(t)   sin(t)]

        // Finally, integral of f(t) between a and b will be:
        // F(b) - F(a) = [sin(b) - sin(a)       -cos(b) + cos(a)]
        //               [cos(b) - cos(a)        sin(b) - sin(a)]

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double t, Matrix result) {
                        // f(t) = e^(A*t) = [cost(t)  sin(t)]
                        //                  [-sin(t)  cos(t)]
                        final double c = Math.cos(t);
                        final double s = Math.sin(t);
                        result.setElementAtIndex(0, c);
                        result.setElementAtIndex(1, -s);
                        result.setElementAtIndex(2, s);
                        result.setElementAtIndex(3, c);
                    }

                    @Override
                    public int getRows() {
                        return 2;
                    }

                    @Override
                    public int getColumns() {
                        return 2;
                    }
                };

        // F(b) - F(a) = [sin(b) - sin(a)       -cos(b) + cos(a)]
        //               [cos(b) - cos(a)        sin(b) - sin(a)]
        final Matrix expectedResult = new Matrix(2, 2);
        final double sinDiff = Math.sin(b) - Math.sin(a);
        final double cosDiff = Math.cos(b) - Math.cos(a);
        expectedResult.setElementAtIndex(0, sinDiff);
        expectedResult.setElementAtIndex(1, cosDiff);
        expectedResult.setElementAtIndex(2, -cosDiff);
        expectedResult.setElementAtIndex(3, sinDiff);

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);

        final Matrix integrationResult = new Matrix(2, 2);
        integrator.integrate(integrationResult);

        // check
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_EXPONENTIAL));
    }

    @Test
    public void integrate_whenExponential3_returnsExpectedResult()
            throws WrongSizeException, IntegrationException {
        // for single dimension functions, integral of f(t) = e^(a*t) is f(t) = 1/a*e^(a*t)
        // Consequently, for matrix function f(t) = e^(A*t) where A is a matrix, should be:
        // f(t) = A^-1 * e^(A*t)
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        // for matrix A = [0    1]
        //                [-1   0]

        // f(t) = e^(A*t) = [cos(t)     sin(t)]
        //                  [-sin(t)    cos(t)]

        // matrix A is antisymmetric and orthonormal, consequently, its inverse is its transpose:
        // A^-1 = A^T = [0  -1]
        //              [1   0]

        // A * A^T = [0   1][0  -1] = [0*0 + 1*1    -1*0 + 1*0 ] = [1  0]
        //           [-1  0][1   0]   [-1*0 + 0*1   -1*-1 + 0*0]   [0  1]

        // Consequently, the integral of f(t) is:
        // F(t) = A^-1*e^(A*t) = [0  -1][cos(t)     sin(t)] = [sin(t)  -cos(t)]
        //                       [1   0][-sin(t)    cos(t)]   [cos(t)   sin(t)]

        // Finally, integral of f(t) between a and b will be:
        // F(b) - F(a) = [sin(b) - sin(a)       -cos(b) + cos(a)]
        //               [cos(b) - cos(a)        sin(b) - sin(a)]

        final Matrix A = new Matrix(2, 2);
        A.setElementAtIndex(0, 0.0);
        A.setElementAtIndex(1, -1.0);
        A.setElementAtIndex(2, 1.0);
        A.setElementAtIndex(3, 0.0);

        final Matrix tmp = new Matrix(2, 2);

        final ExponentialMatrixEstimator exponentialMatrixEstimator =
                new ExponentialMatrixEstimator();

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double t, Matrix result) throws EvaluationException {
                        try {
                            // f(t) = e^(A*t) = [cost(t)  sin(t)]
                            //                  [-sin(t)  cos(t)]
                            tmp.copyFrom(A);
                            tmp.multiplyByScalar(t);
                            exponentialMatrixEstimator.exponential(tmp, result);
                        } catch (final AlgebraException ex) {
                            throw new EvaluationException(ex);
                        }
                    }

                    @Override
                    public int getRows() {
                        return 2;
                    }

                    @Override
                    public int getColumns() {
                        return 2;
                    }
                };

        // F(b) - F(a) = [sin(b) - sin(a)       -cos(b) + cos(a)]
        //               [cos(b) - cos(a)        sin(b) - sin(a)]
        final Matrix expectedResult = new Matrix(2, 2);
        final double sinDiff = Math.sin(b) - Math.sin(a);
        final double cosDiff = Math.cos(b) - Math.cos(a);
        expectedResult.setElementAtIndex(0, sinDiff);
        expectedResult.setElementAtIndex(1, cosDiff);
        expectedResult.setElementAtIndex(2, -cosDiff);
        expectedResult.setElementAtIndex(3, sinDiff);

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);

        final Matrix integrationResult = new Matrix(2, 2);
        integrator.integrate(integrationResult);

        // check
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_EXPONENTIAL));
    }

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegrandWithSingularities_returnsWrongResult()
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(0.0, 1.0, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);
    }

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegralFromZeroToInfinity3_throwsIntegrationException()
            throws IntegrationException, WrongSizeException {

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        result.setElementAtIndex(0,
                                Math.pow(point, -2.0 / 7.0) * Math.exp(-point * point));
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(0.0, ALMOST_INFINITY, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
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

        final RombergTrapezoidalQuadratureMatrixIntegrator integrator =
                new RombergTrapezoidalQuadratureMatrixIntegrator(a, b, listener);

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