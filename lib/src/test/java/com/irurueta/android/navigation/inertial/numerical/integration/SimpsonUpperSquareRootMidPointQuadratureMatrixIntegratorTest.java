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
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class SimpsonUpperSquareRootMidPointQuadratureMatrixIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR_EXPONENTIAL = 1e-3;

    private static final double ABSOLUTE_ERROR_IMPROPER_1 = 1e-5;

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

        final SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                        ABSOLUTE_ERROR_EXPONENTIAL);

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

        final SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(a, b, listener,
                        ABSOLUTE_ERROR_EXPONENTIAL);

        final Matrix integrationResult = new Matrix(2, 2);
        integrator.integrate(integrationResult);

        // check
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_EXPONENTIAL));
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

        final SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final double expected = 2.0 - Math.PI * Math.PI / 6.0;
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

        final SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
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

        final SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonUpperSquareRootMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());
    }
}