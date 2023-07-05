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

import static org.junit.Assert.*;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class SimpsonInfinityMidPointQuadratureMatrixIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double ABSOLUTE_ERROR_GAUSSIAN = 1e-10;

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

        final SimpsonInfinityMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureMatrixIntegrator(a, b, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);

        // check
        final Matrix expectedResult = new Matrix(1, 1);
        expectedResult.setElementAtIndex(0, expected);
        assertTrue(expectedResult.equals(integrationResult, ABSOLUTE_ERROR_GAUSSIAN));
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

        final SimpsonInfinityMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
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

        final SimpsonInfinityMidPointQuadratureMatrixIntegrator integrator =
                new SimpsonInfinityMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());
    }
}