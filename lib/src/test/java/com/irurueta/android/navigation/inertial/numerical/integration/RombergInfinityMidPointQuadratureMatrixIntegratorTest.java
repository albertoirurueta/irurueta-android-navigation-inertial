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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

import org.junit.Test;

public class RombergInfinityMidPointQuadratureMatrixIntegratorTest {

    private static final double ALMOST_INFINITY = 1e99;

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegrandWithSingularities_returnsExpectedResult()
            throws IntegrationException, WrongSizeException {

        final MatrixSingleDimensionFunctionEvaluatorListener listener =
                new MatrixSingleDimensionFunctionEvaluatorListener() {

                    @Override
                    public void evaluate(double point, Matrix result) {
                        result.setElementAtIndex(0, Math.log(point) * Math.log(1.0 - point));
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

        final RombergInfinityMidPointQuadratureMatrixIntegrator integrator =
                new RombergInfinityMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);

        final Matrix integrationResult = new Matrix(1, 1);
        integrator.integrate(integrationResult);
    }

    @Test(expected = IntegrationException.class)
    public void integrate_whenImproperIntegralFromZeroToInfinity3_returnsWrongResult()
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

        final RombergInfinityMidPointQuadratureMatrixIntegrator integrator =
                new RombergInfinityMidPointQuadratureMatrixIntegrator(0.0, ALMOST_INFINITY, listener);

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

        final RombergInfinityMidPointQuadratureMatrixIntegrator integrator =
                new RombergInfinityMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
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

        final RombergInfinityMidPointQuadratureMatrixIntegrator integrator =
                new RombergInfinityMidPointQuadratureMatrixIntegrator(0.0, 1.0, listener);
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());
    }
}