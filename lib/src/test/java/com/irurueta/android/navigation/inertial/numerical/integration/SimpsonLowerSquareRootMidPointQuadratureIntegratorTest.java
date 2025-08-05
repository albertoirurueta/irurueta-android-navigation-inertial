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

import org.junit.Test;

public class SimpsonLowerSquareRootMidPointQuadratureIntegratorTest {

    private static final double ABSOLUTE_ERROR_IMPROPER_1 = 1e-5;

    @Test
    public void integrate_whenImproperIntegrandWithSingularities_returnsExpectedResult()
            throws IntegrationException {
        final double expected = 2.0 - Math.PI * Math.PI / 6.0;

        final SimpsonLowerSquareRootMidPointQuadratureIntegrator integrator =
                new SimpsonLowerSquareRootMidPointQuadratureIntegrator(0.0, 1.0,
                        point -> Math.log(point) * Math.log(1 - point));
        final double result = integrator.integrate();

        assertEquals(expected, result, ABSOLUTE_ERROR_IMPROPER_1);
    }

    @Test
    public void getIntegratorType_returnsExpectedValue() {
        final SimpsonLowerSquareRootMidPointQuadratureIntegrator integrator =
                new SimpsonLowerSquareRootMidPointQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
    }

    @Test
    public void getQuadratureType_returnsExpectedValue() {
        final SimpsonLowerSquareRootMidPointQuadratureIntegrator integrator =
                new SimpsonLowerSquareRootMidPointQuadratureIntegrator(0.0, 1.0, null);
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());
    }
}