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

import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class RombergIntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double EPS = 1e-6;

    @Test
    public void create_whenAccuracyAndQuadratureType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                new SingleDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double point) {
                        return 0.0;
                    }
                };

        RombergIntegrator<?> integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS, QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, EPS,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndQuadratureType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                new SingleDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double point) {
                        return 0.0;
                    }
                };

        RombergIntegrator<?> integrator = RombergIntegrator.create(a, b, listener,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener, QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenNoQuadratureType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                new SingleDimensionFunctionEvaluatorListener() {
                    @Override
                    public double evaluate(double point) {
                        return 0.0;
                    }
                };

        RombergIntegrator<?> integrator = RombergIntegrator.create(a, b, listener, EPS);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = RombergIntegrator.create(a, b, listener);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }
}