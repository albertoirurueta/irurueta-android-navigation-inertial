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
import static org.junit.Assert.fail;

import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.logging.Level;
import java.util.logging.Logger;

public class IntegratorTest {

    private static final double MIN_VALUE = -10.0;

    private static final double MAX_VALUE = 10.0;

    private static final double MIN_LAMBDA = -1.0;

    private static final double MAX_LAMBDA = 1.0;

    private static final double EPS = 1e-6;

    @Test
    public void create_whenAccuracyIntegratorAndQuadratureTypes_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof TrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof MidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof InfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof LowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof UpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof DoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof SimpsonMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof SimpsonInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof SimpsonDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyIntegratorAndQuadratureTypes_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof TrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof MidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof InfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof LowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof UpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof DoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            Integrator.create(a, b, listener, IntegratorType.QUADRATURE,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof SimpsonMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof SimpsonInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof SimpsonUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof SimpsonDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());

        try {
            Integrator.create(a, b, listener, IntegratorType.SIMPSON,
                    QuadratureType.EXPONENTIAL_MID_POINT);
            fail("IllegalArgumentException expected");
        } catch (final IllegalArgumentException ignore) {
        }

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.TRAPEZOIDAL);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.MID_POINT);
        assertTrue(integrator instanceof RombergMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.INFINITY_MID_POINT);
        assertTrue(integrator instanceof RombergInfinityMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.INFINITY_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.LOWER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergLowerSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.LOWER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.UPPER_SQUARE_ROOT_MID_POINT);
        assertTrue(integrator instanceof RombergUpperSquareRootMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.UPPER_SQUARE_ROOT_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.EXPONENTIAL_MID_POINT);
        assertTrue(integrator instanceof RombergExponentialMidPointQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.EXPONENTIAL_MID_POINT, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG,
                QuadratureType.DOUBLE_EXPONENTIAL_RULE);
        assertTrue(integrator instanceof RombergDoubleExponentialRuleQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.DOUBLE_EXPONENTIAL_RULE, integrator.getQuadratureType());
    }

    @Test
    public void create_whenAccuracyAndIntegratorType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener, EPS, IntegratorType.QUADRATURE);
        assertTrue(integrator instanceof TrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.SIMPSON);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, EPS, IntegratorType.ROMBERG);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndIntegratorType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener, IntegratorType.QUADRATURE);
        assertTrue(integrator instanceof TrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.QUADRATURE, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.SIMPSON);
        assertTrue(integrator instanceof SimpsonTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.SIMPSON, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());

        integrator = Integrator.create(a, b, listener, IntegratorType.ROMBERG);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenAccuracyAndDefaultIntegratorType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener, EPS);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void create_whenDefaultAccuracyAndDefaultIntegratorType_returnsExpectedIntegrator() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);

        final SingleDimensionFunctionEvaluatorListener listener =
                point -> 0.0;

        Integrator integrator = Integrator.create(a, b, listener);
        assertTrue(integrator instanceof RombergTrapezoidalQuadratureIntegrator);
        assertEquals(IntegratorType.ROMBERG, integrator.getIntegratorType());
        assertEquals(QuadratureType.TRAPEZOIDAL, integrator.getQuadratureType());
    }

    @Test
    public void comparePerformance() throws IntegrationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double a = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double b = randomizer.nextDouble(a, MAX_VALUE);
        final double lambda = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

        final int[] evaluations = new int[1];
        final SingleDimensionFunctionEvaluatorListener listener =
                point -> {
                    evaluations[0] += 1;
                    return Math.exp(lambda * point);
                };

        final double expected = 1.0 / lambda * (Math.exp(lambda * b) - Math.exp(lambda * a));

        int bestEvaluations = Integer.MAX_VALUE;
        IntegratorType bestEvaluationsIntegratorType = null;
        QuadratureType bestEvaluationsQuadratureType = null;

        long bestDuration = Long.MAX_VALUE;
        IntegratorType bestDurationIntegratorType = null;
        QuadratureType bestDurationQuadratureType = null;

        double bestError = Double.MAX_VALUE;
        IntegratorType bestErrorIntegratorType = null;
        QuadratureType bestErrorQuadratureType = null;

        for (final IntegratorType integratorType : IntegratorType.values()) {
            for (final QuadratureType quadratureType : QuadratureType.values()) {
                final Integrator integrator = Integrator.create(a, b, listener);
                evaluations[0] = 0;
                final long start = System.nanoTime();
                final double result = integrator.integrate();
                final long end = System.nanoTime();
                final long duration = end - start;
                final double error = Math.abs(expected - result);

                Logger.getGlobal().log(Level.INFO, "Integrator type: " + integratorType
                        + ", Quadrature type: " + quadratureType
                        + ", evaluations: " + evaluations[0]
                        + ", duration: " + duration + "ns, error: " + error);

                if (evaluations[0] < bestEvaluations) {
                    bestEvaluations = evaluations[0];
                    bestEvaluationsIntegratorType = integratorType;
                    bestEvaluationsQuadratureType = quadratureType;
                }

                if (duration < bestDuration) {
                    bestDuration = duration;
                    bestDurationIntegratorType = integratorType;
                    bestDurationQuadratureType = quadratureType;
                }

                if (error < bestError) {
                    bestError = error;
                    bestErrorIntegratorType = integratorType;
                    bestErrorQuadratureType = quadratureType;
                }
            }
        }

        Logger.getGlobal().log(Level.INFO,
                "Best evaluations - Integrator type: " + bestEvaluationsIntegratorType
                + ", Quadrature type: " + bestEvaluationsQuadratureType
                + ", evaluations: " + bestEvaluations);

        Logger.getGlobal().log(Level.INFO,
                "Best duration - Integrator type: " + bestDurationIntegratorType
                + ", Quadrature type: " + bestDurationQuadratureType
                + ", duration: " + bestDuration + "ns");

        Logger.getGlobal().log(Level.INFO,
                "Best error - Integrator type: " + bestErrorIntegratorType
                + ", Quadrature type: " + bestErrorQuadratureType
                + ", error: " + bestError);
    }
}