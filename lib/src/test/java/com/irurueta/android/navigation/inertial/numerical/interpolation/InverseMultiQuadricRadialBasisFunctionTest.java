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
package com.irurueta.android.navigation.inertial.numerical.interpolation;

import static org.junit.Assert.assertEquals;

import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class InverseMultiQuadricRadialBasisFunctionTest {

    @Test
    public void evaluate_whenDefaultScale_returnsExpectedValue() {
        final InverseMultiQuadricRadialBasisFunction rbf =
                new InverseMultiQuadricRadialBasisFunction();

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double r = randomizer.nextDouble();
        final double expected = 1.0 / Math.sqrt((r * r) + 1.0);

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }

    @Test
    public void evaluate_whenRandomScale_returnsExpectedValue() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double scale = randomizer.nextDouble();

        final InverseMultiQuadricRadialBasisFunction rbf =
                new InverseMultiQuadricRadialBasisFunction(scale);

        final double r = randomizer.nextDouble();
        final double expected = 1.0 / Math.sqrt((r * r) + (scale * scale));

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }
}