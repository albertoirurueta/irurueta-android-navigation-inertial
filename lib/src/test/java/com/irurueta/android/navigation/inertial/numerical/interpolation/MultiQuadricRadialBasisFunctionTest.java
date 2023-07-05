package com.irurueta.android.navigation.inertial.numerical.interpolation;

import static org.junit.Assert.*;

import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class MultiQuadricRadialBasisFunctionTest {

    @Test
    public void evaluate_whenDefaultScale_returnsExpectedValue() {
        final MultiQuadricRadialBasisFunction rbf =
                new MultiQuadricRadialBasisFunction();

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double r = randomizer.nextDouble();
        final double expected = Math.sqrt((r * r) + 1.0);

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }

    @Test
    public void evaluate_whenRandomScale_returnsExpectedValue() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double scale = randomizer.nextDouble();

        final MultiQuadricRadialBasisFunction rbf =
                new MultiQuadricRadialBasisFunction(scale);

        final double r = randomizer.nextDouble();
        final double expected = Math.sqrt((r * r) + (scale * scale));

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }
}