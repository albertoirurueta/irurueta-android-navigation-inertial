package com.irurueta.android.navigation.inertial.numerical.interpolation;

import static org.junit.Assert.*;

import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

public class ThinPlateRadialBasisFunctionTest {

    @Test
    public void evaluate_whenDefaultScale_returnsExpectedValue() {
        final ThinPlateRadialBasisFunction rbf = new ThinPlateRadialBasisFunction();

        final UniformRandomizer randomizer = new UniformRandomizer();
        final double r = randomizer.nextDouble(-1.0, 1.0);
        final double expected = r <= 0.0 ? 0.0 : r * r * Math.log(r);

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }

    @Test
    public void evaluate_whenRandomScale_returnsExpectedValue() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double scale = randomizer.nextDouble();

        final ThinPlateRadialBasisFunction rbf = new ThinPlateRadialBasisFunction(scale);

        final double r = randomizer.nextDouble();
        final double expected = r <= 0.0 ? 0.0 : r * r * Math.log(r / scale);

        assertEquals(expected, rbf.evaluate(r), 0.0);
    }
}