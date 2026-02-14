/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.processors.filters

import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import org.junit.Assert.*
import org.junit.Test

class LowPassAveragingFilterTest {

    @Test
    fun constructor_whenTimeConstant_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
            timeConstant
        )

        // check
        assertEquals(timeConstant, filter.timeConstant, 0.0)

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)
    }

    @Test
    fun constructor_whenInvalidTimeConstant_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
                -1.0
            )
        }
    }

    @Test
    fun constructor_whenDefault_setsExpectedParameters() {
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        // check
        assertEquals(
            AveragingFilter.DEFAULT_TIME_CONSTANT,
            filter.timeConstant,
            0.0
        )

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)
    }

    @Test
    fun constructor_whenCopy_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter1 = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
            timeConstant
        )

        val timestamp = System.nanoTime()
        setPrivateProperty(
            AveragingFilter::class, filter1, "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class, filter1,
                "previousTimestamp"
            )
        )

        val filter2 = LowPassAveragingFilter(filter1)

        // check
        assertEquals(timeConstant, filter1.timeConstant, 0.0)
        assertEquals(timeConstant, filter2.timeConstant, 0.0)

        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class, filter1,
                "previousTimestamp"
            )
        )
    }

    @Test
    fun filter_whenFirstMeasurement_returnsFalseAndExpectedValues() {
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val inputTriad = AccelerationTriad(valueX1, valueY1, valueZ1)
        val outputTriad = AccelerationTriad()
        val timestamp = System.nanoTime()

        // filter one time
        assertFalse(filter.filter(inputTriad, outputTriad, timestamp))

        assertEquals(0.0, outputTriad.valueX, 0.0)
        assertEquals(0.0, outputTriad.valueY, 0.0)
        assertEquals(0.0, outputTriad.valueZ, 0.0)
    }

    @Test
    fun filter_whenMultipleMeasurement_returnsExpectedValues() {
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val alpha = AveragingFilter.DEFAULT_TIME_CONSTANT /
                (AveragingFilter.DEFAULT_TIME_CONSTANT + TIME_INTERVAL)
        val beta = 1.0 - alpha

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val input1 = AccelerationTriad(valueX1, valueY1, valueZ1)
        val output = AccelerationTriad()
        val timestamp = System.nanoTime()

        // filter one time
        assertFalse(filter.filter(input1, output, timestamp))

        assertEquals(0.0, output.valueX, 0.0)
        assertEquals(0.0, output.valueY, 0.0)
        assertEquals(0.0, output.valueZ, 0.0)

        // filter second time
        val outputX1 = output.valueX
        val outputY1 = output.valueY
        val outputZ1 = output.valueZ
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        val input2 = AccelerationTriad(valueX2, valueY2, valueZ2)
        val timestamp2 = timestamp + TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input2, output, timestamp2))

        // check
        assertEquals(alpha * outputX1 + beta * valueX2, output.valueX, 0.0)
        assertEquals(alpha * outputY1 + beta * valueY2, output.valueY, 0.0)
        assertEquals(alpha * outputZ1 + beta * valueZ2, output.valueZ, 0.0)
    }

    @Test
    fun filter_whenZeroTimeInterval_returnsFalse() {
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val alpha = AveragingFilter.DEFAULT_TIME_CONSTANT /
                (AveragingFilter.DEFAULT_TIME_CONSTANT + TIME_INTERVAL)
        val beta = 1.0 - alpha

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val input1 = AccelerationTriad(valueX1, valueY1, valueZ1)
        val output = AccelerationTriad()
        val timestamp = System.nanoTime()

        // filter one time
        assertFalse(filter.filter(input1, output, timestamp))

        assertEquals(0.0, output.valueX, 0.0)
        assertEquals(0.0, output.valueY, 0.0)
        assertEquals(0.0, output.valueZ, 0.0)

        // filter second time
        val outputX1 = output.valueX
        val outputY1 = output.valueY
        val outputZ1 = output.valueZ
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        val input2 = AccelerationTriad(valueX2, valueY2, valueZ2)
        assertTrue(filter.filter(input2, output, timestamp + TIME_INTERVAL_NANOS))

        // check
        assertEquals(alpha * outputX1 + beta * valueX2, output.valueX, 0.0)
        assertEquals(alpha * outputY1 + beta * valueY2, output.valueY, 0.0)
        assertEquals(alpha * outputZ1 + beta * valueZ2, output.valueZ, 0.0)

        // filter third time (with zero time interval respect previous one sample)
        val outputX2 = output.valueX
        val outputY2 = output.valueY
        val outputZ2 = output.valueZ
        val valueX3 = randomizer.nextDouble()
        val valueY3 = randomizer.nextDouble()
        val valueZ3 = randomizer.nextDouble()
        val input3 = AccelerationTriad(valueX3, valueY3, valueZ3)
        assertFalse(filter.filter(input3, output, timestamp + TIME_INTERVAL_NANOS))

        // check
        assertEquals(outputX2, output.valueX, 0.0)
        assertEquals(outputY2, output.valueY, 0.0)
        assertEquals(outputZ2, output.valueZ, 0.0)
    }

    @Test
    fun reset_resetsPreviousTimestamp() {
        val filter = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val timestamp = System.nanoTime()
        setPrivateProperty(
            AveragingFilter::class,
            filter,
            "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class, filter, "previousTimestamp"
            )
        )

        filter.reset()

        // check
        assertEquals(
            -1L,
            getPrivateProperty(
                AveragingFilter::class,
                filter,
                "previousTimestamp"
            )
        )
    }

    @Test
    fun copyFrom_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val timestamp = System.nanoTime()
        setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class, filter1, "previousTimestamp"
            )
        )

        val filter2 = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        filter2.copyFrom(filter1)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class,
                filter2,
                "previousTimestamp"
            )
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    @Test
    fun copyTo_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        val timestamp = System.nanoTime()
        setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class, filter1, "previousTimestamp"
            )
        )

        val filter2 = LowPassAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

        filter1.copyTo(filter2)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(
                AveragingFilter::class,
                filter2,
                "previousTimestamp"
            )
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    private companion object {
        const val TIME_INTERVAL = 0.02
        const val TIME_INTERVAL_NANOS = 20_000_000
    }
}