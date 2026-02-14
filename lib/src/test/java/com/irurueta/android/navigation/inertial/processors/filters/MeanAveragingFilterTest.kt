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
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class MeanAveragingFilterTest {

    @Test
    fun constructor_whenTimeConstant_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
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
            MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
                -1.0
            )
        }
    }

    @Test
    fun constructor_whenDefault_setsExpectedParameters() {
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
        val filter1 = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>(
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

        val values1: ArrayDeque<AccelerationTriad>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        val value2 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter(filter1)

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

        val values2: ArrayDeque<AccelerationTriad>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertEquals(values1[0], values2[0])
        assertEquals(value1, values2[0])
        assertNotSame(values1[0], values2[0])
        assertEquals(values1[1], values2[1])
        assertEquals(value2, values2[1])
        assertNotSame(values1[1], values2[1])
    }

    @Test
    fun filter_whenFirstMeasurement_returnsFalseAndExpectedValues() {
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        val input2 = AccelerationTriad(valueX2, valueY2, valueZ2)
        val timestamp2 = timestamp + TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input2, output, timestamp2))

        // check
        assertEquals(valueX2, output.valueX, 0.0)
        assertEquals(valueY2, output.valueY, 0.0)
        assertEquals(valueZ2, output.valueZ, 0.0)

        // filter third time
        val valueX3 = randomizer.nextDouble()
        val valueY3 = randomizer.nextDouble()
        val valueZ3 = randomizer.nextDouble()
        val input3 = AccelerationTriad(valueX3, valueY3, valueZ3)
        val timestamp3 = timestamp + 2 * TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input3, output, timestamp3))

        // check
        assertEquals((valueX2 + valueX3) / 2.0, output.valueX, 0.0)
        assertEquals((valueY2 + valueY3) / 2.0, output.valueY, 0.0)
        assertEquals((valueZ2 + valueZ3) / 2.0, output.valueZ, 0.0)

        // filter 4th time
        val valueX4 = randomizer.nextDouble()
        val valueY4 = randomizer.nextDouble()
        val valueZ4 = randomizer.nextDouble()
        val input4 = AccelerationTriad(valueX4, valueY4, valueZ4)
        val timestamp4 = timestamp + 3 * TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input4, output, timestamp4))

        // check
        assertEquals(
            (valueX2 + valueX3 + valueX4) / 3.0,
            output.valueX,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueY2 + valueY3 + valueY4) / 3.0,
            output.valueY,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueZ2 + valueZ3 + valueZ4) / 3.0,
            output.valueZ,
            ABSOLUTE_ERROR
        )

        // filter 5th time
        val valueX5 = randomizer.nextDouble()
        val valueY5 = randomizer.nextDouble()
        val valueZ5 = randomizer.nextDouble()
        val input5 = AccelerationTriad(valueX5, valueY5, valueZ5)
        val timestamp5 = timestamp + 4 * TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input5, output, timestamp5))

        // check
        assertEquals(
            (valueX2 + valueX3 + valueX4 + valueX5) / 4.0,
            output.valueX,
            0.0
        )
        assertEquals(
            (valueY2 + valueY3 + valueY4 + valueY5) / 4.0,
            output.valueY,
            0.0
        )
        assertEquals(
            (valueZ2 + valueZ3 + valueZ4 + valueZ5) / 4.0,
            output.valueZ,
            0.0
        )

        // filter 6th time
        val valueX6 = randomizer.nextDouble()
        val valueY6 = randomizer.nextDouble()
        val valueZ6 = randomizer.nextDouble()
        val input6 = AccelerationTriad(valueX6, valueY6, valueZ6)
        val timestamp6 = timestamp + 5 * TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input6, output, timestamp6))

        // check
        assertEquals(
            (valueX2 + valueX3 + valueX4 + valueX5 + valueX6) / 5.0,
            output.valueX,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueY2 + valueY3 + valueY4 + valueY5 + valueY6) / 5.0,
            output.valueY,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueZ2 + valueZ3 + valueZ4 + valueZ5 + valueZ6) / 5.0,
            output.valueZ,
            ABSOLUTE_ERROR
        )

        // filter 7th time
        val valueX7 = randomizer.nextDouble()
        val valueY7 = randomizer.nextDouble()
        val valueZ7 = randomizer.nextDouble()
        val input7 = AccelerationTriad(valueX7, valueY7, valueZ7)
        val timestamp7 = timestamp + 6 * TIME_INTERVAL_NANOS
        assertTrue(filter.filter(input7, output, timestamp7))

        // check
        assertEquals(
            (valueX3 + valueX4 + valueX5 + valueX6 + valueX7) / 5.0,
            output.valueX,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueY3 + valueY4 + valueY5 + valueY6 + valueY7) / 5.0,
            output.valueY,
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueZ3 + valueZ4 + valueZ5 + valueZ6 + valueZ7) / 5.0,
            output.valueZ,
            ABSOLUTE_ERROR
        )
    }

    @Test
    fun filter_whenZeroTimeInterval_returnsFalse() {
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        val input2 = AccelerationTriad(valueX2, valueY2, valueZ2)
        assertFalse(filter.filter(input2, output, timestamp))

        // check
        assertEquals(0.0, output.valueX, 0.0)
        assertEquals(0.0, output.valueY, 0.0)
        assertEquals(0.0, output.valueZ, 0.0)
    }

    @Test
    fun reset_resetsTimeInterval() {
        val filter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
                AveragingFilter::class,
                filter,
                "previousTimestamp"
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
    fun copyFrom_copiesTimeIntervalEstimatorInitialTimestampAndValues() {
        val filter1 = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
                AveragingFilter::class,
                filter1,
                "previousTimestamp"
            )
        )

        val randomizer = UniformRandomizer()
        val values1: ArrayDeque<AccelerationTriad>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        val value2 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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

        val values2: ArrayDeque<AccelerationTriad>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertEquals(values1[0], values2[0])
        assertEquals(value1, values2[0])
        assertNotSame(values1[0], values2[0])
        assertEquals(values1[1], values2[1])
        assertEquals(value2, values2[1])
        assertNotSame(values1[1], values2[1])
    }

    @Test
    fun copyTo_copiesTimeIntervalEstimatorInitialTimestampAndValues() {
        val filter1 = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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
                AveragingFilter::class,
                filter1,
                "previousTimestamp"
            )
        )

        val randomizer = UniformRandomizer()
        val values1: ArrayDeque<AccelerationTriad>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        val value2 = AccelerationTriad(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()

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

        val values2: ArrayDeque<AccelerationTriad>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertEquals(values1[0], values2[0])
        assertEquals(value1, values2[0])
        assertNotSame(values1[0], values2[0])
        assertEquals(values1[1], values2[1])
        assertEquals(value2, values2[1])
        assertNotSame(values1[1], values2[1])
    }

    private companion object {
        const val TIME_INTERVAL_NANOS = 20000000

        const val ABSOLUTE_ERROR = 1e-12
    }
}