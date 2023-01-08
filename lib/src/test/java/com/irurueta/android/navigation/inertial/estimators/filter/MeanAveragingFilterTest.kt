/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators.filter

import android.os.SystemClock
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class MeanAveragingFilterTest {

    @Test
    fun constructor_whenTimeConstant_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter = MeanAveragingFilter(timeConstant)

        // check
        assertEquals(timeConstant, filter.timeConstant, 0.0)

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)
    }

    @Test
    fun constructor_whenDefault_setsExpectedParameters() {
        val filter = MeanAveragingFilter()

        // check
        assertEquals(AveragingFilter.DEFAULT_TIME_CONSTANT, filter.timeConstant, 0.0)

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)
    }

    @Test
    fun constructor_whenCopy_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter1 = MeanAveragingFilter(timeConstant)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp", timestamp)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp")
        )

        val values1: ArrayDeque<DoubleArray>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        val value2 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter(filter1)

        // check
        assertEquals(timeConstant, filter1.timeConstant, 0.0)
        assertEquals(timeConstant, filter2.timeConstant, 0.0)

        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )

        val values2: ArrayDeque<DoubleArray>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertArrayEquals(values1[0], values2[0], 0.0)
        assertArrayEquals(value1, values2[0], 0.0)
        assertNotSame(values1[0], values2[0])
        assertArrayEquals(values1[1], values2[1], 0.0)
        assertArrayEquals(value2, values2[1], 0.0)
        assertNotSame(values1[1], values2[1])
    }

    @Test(expected = IllegalArgumentException::class)
    fun filter_whenInvalidLength_throwsIllegalArgumentException() {
        val filter = MeanAveragingFilter()

        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()
        val output = DoubleArray(2)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        filter.filter(valueX, valueY, valueZ, output, timestamp)
    }

    @Test
    fun filter_whenFirstMeasurement_returnsFalse() {
        val filter = MeanAveragingFilter()

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val output = DoubleArray(AveragingFilter.OUTPUT_LENGTH)
        val timestamp = SystemClock.elapsedRealtimeNanos()

        // filter one time
        assertFalse(filter.filter(valueX1, valueY1, valueZ1, output, timestamp))
    }

    @Test
    fun filter_whenValidLengthAndEmpty_returnsExpectedValues() {
        val filter = MeanAveragingFilter()

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val output = DoubleArray(AveragingFilter.OUTPUT_LENGTH)
        val timestamp = SystemClock.elapsedRealtimeNanos()

        // filter one time
        assertFalse(filter.filter(valueX1, valueY1, valueZ1, output, timestamp))

        // check
        assertEquals(0.0, output[0], 0.0)
        assertEquals(0.0, output[1], 0.0)
        assertEquals(0.0, output[2], 0.0)

        // filter second time
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX2,
                valueY2,
                valueZ2,
                output,
                timestamp + TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals(valueX2, output[0], 0.0)
        assertEquals(valueY2, output[1], 0.0)
        assertEquals(valueZ2, output[2], 0.0)

        // filter third time
        val valueX3 = randomizer.nextDouble()
        val valueY3 = randomizer.nextDouble()
        val valueZ3 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX3,
                valueY3,
                valueZ3,
                output,
                timestamp + 2 * TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals((valueX2 + valueX3) / 2.0, output[0], 0.0)
        assertEquals((valueY2 + valueY3) / 2.0, output[1], 0.0)
        assertEquals((valueZ2 + valueZ3) / 2.0, output[2], 0.0)

        // filter 4th time
        val valueX4 = randomizer.nextDouble()
        val valueY4 = randomizer.nextDouble()
        val valueZ4 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX4,
                valueY4,
                valueZ4,
                output,
                timestamp + 3 * TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals((valueX2 + valueX3 + valueX4) / 3.0, output[0], ABSOLUTE_ERROR)
        assertEquals((valueY2 + valueY3 + valueY4) / 3.0, output[1], ABSOLUTE_ERROR)
        assertEquals((valueZ2 + valueZ3 + valueZ4) / 3.0, output[2], ABSOLUTE_ERROR)

        // filter 5th time
        val valueX5 = randomizer.nextDouble()
        val valueY5 = randomizer.nextDouble()
        val valueZ5 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX5,
                valueY5,
                valueZ5,
                output,
                timestamp + 4 * TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals(
            (valueX2 + valueX3 + valueX4 + valueX5) / 4.0,
            output[0],
            0.0
        )
        assertEquals(
            (valueY2 + valueY3 + valueY4 + valueY5) / 4.0,
            output[1],
            0.0
        )
        assertEquals(
            (valueZ2 + valueZ3 + valueZ4 + valueZ5) / 4.0,
            output[2],
            0.0
        )

        // filter 6th time
        val valueX6 = randomizer.nextDouble()
        val valueY6 = randomizer.nextDouble()
        val valueZ6 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX6,
                valueY6,
                valueZ6,
                output,
                timestamp + 5 * TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals(
            (valueX2 + valueX3 + valueX4 + valueX5 + valueX6) / 5.0,
            output[0],
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueY2 + valueY3 + valueY4 + valueY5 + valueY6) / 5.0,
            output[1],
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueZ2 + valueZ3 + valueZ4 + valueZ5 + valueZ6) / 5.0,
            output[2],
            ABSOLUTE_ERROR
        )

        // filter 7th time
        val valueX7 = randomizer.nextDouble()
        val valueY7 = randomizer.nextDouble()
        val valueZ7 = randomizer.nextDouble()
        assertTrue(
            filter.filter(
                valueX7,
                valueY7,
                valueZ7,
                output,
                timestamp + 6 * TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals(
            (valueX3 + valueX4 + valueX5 + valueX6  + valueX7) / 5.0,
            output[0],
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueY3 + valueY4 + valueY5 + valueY6 + valueY7) / 5.0,
            output[1],
            ABSOLUTE_ERROR
        )
        assertEquals(
            (valueZ3 + valueZ4 + valueZ5 + valueZ6 + valueZ7) / 5.0,
            output[2],
            ABSOLUTE_ERROR
        )
    }

    @Test
    fun filter_whenZeroTimeInterval_returnsFalse() {
        val filter = MeanAveragingFilter()

        val randomizer = UniformRandomizer()
        val valueX1 = randomizer.nextDouble()
        val valueY1 = randomizer.nextDouble()
        val valueZ1 = randomizer.nextDouble()
        val output = DoubleArray(AveragingFilter.OUTPUT_LENGTH)
        val timestamp = SystemClock.elapsedRealtimeNanos()

        // filter one time
        assertFalse(filter.filter(valueX1, valueY1, valueZ1, output, timestamp))

        // check
        assertEquals(0.0, output[0], 0.0)
        assertEquals(0.0, output[1], 0.0)
        assertEquals(0.0, output[2], 0.0)

        // filter second time
        val valueX2 = randomizer.nextDouble()
        val valueY2 = randomizer.nextDouble()
        val valueZ2 = randomizer.nextDouble()
        assertFalse(
            filter.filter(
                valueX2,
                valueY2,
                valueZ2,
                output,
                timestamp
            )
        )

        // check
        assertEquals(0.0, output[0], 0.0)
        assertEquals(0.0, output[1], 0.0)
        assertEquals(0.0, output[2], 0.0)
    }

    @Test
    fun reset_resetsTimeInterval() {
        val filter = MeanAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(
            AveragingFilter::class,
            filter,
            "previousTimestamp",
            timestamp
        )
        val values: ArrayDeque<DoubleArray>? = filter.getPrivateProperty("values")
        requireNotNull(values)
        val valuesSpy = spyk(values)
        filter.setPrivateProperty("values", valuesSpy)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        )

        filter.reset()

        // check
        assertEquals(
            -1L,
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        )
        verify(exactly = 1) { valuesSpy.clear() }
    }

    @Test
    fun copyFrom_copiesTimeIntervalEstimatorInitialTimestampAndValues() {
        val filter1 = MeanAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp")
        )

        val randomizer = UniformRandomizer()
        val values1: ArrayDeque<DoubleArray>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        val value2 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter()

        filter2.copyFrom(filter1)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)

        val values2: ArrayDeque<DoubleArray>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertArrayEquals(values1[0], values2[0], 0.0)
        assertArrayEquals(value1, values2[0], 0.0)
        assertNotSame(values1[0], values2[0])
        assertArrayEquals(values1[1], values2[1], 0.0)
        assertArrayEquals(value2, values2[1], 0.0)
        assertNotSame(values1[1], values2[1])
    }

    @Test
    fun copyTo_copiesTimeIntervalEstimatorInitialTimestampAndValues() {
        val filter1 = MeanAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "previousTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp")
        )

        val randomizer = UniformRandomizer()
        val values1: ArrayDeque<DoubleArray>? = filter1.getPrivateProperty("values")
        requireNotNull(values1)
        val value1 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        val value2 =
            doubleArrayOf(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble())
        values1.addLast(value1)
        values1.addLast(value2)

        val filter2 = MeanAveragingFilter()

        filter1.copyTo(filter2)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)

        val values2: ArrayDeque<DoubleArray>? = filter2.getPrivateProperty("values")
        requireNotNull(values2)
        assertNotSame(values1, values2)
        assertEquals(values1.size, values2.size)
        assertArrayEquals(values1[0], values2[0], 0.0)
        assertArrayEquals(value1, values2[0], 0.0)
        assertNotSame(values1[0], values2[0])
        assertArrayEquals(values1[1], values2[1], 0.0)
        assertArrayEquals(value2, values2[1], 0.0)
        assertNotSame(values1[1], values2[1])
    }

    private companion object {
        const val TIME_INTERVAL_NANOS = 20000000

        const val ABSOLUTE_ERROR = 1e-12
    }
}