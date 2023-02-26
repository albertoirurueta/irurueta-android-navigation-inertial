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
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class LowPassAveragingFilterTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenTimeConstant_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter = LowPassAveragingFilter(timeConstant)

        // check
        assertEquals(timeConstant, filter.timeConstant, 0.0)

        val previousTimestamp: Long? =
            getPrivateProperty(AveragingFilter::class, filter, "previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)
    }

    @Test
    fun constructor_whenDefault_setsExpectedParameters() {
        val filter = LowPassAveragingFilter()

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
        val filter1 = LowPassAveragingFilter(timeConstant)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp", timestamp)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "previousTimestamp")
        )

        val filter2 = LowPassAveragingFilter(filter1)

        // check
        assertEquals(timeConstant, filter1.timeConstant, 0.0)
        assertEquals(timeConstant, filter2.timeConstant, 0.0)

        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun filter_whenInvalidLength_throwsIllegalArgumentException() {
        val filter = LowPassAveragingFilter()

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
        val filter = LowPassAveragingFilter()

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
    fun filter_whenValidLengthAndNotFirstTime_returnsExpectedValues() {
        val filter = LowPassAveragingFilter()

        val alpha =
            AveragingFilter.DEFAULT_TIME_CONSTANT / (AveragingFilter.DEFAULT_TIME_CONSTANT + TIME_INTERVAL)
        val beta = 1.0 - alpha

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
        val outputX1 = output[0]
        val outputY1 = output[1]
        val outputZ1 = output[2]
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
        assertEquals(alpha * outputX1 + beta * valueX2, output[0], 0.0)
        assertEquals(alpha * outputY1 + beta * valueY2, output[1], 0.0)
        assertEquals(alpha * outputZ1 + beta * valueZ2, output[2], 0.0)
    }

    @Test
    fun filter_whenZeroTimeInterval_returnsFalse() {
        val filter = LowPassAveragingFilter()

        val alpha =
            AveragingFilter.DEFAULT_TIME_CONSTANT / (AveragingFilter.DEFAULT_TIME_CONSTANT + TIME_INTERVAL)
        val beta = 1.0 - alpha

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
        val outputX1 = output[0]
        val outputY1 = output[1]
        val outputZ1 = output[2]
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
        assertEquals(alpha * outputX1 + beta * valueX2, output[0], 0.0)
        assertEquals(alpha * outputY1 + beta * valueY2, output[1], 0.0)
        assertEquals(alpha * outputZ1 + beta * valueZ2, output[2], 0.0)

        // filter third time (with zero time interval respect previous one sample)
        val outputX2 = output[0]
        val outputY2 = output[1]
        val outputZ2 = output[2]
        val valueX3 = randomizer.nextDouble()
        val valueY3 = randomizer.nextDouble()
        val valueZ3 = randomizer.nextDouble()
        assertFalse(
            filter.filter(
                valueX3,
                valueY3,
                valueZ3,
                output,
                timestamp + TIME_INTERVAL_NANOS
            )
        )

        // check
        assertEquals(outputX2, output[0], 0.0)
        assertEquals(outputY2, output[1], 0.0)
        assertEquals(outputZ2, output[2], 0.0)
    }

    @Test
    fun reset_resetsPreviousTimestamp() {
        val filter = LowPassAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        setPrivateProperty(
            AveragingFilter::class,
            filter,
            "previousTimestamp",
            timestamp
        )

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
    }

    @Test
    fun copyFrom_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter()

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

        val filter2 = LowPassAveragingFilter()

        filter2.copyFrom(filter1)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    @Test
    fun copyTo_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter()

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

        val filter2 = LowPassAveragingFilter()

        filter1.copyTo(filter2)

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "previousTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    private companion object {
        const val TIME_INTERVAL = 0.02
        const val TIME_INTERVAL_NANOS = 20_000_000
    }
}