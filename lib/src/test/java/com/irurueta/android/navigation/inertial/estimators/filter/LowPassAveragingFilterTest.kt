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
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.justRun
import io.mockk.mockk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class LowPassAveragingFilterTest {

    @Test
    fun constructor_whenTimeConstant_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter = LowPassAveragingFilter(timeConstant)

        // check
        assertEquals(timeConstant, filter.timeConstant, 0.0)
    }

    @Test
    fun constructor_whenDefault_setsExpectedParameters() {
        val filter = LowPassAveragingFilter()

        // check
        assertEquals(AveragingFilter.DEFAULT_TIME_CONSTANT, filter.timeConstant, 0.0)
    }

    @Test
    fun constructor_whenCopy_setsExpectedParameters() {
        val randomizer = UniformRandomizer()
        val timeConstant = randomizer.nextDouble()
        val filter1 = LowPassAveragingFilter(timeConstant)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "initialTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "initialTimestamp")
        )

        val filter2 = LowPassAveragingFilter(filter1)

        // check
        assertEquals(timeConstant, filter1.timeConstant, 0.0)
        assertEquals(timeConstant, filter2.timeConstant, 0.0)

        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "initialTimestamp")
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
    fun filter_whenValidLength_returnsExpectedValues() {
        val filter = LowPassAveragingFilter()

        val timeIntervalEstimator = mockk<TimeIntervalEstimator>()
        every { timeIntervalEstimator.averageTimeInterval }.returns(TIME_INTERVAL)
        every { timeIntervalEstimator.addTimestamp(any<Double>()) }.returns(true)
        every { timeIntervalEstimator.numberOfProcessedSamples }.returnsMany(0, 1, 2)
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter,
            "timeIntervalEstimator",
            timeIntervalEstimator
        )

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
        assertEquals(beta * valueX1, output[0], 0.0)
        assertEquals(beta * valueY1, output[1], 0.0)
        assertEquals(beta * valueZ1, output[2], 0.0)

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
    fun reset_resetsTimeInterval() {
        val filter = LowPassAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter,
            "initialTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter, "initialTimestamp")
        )

        val timeIntervalEstimator = mockk<TimeIntervalEstimator>()
        every { timeIntervalEstimator.reset() }.returns(true)
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter,
            "timeIntervalEstimator",
            timeIntervalEstimator
        )

        filter.reset()

        // check
        verify(exactly = 1) { timeIntervalEstimator.reset() }
        assertEquals(
            0L,
            getPrivateProperty(AveragingFilter::class, filter, "initialTimestamp")
        )
    }

    @Test
    fun copyFrom_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "initialTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "initialTimestamp")
        )

        val timeIntervalEstimator1 = mockk<TimeIntervalEstimator>()
        justRun { timeIntervalEstimator1.copyFrom(any()) }
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "timeIntervalEstimator",
            timeIntervalEstimator1
        )

        val filter2 = LowPassAveragingFilter()

        val timeIntervalEstimator2 = mockk<TimeIntervalEstimator>()
        justRun { timeIntervalEstimator2.copyFrom(any()) }
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter2,
            "timeIntervalEstimator",
            timeIntervalEstimator2
        )

        filter2.copyFrom(filter1)

        // check
        verify(exactly = 1) { timeIntervalEstimator2.copyFrom(timeIntervalEstimator1) }
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "initialTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    @Test
    fun copyTo_copiesTimeIntervalEstimatorAndInitialTimestamp() {
        val filter1 = LowPassAveragingFilter()

        val timestamp = SystemClock.elapsedRealtimeNanos()
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "initialTimestamp",
            timestamp
        )

        // check
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter1, "initialTimestamp")
        )

        val timeIntervalEstimator1 = mockk<TimeIntervalEstimator>()
        justRun { timeIntervalEstimator1.copyFrom(any()) }
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter1,
            "timeIntervalEstimator",
            timeIntervalEstimator1
        )

        val filter2 = LowPassAveragingFilter()

        val timeIntervalEstimator2 = mockk<TimeIntervalEstimator>()
        justRun { timeIntervalEstimator2.copyFrom(any()) }
        com.irurueta.android.navigation.inertial.setPrivateProperty(
            AveragingFilter::class,
            filter2,
            "timeIntervalEstimator",
            timeIntervalEstimator2
        )

        filter1.copyTo(filter2)

        // check
        verify(exactly = 1) { timeIntervalEstimator2.copyFrom(timeIntervalEstimator1) }
        assertEquals(
            timestamp,
            getPrivateProperty(AveragingFilter::class, filter2, "initialTimestamp")
        )
        assertEquals(filter1.timeConstant, filter2.timeConstant, 0.0)
    }

    private companion object {
        const val TIME_INTERVAL = 0.02
        const val TIME_INTERVAL_NANOS = 20000000
    }
}