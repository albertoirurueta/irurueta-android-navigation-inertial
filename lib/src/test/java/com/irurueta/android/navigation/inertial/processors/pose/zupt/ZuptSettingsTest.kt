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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import android.location.Location
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import kotlin.math.abs

class ZuptSettingsTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenDefaultValues_setsExpectedResults() {
        val settings = ZuptSettings()

        // check
        assertNull(settings.location)
        assertEquals(ZuptProcessor.DEFAULT_WINDOW_NANOSECONDS, settings.windowNanoseconds)
        assertEquals(
            ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD,
            settings.gravityThreshold,
            0.0
        )
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD,
            settings.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            NonAdaptiveZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD,
            settings.gyroscopeVarianceThreshold,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE,
            settings.accelerometerNoiseVariance,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE,
            settings.gyroscopeNoiseVariance,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
            settings.accelerometerFactor,
            0.0
        )
        assertEquals(
            AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
            settings.gyroscopeFactor,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_WEIGHT,
            settings.gravityWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_WEIGHT,
            settings.accelerometerWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_WEIGHT,
            settings.gyroscopeWeight,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GRAVITY_NORMALIZATION_FACTOR,
            settings.gravityNormalizationFactor,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR,
            settings.accelerometerNormalizationFactor,
            0.0
        )
        assertEquals(
            SoftZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR,
            settings.gyroscopeNormalizationFactor,
            0.0
        )
        assertEquals(ZuptProcessorType.NONE, settings.processorType)
    }

    @Test
    fun constructor_whenSpecificValues_setsExpectedValues() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val windowNanoseconds = abs(randomizer.nextLong())
        val gravityThreshold = randomizer.nextDouble()
        val accelerometerVarianceThreshold = randomizer.nextDouble()
        val gyroscopeVarianceThreshold = randomizer.nextDouble()
        val accelerometerNoiseVariance = randomizer.nextDouble()
        val gyroscopeNoiseVariance = randomizer.nextDouble()
        val accelerometerFactor = randomizer.nextDouble()
        val gyroscopeFactor = randomizer.nextDouble()
        val gravityWeight = randomizer.nextDouble()
        val accelerometerWeight = randomizer.nextDouble()
        val gyroscopeWeight = randomizer.nextDouble()
        val gravityNormalizationFactor = randomizer.nextDouble()
        val accelerometerNormalizationFactor = randomizer.nextDouble()
        val gyroscopeNormalizationFactor = randomizer.nextDouble()
        val processorType = ZuptProcessorType.SOFT

        val settings = ZuptSettings(
            location,
            windowNanoseconds,
            gravityThreshold,
            accelerometerVarianceThreshold,
            gyroscopeVarianceThreshold,
            accelerometerNoiseVariance,
            gyroscopeNoiseVariance,
            accelerometerFactor,
            gyroscopeFactor,
            gravityWeight,
            accelerometerWeight,
            gyroscopeWeight,
            gravityNormalizationFactor,
            accelerometerNormalizationFactor,
            gyroscopeNormalizationFactor,
            processorType
        )

        // check
        assertSame(location, settings.location)
        assertEquals(windowNanoseconds, settings.windowNanoseconds)
        assertEquals(gravityThreshold, settings.gravityThreshold, 0.0)
        assertEquals(accelerometerVarianceThreshold, settings.accelerometerVarianceThreshold, 0.0)
        assertEquals(gyroscopeVarianceThreshold, settings.gyroscopeVarianceThreshold, 0.0)
        assertEquals(accelerometerNoiseVariance, settings.accelerometerNoiseVariance, 0.0)
        assertEquals(gyroscopeNoiseVariance, settings.gyroscopeNoiseVariance, 0.0)
        assertEquals(accelerometerFactor, settings.accelerometerFactor, 0.0)
        assertEquals(gyroscopeFactor, settings.gyroscopeFactor, 0.0)
        assertEquals(gravityWeight, settings.gravityWeight, 0.0)
        assertEquals(accelerometerWeight, settings.accelerometerWeight, 0.0)
        assertEquals(gyroscopeWeight, settings.gyroscopeWeight, 0.0)
        assertEquals(gravityNormalizationFactor, settings.gravityNormalizationFactor, 0.0)
        assertEquals(accelerometerNormalizationFactor, settings.accelerometerNormalizationFactor, 0.0)
        assertEquals(gyroscopeNormalizationFactor, settings.gyroscopeNormalizationFactor, 0.0)
        assertEquals(processorType, settings.processorType)
    }

    @Test
    fun windowNanoseconds_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(windowNanoseconds = -1L)
        }
    }

    @Test
    fun gravityThreshold_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gravityThreshold = -1.0)
        }
    }

    @Test
    fun accelerometerVarianceThreshold_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerVarianceThreshold = -1.0)
        }
    }

    @Test
    fun gyroscopeVarianceThreshold_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeVarianceThreshold = -1.0)
        }
    }

    @Test
    fun accelerometerNoiseVariance_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerNoiseVariance = -1.0)
        }
    }

    @Test
    fun gyroscopeNoiseVariance_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeNoiseVariance = -1.0)
        }
    }

    @Test
    fun accelerometerFactor_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerFactor = -1.0)
        }
    }

    @Test
    fun gyroscopeFactor_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeFactor = -1.0)
        }
    }

    @Test
    fun gravityWeight_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gravityWeight = -1.0)
        }
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gravityWeight = 2.0)
        }
    }

    @Test
    fun accelerometerWeight_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerWeight = -1.0)
        }
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerWeight = 2.0)
        }
    }

    @Test
    fun gyroscopeWeight_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeWeight = -1.0)
        }
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeWeight = 2.0)
        }
    }

    @Test
    fun gravityNormalizationFactor_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gravityNormalizationFactor = -1.0)
        }
    }

    @Test
    fun accelerometerNormalizationFactor_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(accelerometerNormalizationFactor = -1.0)
        }
    }

    @Test
    fun gyroscopeNormalizationFactor_whenInvalid_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException::class.java) {
            ZuptSettings(gyroscopeNormalizationFactor = -1.0)
        }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(
            MIN_LATITUDE_DEGREES,
            MAX_LATITUDE_DEGREES
        )
        val longitudeDegrees = randomizer.nextDouble(
            MIN_LONGITUDE_DEGREES,
            MAX_LONGITUDE_DEGREES
        )
        val height = randomizer.nextDouble(
            MIN_HEIGHT,
            MAX_HEIGHT
        )

        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        return location
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0
    }
}