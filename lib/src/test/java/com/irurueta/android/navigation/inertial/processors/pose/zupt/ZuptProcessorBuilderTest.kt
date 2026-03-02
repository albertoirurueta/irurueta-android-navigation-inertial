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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import kotlin.math.abs

class ZuptProcessorBuilderTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenDefault_setsDefaultSettings() {
        val builder = ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        // check settings
        val settings = builder.settings

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
    fun constructor_whenSettings_setsExpectedSettings() {
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

        val builder =
            ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>(settings)

        assertSame(settings, builder.settings)
    }

    @Test
    fun build_whenNoneProcessorType_createsExpectedProcessor() {
        val builder = ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>()

        val processor = builder.build()

        assertTrue(processor is NoneZuptProcessor)
    }

    @Test
    fun build_whenNonAdaptiveProcessorType_createsExpectedProcessor() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val windowNanoseconds = abs(randomizer.nextLong())
        val gravityThreshold = randomizer.nextDouble()
        val accelerometerVarianceThreshold = randomizer.nextDouble()
        val gyroscopeVarianceThreshold = randomizer.nextDouble()
        val processorType = ZuptProcessorType.NON_ADAPTIVE

        val settings = ZuptSettings(
            location,
            windowNanoseconds,
            gravityThreshold,
            accelerometerVarianceThreshold,
            gyroscopeVarianceThreshold,
            processorType = processorType
        )

        val builder =
            ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>(settings)

        val processor = builder.build()

        assertTrue(processor is NonAdaptiveZuptProcessor)

        val nonAdaptiveProcessor = processor as NonAdaptiveZuptProcessor

        assertSame(location, nonAdaptiveProcessor.location)
        assertEquals(
            gravityThreshold,
            nonAdaptiveProcessor.gravityThreshold,
            0.0
        )
        assertEquals(
            accelerometerVarianceThreshold,
            nonAdaptiveProcessor.accelerometerVarianceThreshold,
            0.0
        )
        assertEquals(
            gyroscopeVarianceThreshold,
            nonAdaptiveProcessor.gyroscopeVarianceThreshold,
            0.0
        )
        assertEquals(windowNanoseconds, nonAdaptiveProcessor.windowNanoseconds)
    }

    @Test
    fun build_adaptiveProcessorType_createsExpectedProcessor() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val windowNanoseconds = abs(randomizer.nextLong())
        val gravityThreshold = randomizer.nextDouble()
        val accelerometerNoiseVariance = randomizer.nextDouble()
        val gyroscopeNoiseVariance = randomizer.nextDouble()
        val accelerometerFactor = randomizer.nextDouble()
        val gyroscopeFactor = randomizer.nextDouble()
        val processorType = ZuptProcessorType.ADAPTIVE

        val settings = ZuptSettings(
            location,
            windowNanoseconds,
            gravityThreshold,
            accelerometerNoiseVariance = accelerometerNoiseVariance,
            gyroscopeNoiseVariance = gyroscopeNoiseVariance,
            accelerometerFactor = accelerometerFactor,
            gyroscopeFactor = gyroscopeFactor,
            processorType = processorType
        )

        val builder =
            ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>(settings)

        val processor = builder.build()

        assertTrue(processor is AdaptiveZuptProcessor)

        val adaptiveProcessor = processor as AdaptiveZuptProcessor

        assertSame(location, adaptiveProcessor.location)
        assertEquals(
            gravityThreshold,
            adaptiveProcessor.gravityThreshold,
            0.0
        )
        assertEquals(
            accelerometerNoiseVariance,
            adaptiveProcessor.accelerometerNoiseVariance,
            0.0
        )
        assertEquals(
            gyroscopeNoiseVariance,
            adaptiveProcessor.gyroscopeNoiseVariance,
            0.0
        )
        assertEquals(
            accelerometerFactor,
            adaptiveProcessor.accelerometerFactor,
            0.0
        )
        assertEquals(
            gyroscopeFactor,
            adaptiveProcessor.gyroscopeFactor,
            0.0
        )
        assertEquals(windowNanoseconds, adaptiveProcessor.windowNanoseconds)
    }

    @Test
    fun build_softProcessorType_createsExpectedProcessor() {
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val windowNanoseconds = abs(randomizer.nextLong())
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
            gravityWeight = gravityWeight,
            accelerometerWeight = accelerometerWeight,
            gyroscopeWeight = gyroscopeWeight,
            gravityNormalizationFactor = gravityNormalizationFactor,
            accelerometerNormalizationFactor = accelerometerNormalizationFactor,
            gyroscopeNormalizationFactor = gyroscopeNormalizationFactor,
            processorType = processorType
        )

        val builder =
            ZuptProcessorBuilder<AccelerometerAndGyroscopeSyncedSensorMeasurement>(settings)

        val processor = builder.build()

        assertTrue(processor is SoftZuptProcessor)

        val softProcessor = processor as SoftZuptProcessor

        assertSame(location, softProcessor.location)
        assertEquals(
            gravityWeight,
            softProcessor.gravityWeight,
            0.0
        )
        assertEquals(
            accelerometerWeight,
            softProcessor.accelerometerWeight,
            0.0
        )
        assertEquals(
            gyroscopeWeight,
            softProcessor.gyroscopeWeight,
            0.0
        )
        assertEquals(
            gravityNormalizationFactor,
            softProcessor.gravityNormalizationFactor,
            0.0
        )
        assertEquals(
            accelerometerNormalizationFactor,
            softProcessor.accelerometerNormalizationFactor,
            0.0
        )
        assertEquals(
            gyroscopeNormalizationFactor,
            softProcessor.gyroscopeNormalizationFactor,
            0.0
        )
        assertEquals(windowNanoseconds, softProcessor.windowNanoseconds)
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